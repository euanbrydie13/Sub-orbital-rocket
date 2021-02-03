close all
clear all
clc

%%Optimal Control problem

Tspan = 0:1:225;
x0= [10;1;1.57;1;1;1;2500]; %initial values of state variables

control_0 = [225; 1;1;1;1;1; 1;1;1;1;1]; %initial input
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','sqp');


[u_opt,fval,exitflag,output] = fmincon(@cost3,control_0,[],[],[],[],[],[],@nonlcon,options);
x = ode3(@(t,x,u_opt) Dynamics_test(t,x,u_opt),Tspan,x0,u_opt); 

%% Display of State variables

%Trajectory Plot
figure
plot3(x(:,6).*(180/pi),x(:,5).*(180/pi), x(:,1).*1e-3);
grid on
xlabel('Latitude')
ylabel('Longitude')
zlabel('Altitude')

%Altitude and Velocity Plot
figure 
subplot(1,2,1);
plot(tspan,x(:,1).*1e-3);
xlabel('Time(s)');
ylabel('Altitude (km)');
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(tspan,x(:,2).*1e-3); 
xlabel('Time (s)');
ylabel('Velocity (km/s)');
ax = gca;
ax.FontSize = 15;

% Flight Path and Heading angle Plot
figure 
subplot(1,2,1);
plot(tspan,x(:,3).*(180/pi));
xlabel('Time(s)');
ylabel('Flight path angle (deg)')
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(tspan,x(:,4).*(180/pi));
xlabel('Time(s)');
ylabel('Heading angle (deg)') 
ax = gca;
ax.FontSize = 15;

%Plot of Mass
figure
plot(tspan,x(:,7));
xlabel('Time(s)');
ylabel('Mass(kg)')
ax = gca;
ax.FontSize = 15;

function dx = Dynamics_test(t,x,control)
%% Control model 
tof = control(1);
n=(length(control)-1)/2;
alpha_all = control(2:n+1);
throttle_all = control(n+2:end);

T2 = linspace(0, tof, n);
alpha = interp1(T2,alpha_all,t);
throttle = interp1(T2,throttle_all,t);
bank = 0;

%% State equations from dissertation
h = x(1,:);   % altitude from the surface of the Earth to the vehicle, m
v = x(2,:);   % vehicle velocity relative to the Earth, m/s
fpa = x(3,:); % flight path angle (angle between the relative velocity and the local horizontal axis), rad
chi = x(4,:); % flight heading angle (angle between the velocity vector projected onto the local horizontal plane, and North), rad
lat = x(5,:); % latitude (+ North), rad
lon = x(6,:); % longitude (+ East), rad
m = x(7,:);   % vehicle mass, kg

%% Vehicle Specifications [SKYLARK L]
% Vehicle specifications need to be defined in own script
vehicle.sref = pi*(0.356)^2;         % Reference area (m^2)
vehicle.aexit = pi.*(0.1).^2;        % Exit Nozzle Area  (Assumed - need to find appropriate value) [m^2]
vehicle.isp = 229;                   % Specfic Impulse [s]
vehicle.tvac = 30e3;                 % Thrust Force (kn)

%% Atmospheric model 2
[press,dens] = atmo_euan(x(1));      % Pressure and Density calcualted from atmospheric model

%% Aerodynamic model Simple Rocket
%Determines Lift and drag forces
CL=0;                         % Coefficent of Lift (Since sounding rocket CL can be assumed 0)
CD=0;                         % Coefficent of Drag (Constant) 
Sref = vehicle.sref;          % Aerodynamic Reference Area [m^2]
qdyn = 0.5.*dens.*x(2).^2;    % Dynamic pressure
L = CL.*Sref.*qdyn;           % Force of Lift [N]
D = CD.*Sref.*qdyn;           % Force of Drag [N]

%% Gravity model

rE = 6378137;                % Earths radius [m]
r = h + rE;                  % Distance from centre of the earth [m]
g = 9.8066.*(rE./r).^2;      % Value of gravity at given altitiude [ms-2]
gr = g; gt = 0;              % Radial and Tangential gravitational acceleration [Tangential assumed to be zero]
wE = 7.292118.*10.^-05;      % Angular rotation of earth [rads-1]

%% Propulsion Model

Ae = vehicle.aexit;                             % Reference Area = pi.*(1).^2;
Isp = vehicle.isp;                              % Specfic Impulse 229;
Fsl = vehicle.tvac;                             % Max Thrust at Sea Level [N] - target T/W ~3

FT = throttle.*(Fsl + Ae.*(1e5 - press));       % Force of Thrust [N]
mp = throttle.*(FT./(9.81.*Isp));               % Mass flow [kgs-1]

%% 3DOF Dynamic Equations (6 ODEs)

% Equations taken from Tropico folder/Derivation avaiable in dissertation
dh = v.*sin(fpa);
dlat = v.*cos(fpa).*cos(chi)./r;
if  abs(lat-pi/2) < eps(1)
       dlon = 0;
else
dlon = v.*cos(fpa).*sin(chi)./(r.*cos(lat));
end

Fx = (FT.*cos(alpha) - D)./m - gr.*sin(fpa) + gt.*cos(fpa).*cos(chi);
dv = Fx + wE.^2.*r.*cos(lat).*(sin(fpa).*cos(lat) - cos(fpa).*cos(chi).*sin(lat));
if abs(v) < eps(1)
    dfpa = 0;
else
    Fz = (FT.*sin(alpha)+L).*cos(bank)./m - gr.*cos(fpa) - gt.*sin(fpa).*cos(chi);
    dfpa = (v/r).*cos(fpa) + Fz./v + (wE^2.*r./v).*cos(lat).*(sin(fpa).*cos(chi).*sin(lat) + cos(fpa).*cos(lat)) + 2.*wE.*sin(chi).*cos(lat);
end
%if (abs(lat - pi/2) < eps(1) || abs(lat + pi/2) < eps(1) || abs(fpa - pi/2) < eps(1) || abs(v) < eps(1))
     dchi = 0;
%else
%     Fy = (FT.*sin(alpha)+L).*sin(bank)./m - gt.*sin(chi);
%     dchi = (v./r).*cos(fpa).*sin(chi).*tan(lat) + Fy./(v.*cos(fpa)) ...
%         + wE^2.*r.*(sin(chi).*sin(lat).*cos(lat))./(v.*cos(fpa)) + 2.*wE.*(sin(lat)-tan(fpa).*cos(chi).*cos(lat));
%end
dm = -mp;

dx = [dh; dv; dfpa; dchi; dlat; dlon; dm];  
end

% Cost function 
function J2=cost3(control)
Tspan = 0:1:225;
x0= [10;1;1.57;1;1;1;2500]; %initial values of state variables

x = ode3(@(t,x,control) Dynamics_test(t,x,control),Tspan,x0,control);

J2 = -max(x(:,1));
end

%nonlinear constraints
function [c,ceq]=nonlcon(control)

Tspan = 0:1:225;
x0= [10;1;1.57;1;1;1;2500]; %initial values of state variables

x = ode3(@(t,x,control) Dynamics_test(t,x,control),Tspan,x0,control);
c=[];
ceq(1)= x(end,1) - 10000;
ceq = ceq';
end
