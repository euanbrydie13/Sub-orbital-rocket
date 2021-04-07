%% RAPTOR PEREGRINE / Optimal Control problem - Single Shooting
clear all;

%% Load Location and Heading Seletion
location = location_select;

%% Load Vehicle Selection
vehicle = vehicle_select; 

%% Optimising Burn Stage - Single Shooting

% First Guess for control Law (Needs updated)
control_0 = [150; 0;0;0;0;0; 0.9;0.9;0.9;0.9;0.9; 0;0;0;0;0];                      % Initial input
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','interior-point');

% Lower and Upper Bounds Applied to Control
tlower = 130;
alphalower = -5.*(pi./180).*ones(5,1);
throttlelower = zeros(5,1);
banklower = -10.*(pi./180).*ones(5,1);

tupper = 160;
alphaupper= 15.*(pi./180).*ones(5,1);
throttleupper = ones(5,1);
bankupper = 10.*(pi./180).*ones(5,1);

LB = [tlower; alphalower; throttlelower; banklower];
UB = [tupper; alphaupper; throttleupper; bankupper];

% Calling fmincon
[control_opt,fval,exitflag,output] = fmincon(@Cost_Function,control_0,[],[],[],[],LB,UB,@Non_Linear_Constraints,options);

% Integrating using Optimal Control Law

Tspan1 = 0:1:control_opt(1);        % Tspan calculated by using control output 1
x0 = [100;50;89*(pi/180);location.heading;location.lat;location.lon;vehicle.wet]; % Initial Conditions
xstate1 = ode3(@(t,x,control) dynamics_model(t,x,control_opt),Tspan1,x0); % Integrating using ode3

% Integrate for remainder of trajectory

control_01 = [(600 - control_opt(1)); 0;0;0;0;0; 0;0;0;0;0; 0;0;0;0;0]; 
Tspan2 = control_opt(1):1:600;
x0t2 = xstate1(end,:);
x02 = x0t2';
xstate2 = ode3(@(t,x,control) dynamics_model(t,x,control_01),Tspan2,x02);

% Adding state vectors
x = [xstate1
     xstate2];

% Adding Timespans
Tspan = [Tspan1, Tspan2];

%% Display of State variables

% Trajectory Plot
figure
plot3(x(:,5),x(:,6),x(:,1)*1e-3);
grid on
xlabel('Latitude')
ylabel('Longitude')
zlabel('Altitude')

%Altitude and Velocity Plot
figure 
subplot(1,2,1);
plot(Tspan,x(:,1)*1e-3);
xlabel('Time(s)');
ylabel('Altitude (km)');
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(Tspan,x(:,2)*1e-3); 
xlabel('Time (s)');
ylabel('Velocity (km/s)');
ax = gca;
ax.FontSize = 15;

% Flight Path and Heading angle Plot
figure 
subplot(1,2,1);
plot(Tspan,x(:,3)*(180/pi));
xlabel('Time(s)');
ylabel('Flight path angle (deg)')
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(Tspan,x(:,4)*(180/pi));
xlabel('Time(s)');
ylabel('Heading angle (deg)') 
ax = gca;
ax.FontSize = 15;

%Convert results back into degrees for plotting
lat = rad2deg(x(:,5));
lon = rad2deg(x(:,6));
alt = x(:,1)*10^-3;
headingr = x(:,4);
heading = rad2deg(x(:,4));

%Creating figure pannel to plot the maps
figpos = [500 200 800 400];
uif = uifigure("Name","Trajectory Results","Position",figpos);
ug = uigridlayout(uif,[1,2]);
p1 = uipanel(ug);
p2 = uipanel(ug);
gx = geoaxes(p1,"Basemap","satellite"); 
gx.InnerPosition 

%Plotting trajectory data on maps
hold(gx,"on")
ptrack = geoplot(gx,lat,lon,"c","LineWidth",2);

%Setting markers on 2D map for start, end and current location
hold(gx,"on");
mstart = geoplot(gx,lat(1),lon(1),"ow","MarkerSize",10,"MarkerFaceColor","magenta");
mend = geoplot(gx,lat(end),lon(end),"ow","MarkerSize",10,"MarkerFaceColor","blue");

mstart.DisplayName = "Start Location";
mend.DisplayName = "End Location";
legend(gx)

%% Flight Corridor / Exclusion Zone
% This calculates the flight corridor from specfic start and end coordinates

% Starting coordinates
lat1 = location.lat;
lon1 = location.lon; 

chi1 = location.heading; 

% Drawing Start Circle
latS = rad2deg(x0(5));
lonS = rad2deg(x0(6));

R = 2440/110574;                % radius of circle 
C = [latS,lonS];                % center of circle 
th = linspace((pi/2+location.heading),(3*pi/2+location.heading));
x1  = C(1)+R*cos(th); 
y1 = C(2)+R*sin(th) ;

% Drawing End Circle

latF = rad2deg(x(end,5));
lonF = rad2deg(x(end,6));

R = (5889.36/110574);                          % radius of circle 
C = [latF,lonF];                               % center of circle 
th = linspace(((pi/2)+location.heading), (-pi/2 + location.heading));
x2  = C(1)+R*cos(th); 
y2 = C(2)+R*sin(th);

hold(gx,'on')
exc1 = geoplot(gx,x1,y1,'--r',"LineWidth",2);
exc2 = geoplot(gx,x2,y2,'--r',"LineWidth",2);
exc3 = geoplot(gx,[x1(1) x2(1)], [y1(1) y2(1)],'--r',"LineWidth",2);
exc4 = geoplot(gx,[x1(:,end) x2(:,end)], [y1(:,end) y2(:,end)],'--r',"LineWidth",2);

%% Objective Function - Maximising Altitude
function J2 = Cost_Function(control_opt)

vehicle = vehicle_select;
location = location_select;
Tspan = 0:1:control_opt(1);
x0 = [100; 50; 89*(pi/180); location.heading; location.lat; location.lon; vehicle.wet];

x = ode3(@(t,x,control_opt)dynamics_model(t,x,control_opt),Tspan,x0,control_opt);

J2 = -sum(x(:,1));

end

%% Nonlinear constraints 

function [c,ceq]=Non_Linear_Constraints(control_opt)

vehicle = vehicle_select;
location = location_select;
Tspan = 0:1:control_opt(1);
x0 = [100; 50; 89*(pi/180); location.heading; location.lat; location.lon; vehicle.wet];
x = ode3(@(t,x,control_opt) dynamics_model(t,x,control_opt),Tspan,x0,control_opt);

c = [];
ceq(1) = x(end,1) - 50000;
ceq(1) = x(end,2) - 950;
ceq(2) = x(end,3) - deg2rad(75);
%ceq(3) = x(end,4) - deg2rad(15); 
%ceq(4) = x(end,5) - ;
%ceq(4) = x(end,6) - ;
ceq(4) = x(end,7) - 100;
ceq = ceq';
end