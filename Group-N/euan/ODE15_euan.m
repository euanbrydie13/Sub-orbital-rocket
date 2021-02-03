%% ODE15s Intergration using Optimal Control Law
% This code uses the optimal control vector and plots the corresponding
% outputs

%% Vehicle Specifications [SKYLARK L]
vehicle.sref = pi*(0.356)^2;           % Reference area (m^2)
vehicle.aexit = pi.*(0.1).^2;          % Exit Nozzle Area  (Assumed - need to find appropriate value) [m^2]
vehicle.isp = 229;                     % Specfic Impulse [s]
vehicle.tvac = 0;                   % Thrust Force (kn)

%% Define Inital Conditions

%% Define Optimal Control Settings 
alpha = [3.12 3.21 4.56 5.12 6.13]*pi/180;
throttle = [0.99 0.98 0.97 0 0];
%T = linspace(t0, tend, length(alpha));

%% Apply ODE Solver
function Y = ode3(@(t,x)dynamics_euan1(t,x,T,alpha,throttle,vehicle),tspan,x0,varargin)
%ODE3  Solve differential equations with a non-adaptive method of order 3.
%   Y = ODE3(ODEFUN,TSPAN,Y0) with TSPAN = [T1, T2, T3, ... TN] integrates 
%   the system of differential equations y' = f(t,y) by stepping from T0 to 
%   T1 to TN. Function ODEFUN(T,Y) must return f(t,y) in a column vector.
%   The vector Y0 is the initial conditions at T0. Each row in the solution 
%   array Y corresponds to a time specified in TSPAN.
%
%   Y = ODE3(ODEFUN,TSPAN,Y0,P1,P2...) passes the additional parameters 
%   P1,P2... to the derivative function as ODEFUN(T,Y,P1,P2...). 
%
%   This is a non-adaptive solver. The step sequence is determined by TSPAN
%   but the derivative function ODEFUN is evaluated multiple times per step.
%   The solver implements the Bogacki-Shampine Runge-Kutta method of order 3.
%
%   Example 
%         tspan = 0:0.1:20;
%         y = ode3(@vdp1,tspan,[2 0]);  
%         plot(tspan,y(:,1));
%     solves the system y' = vdp1(t,y) with a constant step size of 0.1, 
%     and plots the first component of the solution.   
%

t0=0;                               % Start time 
tend = 225;                         % End Time
tspan= [t0:0.0001:tend];                  % Time Span

x0(1,1) = 100;                           % Altitude y(1)
x0(2,1) = 0;                             % Velocity y(2)
x0(3,1) = 0.*(pi./180);                  % Flight path angle y(3)
x0(4,1) = 90.*(pi./180);                 % Flight heading angle y(4)
x0(5,1) = 58.5107.*(pi./180);            % Latitude y(5)
x0(6,1) = -4.5121.*(pi./180);            % Longitude y(6)
x0(7,1) = 2466;                          % Mass of the vehicle y(7)

if ~isnumeric(tspan)
  error('TSPAN should be a vector of integration steps.');
end

if ~isnumeric(x0)
  error('Y0 should be a vector of initial conditions.');
end

h = diff(tspan);
if any(sign(h(1))*h <= 0)
  error('Entries of TSPAN are not in order.') 
end  

try
  f0 = feval(dynamics_euan1,tspan(1),x0,varargin{:});
catch
  msg = ['Unable to evaluate the ODEFUN at t0,y0. ',lasterr];
  error(msg);  
end  

x0 = x0(:);   % Make a column vector.
if ~isequal(size(x0),size(f0))
  error('Inconsistent sizes of Y0 and f(t0,y0).');
end  

neq = length(x0);
N = length(tspan);
Y = zeros(neq,N);
F = zeros(neq,3);

Y(:,1) = x0;
for i = 2:N
  ti = tspan(i-1);
  hi = h(i-1);
  yi = Y(:,i-1);
  F(:,1) = feval(dynamics_euan1,ti,yi,varargin{:});
  F(:,2) = feval(dynamics_euan1,ti+0.5*hi,yi+0.5*hi*F(:,1),varargin{:});
  F(:,3) = feval(dynamics_euan1,ti+0.75*hi,yi+0.75*hi*F(:,2),varargin{:});  
  Y(:,i) = yi + (hi/9)*(2*F(:,1) + 3*F(:,2) + 4*F(:,3));
end
Y = Y.';

end 
%options = odeset('RelTol',1e-3, 'AbsTol',1e-5);
%[t,x] = ode23tb(@(t,x)dynamics_euan(t,x,T,alpha,throttle,vehicle),tspan,x0,options); %Stiff solver applied


%% Displaying Results 
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
plot(t,x(:,1).*1e-3);
xlabel('Time(s)');
ylabel('Altitude (km)');
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(t,x(:,2).*1e-3); 
xlabel('Time (s)');
ylabel('Velocity (km/s)');
ax = gca;
ax.FontSize = 15;

% % Flight Path and Heading angle Plot
% figure 
% subplot(1,2,1);
% plot(t,x(:,3).*(180/pi));
% xlabel('Time(s)');
% ylabel('Flight path angle (deg)')
% ax = gca;
% ax.FontSize = 15;
% subplot(1,2,2);
% plot(t,x(:,4).*(180/pi));
% xlabel('Time(s)');
% ylabel('Heading angle (deg)') 
% ax = gca;
% ax.FontSize = 15;
% 
%Plot of Mass
figure
plot(t,x(:,7));
xlabel('Time(s)');
ylabel('Mass(kg)')
ax = gca;
ax.FontSize = 15;
% 
%Plot of Controls 
figure
subplot(1,2,1)
plot(T,(alpha).*(180/pi));
xlabel('Time(s)');
ylabel('Angle of Attack (deg)')
ax = gca;
ax.FontSize = 15;
figure
subplot(1,2,2);
plot(T,(throttle));
xlabel('Time(s)');
ylabel('Throttle')
ax = gca;
ax.FontSize = 15;