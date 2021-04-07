%% Optimal Control problem - Single Shooting
clc 

%% Load Location and Heading Seletion
location = location_select;

%% Load Vehicle Selection
vehicle = vehicle_select; 

%% Optimisation 

%Lower and Upper Bounds

tlower = 0;
alphalower = -5.*(pi./180).*ones(5,1);
throttlelower = zeros(5,1);
banklower = -10.*(pi./180).*ones(5,1);

tupper = 400;
alphaupper= 15.*(pi./180).*ones(5,1);
throttleupper = ones(5,1);
bankupper = 10.*(pi./180).*ones(5,1);

LB = [tlower; alphalower; throttlelower; banklower];
UB = [tupper; alphaupper; throttleupper; bankupper];

%% Setting up First Optimisation

%Initial guess
control_0 = [150; 0;0;0;0;0; 0.9;0.9;0.9;0.9;0.9; 0;0;0;0;0];                      % Initial input
x0 = [100;50;89*(pi/180);location.heading;location.lat;location.lon;vehicle.wet];
%Tspan = 0:1:control_0(1);
%option for fmincon
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','interior-point');
%init function for fmincon
Cost_Function(control_0,x0)
Non_Linear_Constraints(control_0,x0)

%call fmincon
[control_opt,fval,exitflag,output] = fmincon(@(x) Cost_Function(control1,x0),control_0,[],[],[],[],LB,UB,@(x) Non_Linear_Constraints(control1,x0),options);

Tspan = 0:1:control_opt(1);        % Tspan calculated by using control output 1

xState1 = ode3(@(t,x,control_opt) dynamics_model(t,x,control_opt),Tspan,x0,control_opt);

%% Second Optimisation

control_01 = [150; 0;0;0;0;0; 0;0;0;0;0; 0;0;0;0;0];                      % Initial input
x01 = xState1(end,:);
Tspan1 = 0:1:control_opt1(1);
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','interior-point');

cf_1 = @(x)Cost_Function1(control_opt1, x01);
conf_1 =@(x)Non_linear_Constraints(control_opt1,x01,Tspan1);
[control_opt1,fval1,exitflag1,output1] = fmincon(cf_1,control_01,[],[],[],[],LB,UB,conf_1,options);

%Tspan1 = control_opt(1):1:control_opt1(1);     % Tspan calculated by using control output 1

xState2 = ode3(@(t,x,control_opt1) dynamics_model(t,x,control_opt1),Tspan1,x01,control_opt1);

%% Cost function  
function J2 = Cost_Function(control1,x0)

% vehicle = vehicle_select;
% location = location_select;
Tspan = 0:1:control1(1);

% x0 = [100; 50; 89*(pi/180); location.heading; location.lat; location.lon; vehicle.wet];

x = ode3(@(t,x,control)dynamics_model(t,x,control1),Tspan,x0,control1);

J2 = -max(x(:,1)); % + sum(x(:,7))) ;

end

%% Cost Function 2
function J3 = Cost_Function1(control1,x01)

% vehicle = vehicle_select;
% location = location_select;

% x0 = [100;50;89*(pi/180);location.heading;location.lat;location.lon;vehicle.wet];
% Tspan = 0:1:control_opt(1);
% x1 = ode3(@(t,x,control_opt) dynamics_model(t,x,control_opt),Tspan,x0,control_opt);
%x01 = x1(end,:);

Tspan1 = 0:1:control1(1);
%fprintf('@(t,x,control)dynamics_model(t,x,control_opt1)')
%disp(@(t,x,control)dynamics_model(t,x,control_opt1))
%fprintf('Tspan1')
%disp(Tspan1)

% disp(x01)
x = ode3(@(t,x,control)dynamics_model(t,x,control1),Tspan1,x01);

J3 = -min(x(:,1)); % + sum(x(:,7))) ;
end

%% Nonlinear constraints 

function [c,ceq]=Non_Linear_Constraints(control1,x0)

% vehicle = vehicle_select;
% location = location_select;
Tspan = 0:1:control1(1);
%x0 = [100; 50; 90*(pi/180); location.heading; location.lat; location.lon; vehicle.wet];
x = ode3(@(t,x,control) dynamics_model(t,x,control1),Tspan,x0,control1);

c = [];
ceq(1) = x(end,1) - 37800;
ceq(1) = x(end,2) - 950;
ceq(2) = x(end,3) - deg2rad(80);
%ceq(3) = x(end,4) - deg2rad(15); 
%ceq(4) = x(end,5) - ;
%ceq(4) = x(end,6) - ;
ceq(4) = x(end,7) - 500;
ceq = ceq';
end

%% Nonlinear constraints 

function [c,ceq]=Non_linear_Constraints(control_opt1,x01,Tspan1) 

% vehicle = vehicle_select;
% location = location_select;
% Tspan1 = 0:1:control_opt1(1);
%x0 = [100; 50; 90*(pi/180); location.heading; location.lat; location.lon; vehicle.wet];
x = ode3(@(t,x,control) dynamics_model(t,x,control_opt1),Tspan1,x01,control_opt1);

c = [];
ceq(1) = x(end,1) - 120000;
ceq(1) = x(end,2) - 99;
ceq(2) = x(end,3) - deg2rad(0);
%ceq(3) = x(end,4) - deg2rad(15); 
%ceq(4) = x(end,5) - ;
%ceq(4) = x(end,6) - ;
ceq(4) = x(end,7) - 500;
ceq = ceq';
end