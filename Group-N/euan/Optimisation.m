%% Optimisation
close all
clear all 
clc

global t0 tend tspan x0                   % Global Variables 
t0=0;                                     % Start time 
tend = 200;                               % End time
tspan = t0:1:tend;                        % Time Span
x0 = [0 0 90*(pi/180) 0*pi/180 58.5127*(pi/180) -4.512*(pi/180) 2500];
u0 =[200; 0;0;0;0;0; 1;1;1;0;0]; %Initial guess


lb = [40; ...                                                 % Lower Bounds
    -5*pi/180; -5*pi/180; -5*pi/180; -5*pi/180; -5*pi/180; ...
    0; 0; 0; 0; 0];

ub = [400; ...
    20*pi/180; 20*pi/180; 20*pi/180; 20*pi/180; 20*pi/180; ...     % Upper Bounds
    1; 1; 1; 1; 1];

[u, fval, exitflag, output] = fmincon(@obfunction,u0,[],[],[],[],lb,ub, @nlcon);
x = ode3(@(t,x)dynamics_euan1(t,x,u),tspan,x0);

figure
plot(tspan,x(:,1)) 

function J= obfunction(u)
global tspan x0
[x] = ode3(@(t,x)dynamics_euan1(t,x,u),tspan,x0);                              
% Maximise height is the objective
J = trapz(tspan,x(:,1));
end

function [c,ceq] = nlcon(u)
global tspan x0 
x = ode3(@(t,x)dynamics_euan1(t,x,u),tspan,x0);
n = length(x);
x1 = x(:,1)'; 
c = [];
ceq(1) = x1(n)-100000;         % altitude constraint - reach 10000 m 
% ceq(2) = x(end,2);           % velocity constraint - at maximum height v = 0 m/s
ceq = ceq';
end

