%% Intergrating Dynamics 
clc; clear;
addpath ../../


t0 = 0;
tend = 100;
tspan = t0:1:tend;

x0 = [0; 0; 505846];  

u = 5885000; 

[t, y] = ode45(@(t,x,u)(rocketDynamics1(x,u)),tspan,x0);