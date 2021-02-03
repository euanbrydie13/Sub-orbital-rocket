clc; clear;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Problem Bounds                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

h0 = 0.1;     %Rocket starts on the ground
v0 = 0.1;     %Rocket starts stationary
fpa0 = 0;     %Flight path angle is set to 0
chi0 = pi/2;  %Flight heading angle set to 0
lat0 = 1;     %Latitude
lon0 = 1;     %Longitude
m0 = 2500;    %Rocket starts full of fuel

vF = 0;  
fpaF= pi; 
chiF = pi;
latF = 1.5;
lonF = 1.9;
mF = 0.2*2500;  %Assume that we use all of the fuel

hLow = 0;   %Cannot go through the earth
hUpp = inf; %Go as high as you can

vLow = 0;    %Just look at the trajectory as it goes up
vUpp = 1e6;  %Go as fast as you can

fpaLow = -pi; %Lower Bound on flight path angle -180
fpaUpp = +pi; %Upper Bound on flight path angle +180 

chiLow = -pi; %Lower Bound on heading angle -180
chiUpp = +pi; %Upper Bound on heading angle +180

latLow = -pi;  %Lower Latitude Bound - free
latUpp = +pi;  %Upper Latitude bound - free

lonLow = -pi;  %Lower Latitude Bound - free
lonUpp = +pi;  %Upper Latitude Bound - free

mLow = 0.2*2500; %Lower Bound on mass
mUpp = 2500;     %Upper Bound on mass

alphaLow = -pi;  %Lower bound on angle of attack 
alphaUpp = pi;   %Upper bound on angle of attack

throttleLow = 0; %Lower bound on throttle
throttleUpp = 1; %Upper bound on throttle

P.bounds.initialTime.low = 0;
P.bounds.initialTime.upp = 0;

P.bounds.finalTime.low = 0;
P.bounds.finalTime.upp = 300;

P.bounds.state.low = [hLow;vLow;fpaLow;chiLow;latLow;lonLow;mLow];
P.bounds.state.upp = [hUpp;vUpp;fpaUpp;chiUpp;latUpp;lonUpp;mUpp];

P.bounds.initialState.low = [h0;v0;fpa0;chi0;lat0;lon0;m0];
P.bounds.initialState.upp = [h0;v0;fpa0;chi0;lat0;lon0;m0];

P.bounds.finalState.low = [hLow;vF;fpaF;chiF;latF;lonF;mF];
P.bounds.finalState.upp = [hUpp;vF;fpaF;chiF;latF;lonF;mF];

P.bounds.control.low = [alphaLow;throttleLow];
P.bounds.control.upp = [alphaUpp;throttleUpp];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Initial Guess                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
hGuess = 100000;   %(m) guess at the maximum height reached
P.guess.time = [0, 200];  %(s)
P.guess.state = [ [h0;v0;fpa0;chi0;lat0;lon0;m0],  [hGuess;vF;fpaF;chiF;latF;lonF;mF] ];
P.guess.control = [[alphaUpp;throttleUpp], [alphaLow;throttleLow]];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Objective and Dynamic functions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Dynamics function:
P.func.dynamics = @(t,x,u)( Dynamics(x,u));

% Objective function:
P.func.bndObj = @(t0,x0,tF,xF)(-xF(1));  %Maximize final height

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Options and Method selection                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% method = 'trapezoid';
method = 'rungeKutta';
% method = 'chebyshev';

switch method
        
    case 'rungeKutta'
        P.options(1).method = 'rungeKutta';
        P.options(1).defaultAccuracy = 'low';
        
        P.options(2).method = 'rungeKutta';
        P.options(2).defaultAccuracy = 'medium';

end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Solve!                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
soln = optimTraj1(P);

t = linspace(soln(end).grid.time(1),soln(end).grid.time(end),250);
x = soln(end).interp.state(t);
u = soln(end).interp.control(t);

figure(120);
subplot(2,2,1);
plot(t,x(1,:)/1000)
xlabel('time (s)')
ylabel('height (km)')
title('Maximal Height Trajectory')
subplot(2,2,2);
plot(t,x(7,:))
xlabel('time (s)')
ylabel('mass (kg)')
title('Goddard Rocket')
subplot(2,2,3);
plot(t,x(2,:))
xlabel('time (s)')
ylabel('velocity (m/s)')
subplot(2,2,4);
plot(t,u(1,:))
xlabel('time (s)')
ylabel('angle of attack (kN)')
