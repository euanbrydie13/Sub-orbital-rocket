function J= obfunction_euan(control)

% %% Arrangement of Control Vector
% n = (length(u)-1)/2;
% tspan= [0, u(1:1)];                     % Time span
% alpha = u(2:(2+n-1));                   % Angle of attack
% throttle = u(end-n+1:end);              % Throttle
% T = linspace(0, u(1:1), length(alpha)); % Generating Linearly spaced vector
% u = [tspan alpha throttle];             % Control Vector

%% Integrater
% Need to implement ode3 into this part of code, illegal to just import
% need to look into how to call it
t0=0;                                     % Start time 
tend = 350;                               % End time
tspan = t0:1:tend;                        % Time Span
x0 = [1 40 90*(pi/180) 0*pi/180 58.5127*(pi/180) -4.512*(pi/180) 2500]; 
%x = ode3(@(t,x)dynamics_euan1(t,x),tspan,x0);
%Initally used ode45 but intergration tolerances wouldnt allow
%[t,y] =ode15s(@(t,y)dynamics_euan(t,y,T,alpha,throttle,vehicle),tspan,y0);

x = ode3(@(t,x)dynamics_euan1(t,x,control),tspan,x0);                              

% x01(1,1) = 100;                           % Altitude y(1)
% x01(2,1) = 2;                             % Velocity y(2)
% x01(3,1) = 45.*(pi./180);                 % Flight path angle y(3)
% x01(4,1) = 45.*(pi./180);                 % Flight heading angle y(4)
% x01(5,1) = 58.5107.*(pi./180);            % Latitude y(5)
% x01(6,1) = -4.5121.*(pi./180);            % Longitude y(6)
% x01(7,1) = 2466;                          % Mass of the vehicle y(7)

%% Create Objective function
% Minimise Mass is the objective
J = trapz(x(:,1));
end


