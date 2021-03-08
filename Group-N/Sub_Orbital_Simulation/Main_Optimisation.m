%% Optimal Control problem

control_0 = [125; 0;0;0;0;0; 1;1;1;1;1; 0;0;0;0;0]; %initial input
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','sqp');


[control_opt,fval,exitflag,output] = fmincon(@Cost_Function,control_0,[],[],[],[],[],[],@Non_Linear_Constraints,options);

x0(1) = 100;                               % Altitude x(1)
x0(2) = 0;                                 % Velocity x(2)
x0(3) = 90*pi/180;                         % Flight path angle x(3)
x0(4) = 15.*(pi./180);                     % Flight heading angle x(4)
x0(5) = 58.5127.*(pi./180);                % Latitude x(5)
x0(6) = -4.5121.*(pi./180);                % Longitude x(6)
x0(7) = 2466;                              % Mass of the vehicle x(7)

Tspan = 0:1:control_opt(1);                %Tspan calculated by using control output 1

x = ode3(@(t,x,u_opt) dynamics_model(t,x,u_opt),Tspan,x0,control_opt); 


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
plot(Tspan,x(:,1).*1e-3);
xlabel('Time(s)');
ylabel('Altitude (km)');
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(Tspan,x(:,2).*1e-3); 
xlabel('Time (s)');
ylabel('Velocity (km/s)');
ax = gca;
ax.FontSize = 15;

% Flight Path and Heading angle Plot
figure 
subplot(1,2,1);
plot(Tspan,x(:,3).*(180/pi));
xlabel('Time(s)');
ylabel('Flight path angle (deg)')
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(Tspan,x(:,4).*(180/pi));
xlabel('Time(s)');
ylabel('Heading angle (deg)') 
ax = gca;
ax.FontSize = 15;

%Plot of Mass
figure
plot(Tspan,x(:,7));
xlabel('Time(s)');
ylabel('Mass(kg)')
ax = gca;
ax.FontSize = 15;