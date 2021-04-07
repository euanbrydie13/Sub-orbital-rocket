%% RAPTOR_PEREGRINE Optimal Control problem - Single Shooting 3 stages

tBurn = 70;
tCoast = 100;
tDescend = 400;

%% Load Location and Heading Seletion
location = location_select;

%% Load Vehicle Selection
vehicle = vehicle_select; 

% Applying Lower and Upper Bounds on Controls

tlower = 10;
alphalower = -5.*(pi./180).*ones(5,1);
throttlelower = zeros(5,1);
banklower = -10.*(pi./180).*ones(5,1);

tupper = 160;
alphaupper= 5.*(pi./180).*ones(5,1);
throttleupper = ones(5,1);
bankupper = 10.*(pi./180).*ones(5,1);

LB = [tlower; alphalower; throttlelower; banklower];
UB = [tupper; alphaupper; throttleupper; bankupper];

% First Optimisation

% Initial values for control and state variables

y0 = [tBurn; 0;0;0;0;0; 0.9;0.9;0.9;0.5;0.5; 0;0;0;0;0];                      % Initial input
x0 = [100;30;90*(pi/180);location.heading;location.lat;location.lon;vehicle.wet];
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','interior-point','MaxFunctionEvaluations',700);

% Checking initial values for objective and constraint fun
Cost1R(y0, x0)
Confun1R(y0, x0)

% Running Optimiser

tic
[y_opt,fval1,exitflag1,output1] = fmincon(@(y) Cost1R(y, x0),y0,[],[],[],[],LB,UB,@(y) Confun1R(y, x0),options);
toc 

% Determining time span

tspan = 0:1:y_opt(1);

% Integrating first stage 

xState1 = ode3(@(t,x,control) dynamics_model(t,x,y_opt),tspan,x0);

% Second Optimisation

throttlelower2 = zeros(5,1);
throttleupper2 = ones(5,1)*0.01;

LB2 = [150; alphalower; throttlelower2; banklower];
UB2 = [300; alphaupper; throttleupper2; bankupper];

y02 = [y_opt(1) + tCoast ; 0;0;0;0;0; 0;0;0;0;0; 0;0;0;0;0];
x0t2 = xState1(end,:);
x02 = x0t2';

% Checking initial values for objective and constraint fun
Cost2R(y02, x02, y_opt)
Confun2R(y02, x02, y_opt)

% Running 2nd Optimiser

tic
[y_opt2,fval2,exitflag2,output2] = fmincon(@(y2) Cost2R(y2, x02, y_opt),y02,[],[],[],[],LB2,UB2,@(y2) Confun2R(y2, x02, y_opt),options);
toc 

tspan2 = y_opt(1):1:y_opt2(1);

xState2 = ode3(@(t,x,control) dynamics_model(t,x,y_opt2),tspan2,x02);

% Third Optimisation

LB3 = [300; alphalower; throttlelower2; banklower];
UB3 = [600; alphaupper; throttleupper2; bankupper];

y03 = [y_opt2(1) + tDescend; 0;0;0;0;0; 0;0;0;0;0; 0;0;0;0;0];
x0t3 = xState2(end,:);
x03 = x0t3';

% Checking initial values for objective and constraint fun
Cost3R(y03, x03, y_opt2)
Confun3R(y03, x03, y_opt2)

% Running 3rd Optimiser

tic
[y_opt3,fval3,exitflag3,output3] = fmincon(@(y3) Cost3R(y3, x03, y_opt2),y03,[],[],[],[],LB3,UB3,@(y3) Confun2R(y3, x03, y_opt2),options);
toc 

tspan3 = y_opt2(1):1:y_opt3(1);

xState3 = ode3(@(t,x,control) dynamics_model(t,x,y_opt3),tspan3,x03);

x = [xState1
     xState2
     xState3];

Tspan = [tspan, tspan2, tspan3];

%% Displaying controls

Alpha = [y_opt(2:6); y_opt2(2:6); y_opt3(2:6)];

Throttle = [y_opt(7:11);y_opt2(7:11);y_opt3(7:11)];

Bank = [y_opt(12:16);y_opt2(12:16);y_opt(12:16)];

tspanC1 = linspace(0,tspan(end),5);
tspanC2 = linspace(tspan(end),tspan2(end),5);
tspanC3 = linspace(tspan2(end),tspan3(end),5);

TspanC = [tspanC1, tspanC2, tspanC3];

%% Plotting Control Settings

figure('Name','Control Settings','NumberTitle','off')
hold on 
pAlpha = stairs(TspanC,Alpha);
pThrottle = stairs(TspanC,Throttle);
pBank = stairs(TspanC,Bank);
pAlpha.DisplayName = "Angle of Attack";
pThrottle.DisplayName = "Throttle";
pBank.DisplayName = "Bank Angle";
legend

%% Display of State variables

x = [xState1
     xState2
     xState3];

Tspan = [tspan, tspan2, tspan3];

%Trajectory Plot
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

%Plot of Mass
figure
plot(Tspan,x(:,7));
xlabel('Time(s)');
ylabel('Mass(kg)')
ax = gca;
ax.FontSize = 15;


%Convert results back into degrees for plotting
lat = rad2deg(x(:,5));
lon = rad2deg(x(:,6));
alt = x(:,1);
headingr = x(:,4);
heading = rad2deg(x(:,4));

% Creating figure pannel to plot the maps
figure
uif1 = uifigure("Name","Trajectory Results");
% ug = uigridlayout(uif,[1,2]);
% % p1 = uipanel(ug);
% p2 = uipanel(ug);
gx = geoaxes("Basemap","satellite");

%Setting markers on 2D map for start, end and current location
hold(gx,"on");
ptrack = geoplot(gx,lat,lon,"c","LineWidth",2);
mstart = geoplot(gx,lat(1),lon(1),"ow","MarkerSize",10,"MarkerFaceColor","magenta");
mend = geoplot(gx,lat(end),lon(end),"ow","MarkerSize",10,"MarkerFaceColor","blue");
ptrack.DisplayName = "Flight Path";
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
exc1 = geoplot(gx,x1,y1,'-r',"LineWidth",2);
exc2 = geoplot(gx,x2,y2,'-r',"LineWidth",2);
exc3 = geoplot(gx,[x1(1) x2(1)], [y1(1) y2(1)],'-r',"LineWidth",2);
exc4 = geoplot(gx,[x1(:,end) x2(:,end)], [y1(:,end) y2(:,end)],'-r',"LineWidth",2);

uif = uifigure;
g = geoglobe(uif);
geoplot3(g,lat,lon,alt,'c','HeightReference','terrain', ...
     'LineWidth',2),
