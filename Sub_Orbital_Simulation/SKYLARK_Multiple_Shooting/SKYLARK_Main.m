%% SKYLARK_L Optimal Control problem - Single Shooting 3 stages

tBurn = 150;
tCoast = 100;
tDescend = 300;

%% Load Location and Heading Seletion
location = location_select;

%% Load Vehicle Selection
vehicle = vehicle_select; 

% Applying Lower and Upper Bounds on Controls

tlower = 60;
alphalower = -5.*(pi./180).*ones(5,1);
throttlelower = zeros(5,1);
banklower = -10.*(pi./180).*ones(5,1);

tupper = 200;
alphaupper= 5.*(pi./180).*ones(5,1);
throttleupper = ones(5,1);
bankupper = 10.*(pi./180).*ones(5,1);

LB = [tlower; alphalower; throttlelower; banklower];
UB = [tupper; alphaupper; throttleupper; bankupper];

% First Optimisation

% Initial values for control and state variables

y0 = [tBurn; 0;0;0;0;0; 0.9;0.9;0.9;0.9;0.9; 0;0;0;0;0];                      % Initial input
x0 = [100;50;90*(pi/180);location.heading;location.lat;location.lon;vehicle.wet];
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','interior-point','MaxFunctionEvaluations',500);

% Checking initial values for objective and constraint fun
Cost1S(y0, x0)
Confun1S(y0, x0)

% Running Optimiser

tic
[y_opt,fval1,exitflag1,output1] = fmincon(@(y) Cost1S(y, x0),y0,[],[],[],[],LB,UB,@(y) Confun1S(y, x0),options);
toc 

% Determining time span

tspan = 0:1:y_opt(1);

% Integrating first stage 

xState1 = ode3(@(t,x,control) dynamics_model(t,x,y_opt),tspan,x0);

% Second Optimisation

throttlelower2 = ones(5,1)*-0.01;
throttleupper2 = ones(5,1)*0.01;

LB2 = [50; alphalower; throttlelower2; banklower];
UB2 = [300; alphaupper; throttleupper2; bankupper];

y02 = [y_opt(1) + tCoast ; 0;0;0;0;0; 0;0;0;0;0; 0;0;0;0;0];
x0t2 = xState1(end,:);
x02 = x0t2';

% Checking initial values for objective and constraint fun
Cost2S(y02, x02, y_opt)
Confun2S(y02, x02, y_opt)

% Running 2nd Optimiser

tic
[y_opt2,fval2,exitflag2,output2] = fmincon(@(y2) Cost2S(y2, x02, y_opt),y02,[],[],[],[],LB2,UB2,@(y2) Confun2S(y2, x02, y_opt),options);
toc 

tspan2 = y_opt(1):1:y_opt2(1);

xState2 = ode3(@(t,x,control) dynamics_model(t,x,y_opt2),tspan2,x02);

% Third Optimisation

LB3 = [240; alphalower; throttlelower2; banklower];
UB3 = [500; alphaupper; throttleupper2; bankupper];

y03 = [y_opt2(1) + tDescend; 0;0;0;0;0; 0.001;0.001;0.001;0.001;0.001; 0;0;0;0;0];
x0t3 = xState2(end,:);
x03 = x0t3';

% Checking initial values for objective and constraint fun
Cost3S(y03, x03, y_opt2)
Confun3S(y03, x03, y_opt2)

% Running 3rd Optimiser

tic
[y_opt3,fval3,exitflag3,output3] = fmincon(@(y3) Cost3S(y3, x03, y_opt2),y03,[],[],[],[],LB3,UB3,@(y3) Confun2S(y3, x03, y_opt2),options);
toc 

tspan3 = y_opt2(1):1:y_opt3(1);

xState3 = ode3(@(t,x,control) dynamics_model(t,x,y_opt3),tspan3,x03);

%% Display Control & State variables

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

figure('Name','Control Settings','NumberTitle','off')
hold on 
pAlpha = stairs(TspanC,Alpha);
pThrottle = stairs(TspanC,Throttle);
pBank = stairs(TspanC,Bank);
pAlpha.DisplayName = "Angle of Attack";
pThrottle.DisplayName = "Throttle";
pBank.DisplayName = "Bank Angle";
legend

% %% Trajectory and State Plots
% 
%Altitude and Velocity Plot
figure('Name','Altitude and Velcoity','NumberTitle','off')
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
figure('Name','Velcoity frame angles','NumberTitle','off')
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
figure('Name','Vehicle Mass','NumberTitle','off')
plot(Tspan,x(:,7));
xlabel('Time(s)');
ylabel('Mass(kg)')
ax = gca;
ax.FontSize = 15;

%% Mapping Flight Path with Exclusion Zone

%Convert results back into degrees for plotting
lat = rad2deg(x(:,5));
lon = rad2deg(x(:,6));
alt = x(:,1);
headingr = x(:,4);
heading = rad2deg(x(:,4));

%Creating figure pannel to plot the maps
figure
uif1 = uifigure("Name","Trajectory Results");
% ug = uigridlayout(uif,[1,2]);
% p1 = uipanel(ug);
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
% % This calculates the flight corridor from specfic start and end coordinates
% 
% % Starting coordinates
% lat1 = location.lat;
% lon1 = location.lon; 
% 
% chi1 = location.heading; 
% 
% % Drawing Start Circle
% latS = rad2deg(x0(5));
% lonS = rad2deg(x0(6));
% 
% R = 2440/110574;                % radius of circle 
% C = [latS,lonS];                % center of circle 
% th = linspace((pi/2+location.heading),(3*pi/2+location.heading));
% x1  = C(1)+R*cos(th); 
% y1 = C(2)+R*sin(th) ;
% 
% % Drawing End Circle
% 
% latF = rad2deg(x(end,5));
% lonF = rad2deg(x(end,6));
% 
% R = (5889.36/110574);                          % radius of circle 
% C = [latF,lonF];                               % center of circle 
% th = linspace(((pi/2)+location.heading), (-pi/2 + location.heading));
% x2  = C(1)+R*cos(th); 
% y2 = C(2)+R*sin(th);
% 
% hold(gx,'on')
% exc1 = geoplot(gx,x1,y1,'-r',"LineWidth",2);
% exc2=  geoplot(gx,x2,y2,'-r',"LineWidth",2);
% exc3= geoplot(gx,[x1(1) x2(1)], [y1(1) y2(1)],'-r',"LineWidth",2);
% exc4= geoplot(gx,[x1(:,end) x2(:,end)], [y1(:,end) y2(:,end)],'-r',"LineWidth",2);
% exc1.DisplayName = "Exclusion Zone";
% 
% % 3D Trajectory with globe
% uif = uifigure;
% g = geoglobe(uif);
% geoplot3(g,lat,lon,alt,'c','HeightReference','terrain', ...
%      'LineWidth',2),
% 
% % 3D Trajectory 
% figure('Name','3D Trajectory','NumberTitle','off')
% plot3(lat,lon,alt*10e-3);
% hold on 
% grid on 
 
% figure
% streamline(lat,lon,alt,x(:,2),heading,x(:,4),lat(1),lon(1),alt(1))
% hold on;
% plot3(lat(1),lon(1),alt(1))
%% Displaying Results 

%Convert results back into degrees for plotting
lat = rad2deg(x(:,5));
lon = rad2deg(x(:,6));
alt = x(:,1);
headingr = x(:,4);
heading = rad2deg(x(:,4));

% % disp(lat)
% % disp(lon)
% % disp(heading)

%Creating figure pannel to plot the maps
figpos = [500 200 800 400];
uif = uifigure("Name","Trajectory Results","Position",figpos);
ug = uigridlayout(uif,[1,2]);
p1 = uipanel(ug);
p2 = uipanel(ug);
gx = geoaxes(p1,"Basemap","satellite"); 
gg = geoglobe(p2); 
gx.InnerPosition %= gx.OuterPosition;
gg.Position = [0 0 1 1];
% 
%Plotting trajectory data on maps
geoplot3(gg,lat,lon,alt,"c","LineWidth",2,"HeightReference","geoid")
hold(gx,"on")
ptrack = geoplot(gx,lat,lon,"c","LineWidth",2);

% 3D Trajectory with globe
uif = uifigure;
g = geoglobe(uif);
geoplot3(g,lat,lon,alt,'c','HeightReference','terrain', ...
     'LineWidth',2),

%% Simple Flight Corridor 
% This calculates a simple flight corridor from specfic start and end
% coordinates

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

R = (2440/110574);                          % radius of circle 
C = [latF,lonF];                               % center of circle 
th = linspace(((pi/2)+location.heading), (-pi/2 + location.heading));
x2  = C(1)+R*cos(th); 
y2 = C(2)+R*sin(th);

hold(gx,'on')
exc1 = geoplot(gx,x1,y1,'--r',"LineWidth",2);
exc2 = geoplot(gx,x2,y2,'--r',"LineWidth",2);
exc3 = geoplot(gx,[x1(1) x2(1)], [y1(1) y2(1)],'--r',"LineWidth",2);
exc4 = geoplot(gx,[x1(:,end) x2(:,end)], [y1(:,end) y2(:,end)],'--r',"LineWidth",2);

%Setting initial view to launch site
campos(gg,lat(1),lon(1))
camheight(gg,alt(1) + 75)
campitch(gg,-90)
camheading(gg,heading(3))

%Setting markers on 2D map for start, end and current location
marker = geoplot(gx,lat(1),lon(1),"ow","MarkerSize",10,"MarkerFaceColor","k");
mstart = geoplot(gx,lat(1),lon(1),"ow","MarkerSize",10,"MarkerFaceColor","magenta");
mend = geoplot(gx,lat(end),lon(end),"ow","MarkerSize",10,"MarkerFaceColor","blue");

marker.DisplayName = "Current Location";
mstart.DisplayName = "Start Location";
mend.DisplayName = "End Location";
ptrack.DisplayName = "Trajectory";
legend(gx)

% %Calculating 3D distances (taking into account altitude change and ground
% %distance)
% wgs84 = wgs84Ellipsoid;
% N = egm96geoid(lat,lon);
% h = alt + N;
% lat1 = lat(1:end-1);
% lat2 = lat(2:end);
% lon1 = lon(1:end-1);
% lon2 = lon(2:end);
% h1 = h(1:end-1);
% h2 = h(2:end);
% [dx,dy,dz] = ecefOffset(wgs84,lat1,lon1,h1,lat2,lon2,h2);
% distance = hypot(hypot(dx, dy), dz);
% cumulativeDistance = cumsum(distance);
% total_distance = sum(distance);
% tdist =[cumulativeDistance];
% 
% %View flight data in real time
% dt = datatip(ptrack,"DataIndex",1,"Location","southeast");
% dtrow = dataTipTextRow("Distance",tdist);
% dtrow(end+1) = dataTipTextRow("Altitude",alt);
% dtrow(end+1) = dataTipTextRow("Heading",heading);
% ptrack.DataTipTemplate.DataTipRows(end+1:end+3) = dtrow;
% 
% %Animate trajectory
% pitch = -2.789;
% campitch(gg,pitch)
% for k = 2:(length(lat)-1)    
%     campos(gg,lat(k),lon(k))
%     camheight(gg,alt(k)+100)
%     camheading(gg,heading(k))
%     
%     set(marker,"LatitudeData",lat(k),"LongitudeData",lon(k));
%     dt.DataIndex = k;
%     
%     drawnow
%     pause(.25)
% end
% 
% campos(gg,lat(end),lon(end),alt(end)+100)
% dt.DataIndex = length(lat);
% 
% %3D Trajectory Plot
% figure
% plot3(lon,lat,alt), hold on;

% grid minor
% 
% 
% stem3(lon,lat,alt, 'Marker', 'none', 'Color', 'black'), hold on;
% geoshow('landareas.shp', 'FaceColor',[0.5 1 0.5]),hold on %comment out this line to see on normal 3D axis
% grid minor
% 
% grid on
% xlabel('Longitude (degrees)')
% ylabel('Latitude (degrees)')
% zlabel('Altitude (km)')
% 
% %2D Trajectory Plot
% figure
% geoshow('landareas.shp','FaceColor',[0.5 1 0.5]),hold on
% ax = plot(lon,lat,'LineStyle', '--'); hold on;
% mstart = plot(lon(1),lat(1),"ow","MarkerSize",5,"MarkerFaceColor","b"); hold on; %seems to start in the sea, feel the map coordinates could be a bit off
% mend = plot(lon(end),lat(end),"ow","MarkerSize",5,"MarkerFaceColor","r"); hold on; %no idea why this is not getting plotted
% 
% ax.DisplayName = 'Trajectory';
% mstart.DisplayName = "Start Location";
% mend.DisplayName = "End Location";
% legend
% grid minor
% 
% grid on
% xlabel('Longitude (degrees)') 
% ylabel('Latitude (degrees)')
% title('Ground Track')
