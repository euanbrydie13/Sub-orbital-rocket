%% Trajectory Simulation 
% This code uses an optimal control vector and plots the corresponding
% outputs. SKYLARK L simulated from Space Port Sutherland

%% Location and Heading Selection 
location = location_select;

%% Vehicle Selection
vehicle = vehicle_select;

%% Define Inital Conditions

% Supplying simulation with trial and error controls, not optimised. Best
% guess 
control=[100; 0;0;0;0;0; 1;1;1;1;1; 0;0;0;0;0]; % [Time; Angle of Attack; Throttle; Bank Angle]

t0=0;                                      % Start time                                
tspan = t0:1:control(1);                   % Time Span

x0(1) = 10;                                % Altitude x(1) (km)
x0(2) = 50;                                % Velocity x(2) (m/s)
x0(3) = 89*(pi/180);                       % Flight path angle x(3) (degree)
x0(4) = location.heading;                  % Flight heading angle x(4) (degree)
x0(5) = location.lat;                      % Latitude x(5) (degree)
x0(6) = location.lon;                      % Longitude x(6) (degree)
x0(7) = vehicle.wet;                       % Mass of the vehicle x(7) (kg)

%% Apply ODE Solver

x = ode3(@(t,x)dynamics_model(t,x,control),tspan,x0);

%% Display of State variables

%Trajectory Plot
figure
plot3(x(:,6),x(:,5),x(:,1).*1e-3);
grid on
xlabel('Latitude')
ylabel('Longitude')
zlabel('Altitude')
% 
%Altitude and Velocity Plot
figure 
subplot(1,2,1);
plot(tspan,x(:,1).*1e-3);
xlabel('Time(s)');
ylabel('Altitude (km)');
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(tspan,x(:,2).*1e-3); 
xlabel('Time (s)');
ylabel('Velocity (km/s)');
ax = gca;
ax.FontSize = 15;

% Flight Path and Heading angle Plot
figure 
subplot(1,2,1);
plot(tspan,x(:,3).*(180/pi));
xlabel('Time(s)');
ylabel('Flight path angle (deg)')
ax = gca;
ax.FontSize = 15;
subplot(1,2,2);
plot(tspan,x(:,4).*(180/pi));
xlabel('Time(s)');
ylabel('Heading angle (deg)') 
ax = gca;
ax.FontSize = 15;

%Plot of Mass
figure
plot(tspan,x(:,7));
xlabel('Time(s)');
ylabel('Mass(kg)')
ax = gca;
ax.FontSize = 15;

%% Displaying Results 

Alpha = control(2:6);
Throttle = control(7:11);
Bank = control(12:16);

T = linspace(0, tspan(end), 5);

figure('Name','Control Settings','NumberTitle','off')
hold on 
plot(T,Alpha)
% plot(T,Throttle)
% plot(Ts,Bank)

%Convert results back into degrees for plotting
lat = rad2deg(x(:,5));
lon = rad2deg(x(:,6));
alt = x(:,1)*10^-3;
headingr = x(:,4);
heading = rad2deg(x(:,4));

% % disp(lat)
% % disp(lon)
% % disp(heading)

% %Creating figure pannel to plot the maps
% figpos = [500 200 800 400];
% uif = uifigure("Name","Trajectory Results","Position",figpos);
% ug = uigridlayout(uif,[1,2]);
% p1 = uipanel(ug);
% p2 = uipanel(ug);
% gx = geoaxes(p1,"Basemap","satellite"); 
% gg = geoglobe(p2); 
% gx.InnerPosition %= gx.OuterPosition;
% gg.Position = [0 0 1 1];
% 
%Plotting trajectory data on maps
% geoplot3(gg,lat,lon,alt,"c","LineWidth",2,"HeightReference","geoid")
% hold(gx,"on")
% ptrack = geoplot(gx,lat,lon,"c","LineWidth",2);
% 
% %% Simple Flight Corridor 
% % This calculates a simple flight corridor from specfic start and end
% % coordinates
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
% exc1 = geoplot(gx,x1,y1,'--r',"LineWidth",2);
% exc2 = geoplot(gx,x2,y2,'--r',"LineWidth",2);
% exc3 = geoplot(gx,[x1(1) x2(1)], [y1(1) y2(1)],'--r',"LineWidth",2);
% exc4 = geoplot(gx,[x1(:,end) x2(:,end)], [y1(:,end) y2(:,end)],'--r',"LineWidth",2);
% 
% %Setting initial view to launch site
% campos(gg,lat(1),lon(1))
% camheight(gg,alt(1) + 75)
% campitch(gg,-90)
% camheading(gg,heading(3))
% 
% %Setting markers on 2D map for start, end and current location
% marker = geoplot(gx,lat(1),lon(1),"ow","MarkerSize",10,"MarkerFaceColor","k");
% mstart = geoplot(gx,lat(1),lon(1),"ow","MarkerSize",10,"MarkerFaceColor","magenta");
% mend = geoplot(gx,lat(end),lon(end),"ow","MarkerSize",10,"MarkerFaceColor","blue");
% 
% marker.DisplayName = "Current Location";
% mstart.DisplayName = "Start Location";
% mend.DisplayName = "End Location";
% ptrack.DisplayName = "Trajectory";
% legend(gx)
% 
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

T = linspace(0,tspan(end),5);

zero = zeros(length(alt),1);

one = ones(length(alt),1);

%3D Trajectory Plot
figure
plot3(lon,lat,alt), hold on;
%stem3(lon,lat,alt, 'Marker', 'none', 'Color', 'black'), hold on;
%geoshow('landareas.shp', 'FaceColor',[0.5 1 0.5]),hold on %comment out this line to see on normal 3D axis

quiver3(lat,lon,alt,zero,zero,one)

% quiver3(lon(T(2)),lat(T(2)),alt(T(2)),x(T(2),3),x(T(2),4),control(8))
% quiver3(lon(T(3)),lat(T(3)),alt(T(3)),x(T(3),3),x(T(3),4),control(9))
% quiver3(lon(T(4)),lat(T(4)),alt(T(4)),x(T(4),3),x(T(4),4),control(10))
% quiver3(lon(T(5)),lat(T(5)),alt(T(5)),x(T(5,3)),x(T(5),4),control(11))

% quiver3(lon(T(2)),lat(T(2)),alt(T(2)),0,0,1,0.8)%ontrol(8))
% quiver3(lon(T(3)),lat(T(3)),alt(T(3)),0,0,1,0.9)%control(9))
% quiver3(lon(T(4)),lat(T(4)),alt(T(4)),0,0,1,0.7)%control(10))
% quiver3(lon(T(5)),lat(T(5)),alt(T(5)),0,0,1,0.5)%control(11))


grid minor

grid on
xlabel('Longitude (degrees)')
ylabel('Latitude (degrees)')
zlabel('Altitude (km)')

%2D Trajectory Plot
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
