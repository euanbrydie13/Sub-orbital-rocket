%% Trajectory Simulation 
% This code uses an optimal control vector and plots the corresponding
% outputs. SKYLARK L simulated from Space Port Sutherland

%% Define Inital Conditions

control =[200; 0;0;0;0;0; 1;1;0;0;0; 0;0;0;0;0]; % [Time; Angle of Attack; Throttle; Bank Angle]

t0=0;                                      % Start time                                
tspan = t0:1:control(1);                   % Time Span

x0(1) = 100;                               % Altitude x(1)
x0(2) = 0;                                 % Velocity x(2)
x0(3) = 80*pi/180;                         % Flight path angle x(3)
x0(4) = 0.*(pi./180);                      % Flight heading angle x(4)
x0(5) = 58.5127.*(pi./180);                % Latitude x(5)
x0(6) = -4.5121.*(pi./180);                % Longitude x(6)
x0(7) = 2466;                              % Mass of the vehicle x(7)

%% Apply ODE Solver and Controls

% Supplying simulation with trial and error controls, not optimised. Best
% guess 

x = ode3(@(t,x)dynamics_model(t,x,control),tspan,x0);

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

% %% Displaying Results 
% 
% % % Convert results back into degrees for plotting
% lat = x(:,5)*(180/pi);
% lon = x(:,6)*(180/pi);
% alt = x(:,1)*10^-3;
% heading = x(:,4)*(180/pi);
% 
% %Creating geographic axes and globe in same figure
% figpos = [1000 500 800 400];
% uif = uifigure("Position",figpos);
% ug = uigridlayout(uif,[1,2]);
% p1 = uipanel(ug);
% p2 = uipanel(ug);
% gx = geoaxes(p1,"Basemap","satellite"); 
% gg = geoglobe(p2); 
% gx.InnerPosition %= gx.OuterPosition;
% gg.Position = [0 0 1 1];
% 
% %Plotting trajectory data on maps
% geoplot3(gg,lat,lon,alt,"c","LineWidth",2,"HeightReference","geoid")
% hold(gx,"on")
% ptrack = geoplot(gx,lat,lon,"c","LineWidth",2);
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
% 
% % disp(heading)
% %3D Trajectory Plot
% figure
% plot3(lon,lat,alt), hold on;
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

%%h h h h j