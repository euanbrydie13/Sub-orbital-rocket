%% Simple Flight Corridor 
% This calculates a simple flight corridor from specfic start and end
% coordinates

%% From Downrange Script

% Load Locations and vehicle 
location = location_select;
vehicle = vehicle_select; 

% Starting coordinates
lat1 = location.lat;
lon1 = location.lon; 

chi1 = location.heading; 

% Downrage of vehicle
Drange = vehicle.Drange; 

% Calculating end coordinates

dlon = ((sin(chi1)*Drange)/(111320*cos(lat1)))*(pi/180);
dlat = ((cos(chi1)*Drange)/110574)*(pi/180);

lonfinal = lon1 + dlon;
latfinal = lat1 + dlat;

lat11 = rad2deg(lat1);
lon11 = rad2deg(lon1);

latfinal1 = rad2deg(latfinal);
lonfinal1 = rad2deg(lonfinal);

%% Drawing Start Circle

R = 0.001;%2440/110574;                % radius of circle 
C = [lat11,lon11];              % center of circle 
th = linspace(((pi/2)+location.heading),((3*pi/2)+location.heading));
x1  = C(1)+R*cos(th) ; 
y1 = C(2)+R*sin(th) ;

geoplot(x1,y1,'r')
hold on 

%% Drawing End Circle

R = 5889.36/110574;                       % radius of circle 
C = [latfinal1,lonfinal1];                % center of circle 
th = linspace(((pi/2)+location.heading), (-pi/2 + location.heading));
x2  = C(1)+R*cos(th) ; 
y2 = C(2)+R*sin(th) ;

geoplot(x2,y2,'r')
hold on 
geoplot([x1(1) x2(1)], [y1(1) y2(1)],'r')
hold on 
geoplot([x1(:,end) x2(:,end)], [y1(:,end) y2(:,end)],'r')
