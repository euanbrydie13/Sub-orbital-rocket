% Calculates the equivalent latitude and longitude coordinates..
% for a specfic downrange. 

%% Load Locations and vehicle 
location = location_select;
vehicle = vehicle_select; 

%% Starting coordinates
lat1 = location.lat;
lon1 = location.lon; 
chi1 = location.heading; 

%% Downrage of vehicle
Drange = vehicle.Drange; 

%% Calculating end coordinates

dlon = ((sin(chi1)*Drange)/111320*cos(lat1))*(pi/180);
dlat = ((cos(chi1)*Drange)/110574)*(pi/180);

lonfinal = lon1 + dlon;
latfinal = lat1 + dlat;

