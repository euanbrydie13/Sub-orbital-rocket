function location = location_select(~)

ltype = 'Cornwall';

switch ltype
    
    case'Sutherland'
        location.lat = deg2rad(58.5107);         % Latitude (N)
        location.lon = deg2rad(-4.5121);         % Longitude (W) 
        location.heading = deg2rad(5);           % Flight Heading Angle 
    
    case'Shetland'
        location.lat = deg2rad(60.81667);        % Latitude (N)
        location.lon = deg2rad(-0.76667);        % Longitude (W) 
        location.heading = deg2rad(30);          % Flight Heading Angle 
        
    case'Cornwall'
        location.lat = deg2rad(50.44);           % Latitude (N)
        location.lon = deg2rad(-5.0088);         % Longitude (W) 
        location.heading = deg2rad(320);         % Flight Heading Angle
        
    case'South Uist'
        location.lat = deg2rad(57.33);           % Latitude (N)
        location.lon = deg2rad(-7.33);           % Longitude (W) 
        location.heading = deg2rad(270);         % Flight Heading Angle 
    otherwise 
        fprintf('Invalid Selection')
end
