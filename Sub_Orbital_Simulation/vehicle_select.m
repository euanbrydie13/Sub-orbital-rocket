function vehicle = vehicle_select(~)

vtype = 'SKYLARK L';

switch vtype
    
    case'SKYLARK L'
        
        vehicle.sref = pi.*(0.356)^2;         % Aerodynamic Reference area (m^2)
        vehicle.aexit = pi.*(0.1)^2;          % Exit Nozzle Area  (Assumed - need to find appropriate value) [m^2]
        vehicle.isp = 229;                    % Specfic Impulse [s]
        vehicle.tvac = 30e3;                  % Thrust Force at Sea Level (kn)
        vehicle.wet = 2466;                   % Wet mass of vehicle
        vehicle.aero = 1.5; 
        
    case 'RAPTOR MERLIN'
        
        vehicle.sref = pi.*(0.0780)^2;        % Aerodynamic Reference area (m^2)
        vehicle.aexit = pi.*(0.06)^2;         % Exit Nozzle Area  (Assumed - need to find appropriate value) [m^2]
        vehicle.isp = 230;                    % Specfic Impulse [s]
        vehicle.tvac = 10;                    % Thrust Force at Sea Level (kn)
        vehicle.wet = 65.62;                  % Wet mass of vehicle

    case 'RAPTOR PEREGRINE'
        
        vehicle.sref = pi.*(0.1375)^2;        % Aerodynamic Reference area (m^2)
        vehicle.aexit = pi.*(0.07)^2;         % Exit Nozzle Area  (Assumed - need to find appropriate value) [m^2]
        vehicle.isp = 230;                    % Specfic Impulse [s]
        vehicle.tvac = 20e3;                  % Thrust Force at Sea Level (kn)
        vehicle.wet = 250;                    % Wet mass of vehicle
        vehicle.aero = 0.5;
end

