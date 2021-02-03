function vehicle = vehicle_select(~)

vtype = 'SKYLARK_L';

switch vtype
    
    case'SKYLARK_L'
        vehicle.sref = pi*(0.356)^2;         % Aerodynamic Reference area (m^2)
        vehicle.aexit = pi.*(0.1).^2;        % Exit Nozzle Area  (Assumed - need to find appropriate value) [m^2]
        vehicle.isp = 229;                   % Specfic Impulse [s]
        vehicle.tvac = 30e3;                 % Thrust Force at Sea Level (kn)
end
