function [FT, mp] = propulsion_model(throttle,vehicle,press)

FT = throttle.*(vehicle.tvac + vehicle.aexit.*(1e5 - press));          % Force of Thrust [N]
mp = throttle.*(FT./(9.81.*vehicle.isp));                             % Mass flow [kgs-1]

% mp = vehicle.aexit*vehicle.exitV*vehicle.fluidD;                       %throttle.*(FT./(9.81.*vehicle.isp));                           % Mass flow [kgs-1]
% FT = throttle.*(vehicle.tvac*mp + vehicle.aexit.*(1e5 - press));       % Force of Thrust [N]
end