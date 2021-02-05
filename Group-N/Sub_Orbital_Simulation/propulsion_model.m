function [FT, mp] = propulsion_model(throttle,vehicle,press)

mp = throttle.*(FT./(9.81.*vehicle.isp));                              % Mass flow [kgs-1] - Could be fluid density at nozzle x exhaust velocity x nozzle exit area
FT = throttle.*(vehicle.tvac*mp + vehicle.aexit.*(1e5 - press));       % Force of Thrust [N] - Changed by Paddy
                        
end