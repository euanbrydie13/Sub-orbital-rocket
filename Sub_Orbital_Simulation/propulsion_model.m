function [FT, mp] = propulsion_model(throttle,vehicle,press)

% FTvac = vehicle.tvac;                                % Force of Thrust [N]
% FT = throttle*(FTvac + (vehicle.aexit.*(-press)));  % Force of Thrust [N]
% mp = (FT/(9.81*vehicle.isp));                        % Mass flow [kgs-1] 
%exitvelocity = FTvac/mp;                            % Exity Velocity [ms-1]
%totalpressure = 5e6;                                % Total Pressure [Pa]
%gamma = 1.2;                                        % Specific Heat Ratio
%exitpressure = totalpressure*((1 + ((gamma - 1)/2)*((sspeed/exitvelocity)^2)^(-gamma/(gamma-1)))); % Exit Pressure [Pa]

FT = throttle.*(vehicle.tvac + vehicle.aexit.*(1e5 - press));          % Force of Thrust [N]
mp = throttle.*(FT./(9.81.*vehicle.isp));                             % Mass flow [kgs-1]

end