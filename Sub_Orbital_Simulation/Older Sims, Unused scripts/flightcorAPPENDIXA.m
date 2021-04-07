% %% %% APPENDIX A TO PART 420—METHOD FOR DEFINING A FLIGHT CORRIDOR
% % Author: Ali Rajabi - 08/02/2021
% 
% %% Assigning variables
% %function [] = flightcorAPPENDIXA(x)
% 
% %1NM = 1852m
% %1 degree = 111111m 
% 
% D_max = 1.32*1852; % debris dispersion radius (nm) (for guided suborbital vehicles)
% D_oez = 3.18*1852; % overflight exclusion zone downrange distance (nm)(for guided suborbital vehicles)
% 
% % line segment lengths (nm) (for guided suborbital vehicles)
% CF = 39.91*1852;
% DE = 118.10*1852;
% HI = 0;
% 
% %H_ap = max(x(:,1)); % apogee altitude (apogee altitude should equal the highest altitude)
% H_ap = (100001);
% 
% if H_ap < 100000       %check this is in km, might need to multiply by 100
%     IP = 0.4;      %impact range factor, dependant on apogee altitude
% else 
%     IP = 0.7;
% end 
%     
% Disp_Hap = 0.5; % dispersion factor 
% 
% D = H_ap * IP; % impact range (D)
% R = H_ap * Disp_Hap; % impact dispersion radius (R)
% 
% disp(D)


