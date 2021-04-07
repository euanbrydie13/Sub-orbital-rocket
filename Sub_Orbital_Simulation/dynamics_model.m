function dx = dynamics_model(t,x,control)

%% Control model 
tof = control(1);                  %Time of Flight
n=(length(control)-1)/3;           %Length of decision variables
alpha_all = control(2:n+1);        %Decision variable 1 - Angle of Attack
throttle_all = control(7:7+n-1);   %Decision variable 2 - Throttle
bank_all = control(end-n+1:end);   %Decision variable 3 - Bank Angle

T = linspace(0, tof, n);           
alpha = interp1(T,alpha_all,t);
throttle = interp1(T,throttle_all,t);
bank = interp1(T,bank_all,t);

%% Define State Equations 

h = x(1);    % altitude from the surface of the Earth to the vehicle, m
v = x(2);    % vehicle velocity relative to the Earth, m/s
fpa = x(3);  % flight path angle (angle between the relative velocity and the local horizontal axis), rad
chi = x(4);  % flight heading angle (angle between the velocity vector projected onto the local horizontal plane, and North), rad
lat = x(5);  % latitude (+ North), rad
lon = x(6);  % longitude (+ East), rad
m = x(7);    % vehicle mass, kg

%% Define Vehicle Specifications 

vehicle = vehicle_select;
           
%% Define Atmospheric model 

[press,~,dens,sspeed] = atmo_model(h);  % Pressure and Density calcualted from atmospheric model

%% Define Aerodynamic model

[L, D] = Aerodynamic_model(v,alpha,vehicle,dens,sspeed);

%% Propulsion Model

[FT, mp] = propulsion_model(throttle,vehicle,press);

%% Gravity Model

[g,r] = gravity_model(x);
wE = 7.292115e-5;            % Angular rotation of earth [rads-1]

%% 3DOF Dynamic Equations (6 ODEs)

% Constraints on altitude and mass have been set to allow scheme to output
% only feasible results
if h <= 0 %|| m <= 0
    dx =zeros(7,1);
%     disp('Failure (either alt = 0 or mass < 0)');
    return 
end

% Within the system of ODEs there are singularities present at: 
% h = -Re; fpa = +90, -90 deg ; v = 0 ; lat = + - 90 deg. This explains the abs(x) < eps 

% Equations of motion have been found to be slightly inconsistent - two sources avaiable at: 
% 1.https://core.ac.uk/download/pdf/10128521.pdf
% 2.https://strathprints.strath.ac.uk/57162/1/Toso_etal_IAC_2015_optimisation_of_ascent_and_descent_trajectories.pdf
% 3.Tropico folder- Github 
% Need verification from supervisor on what one to select, only slight differences

% Rate of altitude - Same for all cases
dh = v.*sin(fpa);  % Rate of altitude 

% Rate of Latitude - Source 1/4
%dlat = (v.*cos(fpa).*sin(chi))./r; % Rate of latitude

% Rate of Latitude - Source 2/3
dlat = (v.*cos(fpa).*cos(chi))./r; % Rate of latitude
 
if  (abs(lat - pi/2) < eps(1) || abs(lat + pi/2) < eps(1))
        dlon = 0;
else
    
% Rate of Longitude - Source 1
%dlon = (v.*cos(fpa).*cos(chi))./(r.*cos(lat));

% Rate of Longitude - Source 2/3
dlon = (v.*cos(fpa).*sin(chi))./(r.*cos(lat)); 

end

% % Rate of velocity - Source 1
% Fx = (FT.*cos(alpha) - D)./m - g.*sin(fpa);
% dv = Fx + wE.^2.*r.*cos(lat).*(sin(fpa).*cos(lat) - cos(fpa).*sin(chi).*sin(lat));

% % Rate of velocity - Source 2/3/4
Fx = ((FT*cos(alpha) - D)/m) - g*sin(fpa);
dv = Fx + (wE^2)*r*cos(lat)*(sin(fpa)*cos(lat) - cos(fpa)*sin(chi)*sin(lat));

if  abs(v) < eps(1) 
     dfpa = 0;
else
% 
% Rate of flight path angle - Source 1    
% dfpa =((FT*sin(alpha)+L)/(m*v))*cos(bank)+((g/v)+(v/r))*cos(fpa)...
%         + 2*wE*cos(lat)*cos(chi)...
%         + ((wE^2*r)/v)*cos(lat)*(cos(fpa)*cos(lat) + sin(fpa)*sin(lat)*sin(chi));
%  
% Rate of flight path angle - Source 2   
% dfpa =((FT*sin(alpha)+L)/(m*v))*cos(bank)-((g/v)-(v/r))*cos(fpa)...
%          + 2*wE*cos(chi)*cos(lat)...
%          + (wE^2)*(r/v)*cos(lat)*(sin(chi)*sin(fpa)*sin(lat) + cos(fpa)*cos(lat));

% % Rate of flight path angle - Source 3
% Fz = (FT*sin(alpha)+L)*cos(bank)/(m*v) - g*cos(fpa);
%     dfpa = (v/r)*cos(fpa) + Fz/v + (wE^2)*(r/v)*cos(lat)*(sin(fpa)*cos(chi)*sin(lat) + cos(fpa)*cos(lat)) + 2*wE*sin(chi)*cos(lat);

% Rate of flight path angle - Source 4

dfpa =((FT*sin(alpha)+L)/(m*v))*(cos(bank))-((g/v)-(v/r))*cos(fpa)...
          + 2*wE*cos(chi)*cos(lat)...
          + (wE^2)*(r/v)*cos(lat)*(sin(chi)*sin(fpa)*sin(lat) + cos(fpa)*cos(lat));

end
%  
%if  (abs(lat - pi/2) < eps(1) || abs(lat + pi/2) < eps(1) || abs(fpa - pi/2) < eps(1) || abs(v) < eps(1))
     dchi = 0;
%else
% 
% Rate of flight heading angle - Source 1
% dchi = ((FT*sin(alpha)+L)/(m*v*cos(fpa)))*sin(bank) - (v/r)*cos(fpa)*cos(chi)*tan(lat)...     
%           + 2*wE*(tan(fpa)*cos(lat)*sin(chi) - sin(lat))...
%           - (((wE^2)*r)/(v*cos(fpa)))*sin(lat)*cos(lat)*cos(chi);
%  
% Rate of flight heading angle - Source 2
% dchi = ((L)/(m*v*cos(fpa)))*sin(bank) - (v/r)*cos(fpa)*cos(chi)*tan(lat)...     
%           + 2*wE*(sin(chi)*cos(lat)*tan(fpa) - sin(lat))...
%           - wE^2*(r/(v*cos(fpa)))*cos(lat)*sin(fpa)*cos(chi);

% Rate of flight heading angle - Source 3 

% Fy = (FT*sin(alpha)+L)*sin(bank)/m;
%     dchi = (v/r)*cos(fpa)*sin(chi)*tan(lat) + Fy/(v*cos(fpa)) ...
%         + wE^2*r*(sin(chi)*sin(lat)*cos(lat))/(v*cos(fpa)) + 2*wE*(sin(lat)-tan(fpa)*cos(chi)*cos(lat));
% 
% 
% Rate of flight heading angle - Source 4

% dchi = ((L)/(m*v*cos(fpa)))*sin(bank) - (v/r)*cos(fpa)*cos(chi)*tan(lat)...     
%     + 2*wE*(sin(chi)*cos(lat)*tan(fpa) - sin(lat))...
%      - wE^2*(r/(v*cos(fpa)))*cos(lat)*sin(fpa)*cos(chi);
%end
dm = -mp;
 
dx = [dh; dv; dfpa; dchi; dlat; dlon; dm];

%% Local Level Horizon 

% dh = v.*sin(fpa);  % Rate of altitude 
% 
% dlat = (v.*cos(fpa).*cos(chi))./r; % Rate of latitude
% 
% dlon = ((v.*cos(fpa).*sin(chi)))./r; % Rate of longitude 
% 
% % if fpa<0
% %     fpa = -1.*(fpa);
% % 
% % end
% 
% dv = (FT.*cos(alpha)-D)./m - gr.*sin(fpa); %Rate of velocity
% 
% dfpa = ((FT.*sin(alpha)+L).*cos(bank))./(m.*v) - (gr.*cos(fpa))./v; % Rate of flight path angle
% 
% dchi = ((FT.*sin(alpha)+L).*sin(bank))./(m.*v.*cos(fpa)); % Rate of heading 

% dm = -mp; % Mass flow rate 
% 
% dx = [dh; dv; dfpa; dchi; dlat; dlon; dm];

%% Zipfels Equations
% Zipfel, P. (2007).Modeling and Simulation of Aerospace Vehicle Dynamics, Second Edition. AIAAEducation Series
% %% Transformation Matrices 
% 
% %Gravity to Earth frame
% TGE = [-sin(lat)*cos(lon),-sin(lat)*sin(lon), cos(lat);
%                 -sin(lon),          cos(lon),        0;
%       -cos(lat)*cos(lon),-cos(lat)*sin(lon), -sin(lat)];
%      
% %Gravity to Velocity coordinates
% TVG = [cos(fpa)*cos(chi), cos(fpa)*sin(chi), -cos(fpa);
%                -sin(chi),          cos(chi),         0;
%        sin(fpa)*cos(chi), sin(fpa)*sin(chi), cos(fpa)];
% 
% TVE = TVG*TGE;
% 
% %Earth to Velocity coordinates
% % TVE1 =  [cos(fpa)*cos(lon), cos(fpa)*sin(lon), -sin(fpa);
% %        -sin(lon)        ,          cos(lon),         0 ;
% %         sin(fpa)*cos(lon), sin(fpa)*sin(lon), cos(fpa)];
% 
% %Transpose Transformations
% TGV = TVG';     
% TEV = TVE';
% TEG = TGE'; 
% 
% %% Vectors Forces
% 
% % Aerodynamic/Propulsion (VFrame)
% FAP = [            FT*cos(alpha)-D;
%        (FT*sin(alpha)+L)*sin(bank);
%       -(FT*sin(alpha)+L)*cos(bank)];
% 
% % Gravity (GFrame)
% GG =  [0
%        0
%        g];
% 
% % Velocity (VFrame)
% V = [v
%      0
%      0];
% 
% % Skew symmetric matrix of earths angular velocity (Eframe)
% 
% wE = 7.292115e-5;           % Angular rotation of earth [rads-1]
% 
% WE = [0,-wE, 0;
%      wE,  0, 0;
%       0,  0, 0];
%   
% % Distance from ECentre (GFrame)
% R = [0
%      0
%     -r];
% 
% % 6 Equations of Motion
% M = (1/m)*FAP + TVG*GG - 2*TVE*WE*TEV*V - TVE*WE*WE*TEG*R;
% 
% G = TGV*V;