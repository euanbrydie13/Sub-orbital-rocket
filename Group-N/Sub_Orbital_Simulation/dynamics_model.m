function dx = dynamics_model(t,x,control)

%% Control model 
tof = control(1);
n=(length(control)-1)/3;
alpha_all = control(2:n+1);
throttle_all = control(n+2:n+6);
bank_all = control(n+7:end);

T = linspace(0, tof, n);
alpha = interp1(T,alpha_all,t);
throttle = interp1(T,throttle_all,t);
bank = interp1(T,bank_all,t); 

%% Vehicle Specifications

vehicle = vehicle_select;
           
%% State equations from dissertation
h = x(1);   % altitude from the surface of the Earth to the vehicle, m
v = x(2);   % vehicle velocity relative to the Earth, m/s
fpa = x(3); % flight path angle (angle between the relative velocity and the local horizontal axis), rad
chi = x(4); % flight heading angle (angle between the velocity vector projected onto the local horizontal plane, and North), rad
lat = x(5); % latitude (+ North), rad
lon = x(6); % longitude (+ East), rad
m = x(7);   % vehicle mass, kg

%% Atmospheric model 

[press,dens] = atmo_model(x(1));  % Pressure and Density calcualted from atmospheric model

%% Aerodynamic model Simple Rocket

[L, D] = Aerodynamic_model(x,vehicle,dens);

%% Gravity model

rE = 6378137.00*(1-(1/298.257223563)*sin(lat)^2); % Earths radius [m]           
r = h + rE;                 % Distance from centre of the earth [m]
wE = 7.292118e-05;          % Angular rotation of earth [rads-1]
mu = 3.986004418e14;        % Geocentric constant of gravitation (GM) [m3/s2], ref: IERS Numerical Standards
J2 = 1.0826359e-3;          % Second degree term in Earth's gravity potential, ref: IERS Numerical Standards
J3=2.532153e-7;
J4=1.6109876e-7;


phi=pi/2-lat;

gr=(mu/r^2*(1 - 1.5*J2*(3*cos(phi)^2-1)*(rE/r)^2 ...
    -2*J3*cos(phi)*(5*cos(phi)^2-3)*(rE/r)^3 ...
    -(5/8)*J4*(35*cos(phi)^4-30*cos(phi)^2+3)*(rE/r)^4));
gt=-3*mu*sin(phi)*cos(phi)*(rE/r)^2*(J2+0.5*J3*(5*cos(phi)^2-1)...
    *(rE/r)/cos(phi)+(5/6)*J4*(7*cos(phi)^2-1)*(rE/r)^2)/r^2;


%% Propulsion Model

[FT, mp] = propulsion_model(throttle,vehicle,press);

%% 3DOF Dynamic Equations (6 ODEs)

if h <= 0 || m <= 0 
    dx =zeros(7,1);
    disp('Vehicle failure (either alt = 0 or mass < 0)');
    return 
end

dh = v.*sin(fpa);
dlat = v.*cos(fpa).*cos(chi)./r;
if  abs(lat-(pi/2)) < eps(1)
     dlon = 0;
else
    dlon = v.*cos(fpa).*sin(chi)./(r.*cos(lat));
end

Fx = (FT.*cos(alpha) - D)./m - gr.*sin(fpa) + gt*cos(fpa)*cos(chi);
dv = Fx + wE.^2.*r.*cos(lat).*(sin(fpa).*cos(lat) - cos(fpa).*cos(chi).*sin(lat));

if abs(v) < eps(1)
    dfpa = 0;
else
   Fz = ((FT*sin(alpha)+L)*cos(bank))/m - gt*sin(fpa)*cos(chi);
     
   dfpa = (v/r)*cos(fpa) + Fz/v + (wE^2*r/v)*cos(lat)*(sin(fpa)*cos(chi)*sin(lat) + cos(fpa)*cos(lat)) + 2*wE*sin(chi)*cos(lat);
    
    
%dfpa =  ((FT.*sin(alpha)+L).*cos(bank))./(v.*m) - (gr.*cos(fpa))./v + ((wE^2.*r.*cos(lat))./v).*(sin(fpa).*cos(chi).*sin(lat) + cos(fpa).*cos(lat)) + 2.*wE.*sin(chi).*cos(lat);
        
end

if (abs(lat - pi/2) < eps(1) || abs(lat + pi/2) < eps(1) || abs(fpa - pi/2) < eps(1) || abs(v) < eps(1))
     dchi = 0;
else
   Fy = ((FT*sin(alpha)+L)*sin(bank))/m - gt*sin(chi);
   
   dchi = (v/r)*cos(fpa)*sin(chi)*tan(lat) + Fy/(v*cos(fpa)) ...
       + wE^2*r*(sin(chi)*sin(lat)*cos(lat))/(v*cos(fpa)) + 2*wE*(sin(lat)-tan(fpa)*cos(chi)*cos(lat));
   
   %dchi = ((FT.*sin(alpha)+L).*sin(bank))./(v.*cos(fpa).*m) ...
   %    + (wE^2.*r.*sin(chi).*sin(lat).*cos(lat))./(v.*cos(fpa)) + 2.*wE.*(sin(lat)-tan(fpa).*cos(chi).*cos(lat));
   
end
dm = -mp;

dx = [dh; dv; dfpa; dchi; dlat; dlon; dm];

end

