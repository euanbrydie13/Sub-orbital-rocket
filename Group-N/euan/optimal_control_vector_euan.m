%% Optimisation of Suborbital Rocket
% Aims to obtain the final optimal control vector using the fmincon function

%% Vehicle Specifications [SKYLARK L]
% vehicle.sref = pi*(0.356)^2;         % Reference area (m^2)
% vehicle.aexit = pi.*(0.25).^2;       % Exit Nozzle Area  (Assumed - need to find appropriate value) [m^2]
% vehicle.isp = 229;                   % Specfic Impulse [s]
% vehicle.tvac = 30e3;                 % Thrust Force (kn)

%% Initial Guess

u0 =[100; 0;0;0;0;0; 1;1;1;1;1];

lb = [40, ...                                                 % Lower Bounds
    -5*pi/180 -5*pi/180 -5*pi/180 -5*pi/180 -5*pi/180 ...
    0 0 0 0 0];

ub = [200, ...
    20*pi/180 20*pi/180 20*pi/180 20*pi/180 20*pi/180 ...     % Upper Bounds
    1 1 1 1 1];

%nonlinearconstraints = @nlcon

[u, fval] = fmincon(@obfunction_euan,u0,[],[],[],[],lb,ub,[]);
