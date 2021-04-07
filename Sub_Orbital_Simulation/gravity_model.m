function [g, r] = gravity_model(x)

rE = 6375253;               % Earths radius [m]           
r = x(1) + rE;                    % Distance from centre of the earth [m]
% g0= 9.808;
% 
% g = g0*(rE/r)^2;

mu = 3.986004418e14;        % Geocentric constant of gravitation (GM) [m3/s2], ref: IERS Numerical Standards
J2 = 1.0826359e-3;          % Second degree term in Earth's gravity potential, ref: IERS Numerical Standards
J3=2.532153e-7;
J4=1.6109876e-7;
phi=pi./2-x(5);

g=(mu/r^2.*(1 - 1.5.*J2.*(3.*cos(phi).^2-1).*(rE./r).^2 ...
    -2.*J3.*cos(phi).*(5.*cos(phi).^2-3).*(rE./r).^3 ...
    -(5./8).*J4.*(35.*cos(phi).^4-30.*cos(phi).^2+3).*(rE./r).^4));


end