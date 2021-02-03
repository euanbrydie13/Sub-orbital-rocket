%% Atmospheric model
function [press, temp, dens, sspeed] = atmo_euan(h) 
y(1) = h;

%% ATMOSPHERIC DATA ---------------------------
altitudes = [0 11000 20000 32000 47000 51000 71000 84852];
staticpres= [101325 22632.1 5474.89 868.02 110.91 66.94 3.96 0.3734];
standtemp = [288.15 216.65 216.65 228.65 270.65 270.65 214.65 186.87];
lapserate = [-0.0065 0 0.001 0.0028 0 -0.0028 -0.002 0];
R = 8.31432;
g0 = 9.80665;
mi = 0.0289644;
i = 0;
        
if y(1) <= altitudes(1)
press = 101325;
temp = 288.15;
elseif y(1) <= altitudes(2)
i=1;
elseif y(1) <= altitudes(3)
i=2;
elseif y(1) <= altitudes(4)
i=3;
elseif y(1) <= altitudes(5)
i=4;
elseif y(1) <= altitudes(6)
i=5;
elseif y(1) <= altitudes(7)
i=6;
elseif y(1) <= altitudes(8)
i=7;
else y(1) > altitudes(8)
press = 1e-6;
temp = 186.87;
end
       
switch i
case {1,3,4,6,7}
press = staticpres(i)*(standtemp(i)./(standtemp(i)+lapserate(i).*(y(1)-altitudes(i)))).^(g0.*mi./(R.*lapserate(i)));
temp = standtemp(i)+lapserate(i).*(y(1)-altitudes(i));
case {2,5}
press = staticpres(i).*exp((-g0.*mi.*(y(1)-altitudes(i))/(R.*standtemp(i))));
temp = standtemp(i)+lapserate(i).*(y(1)-altitudes(i));
% otherwise
% press = 1e-6;  
% temp = 186.87;
% end
%%this part was overiding the press and temp value for y(1) < altitudes(1)
        
dens= press./(287.058.*temp);
sspeed=sqrt(1.4.*287.058.*temp);
end 