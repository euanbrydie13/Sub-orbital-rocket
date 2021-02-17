FT = 1;
alpha = 1;
L =1;
bank =1;
v =1;
m =1;
gr=1;
fpa =1;
wE =1;
lat =1;
chi=1;
r =1;


%1.7284
%2.8090


%dfpa =  ((FT.*sin(alpha)+L).*cos(bank))./(v.*m) - (gr.*cos(fpa))./v + ((wE^2.*r.*cos(lat))./v).*(sin(fpa).*cos(chi).*sin(lat) + cos(fpa).*cos(lat)) + 2.*wE.*sin(chi).*cos(lat)

Fz = ((FT.*sin(alpha)+L).*cos(bank))./m;
     
dfpa = (v/r).*cos(fpa) + Fz./v + (wE^2.*r./v).*cos(lat).*(sin(fpa).*cos(chi).*sin(lat) + cos(fpa).*cos(lat)) + 2.*wE.*sin(chi).*cos(lat)