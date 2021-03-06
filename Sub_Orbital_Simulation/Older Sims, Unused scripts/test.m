% CF = 2;    %CF distance divided by 2 and changed from NM to degrees
% DE = 6;     %DE distance divided by 2 and changed from NM to degrees
% line1 = 4;     %CF dashed line changed to degrees
% line2 = 7;
% 
% lat = 4;
% lon = 5;
% 
% heading = 30
% 
% latF1 = lat+line1; %Splitting the points of F,E,C and D into coordinates
% latC1 = lat+line1; 
% 
% latE1 = lat+line2; %originating from the launch position (lat and lon)
% latD1 = lat+line2;
% 
% lonF1 = lon+CF;
% lonC1 = lon-CF;
% 
% lonE1 = lon+DE;
% lonD1 = lon-DE;
% 
% %%%%%%
% 
% latF = lat+(line1*cos(heading)-CF*sin(heading)); %Splitting the points of F,E,C and D into coordinates
% latC = lat+(line1*cos(heading)+CF*sin(heading)); 
% 
% latE = lat+(line2*cos(heading)-DE*sin(heading)); %originating from the launch position (lat and lon)
% latD = lat+(line2*cos(heading)+DE*sin(heading));
% 
% lonF = lon+(CF*cos(heading)+line1*sin(heading));
% lonC = lon-(CF*cos(heading)-line1*sin(heading));
% 
% lonE = lon+(DE*cos(heading)+line2*sin(heading));
% lonD = lon-(DE*cos(heading)-line2*sin(heading));
% 
% plot([latF1 latE1], [lonF1 lonE1], 'g')
% hold on
% plot([latC1 latD1], [lonC1 lonD1], 'g')
% hold on
% plot([latF latE], [lonF lonE], 'r')
% hold on
% plot([latC latD], [lonC lonD], 'r')

x = 7;
y = 5; 
theta = 1;
theta1 = deg2nm(theta); 

x1 = x*cos(theta)+y*sin(theta);
y1 = (-x)*sin(theta)+y*cos(theta);

disp(x1)
disp(y1)


