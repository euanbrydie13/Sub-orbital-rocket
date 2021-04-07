function [c,ceq]=Confun2R(y2,x02,y_opt) 

tspan = y_opt(1):1:y2(1);

x = ode3(@(t,x,control) dynamics_model(t,x,y2),tspan,x02);

c = [];

ceq(1) = x(end,1) - 102000;
ceq(2) = x(end,2) - 99;
ceq(3) = x(end,3) - deg2rad(-20);
%ceq(3) = x(end,4) - ; 
%ceq(4) = x(end,5) - ;
%ceq(4) = x(end,6) - ;
ceq(4) = x(end,7) - 800;
ceq = ceq';
end