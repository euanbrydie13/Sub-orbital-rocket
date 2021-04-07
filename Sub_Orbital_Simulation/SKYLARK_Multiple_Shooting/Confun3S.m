function [c,ceq]=Confun3S(y3,x03,y_opt) 

tspan = y_opt(1):1:y3(1);

x = ode3(@(t,x,control) dynamics_model(t,x,y3),tspan,x03);

c = [];

ceq(1) = x(end,1) - 100;
%ceq(1) = x(end,2) - ;
ceq(2) = x(end,3) - deg2rad(-90);
%ceq(3) = x(end,4) - ; 
%ceq(4) = x(end,5) - ;
%ceq(4) = x(end,6) - ;
ceq(3) = x(end,7) - 800;
ceq = ceq';
end