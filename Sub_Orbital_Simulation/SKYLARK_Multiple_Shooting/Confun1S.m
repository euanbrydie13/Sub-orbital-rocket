function [c,ceq]= Confun1S(y, x0) 

tspan = 0:1:y(1);

x = ode3(@(t,x,control) dynamics_model(t,x,y),tspan,x0);

c = [];

ceq(1) = x(end,1) - 37800;
ceq(2) = x(end,2) - 950;
ceq(3) = x(end,3) - 1.51;
%ceq(3) = x(end,4) - ; 
%ceq(4) = x(end,5) - ;
%ceq(4) = x(end,6) - ;
ceq(4) = x(end,7) - 800;
ceq = ceq';
end