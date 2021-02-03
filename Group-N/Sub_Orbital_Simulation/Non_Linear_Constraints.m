%nonlinear constraints
function [c,ceq]=Non_Linear_Constraints(control)

Tspan = 0:1:225;
x0= [10;1;1.57;1;1;1;2500]; %initial values of state variables


x = ode3(@(t,x,control) dynamics_model(t,x,control),Tspan,x0,control);
c=[];
ceq(1)= x(end,1) - 1000;
ceq = ceq';
end