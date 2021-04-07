function J2 = Cost3S(y3,x03,y_opt2)

tspan = y_opt2(1):1:y3(1);

x = ode3(@(t,x,control)dynamics_model(t,x,y3),tspan,x03);

J2 = sum(x(:,1));

end