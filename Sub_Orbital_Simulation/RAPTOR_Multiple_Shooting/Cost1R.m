function J2 = Cost1R(y,x0)

tspan = 0:1:y(1);

x = ode3(@(t,x,control)dynamics_model(t,x,y),tspan,x0);

J2 = -sum(x(:,1));

end
