function J2 = Cost2S(y2,x02,y_opt)

tspan = y_opt(1):1:y2(1);

x = ode3(@(t,x,control)dynamics_model(t,x,y2),tspan,x02);

J2 = -sum(x(:,1));

end