% Cost function 
function J2=Cost_Function(x0,control_opt)

x = ode3(@(t,x,u_opt) dynamics_model(t,x,u_opt),Tspan,x0,control_opt);

J2 = -max(x(:,1));

end