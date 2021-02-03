%% Optimal Control problem

Tspan = 0:1:225;
x0= [10;1;1.57;1;1;1;2500]; %initial values of state variables

control_0 = [225; 1;1;1;1;1; 1;1;1;1;1]; %initial input
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','sqp');

[u_opt,fval,exitflag,output] = fmincon(@Cost_Function,control_0,[],[],[],[],[],[],@Non_Linear_Constraints,options);
x = ode3(@(t,x,u_opt) dynamics_model(t,x,u_opt),Tspan,x0,u_opt); 

figure 
grid on
plot(Tspan,x(:,1))