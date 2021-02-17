%% Optimal Control problem

control_0 = [300; 0;0;0;0;1; 1;1;0;0;0; 0;0;0;0;0]; %initial input
options = optimoptions('fmincon','Display','iter-detailed','Algorithm','sqp');

[control_opt,fval,exitflag,output] = fmincon(@Cost_Function,control_0,[],[],[],[],[],[],@Non_Linear_Constraints,options);

x0= [10;1;1.57;1;1;1;2500];    %initial values of state variables
Tspan = 0:1:control_opt(1);    %Tspan calculated by using control output 1

x = ode3(@(t,x,u_opt) dynamics_model(t,x,u_opt),Tspan,x0,control_opt); 

figure 
grid on
plot(Tspan,x(:,1))