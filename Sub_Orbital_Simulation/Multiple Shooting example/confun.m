function [c, ceq] = confun(y, auxdata)

% Get some parameters
N           = auxdata.N;
T           = auxdata.T;
M           = auxdata.M;
x_init      = auxdata.x0;
x_final     = auxdata.xf;
Nstates     = auxdata.Nstates;
Ncontrols   = auxdata.Ncontrols;

% extract states and controls
x = reshape(y(1:Nstates*N), [], Nstates);
u = reshape(y(Nstates*N+1: end), [], Ncontrols);

% time step
h = T/(N-1)/(M-1);

% Do single shooting in each subinterval
states_atNodes = zeros(N, Nstates);
for i = 1:N-1
    x0 = x(i, :);
    u0 = u(i, :);
    states = zeros(M, Nstates);
    states(1,:) = x0; 
    % integrate to get x(i+1), using RK4 (explicit form)
    for j=1:M-1
       states_atNodes(j+1,:) = ode3(@(t,x,control_opt) dynamics_model(t,x,control_opt),Tspan,x0,control_opt);
    end
    states_atNodes(i+1,:) = states(end,:);
end

% continuity constraints
ceq_temp = x(2:end,:) - states_atNodes(2:end,:);

% initial and final state postions
ceq_temp = [ceq_temp; x(1,:) - x_init];
ceq_temp = [ceq_temp; x(end,:) - x_final];

% equality constraints
ceq = reshape(ceq_temp, [], 1);

% inequality constraints
% c = states_atNodes(:, 1)' - 1/9; % this may not important (for this problem)as we imposed
% lower bound for states at nodes

c = [];
end

