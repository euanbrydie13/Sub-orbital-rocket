% This code demonstrates an example of solving constrained optimization problem 
% with multiple shooting method.
% Author: Vinh Quang Nguyen - University of Massachusetts, Amherst
% Constrained Optimization Problem:
%   minimize L = 1/2 * integral_from_t0_to_T of (u.^2)
%   Subject to: x_dot = (x2; u); with x = (x1, x2)
%   initial value x(0) = (0; 1);
%   final value x(T) = (0; -1);
%   inequality constraint: x1 < 1/9;

%%
clear

N           = 20;           % number of nodes => (N-1) subintervals
M           = 4;            % number of points per subinterval
T           = 200;          % final time
Nstates     = 7;            % number of states
Ncontrols   = 1;            % number of controls
x0          = [0, 0, 0, 0, 0, 0, 1];       % initial states
xf          = [1, 1, 1, 1, 1, 1, 0];       % final states

% Store parameters for the use of constraint and objective function
auxdata.N           = N;         
auxdata.M           = M;          
auxdata.T           = T;          
auxdata.Nstates     = Nstates;    
auxdata.Ncontrols   = Ncontrols;  
auxdata.x0          = x0;    
auxdata.xf          = xf;   

% Set lower bound and upper bound for states and control
x1L = -Inf*ones(N, 1);                  x1U = Inf*ones(N, 1);
x2L = -Inf*ones(N, 1);                  x2U = Inf*ones(N, 1);

% piecewise fuction for control u, there are N-1 values of control
uL = -200*ones(N-1, 1);                 uU = 200*ones(N-1, 1); 

yL = [x1L; x2L; uL];
yU = [x1U; x2U; uU];

% Initial guess
y0 = ones((Nstates + Ncontrols)* N - 1, 1);

% check functions
confun(y0, auxdata)
objfun(y0, auxdata)

%% Solve NLP problem:
% set up options for solver
options = optimoptions('fmincon','Display','Iter','Algorithm','sqp',...
    'MaxFunEvals',Inf); % interior-point

tic
[yopt,f] = fmincon(@(y) objfun(y, auxdata),y0,[],[],[],[],yL,yU,...
        @(y) confun(y, auxdata),options);
toc 

%% Extract states and controls
x = reshape(yopt(1:Nstates*N), [], Nstates);
u = reshape(yopt(Nstates*N+1: end), [], Ncontrols);

% Find the analytic solution for later comparison
t = linspace(0, 1, 1000);
l = 1/9;
[X, U] = analytic_solution(t, l);

%% Plot results
% plot controls
figure()
stairs(linspace(0, T, N), [u; nan])
hold on
plot(t, U, '-.')
legend('Numerical Sol', 'Optimal Sol');
title('Control values')
xlabel('time (s)');
ylabel('u');


% plot states
figure()
plot(linspace(0, T, N), x(:,1))
hold on
plot(linspace(0, T, N), x(:,2))
plot(t, X, '-.')
legend('Analytic Sol x1','Analytic Sol x2','Numerical Sol x1', 'Numerical Sol x2')
title('States')
xlabel('time (s)');
ylabel('states')

