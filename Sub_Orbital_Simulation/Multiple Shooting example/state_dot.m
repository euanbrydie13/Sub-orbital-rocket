function x_dot = state_dot(state, u)
% This define the ODE equations of system
x_dot = [state(2), u];
end

