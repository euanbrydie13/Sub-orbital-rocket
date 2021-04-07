function [L] = objfun( y, auxdata )
N       = auxdata.N;
Nstates = auxdata.Nstates;

u = y(Nstates*N + 1: end);
L = sum(u.^2); % this form could be changed to exact obj function of problem

end

