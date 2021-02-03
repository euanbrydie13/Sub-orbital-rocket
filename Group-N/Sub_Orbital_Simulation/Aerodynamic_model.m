function [L, D] = Aerodynamic_model(x,vehicle,dens)

%Determines Lift and drag forces
CL=0;                         % Coefficent of Lift (Since sounding rocket CL can be assumed 0)
CD=0;                         % Coefficent of Drag (Constant) 
Sref = vehicle.sref;          % Aerodynamic Reference Area [m^2]
qdyn = 0.5.*dens.*x(2).^2;    % Dynamic pressure
L = CL.*Sref.*qdyn;           % Force of Lift [N]
D = CD.*Sref.*qdyn;           % Force of Drag [N]

end 