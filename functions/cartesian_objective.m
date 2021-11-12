function output = cartesian_objective(input)
% =============== Altitude hold objective function ================ 
% Generalised objective function for X-15 trajectory optimisation.
% ================================================================= 

tf      = input.phase.finaltime;
X0      = input.phase.initialstate;
Xf      = input.phase.finalstate;
auxdata = input.auxdata;

Re      = auxdata.Re0;

% N0      = X0(:,1);                  Nf      = Xf(:,1);
% E0      = X0(:,2);                  Ef      = Xf(:,2);
D0      = X0(:,3);                  Df      = Xf(:,3);
% vN0     = X0(:,4);                  vNf     = Xf(:,4);
% vE0     = X0(:,5);                  vEf     = Xf(:,5);
% vD0     = X0(:,6);                  vDf     = Xf(:,6);

% ----------------------------------------------------------------------- %
vBE_L0      = X0(:,4:6);            vBE_Lf  = Xf(:,4:6);
[V0,~,fpa0] = car2pol(vBE_L0);      [Vf,~,fpaf] = car2pol(vBE_Lf);

h           = [-D0; -Df]-Re;
[T,~,~]     = auxdata.atmospheric_model(h);
a           = sqrt(auxdata.gamma.*auxdata.R.*T);
V           = [V0, Vf];
Ma          = V'./a;

% ----------------------------------------------------------------------- %
% output.eventgroup(1).event = [Ma(1), Ma(2)];
% output.eventgroup(2).event = [fpa0, fpaf];


if strcmpi(specification.type, 'hold')
    J = -tf;
end

output.objective = J;
