function output = cart6_objective(input)
% =============== Altitude hold objective function ================ 
% Generalised objective function for X-15 trajectory optimisation.
% ================================================================= 

tf      = input.phase.finaltime;
X0      = input.phase.initialstate;
Xf      = input.phase.finalstate;
auxdata = input.auxdata;
% q = input.phase.integral(:,1);

sBE_L   = [X0(1:3); Xf(1:3)];
vBE_B   = [X0(4:6); Xf(4:6)];

% Re      = auxdata.Re0;
% N0      = X0(:,1);                  Nf      = Xf(:,1);
% E0      = X0(:,2);                  Ef      = Xf(:,2);
% D0      = X0(:,3);                  Df      = Xf(:,3);
% vN0     = X0(:,4);                  vNf     = Xf(:,4);
% vE0     = X0(:,5);                  vEf     = Xf(:,5);
% vD0     = X0(:,6);                  vDf     = Xf(:,6);
% vBE_B0      = X0(:,4:6);            vBE_Bf  = Xf(:,4:6);
% vBE_B = [vBE_B0; vBE_Bf];

% ----------------------------------------------------------------------- %
% h           = [-D0; -Df]-Re;
% [T,~,~]     = auxdata.atmospheric_model(h);
% a           = sqrt(auxdata.gamma.*auxdata.R.*T);
% V           = sqrt(sum(vBE_B.^2,2));
% Ma          = V'./a;

h = -sBE_L(:,3) - auxdata.Re0;
[T,~,rho] = auxdata.atmospheric_model(h);
a = sqrt(auxdata.gamma.*auxdata.R.*T);
V = sqrt(sum(vBE_B.^2,2));
Ma = V./a;

% ----------------------------------------------------------------------- %
% output.eventgroup(1).event = [Ma(1), Ma(2)];
% output.eventgroup(2).event = [fpa0, fpaf];


if auxdata.altitude_hold == 1
    J = (15-tf)^2; % integral of fpa ^ 2
else
    J = tf;
end

output.objective = J;
