function output = cartesian_objective(input)
% =============== Altitude hold objective function ================ 
% Generalised objective function for X-15 trajectory optimisation.
% ================================================================= 

tf      = input.phase.finaltime;
X0      = input.phase.initialstate;
Xf      = input.phase.finalstate;
auxdata = input.auxdata;

N0      = X0(:,1);                  Nf      = Xf(:,1);
E0      = X0(:,2);                  Ef      = Xf(:,2);
D0      = X0(:,3);                  Df      = Xf(:,3);
vN0     = X0(:,4);                  vNf     = Xf(:,4);
vE0     = X0(:,5);                  vEf     = Xf(:,5);
vD0     = X0(:,6);                  vDf     = Xf(:,6);

% ----------------------------------------------------------------------- %
Re0     = GetRe(lat0, ad.radius_model);
Ref     = GetRe(latf, ad.radius_model);
h0      = -(dist0 + Re0);
hf      = -(distf + Ref);

% ----------------------------------------------------------------------- %
delta0  = GetDelta(h0,lat0);       deltaf  = GetDelta(hf,latf);
T_DG0   = TM_DG(delta0);           T_DGf   = TM_DG(deltaf);

vBED0   = [uD0,vD0,wD0];
vBEDf   = [uDf,vDf,wDf];

vBEG0   = T_DG0' * vBED0';
vBEGf   = T_DGf' * vBEDf';

[V0,~,fpa0] = car2pol(vBEG0);
[Vf,~,fpaf] = car2pol(vBEGf);

[Temp,~,~] = GetAtmo(h0);
sos0        = sqrt(ad.gamma*ad.R*Temp);
Ma0 = V0/sos0;

[Temp,~,~] = GetAtmo(hf);
sosf = sqrt(ad.gamma*ad.R*Temp);
Maf = Vf/sosf;

% ----------------------------------------------------------------------- %
output.eventgroup(1).event = [Ma0,Maf];
output.eventgroup(2).event = [fpa0,fpaf];

J = tf;
    
output.objective = J;
