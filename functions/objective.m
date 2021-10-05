function output = objective(input)
% =============== Altitude hold objective function ================ 
% Generalised objective function for X-15 trajectory optimisation.
% ================================================================= 

tf      = input.phase.finaltime;
X0      = input.phase.initialstate;
Xf      = input.phase.finalstate;
q4      = input.phase.integral(:,1);
ad      = input.auxdata;

lat0    = X0(:,1);                  latf    = Xf(:,1);
dist0   = X0(:,3);                  distf   = Xf(:,3);
uD0     = X0(:,4);                  uDf     = Xf(:,4);
vD0     = X0(:,5);                  vDf     = Xf(:,5);
wD0     = X0(:,6);                  wDf     = Xf(:,6);

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
output.eventgroup(3).event = [h0,hf];

if ad.altitude_hold == 1
    J = q4;
else
    J = tf;
end
    
output.objective = J;
