% ================ Minimum time objective function ================ %
% Objective function script for minimising time of mission whilst   %
% trimming the vehicle at the bounds.
% ================================================================= %

function output = MinTimeAndTrim(input)

tf      = input.phase.finaltime;
x0      = input.phase.initialstate;
xf      = input.phase.finalstate;
ad      = input.auxdata;

% Extract state variables
r0      = x0(:,1);                  rf      = xf(:,1);
v0      = x0(:,2);                  vf      = xf(:,2);
aoa0    = x0(:,3);                  aoaf    = xf(:,3);
fpa0    = x0(:,4);                  fpaf    = xf(:,4);
hda0    = x0(:,5);                  hdaf    = xf(:,5);
lat0    = x0(:,6);                  latf    = xf(:,6);
m0      = x0(:,9);                  mf      = xf(:,9);
fda0    = x0(:,10);                 fdaf    = xf(:,10);
thr0    = x0(:,11);                 thrf    = xf(:,11);

% Extract constants
S       = ad.S; 
c       = ad.c;
R       = ad.R;
gamma   = ad.gamma;
rad     = ad.rad;
Re0     = geocradius(lat0*rad);
Ref     = geocradius(latf*rad);

% Calculate intermediate variables
[Temp0, ~, rho0] = GetAtmo(r0 - Re0);
sos0    = sqrt(gamma.*R.*Temp0);
Ma0     = v0./sos0;

[Tempf, ~, rhof] = GetAtmo(rf - Ref);
sosf    = sqrt(gamma.*R.*Tempf);
Maf     = vf./sosf;

[CL0,CD0,Cm0] = GetAero(ad,aoa0*rad,Ma0,fda0*rad);
[CLf,CDf,Cmf] = GetAero(ad,aoaf*rad,Maf,fdaf*rad);

q0      = 0.5.*rho0.*v0.^2;         qf      = 0.5.*rhof.*vf.^2;
L0      = CL0.*q0.*S;               Lf      = CLf.*qf.*S;
D0      = CD0.*q0.*S;               Df      = CDf.*qf.*S;
T0      = thr0*ad.thrust;           Tf      = thrf*ad.thrust;
M0      = Cm0.*q0.*S*c;             Mf      = Cmf.*qf.*S*c;

[gD0,gN0]   = GetGravity(ad,r0,lat0);[gDf,gNf]   = GetGravity(ad,rf,latf);

DCM0        = NED2Body([0,fpa0+aoa0,pi/2-hda0]);
DCMf        = NED2Body([0,fpaf+aoaf,pi/2-hdaf]);
g0          = (DCM0*[gN0,0,gD0]')'; gf          = (DCMf*[gNf,0,gDf]')';
W0          = m0.*g0;               Wf          = mf.*gf;
Wx0         = W0(:,1);              Wxf         = Wf(:,1);
Wz0         = W0(:,3);              Wzf         = Wf(:,3);

% Force Components (body axes)
Fx0         = T0 - D0.*cos(aoa0) + L0.*sin(aoa0) + Wx0;
Fz0         =    - D0.*sin(aoa0) - L0.*cos(aoa0) + Wz0;

Fxf         = Tf - Df.*cos(aoaf) + Lf.*sin(aoaf) + Wxf;
Fzf         =    - Df.*sin(aoaf) - Lf.*cos(aoaf) + Wzf;

Fxs         = Fx0^2 + Fxf^2;
Fzs         = Fz0^2 + Fzf^2;
Ms          = M0^2 + Mf^2;

J           = Fxs + Fzs + Ms + tf;

% output.eventgroup.event = [Ma0,Maf];
output.eventgroup.event = [r0-Re0, rf-Ref,Ma0,Maf];
output.objective = J;
