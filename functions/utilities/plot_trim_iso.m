% NOTE: gravity modelling is out of date. Slight discrepancies will be
% present between exact minimum shown on isosurfaces and actual trim point.

% =========================== IsoPlotter ========================== %
% Visualises the zero-value surfaces of lift, drag and moment error %
% functions to determine trim point of vehicle.                     %
% ================================================================= %

clear all;

h           = 20e3;
Mach        = 6;
N           = 100;

aoamin      = -30;
aoamax      = 30;
d_aoa       = (aoamax-aoamin)/N;
aoa         = aoamin:d_aoa:aoamax;

fdamin      = -50;
fdamax      = 50;
d_fda       = (fdamax-fdamin)/N;
fda         = fdamin:d_fda:fdamax;

T_min       = 0;
T_max       = 250e3;
d_T         = (T_max-T_min)/N;
T           = T_min:d_T:T_max;

[AOA,FDA,THR] = meshgrid(aoa,fda,T);

%-------------------------------------------------------------------%
%                  Load vehicle configuration script                %
%-------------------------------------------------------------------%
auxdata     = get_config('deck1.csv');

Ma          = Mach*ones(size(AOA));
S           = auxdata.S;
c           = auxdata.c;
m           = 10e3;
deg         = auxdata.deg;
Re          = auxdata.Re0;
R           = auxdata.R;
gamma       = auxdata.gamma;
mue         = auxdata.mue;
r           = Re + h;

[CL,CD,Cm]  = GetAero(auxdata,AOA,Ma,FDA);

[Temp, ~, rho]  = GetAtmo(h);
a               = sqrt(gamma.*R.*Temp);

q           = 0.5.*rho.*Ma.^2.*a.^2;
L           = CL.*q.*S;
D           = CD.*q.*S;
M           = Cm.*q.*S.*c;
W           = m.*(mue./r.^2);

d1          = L - W + THR.*sin(AOA.*deg);
d2          = D - THR.*cos(AOA.*deg);
d3          = M;

alt         = join(['h = ',num2str(h/1e3),' km']);
mach        = join(['Ma = ',num2str(Mach)]);
name        = join(['Isosurfaces for ',alt,', ',mach]);

figure(1); clf;
title(name);
hold on; view(3);
p = patch(isosurface(AOA,FDA,THR,d1,0));
p.FaceColor = 'red';
p.EdgeColor = 'none';

q = patch(isosurface(AOA,FDA,THR,d2,0));
q.FaceColor = 'blue';
q.EdgeColor = 'none';

r = patch(isosurface(AOA,FDA,THR,d3,0));
r.FaceColor = 'yellow';
r.EdgeColor = 'none';

grid on;
xlabel('Angle of Attack (deg)');
ylabel('Flap Deflection Angle (deg)');
zlabel('Thrust force (N)');

