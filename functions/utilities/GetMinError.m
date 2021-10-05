function [a_T, fda_T, thr_T, e, T_new] = GetMinError(auxdata,h,Ma,m,bounds,lat,fpa,hda)
hda         = hda*pi/180;
N           = 100;

aoamin      = bounds(1,1);
aoamax      = bounds(1,2);
d_aoa       = (aoamax-aoamin)/N;
aoa         = aoamin:d_aoa:aoamax;

fdamin      = bounds(2,1);
fdamax      = bounds(2,2);
d_fda       = (fdamax-fdamin)/N;
fda         = fdamin:d_fda:fdamax;
[AOA,FDA]   = meshgrid(aoa,fda);

T_min       = bounds(3,1);
T_max       = bounds(3,2);
d_T         = (T_max-T_min)/N;
T           = T_min:d_T:T_max;

THR         = T'.*ones(size(AOA));

Ma          = Ma*ones(size(AOA));
S           = auxdata.S;
c           = auxdata.c;
deg         = auxdata.deg;
Re          = geocradius(lat*180/pi);
R           = auxdata.R;
gamma       = auxdata.gamma;
r           = Re + h;

[CL,CD,Cm]  = GetAero(auxdata,AOA,Ma,FDA);

[Temp, ~, rho]  = GetAtmo(h);
a               = sqrt(gamma.*R.*Temp);

q           = 0.5.*rho.*Ma.^2.*a.^2;
L           = CL.*q.*S;
D           = CD.*q.*S;
M           = Cm.*q.*S.*c;

%-------------------------------------------------------------------%
%      Prepare coordinate transformation for gravity components     %
%-------------------------------------------------------------------%
EulAng      = [0,0,0].*ones(length(aoa),3);
EulAng(:,2) = (EulAng(:,2) + fpa + aoa')*pi/180;
EulAng(:,3) = EulAng(:,3) + pi/2 - hda;
DCM         = NED2Body(EulAng);
[gD,gN]     = JGetGravity(auxdata,r,lat);
g           = zeros(length(DCM),3);
for i = 1:length(DCM)
    g(i,:)  = (DCM(:,:,i)*[gN,0,gD]');
end
W           = m.*g;
[Wx,Wz]     = meshgrid(W(:,1),W(:,3));

%-------------------------------------------------------------------%
%            Calculate force components (body coordinates)          %
%-------------------------------------------------------------------%
Fx          = THR - D.*cos(AOA.*deg) + L.*sin(AOA*deg) + Wx;
Fz          =     - D.*sin(AOA.*deg) - L.*cos(AOA*deg) + Wz';

e1          = sqrt(Fx.^2);
e2          = sqrt(Fz.^2);
e3          = sqrt(M.^2);
eT          = e1 + e2 + e3;

k           = find(eT==min(eT,[],'all'),1);

T_D         = CD(k)*q(k)*S/cos(AOA(k)*pi/180);

a_T         = AOA(k);
fda_T       = FDA(k);
thr_T       = THR(k);
e           = eT(k);
T_new       = T_D;

end