function [CL, CD, Cm] = GetNormedAero(auxdata,aoa,Ma,fda)

nv  = auxdata.normvals;

Xq  = auxdata.aero2.Xq;
Yq  = auxdata.aero2.Yq;
Zq  = auxdata.aero2.Zq;
CLs = auxdata.aero2.CLs;
CDs = auxdata.aero2.CDs;
Cms = auxdata.aero2.Cms;

% normalise inputs
aoa = (aoa-nv(1,1))./(nv(1,2)-nv(1,1));
Ma  = (Ma-nv(2,1))./(nv(2,2)-nv(2,1));
fda = (fda-nv(3,1))./(nv(3,2)-nv(3,1));

CL  = interp3(Xq,Yq,Zq,CLs,aoa,Ma,fda,'spline');
CD  = interp3(Xq,Yq,Zq,CDs,aoa,Ma,fda,'spline');
Cm  = interp3(Xq,Yq,Zq,Cms,aoa,Ma,fda,'spline');

% De-normalise coefficients
CL  = CL.*(nv(4,2)-nv(4,1)) + nv(4,1);
CD  = CD.*(nv(5,2)-nv(5,1)) + nv(5,1);
Cm  = Cm.*(nv(6,2)-nv(6,1)) + nv(6,1);

end