function [CL, CD, Cm] = GetAero(auxdata,aoa,Ma,fda)
%GetAero    Returns the aerodynamic coefficients.
%   [CL, CD, Cm] = GetAero(auxdata,aoa,Ma,fda) returns the lift, drag and
%   moment coeffient at the specified angle of attack, Mach number and flap
%   deflection angle.
%   
%   Inputs:
%       auxdata:    Auxiliary data struct that contains aerodynamic data
%                       and normalisation values.
%       aoa:        Angle of attack (degrees)
%       Ma:         Mach number (-)
%       fda:        Flap deflection angle (degrees)

nv  = auxdata.aero.normvals;

Xq  = auxdata.aero.Xq;
Yq  = auxdata.aero.Yq;
Zq  = auxdata.aero.Zq;
CLs = auxdata.aero.CLs;
CDs = auxdata.aero.CDs;
Cms = auxdata.aero.Cms;

% normalise inputs
aoa = (aoa-nv(1,1))./(nv(1,2)-nv(1,1));
Ma  = (Ma -nv(2,1))./(nv(2,2)-nv(2,1));
fda = (fda-nv(3,1))./(nv(3,2)-nv(3,1));

CL  = interp3(Xq,Yq,Zq,CLs,aoa,Ma,fda,'spline');
CD  = interp3(Xq,Yq,Zq,CDs,aoa,Ma,fda,'spline');
Cm  = interp3(Xq,Yq,Zq,Cms,aoa,Ma,fda,'spline');

% De-normalise coefficients
CL  = CL.*(nv(4,2)-nv(4,1)) + nv(4,1);
CD  = CD.*(nv(5,2)-nv(5,1)) + nv(5,1);
Cm  = Cm.*(nv(6,2)-nv(6,1)) + nv(6,1);

end