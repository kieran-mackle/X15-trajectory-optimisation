function [f_apB, mBB, varargout] = GetFM(input,model)

if model == 1       % no thrust offset or thrust vectoring
    zeta = 0;
    eta  = 0;
elseif model == 2   % no thrust offset with thrust vectoring
    zeta    = input.zet;
    eta     = input.et;
else 
    % this field will be used to declare extra parameters relating to
    % thrust offsets and whatever else will be modelled.
end

aoa     = input.aoa;
Ma      = input.Mach;
fda     = input.fd;
F_T     = input.FT;
qbar    = input.qb;
ad      = input.ad;
S       = ad.S;
b       = ad.b;
c       = ad.c;
rad     = 180/pi;

% Aerodynamics
[CL,CD,Cm]  = GetAero(ad,aoa*rad,Ma,fda*rad);
Cl          = 0;
Cn          = 0;
Cx          =  CL.*sin(aoa) - CD.*cos(aoa);
Cz          = -CL.*cos(aoa) - CD.*sin(aoa);
f_aB        = qbar*S*[Cx; 0; Cz];
maB         = qbar*S*[Cl*b;Cm*c;Cn*b];
% Propulsion
f_pB        = [cos(eta)*cos(zeta);
               cos(eta)*sin(eta);
               -sin(eta)]*F_T;
mpB         = [0;-sin(eta);-cos(eta)*sin(zeta)]* ...
               F_T*7.5;%(xp - xcm);
           
% Force and moment - note that these are absoute, not specific.
f_apB       = f_aB + f_pB;
mBB         = maB + mpB;

if nargout > 2
    varargout{1} = f_aB;
    varargout{2} = f_pB;
    varargout{3} = maB;
    varargout{4} = mpB;
    
    if nargout > 4
        varargout{5} = qbar*S*CL;
        varargout{6} = qbar*S*CD;
        varargout{7} = qbar*S*Cm;
    end
    
end

end