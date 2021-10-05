function [a_T, fda_T, thr_T] = GetTrim(auxdata, h, Ma, m, lat,fpa,hda)
%GetTrim    Return the input parameters to achieve trimmed flight.
%   [a_T, fda_T, thr_T] = GetTrim(auxdata, h, Ma, m, lat,fpa,hda) returns
%   the angle of attack, flap deflection angle (in degrees) and thrust 
%   magnitude required to achieve trimmed flight at the specified input
%   conditions.
%   
%   Current accepted input format:
%       auxdata:    Contains aerodynamic data
%       h:          Altitude (m)
%       Ma:         Mach number (-)
%       m:          Mass of vehicle (kg)
%       lat:        Longitude of vehicle (radians)
%       fpa:        Flight path angle of vehicle (degrees)
%       hda:        Heading direction angle (degrees)

N           = 100;

aoamin      = -30;
aoamax      = 30;

fdamin      = -50;
fdamax      = 50;

T_min       = 0.0;
T_max       = 250e3;

minVals     = [aoamin;fdamin;T_min];
maxVals     = [aoamax;fdamax;T_max];
bounds      = [minVals,maxVals];
boundcheck  = [-minVals,maxVals];

[a_T, fda_T, thr_T, e, T_new] = GetMinError(auxdata,h,Ma,m,bounds,lat,fpa,hda);

K           = 0.5;
tolerance   = 4e-8;
iter        = 0;
delta       = 1;
lim         = 300;

while e > tolerance
    
    if iter > lim -1 || delta == 0
        break
    end
    
    e_old       = e;
    
    minVals     = (1-K)*[a_T;fda_T;T_new];
    maxVals     = (1+K)*[a_T;fda_T;T_new];
    bounds      = [minVals,maxVals];
    
    [a_T,fda_T,thr_T,e,T_new] = GetMinError(auxdata,h,Ma,m,bounds,lat,fpa,hda);
    
    K           = GetK(e, K);
    
    delta       = abs(e-e_old);
    iter        = iter + 1;

end

% Check for convergence
if delta > 1e-1
    [a_T, fda_T, thr_T] = deal(nan, nan, nan);
end

% check solution is within bounds
check       = boundcheck + [a_T, -a_T; fda_T, -fda_T; thr_T, -thr_T];
val         = sum(lt(check,0),'all');

if val > 0
    [a_T, fda_T, thr_T] = deal(nan, nan, nan);
end


end