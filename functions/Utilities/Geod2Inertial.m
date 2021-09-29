function sBII = Geod2Inertial(latd,lond,h,t)
% CADINE84 equivalent
% Determines sBII given geodetic latitude and longitude, and altitude.

a       = 6378137.0;
f       = 3.33528106e-3;
R0      = a.*(1-(f/2).*(1-cos(2.*latd)) + (5.*f.^2./16).*(1-cos(4.*latd)));
we      = 7.29211585e-5;
lG0     = 0;

delta   = GetDelta(h,latd);
li      = lG0 + we*t + lond; % celestial longitude

dbi     = R0 + h;
sBID    = [-dbi*sin(delta);0;-dbi*cos(delta)];
slat    = sin(latd);
clat    = cos(latd);
slon    = sin(li);
clon    = cos(li);
sbid1   = sBID(1);
sbid2   = sBID(2);
sbid3   = sBID(3);
sbii1   = -slat*clon*sbid1 - clat*clon*sbid3;
sbii2   = -slat*slon*sbid1 - clat*slon*sbid3;
sbii3   = clat*sbid1 - slat*sbid3;
sBII    = [sbii1; sbii2; sbii3];

end