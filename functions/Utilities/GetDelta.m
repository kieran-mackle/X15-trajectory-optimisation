function delta = GetDelta(h,latd)
%GETDELTA Calculates geodetic deflection angle
% see page 398 Model and Simulation, Zipfel

a       = 6378137.0;
f       = 3.33528106e-3; % WGS84
R0      = a*(1 - (f/2)*(1-cos(2*latd)) + (5*f^2/16)*(1-cos(4*latd)));

delta   = f*sin(2*latd).*(1 - f/2 - h./R0);

end

