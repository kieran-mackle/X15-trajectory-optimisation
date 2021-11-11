function aoa = trim_aero(auxdata, Ma, fda)
% trim_aero returns the angle of attack to achieve pitch trim at a
% specified Mach number and flap deflection angle.
%
% fda must be inputted in degrees.

function absCm = GetAeroWrapper(aoa)
    [~,~,Cm] = GetAero(auxdata, aoa, Ma, fda);
    absCm = abs(Cm);
end

options = optimoptions('fminimax');
options.Display = 'none';

aoa = fminimax(@GetAeroWrapper, 0, [], [], [], [], -30, 30, [], options);

end