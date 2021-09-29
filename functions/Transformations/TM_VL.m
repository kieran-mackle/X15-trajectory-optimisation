function TM = TM_VL(fpa,hda)
%T_GE Flight-path to geographic coordinate transformation matrix.

% Will need to modify this to ensure it calculates matrices with
% appropriate dimensions (3d probably for each time step in 3rd dimension)

TM = [cos(fpa)*cos(hda),  cos(fpa)*sin(hda),    -sin(fpa);
         -sin(hda),          cos(hda),              0    ;
      sin(fpa)*cos(hda),  sin(fpa)*sin(hda),    cos(fpa)];

end