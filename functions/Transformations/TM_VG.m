function TM = TM_VG(gamma,chi)
%T_GE Flight-path to geographic coordinate transformation matrix.

% Will need to modify this to ensure it calculates matrices with
% appropriate dimensions (3d probably for each time step in 3rd dimension)

TM = [cos(gamma)*cos(chi),  cos(gamma)*sin(chi),    -sin(gamma);
      -sin(chi),            cos(chi),               0          ;
      sin(gamma)*cos(chi),  sin(gamma)*sin(chi),    cos(gamma)];

end