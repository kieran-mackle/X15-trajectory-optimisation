function TM = TM_UI(theta,psi)
%T_GE Flight-path to geographic coordinate transformation matrix.

% Will need to modify this to ensure it calculates matrices with
% appropriate dimensions (3d probably for each time step in 3rd dimension)

TM = [cos(theta)*cos(psi),  cos(theta)*sin(psi),    -sin(theta);
      -sin(psi),            cos(psi),               0          ;
      sin(theta)*cos(psi),  sin(theta)*sin(psi),    cos(theta)];

end