function TM = TM_MV(phi)
%T_GE Load factor wrt velocity coordinate transformation matrix.

% Will need to modify this to ensure it calculates matrices with
% appropriate dimensions (3d probably for each time step in 3rd dimension)

TM = [1,    0,          0;
      0,    cos(phi),   sin(phi);
      0,    -sin(phi),  cos(phi)];

end