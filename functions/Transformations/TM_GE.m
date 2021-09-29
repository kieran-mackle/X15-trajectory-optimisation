function TM = TM_GE(lambda,l)
%T_GE Geographic wrt Earth coordinate transformation matrix.

% Will need to modify this to ensure it calculates matrices with
% appropriate dimensions (3d probably for each time step in 3rd dimension)

TM = [-sin(lambda)*cos(l),  -sin(lambda)*sin(l),    cos(lambda)  ;
      -sin(l),              cos(l),                 0            ;
      -cos(lambda)*cos(l),  -cos(lambda)*sin(l),    -sin(lambda)];

end