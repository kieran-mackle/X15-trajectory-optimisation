WARNING: Has not been modified yet.

function TM = TM_DI(lambdad,li)
%T_DI 

TM = [-sin(lambdad)*cos(li),    -sin(lambdad)*sin(li),  cos(lambdad)  ;
      -sin(li),                 cos(li),                0             ;
      -cos(lambdad)*cos(li),    -cos(lambdad)*sin(li),  -sin(lambdad)];

end