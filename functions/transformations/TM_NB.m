function TM = TM_NB(zeta,eta)

TM = [cos(zeta)*cos(eta), sin(zeta)*cos(eta), -sin(eta);
      -sin(zeta), cos(zeta), 0;
      cos(zeta)*sin(eta),sin(zeta)*sin(eta),cos(eta)];

end