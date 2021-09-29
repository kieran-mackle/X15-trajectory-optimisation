function TM = TM_BV(aoa,aob)

TM = [cos(aoa),  sin(aoa)*sin(aob),    -sin(aoa)*cos(aob);
         0,        cos(aob),              sin(aob)       ;
      sin(aoa),  -cos(aoa)*sin(aob),   cos(aoa)*cos(aob)];

end