function TM = TM_EI(sim_time)

w_E     = 7.29211585e-5;
ha      = w_E*sim_time;     % hour angle, rad

TM = [cos(ha),  sin(ha),    0;
      -sin(ha), cos(ha),    0;
      0,        0,          1];

end