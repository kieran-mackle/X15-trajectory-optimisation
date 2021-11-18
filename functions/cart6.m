function output = cart6(input)
% =============== Cartesian 6 DoF Flight Dynamics ================== 
%  Based on Chapter 10.1.1 of "Modeling and Simulation of Aerospace 
%  Vehicle Dynamics" by Peter H. Zipfel.
% ==================================================================






i=1;
d_sBE_L(i,:) = T_BL' * vBE_B(i,:);


output.dynamics = [d_sBE_L];
