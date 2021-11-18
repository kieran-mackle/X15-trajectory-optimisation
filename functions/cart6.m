function output = cart6(input)
% =============== Cartesian 6 DoF Flight Dynamics ================== 
%  Based on Chapter 10.1.1 of "Modeling and Simulation of Aerospace 
%  Vehicle Dynamics" by Peter H. Zipfel.
% ==================================================================




d_vBE_B(i,:) = f_sp_B - R_BE_B * vBE_B + T_BL * g_L;
d_sBE_L(i,:) = T_BL' * vBE_B(i,:);
d_wBE_B(i,:) = invMOI * (-R_BE_B*MOI*wBE_B(i,:)' + mBB);

output.dynamics = [d_sBE_L, d_vBE_B];
