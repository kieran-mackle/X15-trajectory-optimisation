
% Single MPC Update Iteration
% ===========================
% The following script is set up to run a single MPC update. The purpose 
% is to create a simple case to work towards integrating the 3DOF control
% model into the 6DOF simulation control framework.







% Call MPC Update Solver
U_k = mpc_update(mpc_input);
