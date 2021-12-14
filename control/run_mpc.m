% ======================================================================= %
%                           X-15 MPC Framework                            %
% ======================================================================= %
% This script is a runfile for simulating the control of the X15 using
% MPC. The vehicle is simulated in a 6DOF model, formulated in a polar
% coordinate system. The MPC updates using also a 6DOF model, but expressed
% in a Cartesian coordinate system. Only the control inputs are maintained
% and passed between each environment. 



% ----------------------------------------------------------------------- %
% Define MPC Environment
% ----------------------------------------------------------------------- %
params.timestep     = 0.05;
params.horizon      = 75;
params.sim_time     = 5;
convex_solver       = 'gurobi';       % 'quadprog' / 'gurobi'


% ----------------------------------------------------------------------- %
% Define Simulation Environment
% ----------------------------------------------------------------------- %





