
% Single MPC Update Iteration
% ===========================
% The following script is set up to run a single MPC update. The purpose 
% is to create a simple case to work towards integrating the 3DOF control
% model into the 6DOF simulation control framework.

% Add MPC controller directory
addpath '/home/kieran/Documents/MATLAB/MPC'

clearvars;
deg = pi/180;
rad = 180/pi;

% ----------------------------------------------------------------------- %
% Define MPC Parameters
% ----------------------------------------------------------------------- %
params.timestep     = 0.05;
params.horizon      = 75;
params.sim_time     = 5;
convex_solver       = 'gurobi';       % 'quadprog' / 'gurobi'

% ----------------------------------------------------------------------- %
% Define initial state
% ----------------------------------------------------------------------- %
% Use GPOPS solution to get initial trim state
run('./../inputs/load_paths.m')
load('./../Results/Config1/6DOF/20kmHold/20kmHold.mat')

out = output.result.solution.phase;
x0  = out.state(1,:);
u0  = out.control(1,:);



% MAP STATE FROM 6DOF TO 3DOF MODEL
lat = x0(1); lon = x0(2); dist = x0(3);
uD = x0(4); vD = x0(5); wD = x0(6);
roll = x0(13); pitch = x0(14); yaw = x0(15);



initial.state = x0;
initial.control = u0;

% Define control model - as seen by MPC 
control_model.auxdata       = auxdata;
control_model.dynamics      = @tensor6;
control_model.output        = @X15_outputs;
reference_function          = @X15_reference;

% ----------------------------------------------------------------------- %
% Define cost and constraint matrices
% ----------------------------------------------------------------------- %
cost_weightings.output         = 1e3* [1, 0, 0, 0, 0;   % Altitude
                                       0, 0, 0, 0, 0;   % FDA
                                       0, 0, 0, 0, 0;   % THR
                                       0, 0, 0, 1, 0;   % Ma
                                       0, 0, 0, 0, 1];  % FPA
cost_weightings.control        = eye(length(initial.control));

% Quadprog constraint handling options
constraints.type = 'soft';      % None, soft, hard or mixed
penalty_method = 'linear';      % Quadratic or linear
penalty_weight = 1e3;

constraints.hard.rate    = [-10*deg, 10*deg;
                            -0.2,   0.2];
constraints.hard.input   = [-10*deg, 10*deg;
                            -0.2,   0.2];
constraints.hard.output  = [   0,      0     ;
                            -40*deg, 40*deg  ;
                               0,      1     ;
                               0,      0     ;
                               0,      0     ];

constraints.soft.rate    = [-10*deg, 10*deg;
                            -0.2,   0.2];
constraints.soft.input   = [-10*deg, 10*deg;
                            -0.2,   0.2];
constraints.soft.output  = [   0,      0     ;
                            -40*deg, 40*deg  ;
                               0,      1     ;
                               0,      0     ;
                               0,      0     ];
                           
% NaN for hard constraints
constraints.weights.hard_rate   = [0, 0;
                                   0, 0];
constraints.weights.hard_input  = [0, 0;
                                   0, 0];
constraints.weights.hard_output = [0, 0;
                                   0, 0;
                                   0, 0;
                                   0, 0;
                                   0, 0];

% ----------------------------------------------------------------------- %
% Construct MPC Input
% ----------------------------------------------------------------------- %
mpc_input.control_model     = control_model;
mpc_input.cost              = cost_weightings;
mpc_input.constraints       = constraints;
mpc_input.penalty_method    = penalty_method;
mpc_input.penalty_weight    = penalty_weight;
mpc_input.params            = params;
mpc_input.initial           = initial;
mpc_input.solver            = convex_solver;
mpc_input.t                 = 0;                % From mpc_control.m
mpc_input.reference = [20e3, 0, 0, 6, 0]';      % From X15_reference.m

% Call MPC Update Solver
U_k = mpc_update(mpc_input);
