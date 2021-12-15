% ======================================================================= %
%                           X-15 MPC Framework                            %
% ======================================================================= %
% This script is a runfile for simulating the control of the X15 using
% MPC. The vehicle is simulated in a 6DOF model, formulated in a polar
% coordinate system. The MPC updates using also a 6DOF model, but expressed
% in a Cartesian coordinate system. Only the control inputs are maintained
% and passed between each environment. 

% NOTES
% - seg violation when altitude weight is too high??


% Add MPC controller directory
addpath '/home/kieran/Documents/MATLAB/MPC'

clearvars; deg = pi/180; rad = 180/pi;


% ----------------------------------------------------------------------- %
% Define MPC Environment
% ----------------------------------------------------------------------- %
params.timestep     = 0.05;
params.horizon      = 75;
params.sim_time     = 50;
convex_solver       = 'gurobi';       % 'quadprog' / 'gurobi'


run('./../inputs/load_paths.m')
% Use GPOPS altitude hold solution to get initial trim state
load('./../Results/Config1/6DOF/20km_hold_polar/20km_hold_polar.mat')

out = output.result.solution.phase;
x0  = out.state(1,:);
u0  = out.control(1,:);

initial.state = x0;
initial.control = u0;

% Add cartesian models to auxdata
auxdata.mass_model = @(F)0;
auxdata.aerodynamics_model = @GetAero;
auxdata.gravity_model = @(h)[0,0,9.81];
auxdata.atmospheric_model = @(h)GetAtmo(h);

% Define control model - as seen by MPC 
control_model.auxdata       = auxdata;
control_model.dynamics      = @cart6_euler;
control_model.output        = @cart6_output;
reference_function          = @cart6_reference;

% Define cost and constraint matrices
cost_weightings.output         = 1e3* [1, 0, 0, 0, 0;   % Altitude
                                       0, 0, 0, 0, 0;   % FDA
                                       0, 0, 0, 0, 0;   % THR
                                       0, 0, 0, 1, 0;   % Ma
                                       0, 0, 0, 0, 1];  % FPA
cost_weightings.control        = eye(length(initial.control));


% Quadprog constraint handling options
constraints.type = 'soft';      % None, soft, hard or mixed
penalty_method = 'linear';      % Quadratic or linear
penalty_weight      = 1e3;

constraints.hard.rate    = [-10*deg, 10*deg;
                            -0.2,   0.2];
constraints.hard.input   = [-10*deg, 10*deg;
                            -0.2,   0.2];
constraints.hard.output  = [   0,      0     ;
                            -40*deg, 40*deg  ;
                               0,      1     ;
                               0,      0     ;
                               0,      0     ];

% Set constraint weights
% ------------------------
% Use nan for hard constraints
constraints.weights.hard_rate   = [0, 0;
                                   0, 0];
constraints.weights.hard_input  = 1e8 * [10, 10;
                                         1, 1];
constraints.weights.hard_output = 1e8 *  [0, 0;
                                          10, 10;
                                          1, 1;
                                          0, 0;
                                          0, 0];

mpc_input.control_model     = control_model;
mpc_input.cost              = cost_weightings;
mpc_input.constraints       = constraints;
mpc_input.penalty_method    = penalty_method;
mpc_input.penalty_weight    = penalty_weight;
mpc_input.params            = params;
mpc_input.initial           = initial;
mpc_input.solver            = convex_solver;


% ----------------------------------------------------------------------- %
% Define Simulation Environment
% ----------------------------------------------------------------------- %
% Define plant model - as vehicle responds in the environment
auxdata = configure_inputs(auxdata);
plant_model.auxdata         = auxdata;
plant_model.dynamics        = @tensor6;
plant_model.output          = @X15_outputs;

dyn_input.phase.state       = initial.state;
sim_input.dynamics_input    = dyn_input;
sim_input.plant_model       = plant_model;
sim_input.Ts                = params.timestep;



% ----------------------------------------------------------------------- %
% Run Simulation
% ----------------------------------------------------------------------- %
input.mpc_input = mpc_input;
input.sim_input = sim_input;
input.reference_function = reference_function;

output = mpc_control(input);