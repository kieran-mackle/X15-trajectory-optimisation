% ======================================================================= %
%                           X-15 MPC Framework                            %
% ======================================================================= %
% This script is a runfile for simulating the control of the X15 using
% MPC. The vehicle is simulated in a 6DOF model, formulated in a polar
% coordinate system. The MPC updates using also a 6DOF model, but expressed
% in a Cartesian coordinate system. Only the control inputs are maintained
% and passed between each environment. 

% Initialisation
run('./../inputs/load_paths.m')
addpath '/home/kieran/Documents/MATLAB/MPC'
clearvars; deg = pi/180; rad = 180/pi;

% Define coordinate systems
control_system = 'Cartesian';
simulation_system = 'Polar';

show_plots = 0;

% ----------------------------------------------------------------------- %
% Define MPC Environment
% ----------------------------------------------------------------------- %
% This is where all functions and variable relating to the control model
% are defined.

params.timestep     = 0.025;
params.horizon      = 100;
params.sim_time     = 20;
convex_solver       = 'gurobi'; % 'quadprog' / 'gurobi'

% Get initial trim state from GPOPS solutions
if strcmpi(simulation_system, 'Polar')
    % Load Polar initial state
    load('./../Results/Config1/6DOF/20km_hold_polar/20km_hold_polar.mat')
else
    % Load Cartesian initial state
    load('./../Results/Config1/6DOF/20km_hold/20km_hold.mat')
end

initial.state = output.result.solution.phase.state(1,:);
initial.control = output.result.solution.phase.control(1,:);

% initial.state = mpc_hold_output.state(:,end)';
% initial.control = mpc_hold_output.control(:,end)';

% Add cartesian models to auxdata
auxdata.mass_model = @(F)0;
auxdata.aerodynamics_model = @GetAero;
auxdata.gravity_model = @(h)[0,0,9.81];
auxdata.atmospheric_model = @(h)GetAtmo(h);

% Define control model - as seen by MPC 
control_model.auxdata = auxdata;
control_model.dynamics = @cart6_euler;
control_model.output = @cart6_output;
reference_function = @cart6_reference;

if strcmpi(simulation_system, control_system)
    % one-to-one mapping function
    control_model.mapping_func = @(in)in;
elseif strcmpi(control_system,'Cartesian') && strcmpi(simulation_system,'Polar')
    % Polar to Cartesian mapping function
    control_model.mapping_func = @tensor6plant_to_euler6control;
end

% Define cost and constraint matrices
cost_weightings.output = 1e3* [10, 0, 0, 0, 0;   % Altitude
                               0, 0, 0, 0, 0;   % FDA
                               0, 0, 0, 0, 0;   % THR
                               0, 0, 0, 0.5, 0;   % Ma
                               0, 0, 0, 0, 0.5];  % FPA
cost_weightings.control = eye(length(initial.control));

% Quadprog constraint handling options
constraints.type = 'soft';      % None, soft, hard or mixed
penalty_method = 'linear';      % Quadratic or linear
penalty_weight = 1e5;

constraints.hard.rate    = [-10*deg, 10*deg;
                            -0.2,   0.2];
%                             0, 0]; % dummy input
constraints.hard.input   = [-10*deg, 10*deg;
                            -0.2,   0.2];
%                             0, 0]; % dummy input
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
%                                    0, 0];
constraints.weights.hard_input  = 1e8 * [10, 10;
                                         1, 1];
%                                          0, 0];
constraints.weights.hard_output = 1e9 *  [0, 0;
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
mpc_input.show_plots        = show_plots;

% TODO - add function to check dimensions of constraints (especiall control
% weights etc.) match the initial guess dimensions

% ----------------------------------------------------------------------- %
% Define Simulation Environment
% ----------------------------------------------------------------------- %
% This is where all functions and variables related to the simulation model
% are defined. 
% Define plant model - as vehicle responds in the environment
plant_model = control_model;

auxdata = configure_inputs(auxdata);
plant_model.auxdata         = auxdata;
plant_model.dynamics        = @tensor6;
plant_model.output          = @X15_outputs;

dyn_input.phase.state       = initial.state;
sim_input.dynamics_input    = dyn_input;
sim_input.plant_model       = plant_model;
sim_input.Ts                = params.timestep;
sim_input.N                 = 1000; % Forward simulation steps

% ----------------------------------------------------------------------- %
% Run Simulation
% ----------------------------------------------------------------------- %
input.show_plots = show_plots;
input.mpc_input = mpc_input;
input.sim_input = sim_input;
input.reference_function = reference_function;

mpc_output = mpc_control(input);

