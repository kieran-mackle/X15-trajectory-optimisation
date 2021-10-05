% ======================= Wing Optimisation ======================= %
%  Pseudospectral optimisation problem for X-15 Delta wing.         %
% ================================================================= %

clearvars; clc;

% ----------------------------------------------------------------- %
%                  Initialise environment with paths                %
%------------------------------------------------------------------ %
run('./inputs/load_paths.m')

% ----------------------------------------------------------------- %
%                  Load vehicle configuration script                %
%------------------------------------------------------------------ %
auxdata = get_config('deck1.csv'); % Filename of aerodeck in ./aero/
auxdata.hd  = pwd;

% ----------------------------------------------------------------- %
%                       Load manoeuvre script                       %
%------------------------------------------------------------------ %
manoeuvre_spec.type = 'hold';              % 'climb' / 'hold'
manoeuvre_spec.name = '15to25kmClimb';
manoeuvre_spec.h0 = 20e3;
manoeuvre_spec.hf = 20e3;
manoeuvre_spec.Ma0 = 6;
manoeuvre_spec.Maf = 6;
manoeuvre_spec.use_guess = 0;

[bounds, guess, auxdata] = manoeuvre(manoeuvre_spec, auxdata);

% ----------------------------------------------------------------- %
%                     Assign function handles                       %
%------------------------------------------------------------------ %
dynamics_func               = @tensor6;
% objective_func              = @MinTime6;
objective_func              = @AltHold6;

% ----------------------------------------------------------------- %
%               Provide mesh refinement and method                  %
%------------------------------------------------------------------ %
mesh.method                 = 'hp-PattersonRao';
mesh.tolerance              = 1e-1;
mesh.maxiterations          = 3;

% ----------------------------------------------------------------- %
%                    Construct GPOPS-II input                       %
%------------------------------------------------------------------ %
setup.name                  = 'X-15_Optimisation';
setup.functions.continuous  = dynamics_func;
setup.functions.endpoint    = objective_func;
setup.nlp.solver            = 'ipopt';
setup.bounds                = bounds;
setup.guess                 = guess;
setup.auxdata               = auxdata;
setup.mesh                  = mesh;
setup.method                = 'RPM-Differentiation';
setup.displaylevel          = 2;
setup.scales.method         = 'automatic-bounds';
setup.derivatives.supplier  = 'sparseCD';
setup.derivatives.dependencies = 'sparse';
setup.derivatives.derivativelevel = 'first';
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance = 1e-3;
setup.nlp.ipoptoptions.maxiterations = 2000;
clear objective_func dynamics_func M;

% ----------------------------------------------------------------- %
%                       Call GPOPS-II solver                        %
%------------------------------------------------------------------ %
output = gpops2(setup);

% ----------------------------------------------------------------- %
%                     Post-Processing Routine                       %
%------------------------------------------------------------------ %
post6;
plot6;

