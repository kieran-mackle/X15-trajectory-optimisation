% ======================= Wing Optimisation ======================= 
%      Pseudospectral optimisation problem for X-15 Delta wing.         
% ================================================================= 

clearvars; clc;

% ----------------------------------------------------------------- %
%                  Initialise environment with paths                %
%------------------------------------------------------------------ %
run('./inputs/load_paths.m')

% ----------------------------------------------------------------- %
%                  Load vehicle configuration script                %
%------------------------------------------------------------------ %
auxdata = get_config('deck1.csv');  % Filename of aerodeck in ./aero/
auxdata.hd = pwd;
auxdata.coordinates = 'polar';  % 'polar' or 'cartesian'
auxdata.mass_model = @(F)0;
auxdata.aerodynamics_model = @GetAero;
auxdata.gravity_model = @(h)[0,0,9.81];
auxdata.atmospheric_model = @(h)GetAtmo(h);

% ----------------------------------------------------------------- %
%                       Load manoeuvre script                       %
%------------------------------------------------------------------ %
manoeuvre_spec.type = 'hold';              % 'climb' / 'hold'
manoeuvre_spec.name = '20km_hold';
manoeuvre_spec.h0 = 20e3;
manoeuvre_spec.hf = 20e3;
manoeuvre_spec.Ma0 = 6;
manoeuvre_spec.Maf = 6;
manoeuvre_spec.use_guess = 0;

    % POLAR
[bounds, guess, auxdata] = manoeuvre(manoeuvre_spec, auxdata);
auxdata = configure_inputs(auxdata);

    % CARTESIAN
% [bounds, guess, auxdata] = cart_manoeuvre(manoeuvre_spec, auxdata);
% plot_bounds_and_guess(bounds, guess, auxdata)

% ----------------------------------------------------------------- %
%                     Assign function handles                       %
%------------------------------------------------------------------ %
dynamics_func               = @tensor6;           % @tensor6 / @cart3
objective_func              = @objective; %@cartesian_objective;

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
setup.nlp.ipoptoptions.maxiterations = 100;
clear objective_func dynamics_func M;

% ----------------------------------------------------------------- %
%                       Call GPOPS-II solver                        %
%------------------------------------------------------------------ %
output = gpops2(setup);

% ----------------------------------------------------------------- %
%                     Post-Processing Routine                       %
%------------------------------------------------------------------ %
% t = output.result.solution.phase.time;
% post = post_process_cart3(auxdata, output, t);
% plot_cart3(post)

post6;
plot6;

