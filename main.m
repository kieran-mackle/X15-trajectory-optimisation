% ======================= Wing Optimisation ======================= %
%  Pseudospectral optimisation problem for X-15 Delta wing config.  %
% ================================================================= %

clearvars; clc;

% ----------------------------------------------------------------- %
%                  Initialise environment with paths                %
%------------------------------------------------------------------ %
run('./inputs/Settings_Scripts/LoadPaths.m')
auxdata.hd  = pwd;

% ----------------------------------------------------------------- %
%                  Load vehicle configuration script                %
%------------------------------------------------------------------ %
Config1;

% ----------------------------------------------------------------- %
%                       Load manoeuvre script                       %
%------------------------------------------------------------------ %
Climb6;

% ----------------------------------------------------------------- %
%                        Assign dynamics script                     %
%------------------------------------------------------------------ %
dynamics_func               = @tensor6;

% ----------------------------------------------------------------- %
%                      Assign objective script                      %
%------------------------------------------------------------------ %
objective_func              = @MinTime6;

% ----------------------------------------------------------------- %
%               Provide mesh refinement and method                  %
%------------------------------------------------------------------ %
mesh.method                 = 'hp-PattersonRao';
mesh.tolerance              = 1e-1;
mesh.maxiterations          = 3;

% ----------------------------------------------------------------- %
%                    Construct GPOPS-II input                       %
%------------------------------------------------------------------ %
% setup = get_settings();

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

