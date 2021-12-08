% ----------------------------------------------------------------- %
%                  Initialise environment with paths                %
%------------------------------------------------------------------ %
run('../../inputs/load_paths.m')

% ----------------------------------------------------------------- %
%                  Load vehicle configuration script                %
%------------------------------------------------------------------ %
auxdata = get_config('deck1.csv');  % Filename of aerodeck in ./aero/
auxdata.hd = pwd;
auxdata.coordinates = 'cartesian';  % 'polar' or 'cartesian'
auxdata.mass_model = @(F)0; % Mass rate of change
auxdata.aerodynamics_model = @GetAero;
auxdata.gravity_model = @(h)[0,0,9.81].*ones(size(h,1),1);
auxdata.atmospheric_model = @(h)GetAtmo(h);

% ----------------------------------------------------------------- %
%                       Load manoeuvre script                       %
%------------------------------------------------------------------ %
manoeuvre_spec.type = 'hold';              % 'climb' / 'hold'
manoeuvre_spec.name = '20km_hold_from_trim_MaPenalty';
manoeuvre_spec.h0 = 20e3;
manoeuvre_spec.hf = 20e3;
manoeuvre_spec.Ma0 = 6;
manoeuvre_spec.Maf = 6;
manoeuvre_spec.use_guess = 1;

[bounds, guess, auxdata] = euler_cart6_manoeuvre(manoeuvre_spec, auxdata);

% Extract initial state and control
initial = [guess.phase.state(1,:), guess.phase.control(1,:)];

% Define bounds
lb = [bounds.phase.state.lower, bounds.phase.control.lower];
ub = [bounds.phase.state.upper, bounds.phase.control.upper];

% Define dynamics function
dynamics_func = @cart6_euler;

% Call solver
x = solve_trim(auxdata, dynamics_func, initial, lb, ub);

% Test solution
input.auxdata = auxdata;
input.phase.time = 0;
input.phase.state = x(1:15);
input.phase.control = x(16:end);
out = dynamics_func(input);

