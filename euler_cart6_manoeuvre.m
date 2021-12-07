function [bounds, guess, auxdata] = euler_cart6_manoeuvre(specification, auxdata)
% ===========  Cartesian 6 DoF Manouevre Initialisation ============
%  Dynamics initialisation for Cartesian 6 DoF flight dynamics.
% ==================================================================

d2r             = pi/180;
imported_guess  = specification.use_guess;
auxdata.name    = specification.name;
auxdata.DOF     = '6DOF';

Re0             = auxdata.Re0;
R               = auxdata.R;
gamma           = auxdata.gamma;
atmospheric_model = auxdata.atmospheric_model;

% Define altitude type
if strcmpi(specification.type, 'hold')
    auxdata.altitude_hold = 1;
else
    auxdata.altitude_hold = 0;
end

%-------------------------------------------------------------------%
%               Boundary Conditions (for initial guess)             %
%-------------------------------------------------------------------%
h0      = specification.h0;
hf      = specification.hf;
Ma0     = specification.Ma0;
Maf     = specification.Maf;

[T,~,~] = atmospheric_model([h0, hf]);
a       = sqrt(gamma.*R.*T);

t0      = 0;
tf      = 10;

% ALTITUDE HOLD MANOEUVRE
% --------------------------
% DEFINE POSITION (NED)
N0      = 0;
Nf      = 0;                % (m)
E0      = 0;
Ef      = Maf*a(2)*tf;      % (m)
D0      = -Re0 - h0;
Df      = -Re0 - hf;         % (m)

% DEFINE VELOCITY (Body Coordiantes)
u0      = Ma0*a(1);
uf      = Maf*a(2);         % (m/s)
v0      = 0;
vf      = 0;                % (m/s)
w0      = 0;
wf      = 0;                % (m/s)

% DEFINE ANGULAR VELOCITY (Body rates)
p_0       = 0;
p_f       = 0;
q_0       = 0;
q_f       = 0;
r_0       = 0;
r_f       = 0;

% DEFINE ATTITUDE (Euler angles)
phi0 = 0;                    % Roll (rad)
phif = 0;
theta0 = 0;                  % Pitch (rad)
thetaf = 0;
psi0 = 90*d2r;                % Yaw (rad)
psif = 90*d2r;

% DEFINE MASS
m0      = 10e3;
mf      = m0;               % (kg)

% DEFINE CONTROLS
fda0    = 5.629*pi/180;
fdaf    = fda0;
thr0    = 0.2715; 
thrf    = 0.2715;

dfda0   = 0;
dfdaf   = 0;
dthr0   = 0;
dthrf   = 0;

%-------------------------------------------------------------------%
%                          Variable Limits                          %
%-------------------------------------------------------------------%
t0min    = 0;
t0max    = 0;              % (s)
tfmin    = 10;
tfmax    = 20;              % (s)

% hmin    = 10e3;
% hmax    = 30e3;             % (m)
Mamin   = 4;
Mamax   = 8;                % (-)
aoamin  = -10*d2r;
aoamax  = 10*d2r;          % (rad)
% fpa0    = 0*d2r;
% fpaf    = 0*d2r;            % (rad)

% POSITIONAL BOUNDS
N0min    = N0;
N0max    = N0;                % (m)
E0min    = E0;
E0max    = E0;         % (m)
D0min    = D0;
D0max    = D0;         % (m)
% -------------------
Nmin    = N0;
Nmax    = N0;                % (m)
Emin    = 0;
Emax    = Re0;         % (m)
Dmin    = D0;
Dmax    = Df; % (m)
% -------------------
Nfmin    = Nf;
Nfmax    = Nf;                % (m)
Efmin    = 0;
Efmax    = Re0;         % (m)
Dfmin    = Df;
Dfmax    = Df;          % (m)


% VELOCITY BOUNDS 
u0min   = -Mamax*a(1);
u0max   =  Mamax*a(1);          % (m/s)
v0min   = -Mamax*a(1);
v0max   =  Mamax*a(1);          % (m/s)
w0min   = -Mamax*a(1);
w0max   =  Mamax*a(1);          % (m/s) 
% -------------------
umin   = -Mamax*a(1);
umax   =  Mamax*a(2);           % (m/s)
vmin   = -Mamax*a(1);
vmax   =  Mamax*a(1);           % (m/s)
wmin   = -Mamax*a(1);   
wmax   =  Mamax*a(1);           % (m/s)   
% -------------------
ufmin   = -Mamax*a(1);
ufmax   =  Mamax*a(1);          % (m/s)
vfmin   = -Mamax*a(1);
vfmax   =  Mamax*a(1);          % (m/s)
wfmin   = -Mamax*a(1);
wfmax   =  Mamax*a(1);          % (m/s) 


% ANGULAR VELOCITY BOUNDS
p_0min  = -10;
p_0max  = 10;
q_0min  = -10;
q_0max  = 10;
r_0min  = -10;
r_0max  = 10;
% -------------------
pmin    = -10;
pmax    = 10;
qmin    = -10;
qmax    = 10;
rmin    = -10;
rmax    = 10;
% -------------------
p_fmin  = -10;
p_fmax  = 10;
q_fmin  = -10;
q_fmax  = 10;
r_fmin  = -10;
r_fmax  = 10;


% ATTITUDE BOUNDS
phi0min = 0;                    % Roll (rad)
phi0max = 0;
theta0min = -10*d2r;                  % Pitch (rad)
theta0max = 10*d2r;
psi0min = 90*d2r;                    % Yaw (rad)
psi0max =  90*d2r;
% -------------------
phimin = 0*d2r;                    % Roll (rad)
phimax = 0*d2r;
thetamin = -80*d2r;                  % Pitch (rad)
thetamax = 80*d2r;
psimin = 90*d2r;                    % Yaw (rad)
psimax =  90*d2r;
% -------------------
phifmin = 0;                    % Roll (rad)
phifmax = 0;
thetafmin = 10*d2r;                  % Pitch (rad)
thetafmax = 10*d2r;
psifmin = 90*d2r;                    % Yaw (rad)
psifmax =  90*d2r;


% MASS BOUNDS
m0min    = m0;
m0max    = m0;               % (kg)
% -------------------
mmin    = m0;
mmax    = m0;               % (kg)
% -------------------
mfmin    = mf;
mfmax    = mf;               % (kg)


% CONTROL BOUNDS
fda0min  = -40*d2r;
fda0max  = 40*d2r;       % (rad)
thr0min  = 0;
thr0max  = 1;  
% -------------------
fdamin  = -40*d2r;
fdamax  = 40*d2r;          % (rad)
thrmin  = 0;
thrmax  = 1;  
% -------------------
fdafmin  = -40*d2r;
fdafmax  = 40*d2r;          % (rad)
thrfmin  = 0;
thrfmax  = 1;  

% CONTROL RATE BOUNDS
% -------------------
dfdamin = -1e1*d2r;
dfdamax =  1e1*d2r;
dthrmin = -0.2; 
dthrmax =  0.2;
% -------------------


%-------------------------------------------------------------------%
%                           Construct bounds                        %
%-------------------------------------------------------------------%
bounds.phase.initialtime.lower  = t0min;
bounds.phase.initialtime.upper  = t0max;
bounds.phase.finaltime.lower    = tfmin;
bounds.phase.finaltime.upper    = tfmax;

bounds.phase.initialstate.lower = [N0min, E0min, D0min,     ...
                                   u0min, v0min, w0min,     ...
                                   p_0min, q_0min, r_0min,  ...
                                   phi0min, theta0min, psi0min, ...
                                   m0min, fda0min, thr0min];
bounds.phase.initialstate.upper = [N0max, E0max, D0max,     ...
                                   u0max, v0max, w0max,     ...
                                   p_0max, q_0max, r_0max,  ...
                                   phi0max, theta0max, psi0max, ...
                                   m0max, fda0max, thr0max];

bounds.phase.state.lower        = [Nmin, Emin, Dmin,        ...
                                   umin, vmin, wmin,     ...
                                   pmin, qmin, rmin,        ...
                                   phimin, thetamin, psimin, ...
                                   mmin, fdamin, thrmin];
bounds.phase.state.upper        = [Nmax, Emax, Dmax,        ...
                                   umax, vmax, wmax,     ...
                                   pmax, qmax, rmax,        ...
                                   phimax, thetamax, psimax, ...
                                   mmax, fdamax, thrmax];

bounds.phase.finalstate.lower   = [Nfmin, Efmin, Dfmin,     ...
                                   ufmin, vfmin, wfmin,     ...
                                   p_fmin, q_fmin, r_fmin,  ...
                                   phifmin, thetafmin, psifmin, ...
                                   mfmin, fdafmin, thrfmin];
bounds.phase.finalstate.upper   = [Nfmax, Efmax, Dfmax,     ...
                                   ufmax, vfmax, wfmax,     ...
                                   p_fmax, q_fmax, r_fmax,  ...
                                   phifmax, thetafmax, psifmax, ...
                                   mfmax, fdafmax, thrfmax];

bounds.phase.control.lower      = [dfdamin, dthrmin, -1];
bounds.phase.control.upper      = [dfdamax, dthrmax, 1];

bounds.phase.path.lower         = [aoamin, Mamin];
bounds.phase.path.upper         = [aoamax, Mamax];

% bounds.phase.integral.lower     = [0];
% bounds.phase.integral.upper     = [10e3];

% bounds.eventgroup(1).lower      = [Ma0, Maf];
% bounds.eventgroup(1).upper      = [Ma0, Maf];

%-------------------------------------------------------------------%
%                     Construct initial guess                       %
%-------------------------------------------------------------------%
if imported_guess == 1
    % Use initial guess file
    load('NewGuess.mat');
    guess.phase.time            = newguess.t;
    guess.phase.state           = newguess.state;
    guess.phase.control         = newguess.control;
    
else
    % Linearly interpolate guess
    steps   = 10;
    t       = linspace(t0, tf, steps)';
    N       = linspace(N0, Nf, steps);
    E       = linspace(E0, Ef, steps);
    D       = linspace(D0, Df, steps);
    u       = linspace(u0, uf, steps);
    v       = linspace(v0, vf, steps);
    w       = linspace(w0, wf, steps);
    p       = linspace(p_0, p_f, steps);
    q       = linspace(q_0, q_f, steps);
    r       = linspace(r_0, r_f, steps);
    phi     = linspace(phi0, phif, steps);
    theta   = linspace(theta0, thetaf, steps);
    psi     = linspace(psi0, psif, steps);
    m       = linspace(m0, mf, steps);
    fda     = linspace(fda0, fdaf, steps);
    dfda    = linspace(dfda0, dfdaf, steps);
    thr     = linspace(thr0, thrf, steps);
    dthr    = linspace(dthr0, dthrf, steps);

    guess.phase.time                = t;
    
    guess.phase.state(:,1)          = N;
    guess.phase.state(:,2)          = E;
    guess.phase.state(:,3)          = D;
    
    guess.phase.state(:,4)          = u;
    guess.phase.state(:,5)          = v;
    guess.phase.state(:,6)          = w;
    
    guess.phase.state(:,7)          = p;
    guess.phase.state(:,8)          = q;
    guess.phase.state(:,9)          = r;
    
    guess.phase.state(:,10)         = phi;
    guess.phase.state(:,11)         = theta;
    guess.phase.state(:,12)         = psi;
    
    guess.phase.state(:,13)         = m;
    guess.phase.state(:,14)         = fda;
    guess.phase.state(:,15)         = thr;
    
    guess.phase.control(:,1)        = dfda;
    guess.phase.control(:,2)        = dthr;
    guess.phase.control(:,3)        = zeros(size(t));

    guess.phase.integral            = zeros(1,1);
    
end

end