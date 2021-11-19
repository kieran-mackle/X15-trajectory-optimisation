function [bounds, guess, auxdata] = cart6_manoeuvre(specification, auxdata)
% ===========  Cartesian 6 DoF Manouevre Initialisation ============
%  Dynamics initialisation for Cartesian 6 DoF flight dynamics.
% ==================================================================

d2r             = pi/180;
imported_guess  = specification.use_guess;
auxdata.name    = specification.name;
auxdata.DOF     = '6DOF';

Re0             = auxdata.Re0;
% R               = auxdata.R;
% gamma           = auxdata.gamma;
% atmospheric_model = auxdata.atmospheric_model;

%-------------------------------------------------------------------%
%                        Boundary Conditions                        %
%-------------------------------------------------------------------%
h0      = specification.h0;
hf      = specification.hf;
Ma0     = specification.Ma0;
Maf     = specification.Maf;

% ALTITUDE HOLD MANOEUVRE
% --------------------------
% DEFINE POSITION
N0      = 0;
Nf      = 0;                % (m)
E0      = 0;
Ef      = Maf*a(2)*tf;      % (m)
D0      = -Re - h0;
Df      = -Re - hf;         % (m)

% DEFINE VELOCITY
u0      = Ma0*a(1);
uf      = Maf*a(2);         % (m/s)
v0      = 0;
vf      = 0;                % (m/s)
w0      = 0;
wf      = 0;                % (m/s)

% DEFINE ANGULAR VELOCITY
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
psi0 = 0;                    % Yaw (rad)
psif = 0;

% DEFINE MASS
m0      = 10e3;
mf      = m0;               % (kg)

% DEFINE CONTROLS
fda0    = 5.7*pi/180;
fdaf    = fda0;
thr0    = 0.1; 
thrf    = 0.1;

dfda0   = 0;
dfdaf   = 0;
dthr0   = 0;
dthrf   = 0;

%-------------------------------------------------------------------%
%                          Variable Limits                          %
%-------------------------------------------------------------------%
t0min    = 0;
t0max    = 0;              % (s)
tfmin    = 15;
tfmax    = 15;              % (s)

hmin    = 10e3;
hmax    = 30e3;             % (m)
Mamin   = Ma0;
Mamax   = Ma0;                % (-)
aoamin  = -30*d2r;
aoamax  = -aoamin;          % (rad)
fpa0    = 0*d2r;
fpaf    = 0*d2r;            % (rad)

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
Emin    = -Re;
Emax    = Re;         % (m)
Dmin    = D0;
Dmax    = D0; % (m)
% -------------------
Nfmin    = Nf;
Nfmax    = Nf;                % (m)
Efmin    = 0;
Efmax    = Re0;         % (m)
Dfmin    = Df;
Dfmax    = Df;          % (m)


% VELOCITY BOUNDS 
u0min   = u0;
u0max   = u0;         % (m)
v0min   = v0;
v0max   = v0;         % (m)
w0min   = w0;
w0max   = w0;         % (m) 
% -------------------
umin   = -Mamin*a(1);
umax   = Mamax*a(2); % (m)
vmin   = v0;
vmax   = v0;         % (m)
wmin   = w0;
wmax   = w0;         % (m)   
% -------------------
ufmin   = uf;
ufmax   = uf; % (m)
vfmin   = vf;
vfmax   = vf;         % (m)
wfmin   = wf;
wfmax   = wf;         % (m) 


% ANGULAR VELOCITY BOUNDS
p_0min       = 0;
p_0max       = 0;
q_0min       = 0;
q_0max       = 0;
r_0min       = 0;
r_0max       = 0;
% -------------------
pmin       = 0;
pmax       = 0;
qmin       = 0;
qmax       = 0;
rmin       = 0;
rmax       = 0;
% -------------------
p_fmin       = 0;
p_fmax       = 0;
q_fmin       = 0;
q_fmax       = 0;
r_fmin       = 0;
r_fmax       = 0;


% ATTITUDE BOUNDS
phi0min = 0;                    % Roll (rad)
phi0max = 0;
theta0min = 0;                  % Pitch (rad)
theta0max = 0;
psi0min = 0;                    % Yaw (rad)
psi0max = 0;
% -------------------
phimin = 0;                    % Roll (rad)
phimax = 0;
thetamin = 0;                  % Pitch (rad)
thetamax = 0;
psimin = 0;                    % Yaw (rad)
psimax = 0;
% -------------------
phifmin = 0;                    % Roll (rad)
phifmax = 0;
thetafmin = 0;                  % Pitch (rad)
thetafmax = 0;
psifmin = 0;                    % Yaw (rad)
psifmax = 0;


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
dfdamax = -dfdamin;
dthrmin = -0.2; 
dthrmax = -dthrmin;
% -------------------


% Initialise Quaternions
q0_0 = cos(psi0/2)*cos(theta0/2)*cos(phi0/2) + sin(psi0/2)*sin(theta0/2)*sin(phi0/2);
q1_0 = cos(psi0/2)*cos(theta0/2)*sin(phi0/2) - sin(psi0/2)*sin(theta0/2)*cos(phi0/2);
q2_0 = cos(psi0/2)*sin(theta0/2)*cos(phi0/2) + sin(psi0/2)*cos(theta0/2)*sin(phi0/2);
q3_0 = sin(psi0/2)*cos(theta0/2)*cos(phi0/2) - cos(psi0/2)*sin(theta0/2)*sin(phi0/2);

q0_f = cos(psif/2)*cos(thetaf/2)*cos(phif/2) + sin(psif/2)*sin(thetaf/2)*sin(phif/2);
q1_f = cos(psif/2)*cos(thetaf/2)*sin(phif/2) - sin(psif/2)*sin(thetaf/2)*cos(phif/2);
q2_f = cos(psif/2)*sin(thetaf/2)*cos(phif/2) + sin(psif/2)*cos(thetaf/2)*sin(phif/2);
q3_f = sin(psif/2)*cos(thetaf/2)*cos(phif/2) - cos(psif/2)*sin(thetaf/2)*sin(phif/2);

% Define limits on quaternions
q0_0min = -10;
q0_0max = 10;
q1_0min = -10;
q1_0max = 10;
q2_0min = -10;
q2_0max = 10;
q3_0min = -10;
q3_0max = 10;
% -------------------
q0min = -10;
q0max = 10;
q1min = -10;
q1max = 10;
q2min = -10;
q2max = 10;
q3min = -10;
q3max = 10;
% -------------------
q0_fmin = -10;
q0_fmax = 10;
q1_fmin = -10;
q1_fmax = 10;
q2_fmin = -10;
q2_fmax = 10;
q3_fmin = -10;
q3_fmax = 10;



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
                                   q0_0min, q1_0min, q2_0min, q3_0min, ...
                                   m0min, fda0min, thr0min];
bounds.phase.initialstate.upper = [N0max, E0max, D0max,     ...
                                   u0max, v0max, w0max,     ...
                                   p_0max, q_0max, r_0max,  ...
                                   q0_0max, q1_0max, q2_0max, q3_0max, ...
                                   m0max, fda0max, thr0max];

bounds.phase.state.lower        = [Nmin, Emin, Dmin,        ...
                                   umin, vmin, wmin,     ...
                                   pmin, qmin, rmin,        ...
                                   q0min, q1min, q2min, q3min, ...
                                   mmin, fdamin, thrmin];
bounds.phase.state.upper        = [Nmax, Emax, Dmax,        ...
                                   umax, vmax, wmax,     ...
                                   pmax, qmax, rmax,        ...
                                   q0max, q1max, q2max, q3max, ...
                                   mmax, fdamax, thrmax];

bounds.phase.finalstate.lower   = [Nfmin, Efmin, Dfmin,     ...
                                   ufmin, vfmin, wfmin,     ...
                                   p_fmin, q_fmin, r_fmin,  ...
                                   q0_fmin, q1_fmin, q2_fmin, q3_fmin, ...
                                   mfmin, fdafmin, thrfmin];
bounds.phase.finalstate.upper   = [Nfmax, Efmax, Dfmax,     ...
                                   ufmax, vfmax, wfmax,     ...
                                   p_fmax, q_fmax, r_fmax,  ...
                                   q0_fmax, q1_fmax, q2_fmax, q3_fmax, ...
                                   mfmax, fdafmax, thrfmax];

bounds.phase.control.lower      = [dfdamin, dthrmin];
bounds.phase.control.upper      = [dfdamax, dthrmax];

bounds.phase.path.lower         = [aoamin, Mamin];
bounds.phase.path.upper         = [aoamax, Mamax];


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
    t       = linspace(t0, tf, steps);
    N       = linspace(N0, Nf, steps);
    E       = linspace(E0, Ef, steps);
    D       = linspace(D0, Df, steps);
    u       = linspace(u0, uf, steps);
    v       = linspace(v0, vf, steps);
    w       = linspace(w0, wf, steps);
    p       = linspace(p_0, p_f, steps);
    q       = linspace(q_0, q_f, steps);
    r       = linspace(r_0, r_f, steps);
    q0      = linspace(q0_0, q0_f, steps);
    q1      = linspace(q1_0, q1_f, steps);
    q2      = linspace(q2_0, q2_f, steps);
    q3      = linspace(q3_0, q3_f, steps);
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
    
    guess.phase.state(:,10)         = q0;
    guess.phase.state(:,11)         = q1;
    guess.phase.state(:,12)         = q2;
    guess.phase.state(:,13)         = q3;
    
    guess.phase.state(:,14)         = m;
    guess.phase.state(:,15)         = fda;
    guess.phase.state(:,16)         = thr;
    
    guess.phase.control(:,1)        = dfda;
    guess.phase.control(:,2)        = dthr;
    
end















end