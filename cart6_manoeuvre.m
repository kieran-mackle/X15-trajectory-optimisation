function [bounds, guess, auxdata] = cart6_manoeuvre(specification, auxdata)
% ===========  Cartesian 6 DoF Manouevre Initialisation ============
%  Dynamics initialisation for Cartesian 6 DoF flight dynamics.
% ==================================================================

d2r             = pi/180;
imported_guess  = specification.use_guess;
auxdata.name    = specification.name;
auxdata.DOF     = '6DOF';

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
Efmax    = Re;         % (m)
Dfmin    = Df;
Dfmax    = Df;         % (m)


% VELOCITY BOUNDS 
vN0min   = vN0;
vN0max   = vN0; % (m)
vE0min   = vE0;
vE0max   = vE0;         % (m)
vD0min   = vD0;
vD0max   = vD0;         % (m) 
% -------------------
vNmin   = vN0;
vNmax   = vN0; % (m)
vEmin   = -Mamin*a(1);
vEmax   = Mamax*a(2);         % (m)
vDmin   = vD0;
vDmax   = vD0;         % (m)   
% -------------------
vNfmin   = vNf;
vNfmax   = vNf; % (m)
vEfmin   = vEf;
vEfmax   = vEf;         % (m)
vDfmin   = vDf;
vDfmax   = vDf;         % (m) 

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
fdamax  = -fdamin;          % (rad)
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
q0min = 0;
q0max = 0;
q1min = 0;
q1max = 0;
q2min = 0;
q2max = 0;
q3min = 0;
q3max = 0;


%-------------------------------------------------------------------%
%                           Construct bounds                        %
%-------------------------------------------------------------------%
bounds.phase.initialtime.lower  = t0min;
bounds.phase.initialtime.upper  = t0max;
bounds.phase.finaltime.lower    = tfmin;
bounds.phase.finaltime.upper    = tfmax;

bounds.phase.initialstate.lower = [];
bounds.phase.initialstate.upper = [];

bounds.phase.state.lower        = [];
bounds.phase.state.upper        = [];

bounds.phase.finalstate.lower   = [];
bounds.phase.finalstate.upper   = [];

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
    p       = linspace();
    q       = linspace();
    r       = linspace();
    q0      = linspace();
    q1      = linspace();
    q2      = linspace();
    q3      = linspace();
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