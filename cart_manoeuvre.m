function [bounds, guess, auxdata] = cart_manoeuvre(specification, auxdata)
% ======================== Altitude Change ======================== %
% Altitude change manoeuvre. Specify initial and final altitude.    %
% ================================================================= %

d2r             = pi/180;
imported_guess  = specification.use_guess;
auxdata.name    = specification.name;
auxdata.DOF     = '3DOF';

Re              = auxdata.Re0;
R               = auxdata.R;
gamma           = auxdata.gamma;
atmospheric_model = auxdata.atmospheric_model;

%-------------------------------------------------------------------%
%                     Define Boundary Conditions                    %
%-------------------------------------------------------------------%
h0      = specification.h0;     hf      = specification.hf;     % (m)
Ma0     = specification.Ma0;    Maf     = specification.Maf;    % (-)

t0      = 0;                    tf      = 20;               % (s)

% DEFINE POSITION
N0      = 0;                    Nf      = 0;                % (m)
E0      = 0;                    Ef      = 1700*20;                % (m)
D0      = -Re - h0;             Df      = -Re - hf;         % (m)

% DEFINE VELOCITY - flying due East
[T,~,~] = atmospheric_model([h0, hf]);
a       = sqrt(gamma.*R.*T);
vN0     = 0;                    vNf     = 0;                % (m/s)
vE0     = Ma0*a(1);             vEf     = Maf*a(2);         % (m/s)
vD0     = 0;                    vDf     = 0;                % (m/s)
m0      = 10e3;                 mf      = m0;               % (kg)

fda0    = 0;                    fdaf    = 0;
thr0    = 0;                    thrf    = 0;

% Control inputs
dfda0   = 0;                    dfdaf   = 0;
dthr0   = 0;                    dthrf   = 0;

%-------------------------------------------------------------------%
%                          Variable Limits                          %
%-------------------------------------------------------------------%
tmin    = 0;                    tmax    = 100;              % (s)
hmin    = 10e3;                 hmax    = 50e3;             % (m)
Mamin   = 4;                    Mamax   = 8;                % (-)
aoamin  = -30*d2r;              aoamax  = -aoamin;          % (rad)

fpa0    = 0*d2r;                fpaf    = 0*d2r;            % (rad)

% POSITIONAL BOUNDS
Nmin    = 0;                    Nmax    = 0;                % (m)
Emin    = 0;                    Emax    = vEf*tmax;         % (m)
Dmin    = -Re-hmax;             Dmax    = -Re-hmin;         % (m)

% VELOCITY BOUNDS  % TODO - relax velocity min/max
vNmin   = 0;                    vNmax   = 0;                % (m)
vEmin   = Mamin*a(1);           vEmax   = Mamax*a(2);         % (m)
vDmin   = Mamin*a(1);           vDmax   = Mamax*a(2);         % (m)   
mmin    = m0;                   mmax    = m0;               % (kg)

fdamin  = -40*d2r;              fdamax  = -fdamin;          % (rad)
dfdamin = -1e1*d2r;             dfdamax = -dfdamin;         % (rad/s)
thrmin  = 0;                    thrmax  = 1;                % (-)
dthrmin = -0.2;                 dthrmax = -dthrmin;         % (N/s)


% END OF USER INPUTS --------------------------------------------------- %
% ---------------------------------------------------------------------- %
sBE_Lmin = [Nmin, Emin, Dmin];      sBE_Lmax = [Nmax, Emax, Dmax];
vBE_Lmin = [vNmin, vEmin, vDmin];   vBE_Lmax = [vNmax, vEmax, vDmax];




% Construct bounds ----------------------------------------------------- %
bounds.phase.initialtime.lower  = t0;
bounds.phase.initialtime.upper  = t0;
bounds.phase.finaltime.lower    = tmin;
bounds.phase.finaltime.upper    = tmax;

bounds.phase.initialstate.lower = [sBE_Lmin, vBE_Lmin, m0,  ...
                                   fdamin, thrmin];
bounds.phase.initialstate.upper = [sBE_Lmax, vBE_Lmax, m0,  ...
                                   fdamax, thrmax];

bounds.phase.state.lower        = [sBE_Lmin, vBE_Lmin, mmin,...
                                   fdamin, thrmin];
bounds.phase.state.upper        = [sBE_Lmax, vBE_Lmax, mmax,  ...
                                   fdamax, thrmax];

bounds.phase.finalstate.lower   = [sBE_Lmin, vBE_Lmin, mmin,...
                                   fdamin, thrmin];
bounds.phase.finalstate.upper   = [sBE_Lmax, vBE_Lmax, mmax,...
                                   fdamax, thrmax];

bounds.phase.control.lower      = [dfdamin, dthrmin];
bounds.phase.control.upper      = [dfdamax, dthrmax];

bounds.phase.path.lower         = [aoamin, Mamin];
bounds.phase.path.upper         = [aoamax, Mamax];

% Eventgroups ---------------------------------------------------------- %
bounds.eventgroup(1).lower      = [Ma0, Maf];
bounds.eventgroup(1).upper      = [Ma0, Maf];

bounds.eventgroup(2).lower      = [fpa0, fpaf];
bounds.eventgroup(2).upper      = [fpa0, fpaf];

% Construct guess ------------------------------------------------------ %

if imported_guess == 1
    load('NewGuess.mat');
    guess.phase.time            = newguess.t;
    guess.phase.state           = newguess.state;
    guess.phase.control         = newguess.control;
else
    steps   = 10;
    t       = linspace(t0, tf, steps)';
    N       = linspace(N0, Nf, steps);
    E       = linspace(E0, Ef, steps);
    D       = linspace(D0, Df, steps);
    vN      = linspace(vN0, vNf, steps);
    vE      = linspace(vE0, vEf, steps);
    vD      = linspace(vD0, vDf, steps);
    m       = linspace(m0,mf,steps)';
    fda     = linspace(fda0,fdaf,steps)';
    dfda    = linspace(dfda0,dfdaf,steps)';
    thr     = linspace(thr0,thrf,steps)';
    dthr    = linspace(dthr0,dthrf,steps)';

    guess.phase.time                = t;
    guess.phase.state(:,1)          = N;
    guess.phase.state(:,2)          = E;
    guess.phase.state(:,3)          = D;
    guess.phase.state(:,4)          = vN;
    guess.phase.state(:,5)          = vE;
    guess.phase.state(:,6)          = vD;
    guess.phase.state(:,7)          = m;
    guess.phase.state(:,8)          = fda;
    guess.phase.state(:,9)          = thr;

    guess.phase.control(:,1)        = dfda;
    guess.phase.control(:,2)        = dthr;
    
end

