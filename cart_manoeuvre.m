function [bounds, guess, auxdata] = cart_manoeuvre(specification, auxdata)

% TODO
% 1) Develop for altitude hold
% 2) Generalise
% 3) Develop altitude change

d2r             = pi/180;
imported_guess  = specification.use_guess;
auxdata.name    = specification.name;
auxdata.DOF     = '3DOF';

Re              = auxdata.Re0;
R               = auxdata.R;
gamma           = auxdata.gamma;
atmospheric_model = auxdata.atmospheric_model;

if strcmpi(specification.type, 'hold')
    auxdata.altitude_hold = 1;
else
    auxdata.altitude_hold = 0;
end


%-------------------------------------------------------------------%
%                        Boundary Conditions                        %
%-------------------------------------------------------------------%
h0      = specification.h0;
hf      = specification.hf;
Ma0     = specification.Ma0;
Maf     = specification.Maf;

[T,~,~] = atmospheric_model([h0, hf]);
a       = sqrt(gamma.*R.*T);

t0      = 0;
tf      = 20;

if strcmpi(specification.type, 'hold')
    % ALTITUDE HOLD MANOEUVRE
    % --------------------------
    % DEFINE POSITION
    N0      = 0;
    Nf      = 0;                % (m)
    E0      = 0;
    Ef      = Maf*a(2)*tf;      % (m)
    D0      = -Re - h0;
    Df      = -Re - hf;         % (m)

    % DEFINE VELOCITY - flying due East
    vN0     = 0;
    vNf     = 0;                % (m/s)
    vE0     = Ma0*a(1);
    vEf     = Maf*a(2);         % (m/s)
    vD0     = 0;
    vDf     = 0;                % (m/s)

    % DEFINE MASS
    m0      = 10e3;
    mf      = m0;               % (kg)

    % DEFINE CONTROLS
    fda0    = 10*pi/180;
    fdaf    = fda0;
    thr0    = 0.2; 
    thrf    = 0.2;

    dfda0   = 0;
    dfdaf   = 0;
    dthr0   = 0;
    dthrf   = 0;

    %-------------------------------------------------------------------%
    %                          Variable Limits                          %
    %-------------------------------------------------------------------%
    t0min    = 0;
    t0max    = 0;              % (s)
    tfmin    = 0;
    tfmax    = 100;              % (s)

    hmin    = 10e3;
    hmax    = 50e3;             % (m)
    Mamin   = 4;
    Mamax   = 8;                % (-)
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

else
    % ALTITUDE CHANGE MANOEUVRE
    % --------------------------
    % DEFINE POSITION
    N0      = 0;
    Nf      = 0;                % (m)
    E0      = 0;
    Ef      = Maf*a(2)*tf;      % (m)
    D0      = -Re - h0;
    Df      = -Re - hf;         % (m)

    % DEFINE VELOCITY - flying due East
    vN0     = 0;
    vNf     = 0;                % (m/s)
    vE0     = Ma0*a(1);
    vEf     = Maf*a(2);         % (m/s)
    vD0     = 0;
    vDf     = 0;                % (m/s)

    % DEFINE MASS
    m0      = 10e3;
    mf      = m0;               % (kg)

    % DEFINE CONTROLS
    fda0    = 10*pi/180;
    fdaf    = fda0;
    thr0    = 0.2; 
    thrf    = 0.2;

    dfda0   = 0;
    dfdaf   = 0;
    dthr0   = 0;
    dthrf   = 0;

    %-------------------------------------------------------------------%
    %                          Variable Limits                          %
    %-------------------------------------------------------------------%
    t0min    = 0;
    t0max    = 0;              % (s)
    tfmin    = 20;
    tfmax    = 20;              % (s)

    hmin    = 10e3;
    hmax    = 50e3;             % (m)
    Mamin   = 4;
    Mamax   = 8;                % (-)
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
end


% Construct bounds ----------------------------------------------------- %
bounds.phase.initialtime.lower  = t0min;
bounds.phase.initialtime.upper  = t0max;
bounds.phase.finaltime.lower    = tfmin;
bounds.phase.finaltime.upper    = tfmax;

bounds.phase.initialstate.lower = [N0min, E0min, D0min,     ...
                                   vN0min, vE0min, vD0min,  ...
                                   m0min, fda0min, thr0min];
bounds.phase.initialstate.upper = [N0max, E0max, D0max,     ...
                                   vN0max, vE0max, vD0max,   ...
                                   m0max, fda0max, thr0max];

bounds.phase.state.lower        = [Nmin, Emin, Dmin,        ...
                                   vNmin, vEmin, vDmin,     ...
                                   mmin, fdamin, thrmin];
bounds.phase.state.upper        = [Nmax, Emax, Dmax,        ...
                                   vNmax, vEmax, vDmax,     ...
                                   mmax, fdamax, thrmax];

bounds.phase.finalstate.lower   = [Nfmin, Efmin, Dfmin,     ...
                                   vNfmin, vEfmin, vDfmin,  ...
                                   mfmin, fdafmin, thrfmin];
bounds.phase.finalstate.upper   = [Nfmax, Efmax, Dfmax,     ...
                                   vNfmax, vEfmax, vDfmax,   ...
                                   mfmax, fdafmax, thrfmax];

bounds.phase.control.lower      = [dfdamin, dthrmin];
bounds.phase.control.upper      = [dfdamax, dthrmax];

bounds.phase.path.lower         = [aoamin, Mamin];
bounds.phase.path.upper         = [aoamax, Mamax];


% if strcmpi(specification.type, 'hold')
%     % Penalise deviation from fpa=0
%     % ---------------------------------------------------
%     bounds.phase.integral.lower = [0];
%     bounds.phase.integral.upper = [5];
% end

% Eventgroups ---------------------------------------------------------- %
% bounds.eventgroup(1).lower      = [Ma0, Maf];
% bounds.eventgroup(1).upper      = [Ma0, Maf];

% bounds.eventgroup(2).lower      = [fpa0, fpaf];
% bounds.eventgroup(2).upper      = [fpa0, fpaf];

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
    fda     = linspace(fda0, fdaf,steps)';
    dfda    = linspace(dfda0, dfdaf,steps)';
    thr     = linspace(thr0, thrf,steps)';
    dthr    = linspace(dthr0, dthrf,steps)';

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
    
%     if strcmpi(specification.type, 'hold')
%         guess.phase.integral = zeros(1,1);
%     end
    
end

