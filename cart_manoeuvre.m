function [bounds, guess, auxdata] = cart_manoeuvre(specification, auxdata)
% ======================== Altitude Change ======================== %
% Altitude change manoeuvre. Specify initial and final altitude.    %
% ================================================================= %

d2r             = pi/180;
imported_guess  = specification.use_guess;
auxdata.name    = specification.name;
auxdata.DOF     = '3DOF';

Re              = auxdata.Re0;
R               = ad.R;
gamma           = ad.gamma;

%-------------------------------------------------------------------%
%                     Define Boundary Conditions                    %
%-------------------------------------------------------------------%
h0      = specification.h0;     hf      = specification.hf;     % (m)
Ma0     = specification.Ma0;    Maf     = specification.Maf;    % (-)

t0      = 0;                    tf      = 20;               % (s)

% DEFINE POSITION
N0      = 0;                    Nf      = 0;                % (m)
E0      = 0;                    Ef      = 0;                % (m)
D0      = -Re - h0;             Df      = -Re - hf;         % (m)

% DEFINE VELOCITY - flying due East
[T,~,~] = atmospheric_model([h0, hf]);
a       = sqrt(gamma.*R.*T);
vN0     = 0;                    vNf     = 0;                % (m/s)
vE0     = Ma0*a(1);             vEf     = Maf*a(2);         % (m/s)
vD0     = 0;                    vDf     = 0;                % (m/s)
m0      = 10e3;                 mf      = 10e3;

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

mmin    = m0;                   mmax    = m0;               % (kg)

fdamin  = -40*d2r;              fdamax  = -fdamin;          % (rad)
dfdamin = -1e1*d2r;             dfdamax = -dfdamin;         % (rad/s)
thrmin  = 0;                    thrmax  = 1;                % (-)
dthrmin = -0.2;                 dthrmax = -dthrmin;         % (N/s)


% END OF USER INPUTS --------------------------------------------------- %
% ---------------------------------------------------------------------- %
sBE_Lmin = [Nmin, Emin, Dmin];          sBE_Lmax = [Nmax, Emax, Dmax];
vBE_Lmin = [vNmin, vEmin, vDmin];       vBE_Lmax = [vNmax, vEmax, vDmax];





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
bounds.phase.state.upper        = [sBE_Lmax, vBE_Lmax, m0,  ...
                                   fdamax, thrmax];

bounds.phase.finalstate.lower   = [sBE_Lmin, vBE_Lmin, mmin,...
                                   fdamin, thrmin];
bounds.phase.finalstate.upper   = [sBE_Lmax, vBE_Lmax, mmax,...
                                   fdamax, thrmax];

bounds.phase.control.lower      = [dfdamin,dthrmin];
bounds.phase.control.upper      = [dfdamax,dthrmax];

bounds.phase.path.lower         = [aoamin, Mamin];
bounds.phase.path.upper         = [aoamax, Mamax];

% Eventgroups ---------------------------------------------------------- %
bounds.eventgroup(1).lower      = [Ma0,Maf];
bounds.eventgroup(1).upper      = [Ma0,Maf];

bounds.eventgroup(2).lower      = [fpa0, fpaf];
bounds.eventgroup(2).upper      = [fpa0, fpaf];

% Construct guess ------------------------------------------------------ %

if imported_guess == 1
    load('NewGuess.mat');
    guess.phase.time            = newguess.t;
    guess.phase.state           = newguess.state;
    guess.phase.control         = newguess.control;
else
    N   = 100;
    t   = linspace(t0,tf,N)';
    h   = linspace(h0,hf,N)';
    dist = linspace(dist0,distf,N)';
    lon = linspace(lon0,lonf,N)';
    lat = linspace(lat0,latf,N)';
    V   = linspace(V0,Vf,N)';
    hda = linspace(hda0,hdaf,N)';
    fpa = linspace(fpa0,fpaf,N)';
    m   = linspace(m0,mf,N)';
    fda = linspace(fda0,fdaf,N)';
    dfda = linspace(dfda0,dfdaf,N)';
    thr = linspace(thr0,thrf,N)';
    dthr = linspace(dthr0,dthrf,N)';

    for i = 1:length(t)
        delta       = GetDelta(h(i),lat(i));

        T_DG        = TM_DG(delta);
        sBED(i,:)   = [lat(i),lon(i),dist(i)];

        vBEG        = pol2car(V(i),hda(i),fpa(i));
        vBED(i,:)   = (T_DG * vBEG)';

        wBIB(i,:)   = [p0,q0,r0];

        Euls(i,:)   = [roll0,pitch0,yaw0];
    end

    guess.phase.time                = t;
    guess.phase.state(:,1)          = sBED(:,1);
    guess.phase.state(:,2)          = sBED(:,2);
    guess.phase.state(:,3)          = sBED(:,3);
    guess.phase.state(:,4)          = vBED(:,1);
    guess.phase.state(:,5)          = vBED(:,2);
    guess.phase.state(:,6)          = vBED(:,3);
    guess.phase.state(:,7)          = wBIB(:,1);
    guess.phase.state(:,8)          = wBIB(:,2);
    guess.phase.state(:,9)          = wBIB(:,3);
    guess.phase.state(:,10)         = m;
    guess.phase.state(:,11)         = fda;
    guess.phase.state(:,12)         = thr;
    guess.phase.state(:,13)         = Euls(:,1);
    guess.phase.state(:,14)         = Euls(:,2);
    guess.phase.state(:,15)         = Euls(:,3);

    guess.phase.control(:,1)        = dfda;
    guess.phase.control(:,2)        = dthr;
    
end

