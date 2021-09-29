% ======================== Altitude Change ======================== %
% Altitude change manoeuvre. Specify initial and final altitude.    %
% ================================================================= %

d2r             = pi/180;
imported_guess  = 1;
auxdata.name    = '15to25km';
auxdata.DOF     = '6DOF';

%-------------------------------------------------------------------%
%                            Define Climb                           %
%-------------------------------------------------------------------%
h0      = 15e3;                 hf      = 25e3;             % (m)
Ma0     = 6.0;                  Maf     = 6.0;              % (-)

%-------------------------------------------------------------------%
%                        Boundary Conditions                        %
%-------------------------------------------------------------------%
t0      = 0;                    tf      = 20;               % (s)

% Define speed, heading and flight-path angle boundaries
V0      = Ma2V(Ma0,h0);         Vf      = Ma2V(Maf,hf);     % (m/s)
hda0    = 0*d2r;                hdaf    = 0*d2r;            % (rad)
fpa0    = 0*d2r;                fpaf    = 0*d2r;            % (rad)

% Define geodetic longitude and latitude
lon0    = 0*d2r;                lonf    = 0*d2r;            % (rad)
lat0    = 0*d2r;                latf    = 0*d2r;            % (rad)
% define initial Greenwich celestial longitude:
l_G0    = 0*d2r;
auxdata.l_G0 = l_G0;

% Define body rates
p0      = 0*d2r;                pf      = 0*d2r;
q0      = 0*d2r;                qf      = 0*d2r;
r0      = 0*d2r;                rf      = 0*d2r;

% Define Euler angles
roll0   = 0*d2r;                rollf   = 0*d2r;
pitch0  = 0*d2r;                pitchf  = 0*d2r;
yaw0    = 0*d2r;                yawf    = 0*d2r;

% Define remaining state variable parameters
m0      = 10.5e3;               mf      = 8.4e3;            % (kg)
fda0    = 10*d2r;               fdaf    = 10*d2r;           % (rad)
dfda0   = 0;                    dfdaf   = 0;                % (rad/s)
thr0    = 0.3;                  thrf    = 0.3;              % (-)
dthr0   = 0;                    dthrf   = 0;                % (N/s)

%-------------------------------------------------------------------%
%                          Variable Limits                          %
%-------------------------------------------------------------------%
tmin    = 0;                    tmax    = 50;               % (s)
hmin    = 10e3;                 hmax    = 50e3;             % (m)
Mamin   = 5;                    Mamax   = 7;                % (-)
aoamin  = -30*d2r;              aoamax  = -aoamin;          % (rad)
fpamin  = -40*d2r;              fpamax  = -fpamin;          % (rad)
hdamin  = -180*d2r;             hdamax  = -hdamin;          % (rad)
latmin  = -90*d2r;              latmax  = -latmin;          % (rad)
lonmin  = -180*d2r;             lonmax  = -lonmin;          % (rad)
mmin    = auxdata.drymass;      mmax    = m0;               % (kg)
fdamin  = -40*d2r;              fdamax  = -fdamin;          % (rad)
dfdamin = -1e1*d2r;             dfdamax = -dfdamin;         % (rad/s)
thrmin  = 0;                    thrmax  = 1;                % (-)
dthrmin = -0.2;                 dthrmax = -dthrmin;         % (N/s)

% END OF USER INPUTS --------------------------------------------------- %
% ---------------------------------------------------------------------- %
% Initialise state variables from input
a       = 6378137.0;
f       = 1/298.257223563;
Re0      = a*(1-(f/2)*(1-cos(2*lat0)) + (5*f^2/16)*(1-cos(4*lat0)));
Ref      = a*(1-(f/2)*(1-cos(2*latf)) + (5*f^2/16)*(1-cos(4*latf)));
dist0   = -(Re0 + h0);
distf   = -(Ref + hf);
distmin = 1.2*min(dist0,distf);
distmax = 0.8*max(dist0,distf);

% Define minimum and maximum bounds
vxmin   = -1e4;                 vxmax   = -vxmin;
vymin   = -1e4;                 vymax   = -vymin;
vzmin   = -1e4;                 vzmax   = -vzmin;

pmin    = -1e1*d2r;             pmax    = -pmin;
qmin    = -1e1*d2r;             qmax    = -qmin;
rmin    = -1e1*d2r;             rmax    = -rmin;

rollmin = -180*d2r;             rollmax = -rollmin;
pitchmin = -89*d2r;             pitchmax = -pitchmin;
yawmin  = -180*d2r;             yawmax  = -yawmin;

vBEDmin = [vxmin,vymin,vzmin];
wBIBmin = [pmin,qmin,rmin];
Eulmin  = [rollmin,pitchmin,yawmin];

vBEDmax = [vxmax,vymax,vzmax];
wBIBmax = [pmax,qmax,rmax];
Eulmax  = [rollmax,pitchmax,yawmax];

% Want to try a better start point:
delta0  = GetDelta(h0,lat0);       deltaf  = GetDelta(hf,latf);
T_DG0   = TM_DG(delta0);           T_DGf   = TM_DG(deltaf);

vBEG0   = pol2car(V0,hda0,fpa0);
vBEGf   = pol2car(Vf,hdaf,fpaf);

vBED0   = T_DG0 * vBEG0;
vBEDf   = T_DGf * vBEGf;

% Construct bounds ----------------------------------------------------- %
bounds.phase.initialtime.lower  = t0;
bounds.phase.initialtime.upper  = t0;
bounds.phase.finaltime.lower    = tmin;
bounds.phase.finaltime.upper    = tmax;

bounds.phase.initialstate.lower = [latmin,lonmin,dist0,     ...
                                   vBEDmin,wBIBmin,     ...
                                   m0,fdamin,thrmin,Eulmin];
bounds.phase.initialstate.upper = [latmax,lonmax,dist0,     ...
                                   vBEDmax,wBIBmax,     ...
                                   m0,fdamax,thrmax,Eulmax];

bounds.phase.state.lower        = [latmin,lonmin,distmin,   ...
                                   vBEDmin,wBIBmin,     	...
                                   mmin,fdamin,thrmin,Eulmin];
bounds.phase.state.upper        = [latmax,lonmax,distmax,   ...
                                   vBEDmax,wBIBmax,     	...
                                   mmax,fdamax,thrmax,Eulmax];

bounds.phase.finalstate.lower   = [latmin,lonmin,distf,   ...
                                   vBEDmin,wBIBmin,     	...
                                   mmin,fdamin,thrmin,Eulmin];
bounds.phase.finalstate.upper   = [latmax,lonmax,distf,   ...
                                   vBEDmax,wBIBmax,     	...
                                   mmax,fdamax,thrmax,Eulmax];

bounds.phase.control.lower      = [dfdamin,dthrmin];
bounds.phase.control.upper      = [dfdamax,dthrmax];

bounds.phase.path.lower         = [aoamin,Mamin];
bounds.phase.path.upper         = [aoamax,Mamax];

% Eventgroups ---------------------------------------------------------- %
bounds.eventgroup(1).lower      = [Ma0,Maf];
bounds.eventgroup(1).upper      = [Ma0,Maf];

bounds.eventgroup(2).lower      = [fpa0,fpaf];
bounds.eventgroup(2).upper      = [fpa0,fpaf];

bounds.eventgroup(3).lower      = [h0,hf];
bounds.eventgroup(3).upper      = [h0,hf];

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
%-------------------------------------------------------------------%
%                     Clear unrequired variables                    %
%-------------------------------------------------------------------%
clear *min *max *0 *f d2r delta Euler i li latc pitch roll yaw ad a
clear sBII vBII wBIB vBEG T_DG T_DI
clear N t h lon lat V hda fpa m fda dfda thr dthr
clear dist Re sBED vBED Euls imported_guess newguess
