% =================== Nominal X-15 Configuration ================== %
% Configuration file for nominal configuration of X-15 with delta   %
% wing (NASA TN D-5498).                                            %
% ================================================================= %
auxdata.config  = 'Config1';

%-------------------------------------------------------------------%
%                       Define physical constants                   %
%-------------------------------------------------------------------%
auxdata.Re0     = geocradius(0);    % m
auxdata.mue     = 3.986012e14;      % m^3/s^2
auxdata.we      = 7.29211585e-5;    % rad/s
auxdata.g0      = 9.80665;          % m/s^2
auxdata.R       = 287.053;
auxdata.gamma   = 1.4;
auxdata.fda0    = 0*pi/180;
auxdata.deg     = pi/180;
auxdata.rad     = 180/pi;

% Import aerodynamic force coefficient database
% (Old aerodeck)
% temp            = importdata('C1AeroDeck.csv');
% dat             = sortrows(temp.data,1);
% dat             = sortrows(dat,2);
% data1           = sortrows(dat,3);
% auxdata.aero    = NormDB(data);

% Aerodeck from re-run:
temp            = importdata('deck1.csv');
dat             = sortrows(temp.data,1);
dat             = sortrows(dat,2);
data1           = sortrows(dat,3);
auxdata.aero    = NormDB(data1);

% % File containing viscous additions:
% temp            = importdata('viscdeck.csv');
% dat             = sortrows(temp.data,1);
% dat             = sortrows(dat,2);
% data            = sortrows(dat,3);
% visc_coefs      = data1(:,4:6) + data(:,4:6);
% data2           = data;
% data2(:,4:6)    = visc_coefs;
% auxdata.aero    = NormDB(data2);

% SurfacedAero
% ScatteredInterpfuncts
clear temp dat data inviscDat

% temp            = importdata('x15_aerodeck.txt');
% temp2           = temp.data(:,[1,4,3,5,6,9]);
% dat             = sortrows(temp2,2);
% data            = sortrows(dat,3);
% auxdata.aero    = NormDB(data);
% clear temp* dat data

% Vehcicle shape parameters (scale by 50 for lengths, 50^2 for area)
auxdata.S       = 2*56.0205;
auxdata.c       = 2*10.181;
auxdata.b       = 2*6.8155;
auxdata.xcg     = 21.923*50/100;
auxdata.xcp     = 21.923*50/100;

% Vehicle performance parameters - wiki/Reaction_Motors_XLR99
auxdata.thrust  = 253.55e3;
auxdata.Isp     = 277;
auxdata.x_T     = -7.479; % ( (36.881-21.923)*(50/100) m )
auxdata.z_T     = 0.2;  % thrust offset in z direction (body) (arbitrary)

% Vehicle mass properties - wiki/X-15_Flight_91
auxdata.wetmass = 15195;
auxdata.drymass = 6577;
auxdata.Iyy     = 107109.0;  % NASA TM-207
auxdata.MOI     = 107109.0*eye(3,3);
auxdata.invMOI  = inv(auxdata.MOI);
