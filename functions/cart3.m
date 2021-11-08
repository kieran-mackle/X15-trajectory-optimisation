function output = cart3(input)
% =============== Cartesian 3 DoF Flight Dynamics ================== 
%  Based on Chapter 8.1 of "Modeling and Simulation of Aerospace 
%  Vehicle Dynamics" by Peter H. Zipfel. Dynamics are expressed in
%  the local-level coordinate system.
% ==================================================================

%-------------------------------------------------------------------%
%           Extract state and control variables from input          %
%-------------------------------------------------------------------%
t       = input.phase.time;             % Time vector
X       = input.phase.state;            % State vector
U       = input.phase.control;          % Control vector
auxdata = input.auxdata;                % Auxiliary data struct

% State variables
sBE_L   = X(:,1:3);                     % [N, E, D]
vBE_L   = X(:,4:6);                     % [uL, vL, wL]
m       = X(:,7);

% Control variables
F       = U(:,1);                       % Thrust force
AoA     = U(:,2);                       % Angle of attack
AoB     = U(:,3);                       % Angle of bank

% Constants
Re      = auxdata.Re;                   % Radius of earth
S       = auxdata.S;                    % Reference area 

% Models
gravity_model = auxdata.gravity_model;
aerodynamics_model = auxdata.aerodynamics_model;
mass_model = auxdata.mass_model;

%-------------------------------------------------------------------%
%                         Calculate Dynamics                        %
%-------------------------------------------------------------------%
% Gravity modelling
g_L         = gravity_model(sBE_L);

% Transformation matrix
[V,hda,fpa] = car2pol(vBE_L);
T_VL        = TM_VL(fpa, hda);

% Atmospheric properties
h           = sBE_L(:,3) - Re;
[~,~,rho]   = GetAtmo(h);
qbar        = 0.5*rho.*V.^2;

% Aerodynamic and propulsive forces
[CL,CD,~]   = aerodynamics_model(AoA);
f_ap_M      = [ F*cos(AoA) - qbar*S*CD; 
                         0; 
               -F*sin(AoA) - qbar*S*CL];
f_ap_V      = TM_MV(AoB)' * f_ap_M;
f_sp_V      = f_ap_V./m;

% Equations of motion
d_vBE_L     = T_VL' * f_sp_V + g_L;
d_sBE_L     = vBE_L;

mdot        = mass_model();

%-------------------------------------------------------------------%
%                       Construct Dynamics output                   %
%-------------------------------------------------------------------%
output.dynamics = [d_sBE_L, d_vBE_L', mdot];

