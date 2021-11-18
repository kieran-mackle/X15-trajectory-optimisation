function output = cart6(input)
% =============== Cartesian 6 DoF Flight Dynamics ================== 
%  Based on Chapter 10.1.1 of "Modeling and Simulation of Aerospace 
%  Vehicle Dynamics" by Peter H. Zipfel.
% ==================================================================

%-------------------------------------------------------------------%
%                            Extract Input                          %
%-------------------------------------------------------------------%
auxdata = input.auxdata;

sBE_L = input.phase.state(:,1:3);   % [N, E, D]
vBE_B = input.phase.state(:,4:6);   % [u, v, w]
wBE_B = input.phase.state(:,7:9);   % [p, q, r]
fda = input.phase.state(:,1);       % flap deflection angle
thr = input.phase.state(:,1);       % thrust setting

% Control variables
dfda = input.phase.control(:,1);    % Flap angle rate
dthr = input.phase.control(:,2);    % Thrust setting rate

% Constants
invMOI = auxdata.invMOI;
MOI = auxdata.MOI;

% Models
gravity_model = auxdata.gravity_model;
aerodynamics_model = auxdata.aerodynamics_model;
mass_model = auxdata.mass_model;
atmospheric_model = auxdata.atmospheric_model;


%-------------------------------------------------------------------%
%                       Calculate Dependencies                      %
%-------------------------------------------------------------------%
h = -sBE_L(:,3) - Re;
[T,~,rho] = atmospheric_model(h);
a = sqrt(gamma.*R.*T);


%-------------------------------------------------------------------%
%                         Calculate Dynamics                        %
%-------------------------------------------------------------------%
% Initialisation
d_vBE_B = zeros(length(t),3);
d_sBE_L = zeros(length(t),3);
d_wBE_B = zeros(length(t),3);


% Euler angles 

% Quaternions
q0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
q1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
q2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
q3 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);

% Quaternion matrix
qtm = [0, -p, -q, -r;
       p,  0,  r, -q;
       q, -r,  0,  p;
       r,  q, -p,  0];


for i = 1:length(t)
% Equations of motion
d_vBE_B(i,:) = f_sp_B - R_BE_B * vBE_B + T_BL * g_L;
d_sBE_L(i,:) = T_BL' * vBE_B(i,:);
d_wBE_B(i,:) = invMOI * (-R_BE_B*MOI*wBE_B(i,:)' + mBB);
end

% Quaternions


output.dynamics = [d_sBE_L, d_vBE_B, d_wBE_B];
