function output = cart6(input)
% =============== Cartesian 6 DoF Flight Dynamics ================== 
%  Based on Chapter 10.1.1 of "Modeling and Simulation of Aerospace 
%  Vehicle Dynamics" by Peter H. Zipfel.
% ==================================================================

% ToDo: Check indexing and dimensionality throughout code (looking at
% quaternions specifically)

%-------------------------------------------------------------------%
%                            Extract Input                          %
%-------------------------------------------------------------------%
auxdata = input.auxdata;
t = input.phase.time;

% State variables
sBE_L = input.phase.state(:,1:3);   % [N, E, D]
vBE_B = input.phase.state(:,4:6);   % [u, v, w]

% Body rates
wBE_B = input.phase.state(:,7:9);   % [p, q, r]
p = input.phase.state(:,7);
q = input.phase.state(:,8);
r = input.phase.state(:,9);

% Quaternions
q0 = input.phase.state(:,10);
q1 = input.phase.state(:,11);
q2 = input.phase.state(:,12);
q3 = input.phase.state(:,13);

% Mass
m = input.phase.state(:,14);

% Control variables
fda = input.phase.state(:,15);       % flap deflection angle
thr = input.phase.state(:,16);       % thrust setting

% Control inputs
dfda = input.phase.control(:,1);    % Flap angle rate
dthr = input.phase.control(:,2);    % Thrust setting rate
dr = input.phase.control(:,3); % Arbitrary control input

% Constants
max_thrust = auxdata.thrust;
invMOI = auxdata.invMOI;
MOI = auxdata.MOI;
S = auxdata.S;                      % (m^2)
b = auxdata.b;                      % (m)
c = auxdata.c;                      % (m)
R = auxdata.R;                      % (J/kg.K)
gamma = auxdata.gamma;              % (-)
rad = auxdata.rad;                  % (deg/rad)

% Isp = auxdata.Isp;                  % (N-s/kg)
Re0 = auxdata.Re0;                  % (m)
% we = auxdata.we;                    % (rad/s)
% g0 = auxdata.g0;                    % (m/s)

% Models
gravity_model = auxdata.gravity_model;
aerodynamics_model = auxdata.aerodynamics_model;
mass_model = auxdata.mass_model;
atmospheric_model = auxdata.atmospheric_model;


%-------------------------------------------------------------------%
%                       Calculate Dependencies                      %
%-------------------------------------------------------------------%
h = -sBE_L(:,3) - Re0;
[T,~,rho] = atmospheric_model(h);
a = sqrt(gamma.*R.*T);
V = sqrt(sum(vBE_B.^2,2));
qbar = 0.5.*rho.*V.^2;
Ma = 6*ones(length(t),1);
if any(a > 0)
    Ma(a>0) = V(a>0)./a(a>0);
end
g_L = gravity_model(sBE_L);

% Propulsion
F_T = thr * max_thrust;
eta = 0;                        % Thrust vector angle 1
zeta = 0;                       % Thrust vector angle 2

%-------------------------------------------------------------------%
%                         Calculate Dynamics                        %
%-------------------------------------------------------------------%

% Initialisation
d_vBE_B = zeros(length(t),3);
d_sBE_L = zeros(length(t),3);
d_wBE_B = zeros(length(t),3);
d_quaternions = zeros(length(t),4);
dm = zeros(length(t),1);
aoas = zeros(size(t));
phis = zeros(size(t));
psis = zeros(size(t));

for i = 1:length(t)
    % Dependencies
    R_BE_B = R_tensor([p(i), q(i), r(i)]);
    
    aoa = atan(vBE_B(i,3)/vBE_B(i,1));
    
    % Aerodynamics
    [CL,CD,Cm] = aerodynamics_model(auxdata, aoa*rad, Ma(i), fda(i)*rad);
    Cl = 0;
    Cn = 0;
    Cx =  CL.*sin(aoa) - CD.*cos(aoa);
    Cz = -CL.*cos(aoa) - CD.*sin(aoa);
    f_aB = qbar(i) * S * [Cx; 0; Cz];
    maB = qbar(i) * S * [Cl*b; Cm*c; Cn*b];
    
    % Propulsion
    f_pB = [cos(eta) * cos(zeta);
            cos(eta) * sin(eta);
            -sin(eta)] * F_T(i);
    mpB = [        0; 
               -sin(eta); 
           -cos(eta)*sin(zeta)] * F_T(i)*7.5; %(xp - xcm);
    
    % Absolute force and moment
    f_apB = f_aB + f_pB;
    m_BB = maB + mpB;

    % Specific forces
    f_sp_B = (1./m(i)) * f_apB;
    
    % Quaternion matrix
    qtm = [ 0,   -p(i), -q(i), -r(i);
           p(i),   0,    r(i), -q(i);
           q(i), -r(i),   0,    p(i);
           r(i),  q(i), -p(i),   0 ];
   
    % Body to local-level transformation matrix
    T_BL = [q0(i).^2 + q1(i).^2 - q2(i).^2 - q3(i).^2,  ...
                2*(q1(i)*q2(i) + q0(i)*q3(i)),          ...
                2*(q1(i)*q3(i) - q0(i)*q2(i));          ...
            2*(q1(i)*q2(i) - q0(i)*q3(i)),              ...
                q0(i).^2 - q1(i).^2 + q2(i).^2 - q3(i).^2,  ...
                2*(q2(i)*q3(i) + q0(i)*q1(i));          ...
            2*(q1(i)*q3(i) + q0(i)*q2(i)),              ...
                2*(q2(i)*q3(i) - q0(i)*q1(i)),          ...
                q0(i).^2 - q1(i).^2 - q2(i).^2 + q3(i).^2   ...
            ];
    
    % Equations of motion
    d_vBE_B(i,:) = f_sp_B - R_BE_B * vBE_B(i,:)' + T_BL * g_L(i,:)';
    d_sBE_L(i,:) = T_BL' * vBE_B(i,:)';
    d_wBE_B(i,:) = invMOI * (-R_BE_B*MOI*wBE_B(i,:)' + m_BB);
    d_quaternions(i,:) = 0.5 * qtm * [q0(i); q1(i); q2(i); q3(i)];
    dm(i) = mass_model(F_T(i));
    
    % Euler angles
    psi = atan( 2*(q1(i)*q2(i) + q0(i)*q3(i)) / ...
               (q0(i).^2 + q1(i).^2 - q2(i).^2 - q3(i).^2) );
    theta = asin( -2 * (q1(i)*q3(i) - q0(i)*q2(i)) );
    phi = atan( 2*(q2(i)*q3(i) + q0(i)*q1(i)) / ...
               (q0(i).^2 - q1(i).^2 - q2(i).^2 - q3(i).^2) );
    
    % Append path variables
    aoas(i) = aoa;
    phis(i) = phi;
    psis(i) = psi;
    
end

plot(t,h);

output.dynamics = [d_sBE_L, d_vBE_B, d_wBE_B, d_quaternions, ...
                   dm, dfda, dthr];
output.path     = [aoas, Ma];

