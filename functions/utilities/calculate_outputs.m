function [Ma, h] = calculate_outputs(auxdata, x)

state = x(1:15);

% State variables
sBE_L = state(:,1:3);   % [N, E, D]
vBE_B = state(:,4:6);   % [u, v, w]

% Body rates
wBE_B = state(:,7:9);   % [p, q, r]
p = state(:,7);
q = state(:,8);
r = state(:,9);

% Euler angles
phi = state(:,10);
theta = state(:,11);
psi = state(:,12);

% Mass
m = state(:,13);

% Control variables
fda = state(:,14);       % flap deflection angle
thr = state(:,15);       % thrust setting

% Control inputs
% dfda = input.phase.control(:,1);    % Flap angle rate
% dthr = input.phase.control(:,2);    % Thrust setting rate


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
Re0 = auxdata.Re0;                  % (m)

h = -sBE_L(:,3) - Re0;
[T,~,rho] = auxdata.atmospheric_model(h);
a = sqrt(gamma.*R.*T);
V = sqrt(sum(vBE_B.^2,2));
Ma = V./a;

% 
% aoas = zeros(size(t));
% fpas = zeros(size(t));
% hdas = zeros(size(t));
% CLs = zeros(size(t));
% CDs = zeros(size(t));
% Cms = zeros(size(t));
% 
% for i = 1:length(t)
%     
%     % Aerodynamics
%     aoa = atan(vBE_B(i,3)/vBE_B(i,1));
%     [CL,CD,Cm] = auxdata.aerodynamics_model(auxdata, aoa*rad, Ma(i), fda(i)*rad);
%     
%     vBE_G = TM_BG(phi(i), theta(i), psi(i))' * vBE_B(i,:)';
%     [~,~,fpa] = car2pol(vBE_G);
%     
%     % Append
%     aoas(i) = aoa;
%     fpas(i) = fpa;
%     CLs(i) = CL;
%     CDs(i) = CD;
%     Cms(i) = Cm;
% end




