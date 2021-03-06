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
fda     = X(:,8);                       % Flap deflection angle
thr     = X(:,9);                       % Thrust setting

% Set AoB = 0, derive AoA from pitch-trimmed lookup
AoB     = zeros(size(t));

% Control variables
dfda    = U(:,1);                       % Flap angle rate
dthr    = U(:,2);                       % Thrust setting rate

% Constants
Re      = auxdata.Re0;                   % Radius of earth (m)
S       = auxdata.S;                    % Reference area (m^2)
max_thrust = auxdata.thrust;            % Maximum engine thrust force (N)
R       = auxdata.R;                         % (J/kg.K)
gamma   = auxdata.gamma;                     % (-)
rad2deg = 180/pi;

% Models
gravity_model = auxdata.gravity_model;
aerodynamics_model = auxdata.aerodynamics_model;
mass_model = auxdata.mass_model;
atmospheric_model = auxdata.atmospheric_model;

%-------------------------------------------------------------------%
%                         Calculate Dynamics                        %
%-------------------------------------------------------------------%
g_L         = gravity_model(sBE_L);
h           = -sBE_L(:,3) - Re;
[T,~,rho]   = atmospheric_model(h);
a           = sqrt(gamma.*R.*T);
F           = thr.*max_thrust;

d_vBE_L = zeros(length(t),3);
d_sBE_L = zeros(length(t),3);
mdot = zeros(size(t));
aoas = zeros(size(t));
machs = zeros(size(t));
fpas = zeros(size(t));

for i = 1:length(t)
    [V,hda,fpa] = car2pol(vBE_L(i,:));
    T_VL        = TM_VL(fpa, hda);
    
    Ma          = V/a(i);
    qbar        = 0.5*rho(i)*V^2;
    
    AoA         = auxdata.trimmed_aero.aoa(Ma, fda(i)*rad2deg)*pi/180;
    CL          = auxdata.trimmed_aero.CL(Ma, fda(i)*rad2deg);
    CD          = auxdata.trimmed_aero.CD(Ma, fda(i)*rad2deg);
    f_ap_M      = [ F(i)*cos(AoA) - qbar*S*CD; 
                             0;
                   -F(i)*sin(AoA) - qbar*S*CL];
    f_ap_V      = TM_MV(AoB(i))' * f_ap_M;
    f_sp_V      = f_ap_V/m(i);

    % Equations of motion
    d_vBE_L(i,:)    = (T_VL' * f_sp_V)' + g_L;
    d_sBE_L(i,:)    = vBE_L(i,:);
    mdot(i)         = mass_model(F(i));
    
    % Append path variables
    aoas(i) = AoA;
    machs(i) = Ma;
    fpas(i) = fpa;
end

% figure(2);
% plot(t,h)


% figure(1);
% clf;
% subplot(3,1,1); grid on; hold on;
% title("North");
% plot(t, sBE_L(:,1));
% xlabel('time (s)');
% 
% subplot(3,1,2); grid on; hold on;
% title('East');
% plot(t, sBE_L(:,2));
% xlabel('time (s)');
% 
% subplot(3,1,3); grid on; hold on;
% title("h = -Down - Re");
% plot(t, -sBE_L(:,3) - Re);
% xlabel('time (s)');
% 
% figure(2);
% clf;
% subplot(3,1,1); grid on; hold on;
% title("North acceleration");
% plot(t,d_vBE_L(:,1));
% xlabel('time (s)');
% 
% subplot(3,1,2); grid on; hold on;
% title("East acceleration");
% plot(t,d_vBE_L(:,2));
% xlabel('time (s)');
% 
% subplot(3,1,3); grid on; hold on;
% title("Down acceleration");
% plot(t,d_vBE_L(:,3));
% xlabel('time (s)');

%-------------------------------------------------------------------%
%                       Construct Dynamics output                   %
%-------------------------------------------------------------------%
output.dynamics = [d_sBE_L, d_vBE_L, mdot, dfda, dthr];
output.path     = [aoas, machs];
