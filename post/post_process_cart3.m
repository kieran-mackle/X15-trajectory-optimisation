function output = post_process_cart3(auxdata, state, t)
% ====== Post Processing for Cartesian 3 DoF Flight Dynamics =======
%  A post-processing routine for the cartesian 3DoF flight dynamics
%  model.
% ==================================================================

N = state(:,1);
E = state(:,2);
D = state(:,3);
vN = state(:,4);
vE = state(:,5);
vD = state(:,6);
m = state(:,7);
fda = state(:,8);
thr = state(:,9);

% Constants
Re = auxdata.Re0; 
S = auxdata.S;  
max_thrust = auxdata.thrust; 
R = auxdata.R; 
gamma = auxdata.gamma;
rad2deg = 180/pi;

% Models
gravity_model = auxdata.gravity_model;
aerodynamics_model = auxdata.aerodynamics_model;
mass_model = auxdata.mass_model;
atmospheric_model = auxdata.atmospheric_model;



sBE_L   = state(:,1:3);
vBE_L   = state(:,4:6);

h           = -sBE_L(:,3) - Re;
[T,~,rho]   = atmospheric_model(h);
a           = sqrt(gamma.*R.*T);

F           = thr.*max_thrust;

aoas = zeros(size(t));
machs = zeros(size(t));
fpas = zeros(size(t));
AoB     = zeros(size(t));
CLs = zeros(size(t));
CDs = zeros(size(t));

for i = 1:length(t)
    [V,hda,fpa] = car2pol(vBE_L(i,:));
    T_VL        = TM_VL(fpa, hda);
    
    Ma          = V/a(i);
    qbar        = 0.5*rho(i)*V^2;
    
%     AoA         = trim_aero(auxdata, Ma, fda(i)*rad2deg);
%     [CL,CD,~]   = aerodynamics_model(auxdata, AoA, Ma, fda(i)*rad2deg);
    AoA         = auxdata.trimmed_aero.aoa(Ma, fda(i)*rad2deg)*pi/180;
    CL          = auxdata.trimmed_aero.CL(Ma, fda(i)*rad2deg);
    CD          = auxdata.trimmed_aero.CD(Ma, fda(i)*rad2deg);

    % Append path variables
    aoas(i) = AoA;
    machs(i) = Ma;
    fpas(i) = fpa;
    CLs(i) = CL;
    CDs(i) = CD;
end

output.t = t;
output.N = N;
output.E = E;
output.D = D;
output.vN = vN;
output.vE = vE;
output.vD = vD;
output.m = m;
output.fda = fda;
output.thr = thr;
output.Ma = machs;
output.aoa = aoas;
output.fpa = fpas;
output.CL = CLs;
output.CD = CDs;
