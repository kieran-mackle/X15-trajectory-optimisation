function post = euler_cart6_post(auxdata, output, t)
% ====== Post Processing for Cartesian 6 DoF Flight Dynamics =======
%  A post-processing routine for the cartesian 6DoF flight dynamics
%  model.
% ==================================================================

state = output.result.solution.phase.state;

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
% qbar = 0.5.*rho.*V.^2;
Ma = V./a;


% Euler angles - not sure if required in the dynamics
% psi = atan( 2*(q1*q2 + q0*q3) / (q0.^2 + q1.^2 - q2.^2 - q3.^2) );
% theta = asin( -2 * (q1*q3 - q0*q2) );
% phi = atan( 2*(q2*q3 + q0*q1) / (q0.^2 - q1.^2 - q2.^2 - q3.^2) );

% psi = zeros(size(t));
% theta = zeros(size(t));
% phi = zeros(size(t));
aoas = zeros(size(t));
fpas = zeros(size(t));
hdas = zeros(size(t));
CLs = zeros(size(t));
CDs = zeros(size(t));
Cms = zeros(size(t));

for i = 1:length(t)
    
    % Aerodynamics
    aoa = atan(vBE_B(i,3)/vBE_B(i,1));
    [CL,CD,Cm] = auxdata.aerodynamics_model(auxdata, aoa*rad, Ma(i), fda(i)*rad);
    
    vBE_G = TM_BG(phi(i), theta(i), psi(i))' * vBE_B(i,:)';
    [~,~,fpa] = car2pol(vBE_G);
    
    % Append
    aoas(i) = aoa;
    fpas(i) = fpa;
    CLs(i) = CL;
    CDs(i) = CD;
    Cms(i) = Cm;
end


% Construct output struct
post.t = t;
post.sBE_L = sBE_L;
post.vBE_B = vBE_B;
post.wBE_B = wBE_B;
post.psi = psi;
post.theta = theta;
post.phi = phi;
post.h = h;
post.p = p;
post.q = q;
post.r = r;
post.N = sBE_L(:,1);
post.E = sBE_L(:,2);
post.D = sBE_L(:,3);
post.u = vBE_B(:,1);
post.v = vBE_B(:,2);
post.w = vBE_B(:,3);
post.m = m;
post.fda = fda;
post.thr = thr;

post.Ma = Ma;
post.aoa = aoas;
post.fpa = fpas;
post.hda = hdas;

post.CL = CLs;
post.CD = CDs;
post.Cm = Cms;
% post.F = F;



%-------------------------------------------------------------------%
%                     Prepare save directory                        %
%-------------------------------------------------------------------%
ad = auxdata;
if exist([ad.hd,'/Results/',ad.config,'/',ad.DOF,'/',ad.name],'dir')
    reply = input('Directory already exists. Overwrite? (y/n) [y]: ','s');
    if isempty(reply)
        reply = 'y';
    end
    
    if reply == 'n'
        reply2 = input('Post process without saving? [y]: ','s');
        if isempty(reply2)
            reply2 = 'y';
        end
        
        if reply2 == 'n'
            disp('Post processing cancelled.');
            clear solution name config reply
            return
        else
            reply2 = 'y';
        end
        
    end
else
    mkdir([ad.hd,'/Results/',ad.config,'/',ad.DOF,'/',ad.name]);
    reply = 'y';
    reply2 = 'n';
end

folder      = join([ad.hd,'/Results/',ad.config,'/',ad.DOF,'/',ad.name,'/']);

if reply == 'n' && reply2 == 'y'
    clear solution config reply* total
    return
else
    clear solution config reply* total
    save([folder,auxdata.name]);
end
