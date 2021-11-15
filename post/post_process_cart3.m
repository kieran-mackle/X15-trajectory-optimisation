function post = post_process_cart3(auxdata, output, t)
% ====== Post Processing for Cartesian 3 DoF Flight Dynamics =======
%  A post-processing routine for the cartesian 3DoF flight dynamics
%  model.
% ==================================================================

state = output.result.solution.phase.state;

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
hdas = zeros(size(t));
Vs = zeros(size(t));
% AoB = zeros(size(t));
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
    Vs(i) = V;
    aoas(i) = AoA;
    machs(i) = Ma;
    fpas(i) = fpa;
    hdas(i) = hda;
    CLs(i) = CL;
    CDs(i) = CD;
end

post.t = t;
post.N = N;
post.E = E;
post.D = D;
post.h = h;
post.vN = vN;
post.vE = vE;
post.vD = vD;
post.V = Vs;
post.m = m;
post.fda = fda;
post.thr = thr;
post.Ma = machs;
post.aoa = aoas;
post.fpa = fpas;
post.hda = hdas;
post.CL = CLs;
post.CD = CDs;
post.F = F;

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

