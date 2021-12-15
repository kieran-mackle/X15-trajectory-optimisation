% ============ 6DOF Optimisation Post Process Script ============== %
% Extracts solution from GPOPS-II output and prepares inputs        %
% readable by Simulink forward simulator.
% ================================================================= %

writeCSVfile        = 0;

%-------------------------------------------------------------------%
%                          Extract solution                         %
%-------------------------------------------------------------------%
solution            = output.result.solution.phase;
mesh_hist           = output.meshhistory;
ad                  = auxdata;

plotting.t          = solution.time;
plotting.latd       = solution.state(:,1);
plotting.lond       = solution.state(:,2);
plotting.dist       = solution.state(:,3);
plotting.uD         = solution.state(:,4);
plotting.vD         = solution.state(:,5);
plotting.wD         = solution.state(:,6);
plotting.p          = solution.state(:,7);
plotting.q          = solution.state(:,8);
plotting.r          = solution.state(:,9);
plotting.m          = solution.state(:,10);
plotting.fda        = solution.state(:,11);
plotting.thr        = solution.state(:,12);
plotting.roll       = solution.state(:,13);
plotting.pitch      = solution.state(:,14);
plotting.yaw        = solution.state(:,15);

plotting.dfda       = solution.control(:,1);
plotting.dthr       = solution.control(:,2);

for i=1:length(mesh_hist)
    mesh(i).meshPoints      = [0 cumsum(mesh_hist(i).result.setup.mesh.phase.fraction)];
    mesh(i).time            = mesh_hist(i).result.solution.phase.time;
    mesh(i).iteration       = i*ones(size(mesh(i).meshPoints));
    mesh(i).iterationTime   = i*ones(size(mesh(i).time));
end

t       = plotting.t;
zeta    = zeros(size(t));
eta     = zeros(size(t));

%-------------------------------------------------------------------%
%                     Prepare save directory                        %
%-------------------------------------------------------------------%
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

%-------------------------------------------------------------------%
%                      Post-process procedure                       %
%-------------------------------------------------------------------%
% Constants
Re0     = ad.Re0;        % (m)
we      = ad.we;        % (rad/s)
g0      = ad.g0;        % (m/s)
S       = ad.S;         % (m^2)
b       = ad.b;         % (m)
c       = ad.c;         % (m)
Isp     = ad.Isp;       % (N-s/kg)
R       = ad.R;         % (J/kg.K)
gamma   = ad.gamma;     % (-)
rad     = ad.rad;       % (deg/rad)
MOI     = ad.MOI;       % (kg.m^2)
invMOI  = ad.invMOI;    % ()
lG0     = ad.l_G0;      % (rad)

%-------------------------------------------------------------------%
%                     State tensor construction                     %
%-------------------------------------------------------------------%
sBED    = [zeros(size(t)),zeros(size(t)),plotting.dist];
vBED    = [plotting.uD,plotting.vD,plotting.wD];
wBIB    = [plotting.p,plotting.q,plotting.r];
sEII    = [0;0;0];

%-------------------------------------------------------------------%
%                       Calculate dependencies                      %
%-------------------------------------------------------------------%
a               = 6378137.0;
f               = 3.33528106e-3;
Re              = a.*(1-(f/2).*(1-cos(2.*plotting.latd)) + ...
                     (5.*f.^2./16).*(1-cos(4.*plotting.latd)));
plotting.h      = -(plotting.dist + Re);
[Temp,~,rho]    = GetAtmo(plotting.h);
sos             = sqrt(auxdata.gamma.*auxdata.R.*Temp);

delta           = GetDelta(plotting.h,plotting.latd);
latc            = plotting.latd - delta;
li              = auxdata.l_G0 + auxdata.we*...
                  (t-t(1)) + plotting.lond;

plotting.V      = sqrt(sum(vBED.^2,2));

plotting.Ma     = plotting.V./sos;
qbar            = 0.5.*rho.*plotting.V.^2;
plotting.F_T    = plotting.thr*auxdata.thrust;

R_EII   = R_tensor([0,0,we]);

aoas    = zeros(size(t));
beta    = zeros(size(t));
V2      = zeros(size(t));
hda     = zeros(size(t));
fpa     = zeros(size(t));
L       = zeros(size(t));
D       = zeros(size(t));
M       = zeros(size(t));
f_aB    = zeros(length(t),3);
f_pB    = zeros(length(t),3);
f_apB   = zeros(length(t),3);
Fsp_aD  = zeros(length(t),3);
Fsp_pD  = zeros(length(t),3);
g_G     = zeros(length(t),3);
bForces = zeros(length(t),3);
f_spNED = zeros(length(t),3);
moments = zeros(length(t),3);
g_B     = zeros(length(t),3);
gNED    = zeros(length(t),3);
vBEDdot = zeros(length(t),3);
vBEB    = zeros(length(t),3);
sBII    = zeros(length(t),3);
vBEGs   = zeros(length(t),3);

for i = 1:length(plotting.t)
    
    % TRANSFORMATIONS ------------------------------------ %
    T_DG        = TM_DG(delta(i));
    T_DI        = TM_DI(plotting.latd(i),li(i));
    T_BG        = TM_BG(plotting.roll(i),plotting.pitch(i),plotting.yaw(i));
    T_BD        = T_BG * T_DG';
    
    vBEB(i,:)   = T_BD * vBED(i,:)';
    beta(i)     = asin(vBEB(i,2)/norm(vBEB(i,:)));
    aoa         = atan(vBEB(i,3)/vBEB(i,1));
    aoas(i)     = aoa;
    
    vBEG        = T_DG'*vBED(i,:)';
    [V2(i),hda(i),fpa(i)] = car2pol(vBEG);
    
    % FORCES AND MOMENTS --------------------------------- %
    qb = qbar(i); FT = plotting.F_T(i); Mach = plotting.Ma(i); fd = plotting.fda(i);
    FMinput = MakeStruc(ad,aoa,Mach,fd,qb,FT);
    [f_apB, mBB, f_aB, f_pB, maB, mpB, L(i), D(i), M(i)] = GetFM(FMinput,1);
    f_spB       = (1./plotting.m(i)) * f_apB;
    f_spD       = T_BD'*f_spB;
    
    % Convert aero and propulsion forces to geodetic frame
    f_aD       = (1./plotting.m(i)) * T_BD'*f_aB;
    f_pD       = (1./plotting.m(i)) * T_BD'*f_pB;
    
    % Gravity
    sBII        = T_DI'*sBED(i,:)' + sEII;
    g_G         = GetGravity(sBII,latc(i),4);
    gs(i)       = norm(g_G);
    
    % Equations of motion -------------------------------- %
    vBEDdot(i,:) = (T_BD'*f_spB + T_DG * g_G -          ...
                    2*T_DI*R_EII*T_DI'*vBED(i,:)' -     ...
                    T_DI*R_EII*R_EII*sBII)';
    
    
    % Store results in arrays
    vBEGs(i,:)      = vBEG;
    bForces(i,:)    = f_spB;
    f_spNED(i,:)    = f_spD;
    moments(i,:)    = mBB;
    gNED(i,:)       = g_G;
    g_B(i,:)        = (T_BG * g_G)';
    
    Fsp_aD(i,:)     = f_aD;
    Fsp_pD(i,:)     = f_pD;
    
end

plotting.aoa    = aoas;
plotting.beta   = beta;
plotting.fpa    = fpa;
plotting.hda    = hda;
plotting.f_spB  = bForces;
plotting.mBB    = moments;
plotting.g_B    = g_B;
plotting.g_NED  = gNED;
plotting.f_spD  = f_spNED;
plotting.Fsp_aD = Fsp_aD;
plotting.Fsp_pD = Fsp_pD;
plotting.aBED   = vBEDdot;
plotting.vBEG   = vBEGs;
plotting.L      = L;
plotting.D      = D;
plotting.M      = M;


% Convert latitude and longitude
plotting.xtrack = plotting.latd*Re0; % North equivalent
plotting.ytrack = plotting.lond.*Re; % East equivalent

%-------------------------------------------------------------------%
%            Prepare simulink inputs and save workspace             %
%-------------------------------------------------------------------%
forwardsim.FDA      = [plotting.t,plotting.fda*auxdata.rad];
forwardsim.Thrust   = [plotting.t,plotting.thr];
forwardsim.pos0     = [plotting.latd(1)*180/pi ...
                       plotting.lond(1)*180/pi plotting.h(1)];
forwardsim.vel0     = [vBEB(1,:)];
forwardsim.EO0      = [plotting.roll(1) plotting.pitch(1) ...
                       plotting.yaw(1)];
forwardsim.rot0     = [plotting.p(1) plotting.q(1) plotting.r(1)];
forwardsim.m0       = plotting.m(1);

%-------------------------------------------------------------------%
%                      Run forward simulation                       %
%-------------------------------------------------------------------%
% FSResults = sim('forwardsim6',plotting.t(end));

%-------------------------------------------------------------------%
%                     Write CSV file of Results                     %
%-------------------------------------------------------------------%
if writeCSVfile == 1
    t       = plotting.t;
    h       = plotting.h;
    lat     = plotting.latd*180/pi;
    lon     = plotting.lond*180/pi;
    Ma      = plotting.Ma;
    V       = plotting.V;
    fpa     = plotting.fpa*180/pi; 
    hda     = plotting.hda*180/pi;
    aoa     = plotting.aoa*180/pi;
    roll    = plotting.roll*180/pi;
    pitch   = plotting.pitch*180/pi;
    yaw     = plotting.yaw*180/pi;
    p       = plotting.p*180/pi;
    q       = plotting.q*180/pi;
    r       = plotting.r*180/pi;
    m       = plotting.m;
    F_T     = plotting.F_T;
    thrpc   = plotting.thr;
    fda     = plotting.fda*180/pi;
    L       = plotting.L;
    D       = plotting.D;
    M       = plotting.M;
    f_spB   = plotting.f_spB;

    T       = table(t, h, lat, lon, Ma, V, fpa, hda, aoa, roll, pitch,  ...
                    yaw, p, q, r, F_T, fda, L, D, M, f_spB, m, thrpc);
    T.Properties.VariableNames = {'Time (s)', 'Altitude (m)',           ...
              'Latitude (deg)',     ...
              'Longitude (deg)', 'Mach number', 'Velocity (m/s)',       ...
              'Flight path angle (deg)', 'Heading angle (deg)',         ...
              'Angle of attack (deg)', 'Roll angle (deg)',              ...
              'Pitch angle (deg)', 'Yaw angle (deg)', 'p (deg/s)',      ...
              'q (deg/s)', 'r (deg/s)', 'Thrust force (N)',             ...
              'Flap deflectiona angle (deg)', 'Lift force (N)',         ...
              'Drag force (N)', 'Pitching moment (Nm)',                 ...
              'Specific force (body axes) (m/s)', 'Vehicle mass (kg)',  ...
              'Thrust throttle setting (fraction)'};
          
    writetable(T, 'AltitudeClimb.csv');
end


%-------------------------------------------------------------------%
%         Clear unrequired variables and save workspace             %
%-------------------------------------------------------------------%
clearvars -except aerodeck auxdata bounds folder forwardsim     ...
                  FSResults guess mesh output plotting setup reply*

if reply == 'n' && reply2 == 'y'
    clear solution config reply* total
    return
else
    clear solution config reply* total
    save([folder,auxdata.name]);
end

