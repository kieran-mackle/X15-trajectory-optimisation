function output = tensor6(input)
% ==================== Tensor6 Flight Dynamics ===================== 
%  This script contains the dynamics input function for GPOPS-II.
%  The equations of motion model a hypersonic vehicle flying around
%  a rotating, oblate spheroid Earth in six degrees-of-freedon. The
%  modelling approach has been adapted from "Modeling and Simulation 
%  of Aerospace Vehicle Dynamics" by Zipfel. The velocity and 
%  position of the vehicle is integrated in the geodetic coordinate  
%  system, wherease the body rates are integrated in body
%  coordinates.
% ==================================================================

%-------------------------------------------------------------------%
%           Extract state and control variables from input          %
%-------------------------------------------------------------------%
t       = input.phase.time;
X       = input.phase.state;
u       = input.phase.control;
ad      = input.auxdata;

% State variables
latd    = X(:,1);
lond    = X(:,2);
dist    = X(:,3);
uD      = X(:,4);

vD      = X(:,5);
wD      = X(:,6);
p       = X(:,7);
q       = X(:,8);
r       = X(:,9);
m       = X(:,10);
fda     = X(:,11);
thr     = X(:,12);
roll    = X(:,13);
pitch   = X(:,14);
yaw     = X(:,15);

% Control variables
dfda    = u(:,1);
dthr    = u(:,2);

% Constants
Re0     = ad.Re0;       % (m)
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
vBED    = [uD,vD,wD];
wBIB    = [p,q,r];
sEII    = [0;0;0];

%-------------------------------------------------------------------%
%                   Aerodynamics initialisation                     %
%-------------------------------------------------------------------%
a       = 6378137.0;
f       = 3.33528106e-3;
Re      = a.*(1-(f/2).*(1-cos(2.*latd)) + ...
          (5.*f.^2./16).*(1-cos(4.*latd)));
h               = -(dist + Re);
V               = sqrt(sum(vBED.^2,2));
[Temp,P,rho]    = GetAtmo(h);
sos             = sqrt(gamma.*R.*Temp);
Ma = 6*ones(length(t),1);
if any(sos > 0)
    Ma(sos>0) = V(sos>0)./sos(sos>0);
end
qbar    = 0.5.*rho.*V.^2;
F_T     = thr*ad.thrust;

sBED    = [zeros(size(t)),zeros(size(t)),dist];

%-------------------------------------------------------------------%
%                       Dynamics initialisation                     %
%-------------------------------------------------------------------%
R_EII   = R_tensor([0,0,we]);

delta   = GetDelta(h,latd);
latc    = latd - delta;
li      = lG0 + we*(t-t(1)) + lond;

%-------------------------------------------------------------------%
%                         Calculate dynamics                        %
%-------------------------------------------------------------------%
vBEDdot = zeros(length(t),3);
wBIBdot = zeros(length(t),3);
sdot    = zeros(length(t),3);

rolldot = zeros(size(t));
pitchdot = zeros(size(t));
yawdot = zeros(size(t));

for i = 1:length(t)
    
    % TRANSFORMATIONS ------------------------------------ %
    T_DG        = TM_DG(delta(i));
    T_DI        = TM_DI(latd(i),li(i));
    T_BG        = TM_BG(roll(i),pitch(i),yaw(i));
    T_BD        = T_BG * T_DG';
    R_BIB       = R_tensor(wBIB(i,:)');
    
    vBEB        = T_BD * vBED(i,:)';
    beta        = asin(vBEB(2)/norm(vBEB));
    aoa         = atan(vBEB(3)/vBEB(1));
    aoas(i,1)   = aoa;
    
    % FORCES AND MOMENTS --------------------------------- %
    qb = qbar(i); 
    FT = F_T(i); 
    Mach = Ma(i); 
    fd = fda(i);
    FMinput = MakeStruc(ad,aoa,Mach,fd,qb,FT);
    [f_apB, mBB] = GetFM(FMinput,1);
    f_spB       = (1./m(i)) * f_apB;
    
    % Gravity
    sBII        = T_DI'*sBED(i,:)' + sEII;
    g_G         = GetGravity(sBII,latc(i), ad.gravity_model);
    gs(i)       = norm(g_G);
    
    % Equations of motion -------------------------------- %
    vBEDdot(i,:) = (T_BD'*f_spB + T_DG * g_G -          ...
                    2*T_DI*R_EII*T_DI'*vBED(i,:)' -     ...
                    T_DI*R_EII*R_EII*sBII)';
    sBEDdot     = vBED(i,:);
    latdot      = sBEDdot(1)/Re0;
    londot      = sBEDdot(2)/Re(i);
    distdot     = sBEDdot(3);
    sdot(i,:)   = [latdot londot distdot];
    
    wBIBdot(i,:) = invMOI * (-R_BIB*MOI*wBIB(i,:)' + mBB);
    
    if pitch(i) == 0
        pitch(i) = 1e-5;
    end
    tryme = [1, sin(roll(i))*tan(pitch(i)), cos(roll(i))*tan(pitch(i));
             0, cos(roll(i)), -sin(roll(i));
             0, sin(roll(i))/cos(pitch(i)), cos(roll(i))/cos(pitch(i))];
    
    Eulsdot = tryme*wBIB(i,:)';
    rolldot(i) = Eulsdot(1);
    pitchdot(i) = Eulsdot(2);
    yawdot(i) = Eulsdot(3);

    if ad.altitude_hold == 1
        vBEG   = T_DG' * vBED(i,:)';
        [~,~,fpa(i)] = car2pol(vBEG);
    end
end

Euldot      = [rolldot pitchdot yawdot];

mdot        = -F_T./(g0.*Isp);

%-------------------------------------------------------------------%
%                 Construct GPOPS-II Dynamics output                %
%-------------------------------------------------------------------%
output.dynamics = [sdot vBEDdot wBIBdot mdot dfda dthr Euldot];

output.path     = [aoas,Ma];

if ad.altitude_hold == 1
    output.integrand = (P - ad.P_targ).^2;
    output.path     = [output.path, fpa'];
end


% Debugging --------------- %
plot(t,h);

end
