function post = cart6_post(auxdata, output, t)
% ====== Post Processing for Cartesian 6 DoF Flight Dynamics =======
%  A post-processing routine for the cartesian 6DoF flight dynamics
%  model.
% ==================================================================

auxdata = auxdata;
state = output.result.solution.phase.state;

% State variables
sBE_L = state(:,1:3);   % [N, E, D]
vBE_B = state(:,4:6);   % [u, v, w]

% Body rates
wBE_B = state(:,7:9);   % [p, q, r]
p = state(:,7);
q = state(:,8);
r = state(:,9);

% Quaternions
q0 = state(:,10);
q1 = state(:,11);
q2 = state(:,12);
q3 = state(:,13);




% Euler angles - not sure if required in the dynamics
% psi = atan( 2*(q1*q2 + q0*q3) / (q0.^2 + q1.^2 - q2.^2 - q3.^2) );
% theta = asin( -2 * (q1*q3 - q0*q2) );
% phi = atan( 2*(q2*q3 + q0*q1) / (q0.^2 - q1.^2 - q2.^2 - q3.^2) );

for i = 1:length(t)
    psi(i) = atan( 2*(q1(i)*q2(i) + q0(i)*q3(i)) / ...
               (q0(i).^2 + q1(i).^2 - q2(i).^2 - q3(i).^2) );
    theta(i) = asin( -2 * (q1(i)*q3(i) - q0(i)*q2(i)) );
    phi(i) = atan( 2*(q2(i)*q3(i) + q0(i)*q1(i)) / ...
               (q0(i).^2 - q1(i).^2 - q2(i).^2 - q3(i).^2) );
end


% Construct output struct
post.psi = psi;
post.theta = theta;
post.phi = phi;




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
