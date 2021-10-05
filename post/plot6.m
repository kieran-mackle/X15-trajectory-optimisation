% ================== Plot Optimisation results ==================== %
% This script plots the optimised trajectory from the GPOPS-II      %
% output and optionally compares it to the results of the Simulink  %
% forward simulation.                                               %
% ================================================================= %

%-------------------------------------------------------------------%
%                       Post-Processing Options                     %
%-------------------------------------------------------------------%
                % (Could package this into an input script)
% yes/no (1/0)
SaveFigures     = 0;
CloseFigures    = 0;
createGuess     = 0;

% Toggle 0/1 to specify what to plot
PlotTrajectory  = 0;
PlotGeographic  = 0;
PlotState       = 0;
PlotStateFS     = 0;
PlotAngles      = 0;
PlotControls    = 0;
PlotControlRates = 0;
PlotVelocity    = 0;
PlotMass        = 0;
PlotForwardSim  = 1;
PlotAero        = 0;
PlotForces      = 0;
PlotAccell      = 0;
PlotAll         = 1;
PlotMesh        = 0;
PlotFSonly      = 0;

%-------------------------------------------------------------------%
%                           Plot solution                           %
%-------------------------------------------------------------------%
close all;
t   = plotting.t;
ad  = auxdata;

% Trajectory overview --------------------------------------------- %
if PlotTrajectory == 1
    figure();
    clf;
    plot3(plotting.latd*180/pi,plotting.lond*180/pi,plotting.h/1000);
    title('Overview of vehicle trajectory');
    xlabel('Latitude (deg)');
    ylabel('Longitude (deg)');
    zlabel('Altitude (km)');
    xlim([-0.5,0.5]);
    ylim([-0.5,0.5]);
    zlim([min(plotting.h/1000),max(plotting.h/1000)]);
    hold on;
    box on; grid on;

    % Interpolate for even spacing
    Ni      = 1000;
    tt      = linspace(t(1),t(end),Ni)';
    hh      = interp1(t,plotting.h/1000,tt);
    la      = interp1(t,plotting.latd*180/pi,tt);
    lo      = interp1(t,plotting.lond*180/pi,tt);

    % Create matrices for lines
    Np      = 15;               % Number of lines
    spacing = round(Ni/Np);     
    plat    = la(1:spacing:end);
    plon    = lo(1:spacing:end);
    ph      = hh(1:spacing:end);

    % Plot lines on figure
    h_min   = min(plotting.h/1000);
    for i = 1:Np
        line([plat(i),plat(i)],[plon(i),plon(i)],[h_min,ph(i)]);
    end
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Trajectory-overview.png']));
    end
end

% Trajectory overview --------------------------------------------- %
if PlotGeographic == 1
    figure();
    clf;
    
    % Will be implemented for longer range missions.
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Geographic-overview.png']));
    end
end

% State overview -------------------------------------------------- %
if PlotState == 1
    figure(); clf;
    sp1 = tight_subplot(2,2,[.12 .1],[.08 .09],[.07 .08]);
    set(gcf, 'Position', [500,200,1000,700]);
    
    % Altitude Plot
    axes(sp1(1));
    hold on; grid on;
    plot(t, plotting.h/1e3, 'k-');
    title('Altitude Profile');
    xlabel('t (s)');
    ylabel('h (km)');
    
    % Velocity
    axes(sp1(2));
    hold on; grid on;
%     plot(t, plotting.V, 'k-');
    title('Velocity Profile');
    xlabel('t (s)');
%     ylabel('V (m/s)');
%     yyaxis right;
    plot(t, plotting.Ma, 'k-');
    ylabel('Mach number', 'Color', 'k');
    set(gca,'YColor',[0 0 0]);
    
    % Flight path
    axes(sp1(3));
    hold on; grid on;
    plot(t, plotting.fpa*180/pi, 'k-');
    title('Flight Path Angle Profile');
    xlabel('t (s)');
    ylabel('\gamma (deg)');
    
    % Flight path
    axes(sp1(4));
    hold on; grid on;
    plot(t, plotting.aoa*180/pi, 'k-');
    title('Angle of Attack Profile');
    xlabel('t (s)');
    ylabel('\alpha (deg)');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'State-overview.png']));
    end
end

% State overview with forward simulation results ------------------ %
if PlotStateFS == 1
    fs = FSResults;
    fst = fs.h.time;
    
    % Need to reconstruct the fpa from the FS:
    FS_vBEG  = [fs.w_ECEF.data, fs.v_ECEF.data, -fs.u_ECEF.data];
    for ii = 1:length(fst)
        [~,~,FS_fpa(ii)] = car2pol(FS_vBEG(ii,:));
    end
    
    figure(); clf;
    sp1 = tight_subplot(2,2,[.12 .1],[.08 .09],[.07 .08]);
    set(gcf, 'Position', [500,200,1000,700]);
    set(gcf,'color','w');
    
    % Altitude Plot
    axes(sp1(1));
    hold on; grid on;
    plot(t, plotting.h/1e3, 'k-');
    title('Altitude Profile');
    plot(fst, fs.h.data*1e-3);
    xlabel('t (s)');
    ylabel('h (km)');
    legend('Optimal Trajectory', 'Forward Simulation', 'Location', 'best');
    
    % Velocity
    axes(sp1(2));
    hold on; grid on;
%     plot(t, plotting.V, 'k-');
%     plot(fst, fs.v.data, 'r-');
%     title('Velocity Profile');
%     xlabel('t (s)');
%     ylabel('V (m/s)');
%     yyaxis right;
    title('Mach Number Profile');
    plot(t, plotting.Ma, 'k-');
    plot(fst, fs.Ma.data, 'r-');
    ylabel('Mach number', 'Color', 'k');
    set(gca,'YColor',[0 0 0]);
    legend('Optimal Trajectory', 'Forward Simulation', 'Location', 'southeast');
    
    % Flight path
    axes(sp1(3));
    hold on; grid on;
    plot(t, plotting.fpa*180/pi, 'k-');
    plot(fst, FS_fpa*180/pi);
    title('Flight Path Angle Profile');
    xlabel('t (s)');
    ylabel('\gamma (deg)');
    legend('Optimal Trajectory', 'Forward Simulation', 'Location', 'best');
    
    % Angle of attack 
    axes(sp1(4));
    hold on; grid on;
    plot(t, plotting.aoa*180/pi, 'k-');
    plot(fst, fs.aoa.data*180/pi);
    title('Angle of Attack Profile');
    xlabel('t (s)');
    ylabel('\alpha (deg)');
    legend('Optimal Trajectory', 'Forward Simulation', 'Location', 'best');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'FS-state-overview.png']));
    end
end

% Angles overview ------------------------------------------------- %
if PlotAngles == 1
    figure(); clf;
    sp1 = tight_subplot(2,2,[.13 .09],[.09 .09],[.07 .04]);
    set(gcf, 'Position', [450,200,1000,700]);
    sgtitle('Vehicle Incidence Angles Profile');
    
    % Angle of attack Plot
    axes(sp1(1));
    hold on; grid on;
    plot(t, plotting.aoa*180/pi, 'k-');
    title('Angle of Attack Profile');
    xlabel('t (s)');
    ylabel('\alpha (deg)');
    
    % Pitch
    axes(sp1(2));
    hold on; grid on;
    plot(t, plotting.pitch, 'k-');
    title('Pitch Profile');
    xlabel('t (s)');
    ylabel('\theta (deg)');
    
    % Flight path
    axes(sp1(3));
    hold on; grid on;
    plot(t, plotting.fpa*180/pi, 'k-');
    title('Flight Path Angle Profile');
    xlabel('t (s)');
    ylabel('\gamma (deg)');
    
    % Heading angle
    axes(sp1(4));
    hold on; grid on;
    plot(t, plotting.hda*180/pi, 'k-');
    title('Heading Angle Profile');
    xlabel('t (s)');
    ylabel('\chi (deg)');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Angles-overview.png']));
    end
end

% Optimal Controls ------------------------------------------------ %
if PlotControls == 1
    figure();
    clf;
    sgtitle('Optimal Controls');
    sp1 = tight_subplot(1,2,[.1 .1],[.1 .14],[.09 .04]);
    set(gcf, 'Position', [401,266,841,468]);

    axes(sp1(1));
    plot(t,plotting.fda*180/pi);
    grid on;
    xlabel('Time (s)');
    ylabel('\delta (deg)');
    title('Flap deflection angle');

    axes(sp1(2));
    plot(t,100*plotting.thr);
    grid on;
    xlabel('Time (s)');
    ylabel('\delta_T (%)');
    title('Throttle setting');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Optimal-controls.png']));
    end
end

% Optimal Control Rates ------------------------------------------- %
if PlotControlRates == 1
    figure();
    clf;
    sgtitle('Optimal Control Rates');
    sp1 = tight_subplot(1,2,[.1 .1],[.1 .14],[.09 .04]);
    set(gcf, 'Position', [401,266,841,468]);

    axes(sp1(1));
    plot(t,plotting.dfda*180/pi);
    grid on;
    xlabel('Time (s)');
    ylabel('\delta (deg)');
    title('Flap deflection rate');

    axes(sp1(2));
    plot(t,100*plotting.dthr);
    grid on;
    xlabel('Time (s)');
    ylabel('\delta_T (%)');
    title('Throttle setting rate');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Optimal-controls.png']));
    end
end


% Speed Profile --------------------------------------------------- %
if PlotVelocity == 1
    figure();
    clf;
    sgtitle('Speed Profile');
    sp2 = tight_subplot(1,3,[.1 .1],[.1 .14],[.04 .04]);
    set(gcf, 'Position', [225,580,1020,389]);

    axes(sp2(1));
    plot(t,plotting.Ma);
    grid on;
    xlabel('Time (s)');
    title('Mach Number');
 
    axes(sp2(2));
    plot(t,plotting.aoa*180/pi);
    grid on;
    xlabel('Time (s)');
    title('Angle of Attack (deg)');

    axes(sp2(3));
    plot(t,plotting.fpa*180/pi);
    grid on;
    xlabel('Time (s)');
    title('Flight-path angle (deg)');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Velocity-profile.png']));
    end
end

% Mass Profile ---------------------------------------------------- %
if PlotMass == 1
    figure();
    plot(t,plotting.m);
    grid on;
    title('Mass profile');
    xlabel('Time (s)');
    ylabel('Mass (kg)');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Mass-profile.png']));
    end
end

% Forward Simulation Results Comparison --------------------------- %
if PlotForwardSim == 1
    fs = FSResults;
    fst = fs.h.time;
    
    figure();
    sgtitle('Forward Simulation Results Comparison');
    fsa = tight_subplot(3,3,[.07 .06],[.08 .14],[.04 .04]);
    set(gcf, 'Position', [62,38,1838,869]);
    
    % Postion results
    axes(fsa(1));
    hold on;
    title('Altitude Comparison');
    plot(t,plotting.h*1e-3);
    plot(fst,fs.h.data*1e-3);
    ylabel('h (km)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    axes(fsa(4));
    hold on;
    title('Latitude Comparison');
    plot(t,plotting.latd*180/pi);
    plot(fst,fs.lat.data);
    ylabel('\lambda (deg)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    axes(fsa(7));
    hold on;
    title('Longitude Comparison');
    plot(t,plotting.lond*180/pi);
    plot(fst,fs.lon.data);
    xlabel('Time (s)')
    ylabel('\phi (deg)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    % Velocity results
    axes(fsa(2));
    hold on;
    title('Geographic Velocity');
    plot(t,plotting.V);
    plot(fst,fs.v.data);
    ylabel('V (m/s)');
    yyaxis right
    plot(t,plotting.Ma, '--');
    plot(fst,fs.Ma.data, '--');
    ylabel('Mach');
    grid on;
    legend('GPOPS-II Velocity','Forward Simulation Velocity',...
        'GPOPS-II Mach number','Forward Simulation Mach number', ...
        'Location','best');
    
    axes(fsa(5));
    hold on;
    title('Angle of Attack');
    plot(t,plotting.aoa*180/pi);
    plot(fst,fs.aoa.data*180/pi);
    xlabel('Time (s)')
    ylabel('\alpha (deg)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    axes(fsa(8));
    hold on;
    title('Sideslip Angle');
    plot(t,plotting.beta*180/pi);
    plot(fst,fs.beta.data*180/pi);
    ylabel('\beta (deg)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    % Euler angle results
    axes(fsa(3));
    hold on;
    title('Roll Angle');
    plot(t,plotting.roll*180/pi);
    plot(fst,fs.roll.data*180/pi);
    ylabel('\phi (deg)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    axes(fsa(6));
    hold on;
    title('Pitch Angle');
    plot(t,plotting.pitch*180/pi);
    plot(fst,fs.pitch.data*180/pi);
    ylabel('\theta (deg)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    axes(fsa(9));
    hold on;
    title('Yaw Angle');
    plot(t,plotting.yaw*180/pi);
    plot(fst,fs.yaw.data*180/pi);
    xlabel('Time (s)')
    ylabel('\psi (deg)');
    grid on;
    legend('GPOPS-II Result','Forward Simulation','Location','best');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Opt-vs-FS.png']));
    end
    
end

% Aero coefficient overview --------------------------------------- %
if PlotAero == 1
    [CL, CD, Cm] = GetAero(auxdata, plotting.aoa*180/pi,    ...
                           plotting.Ma, plotting.fda*180/pi);
    
    figure(); clf;
    sp1 = tight_subplot(2,3,[.1 .06],[.09 .14],[.06 .04]);
    set(gcf, 'Position', [200,200,1500,700]);
    sgtitle('Aerodynamic Coefficient Profile');
    
    % Angle of attack Plot
    axes(sp1(1));
    hold on; grid on;
    plot(t, plotting.aoa*180/pi, 'k-');
    title('Angle of Attack Profile');
    xlabel('t (s)');
    ylabel('\alpha (deg)');
    
    % Mach Number Plot
    axes(sp1(2));
    hold on; grid on;
    plot(t, plotting.Ma, 'k-');
    title('Mach Number Profile');
    xlabel('t (s)');
    ylabel('Mach ');
    
    % FDA Plot
    axes(sp1(3));
    hold on; grid on;
    plot(t, plotting.fda*180/pi, 'k-');
    title('Flap Deflection Profile');
    xlabel('t (s)');
    ylabel('\delta (deg) ');
    
    % CL Plot
    axes(sp1(4));
    hold on; grid on;
    plot(t, CL, 'k-');
    title('Lift Coefficient Profile');
    xlabel('t (s)');
    ylabel('C_L');
    
    % CD Plot
    axes(sp1(5));
    hold on; grid on;
    plot(t, CD, 'k-');
    title('Drag Coefficient Profile');
    xlabel('t (s)');
    ylabel('C_D');
    
    % Cm Plot
    axes(sp1(6));
    hold on; grid on;
    plot(t, Cm, 'k-');
    title('Pitching Moment Coefficient Profile');
    xlabel('t (s)');
    ylabel('C_m');
%     ylim([-0.04297, 0.03365]);
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Aero-overview.png']));
    end
end

% Body Forces overview -------------------------------------------- %
if PlotForces == 1
    xtrack  = plotting.xtrack * 1e-3;
    ytrack  = plotting.ytrack * 1e-3;
    h       = plotting.h * 1e-3;
    
    figure();
    clf;
    sgtitle('Forces Profile - INCOMPLETE');
    sp2 = tight_subplot(2,3,[.1 .1],[.1 .14],[.04 .04]);
    set(gcf, 'Position', [225,580,1500,900]);
    
    axes(sp2(1));
    plot3(xtrack, ytrack, h, 'k-');
    title('Overview of vehicle trajectory');
    xlabel('North (km)');
    ylabel('East (km)');
    zlabel('Altitude (km)');
    hold on;
    box on; grid on;

    SF      = 0.01;
    N       = 20;
    space   = round(length(t)/N);
    ix      = 0;

    for i = 1:length(t)
        ix = ix + 1;

        if mod(ix, space) == 0
            fx  = plotting.f_spD(ix,1);
            fy  = plotting.f_spD(ix,2);
            fz  = plotting.f_spD(ix,3);
            quiver3(xtrack(ix), ytrack(ix), h(ix), fx, fy, -fz, 'r', ...
                    'LineWidth', 1.1, 'AutoScaleFactor', SF);
        end

    end
    
    axes(sp2(2));
    hold on; grid on;
    plot(ytrack, h, 'k-');
    title('Overview of vehicle trajectory');
    xlabel('East (km)');
    ylabel('Altitude (km)');

    ix      = 0;
    for i = 1:length(t)
        ix = ix + 1;

        if mod(ix, space) == 0
            fx  = plotting.f_spD(ix,1);
            fy  = plotting.f_spD(ix,2);
            fz  = plotting.f_spD(ix,3);

            fya = plotting.Fsp_aD(ix,2);
            fza = plotting.Fsp_aD(ix,3);

            fyp = plotting.Fsp_pD(ix,2);
            fzp = plotting.Fsp_pD(ix,3);
            
            gz  = plotting.g_NED(ix,3);

            quiver(ytrack(ix), h(ix), fy, -fz, 'k-', ...
                    'LineWidth', 1, 'AutoScaleFactor', SF);
            quiver(ytrack(ix), h(ix), fyp, -fzp, 'r-', ...
                    'LineWidth', 1, 'AutoScaleFactor', SF);
            quiver(ytrack(ix), h(ix), fya, -fza, 'b-', ...
                    'LineWidth', 1, 'AutoScaleFactor', SF);
            quiver(ytrack(ix), h(ix), 0, -gz, 'g-', ...
                    'LineWidth', 1, 'AutoScaleFactor', SF);
        end

    end
    
    axes(sp2(3));
    hold on; grid on;
    plot(t, plotting.f_spB(:,1));
    plot(t, plotting.f_spB(:,2));
    plot(t, plotting.f_spB(:,3));
    title('Specific forces in body coordinates');
    xlabel('t (s)');
    ylabel('f_{sp} (m/s)');
    legend("[f_{sp_x}]^B","[f_{sp_y}]^B","[f_{sp_z}]^B");
    
    axes(sp2(6));
    hold on; grid on;
    plot(t, plotting.f_spD(:,1));
    plot(t, plotting.f_spD(:,2));
    plot(t, plotting.f_spD(:,3));
    title('Specific forces in geodetic coordinates');
    xlabel('t (s)');
    ylabel('f_{sp} (m/s)');
    legend("[f_{sp_x}]^B","[f_{sp_y}]^B","[f_{sp_z}]^B");
    
    
    axes(sp2(4));
    hold on; grid on;
    plot(t, plotting.aBED(:,1));
    plot(t, plotting.aBED(:,2));
    plot(t, plotting.aBED(:,3));
    title('Vehicle Acceleration in Geodetic Coords');
    xlabel('t (s)');
    ylabel('a (m/s)');
    legend("1^D","2^D","3^D");
    
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Force-overview.png']));
    end
end

% Speed Profile --------------------------------------------------- %
if PlotAccell == 1
    figure();
    clf;
    sgtitle('Acceleration Profile');
    sp2 = tight_subplot(2,2,[.12 .1],[.08 .09],[.07 .08]);
    set(gcf, 'Position', [500,200,1000,700]);
    % Specific force in body and geodetic
    % Acceleration in geodetic, gravity in geodetic
    
    axes(sp2(1));
    hold on; grid on;
    plot(t, plotting.f_spB(:,1));
    plot(t, plotting.f_spB(:,2));
    plot(t, plotting.f_spB(:,3));
    title('Specific forces in body coordinates');
    xlabel('t (s)');
    ylabel('f_{sp} (m/s)');
    legend("[f_{sp_x}]^B","[f_{sp_y}]^B","[f_{sp_z}]^B",'Location', 'best');

    axes(sp2(2));
    hold on; grid on;
    plot(t, plotting.f_spD(:,1));
    plot(t, plotting.f_spD(:,2));
    plot(t, plotting.f_spD(:,3));
    title('Specific forces in geodetic coordinates');
    xlabel('t (s)');
    ylabel('f_{sp} (m/s)');
    legend("[f_{sp_x}]^D","[f_{sp_y}]^D","[f_{sp_z}]^D",'Location', 'best');

    axes(sp2(3));
    hold on; grid on;
    plot(t, plotting.g_NED(:,1));
    title('Gravitational acceleration');
    xlabel('t (s)');
    ylabel('g (m/s)');
    yyaxis right;
    plot(t, plotting.g_NED(:,3));
    legend("g_{\phi}","g_{r}",'Location', 'best');
    
    axes(sp2(4));
    hold on; grid on;
    plot(t, plotting.aBED(:,1)/9.81);
    plot(t, plotting.aBED(:,2)/9.81);
    plot(t, plotting.aBED(:,3)/9.81);
    title('Vehicle Acceleration in Geodetic Coords');
    xlabel('t (s)');
    ylabel("a (g's)");
    legend("1^D","2^D","3^D",'Location', 'best');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Acceleration-profile.png']));
    end
end

% Full overview -------------------------------------------------- %
if PlotAll == 1
    fs = FSResults;
    fst = fs.h.time;
    
    figure();
    clf;
    sgtitle('Trajectory Overview');
    sp1 = tight_subplot(3,5,[.07 .06],[.05 .1],[.03 .03]);
    set(gcf, 'Position', [300, 30, 1600, 900]);

    % Position
    axes(sp1(1));
    hold on; grid on;
    plot(t, plotting.h/1e3);
    title('Altitude (km)');

    axes(sp1(6)); 
    hold on; grid on;
    plot(t, plotting.latd*180/pi);
    title('Geodetic latitude (deg)');

    axes(sp1(11));
    hold on; grid on;
    plot(t, plotting.lond*180/pi);
    title('Geodetic longitude (deg)');

    % Velocity
    axes(sp1(2));
    hold on; grid on;
    plot(t, plotting.V);
    title('Velocity (m/s)');

    axes(sp1(7));
    hold on; grid on;
    plot(t, plotting.fpa*180/pi);
    title('FPA (deg)');

    axes(sp1(12));
    hold on; grid on;
    plot(t, plotting.hda*180/pi);
    title('HDA (deg)');

    % Body rates
    axes(sp1(3));
    hold on; grid on;
    plot(t, plotting.p*180/pi);
    title('p (deg/s)');
    ytickformat('%.2f');

    axes(sp1(8));
    hold on; grid on;
    plot(t, plotting.q*180/pi);
    title('q (deg/s)');

    axes(sp1(13));
    hold on; grid on;
    plot(t, plotting.r*180/pi);
    title('r (deg/s)');
    ytickformat('%.2f');

    % Euler angles
    axes(sp1(4));
    hold on; grid on;
    plot(t, plotting.roll*180/pi);
    title('Roll (deg)');

    axes(sp1(9));
    hold on; grid on;
    plot(t, plotting.pitch*180/pi);
    title('Pitch (deg)');

    axes(sp1(14));
    hold on; grid on;
    plot(t, plotting.yaw*180/pi);
    title('Yaw (deg)');

    % Extra
    axes(sp1(15));
    hold on; grid on;
    plot(t, plotting.fda*180/pi);
    title('FDA (deg)');

    axes(sp1(10));
    hold on; grid on;
    plot(t, plotting.thr*auxdata.thrust);
    title('Thrust (N)');

    axes(sp1(5));
    hold on; grid on;
    plot(t, plotting.aoa*180/pi);
    title('AoA (deg)');
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'Full-overview.png']));
    end
end


% Aero coefficient overview --------------------------------------- %
if PlotMesh == 1
    tf = t(end);
    figure(); clf;
    sp1 = tight_subplot(1,2,[.1 .06],[.1 .14],[.06 .04]);
    set(gcf, 'Position', [500,300,1000,500]);
    sgtitle('Mesh History');
    
    axes(sp1(1));
    for i=1:length(mesh)
        pp = plot(mesh(i).meshPoints*tf,mesh(i).iteration,'bo');
        set(pp,'LineWidth',1.25);
        hold on;
    end
    xlabel('Mesh Point Location');
    ylabel('Mesh Iteration');
    grid on;

    axes(sp1(2));
    for i=1:length(mesh)
        pp = plot(mesh(i).time, mesh(i).iterationTime,'bo');
        set(pp,'LineWidth', 1.25);
        hold on;
    end
    xlabel('Collocation Point Location');
    ylabel('Mesh Iteration');
    grid on;
    
    
    if SaveFigures == 1
    saveas(gcf,join([folder,ad.name,'AeroCoefficient-overview.png']));
    end
end

% Close figures --------------------------------------------------- %
if CloseFigures == 1
    close all;
end

% Create new guess for re-run ------------------------------------- %
if createGuess == 1
    solution            = output.result.solution.phase;
    newguess.t          = solution.time;
    newguess.state      = solution.state;
    newguess.control    = solution.control;
    save('NewGuess.mat', 'newguess');
end
