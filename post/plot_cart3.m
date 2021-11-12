function plot_cart3(input)

i = 3;
j = 2;

figure(1); clf;
set(gcf,'color','w');
subplot(i,j,1);
hold on; grid on;
title("Trajectory");
plot(input.t, input.h*1e-3);
xlabel('time (s)');
ylabel('h (km)');

subplot(i,j,2);
hold on; grid on;
title('Ground path');
plot(input.E, input.N);
xlabel('East (m)');
ylabel('North (m)');
axis equal;

subplot(i,j,3);
hold on; grid on;
title('Flight path');
plot(input.t, input.fpa*180/pi);
xlabel('time (s)');
ylabel('\gamma (deg)');
axis equal;

subplot(i,j,4);
hold on; grid on;
title('Mach number');
plot(input.t, input.Ma);
xlabel('time (s)');
ylabel('Ma');

subplot(i,j,5);
hold on; grid on;
title('Angle of Attack');
plot(input.t, input.aoa*180/pi);
xlabel('time (s)');
ylabel('\alpha (deg)');
axis equal;

subplot(i,j,6);
hold on; grid on;
title('Flap Deflection Angle');
plot(input.t, input.fda*180/pi);
xlabel('time (s)');
ylabel('\delta_f (deg)');
axis equal;


