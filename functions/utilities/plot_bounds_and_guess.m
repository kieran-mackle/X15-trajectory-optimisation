function plot_bounds_and_guess(bounds, guess, auxdata)



% Constants
Re = auxdata.Re0; 
S = auxdata.S;  
max_thrust = auxdata.thrust; 
R = auxdata.R; 
gamma = auxdata.gamma;
rad2deg = 180/pi;


% ----------------------------------------------------------------- %
%                       Extract Initial Guess                       %
%------------------------------------------------------------------ %
t = guess.phase.time;

state = guess.phase.state;
N = state(:,1);
E = state(:,2);
D = state(:,3);
vN = state(:,4);
vE = state(:,5);
vD = state(:,6);
m = state(:,7);
fda = state(:,8);
thr = state(:,9);

h = -D - Re;




% ----------------------------------------------------------------- %
%                           Extract Bounds                          %
%------------------------------------------------------------------ %

% MINIMUM BOUNDS
N0min = bounds.phase.initialstate.lower(:,1);
E0min = bounds.phase.initialstate.lower(:,2);
D0min = bounds.phase.initialstate.lower(:,3);
vN0min = bounds.phase.initialstate.lower(:,4);
vE0min = bounds.phase.initialstate.lower(:,5);
vD0min = bounds.phase.initialstate.lower(:,6);
m0min = bounds.phase.initialstate.lower(:,7);
fda0min = bounds.phase.initialstate.lower(:,8);
thr0min = bounds.phase.initialstate.lower(:,9);

Nmin = bounds.phase.state.lower(:,1);
Emin = bounds.phase.state.lower(:,2);
Dmin = bounds.phase.state.lower(:,3);
vNmin = bounds.phase.state.lower(:,4);
vEmin = bounds.phase.state.lower(:,5);
vDmin = bounds.phase.state.lower(:,6);
mmin = bounds.phase.state.lower(:,7);
fdamin = bounds.phase.state.lower(:,8);
thrmin = bounds.phase.state.lower(:,9);

Nfmin = bounds.phase.finalstate.lower(:,1);
Efmin = bounds.phase.finalstate.lower(:,2);
Dfmin = bounds.phase.finalstate.lower(:,3);
vNfmin = bounds.phase.finalstate.lower(:,4);
vEfmin = bounds.phase.finalstate.lower(:,5);
vDfmin = bounds.phase.finalstate.lower(:,6);
mfmin = bounds.phase.finalstate.lower(:,7);
fdafmin = bounds.phase.finalstate.lower(:,8);
thrfmin = bounds.phase.finalstate.lower(:,9);

% MAXIMUM BOUNDS
N0max = bounds.phase.initialstate.upper(:,1);
E0max = bounds.phase.initialstate.upper(:,2);
D0max = bounds.phase.initialstate.upper(:,3);
vN0max = bounds.phase.initialstate.upper(:,4);
vE0max = bounds.phase.initialstate.upper(:,5);
vD0max = bounds.phase.initialstate.upper(:,6);
m0max = bounds.phase.initialstate.upper(:,7);
fda0max = bounds.phase.initialstate.upper(:,8);
thr0max = bounds.phase.initialstate.upper(:,9);

Nmax = bounds.phase.state.upper(:,1);
Emax = bounds.phase.state.upper(:,2);
Dmax = bounds.phase.state.upper(:,3);
vNmax = bounds.phase.state.upper(:,4);
vEmax = bounds.phase.state.upper(:,5);
vDmax = bounds.phase.state.upper(:,6);
mmax = bounds.phase.state.upper(:,7);
fdamax = bounds.phase.state.upper(:,8);
thrmax = bounds.phase.state.upper(:,9);

Nfmax = bounds.phase.finalstate.upper(:,1);
Efmax = bounds.phase.finalstate.upper(:,2);
Dfmax = bounds.phase.finalstate.upper(:,3);
vNfmax = bounds.phase.finalstate.upper(:,4);
vEfmax = bounds.phase.finalstate.upper(:,5);
vDfmax = bounds.phase.finalstate.upper(:,6);
mfmax = bounds.phase.finalstate.upper(:,7);
fdafmax = bounds.phase.finalstate.upper(:,8);
thrfmax = bounds.phase.finalstate.upper(:,9);

h0min = -D0min - Re;
h0max = -D0max - Re;
hmin = -Dmin - Re;
hmax = -Dmax - Re;
hfmin = -Dfmin - Re;
hfmax = -Dfmax - Re;

i = 3;
j = 2;

figure(3); clf;
set(gcf,'color','w');

% subplot(i,j,1);
hold on; grid on;
title("Altitude");
plot(t, h*1e-3, 'k-')
xlabel('time (s)');
ylabel('h (km)');
plot(t, hmin*ones(size(t))*1e-3, 'r--');
plot(t, hmax*ones(size(t))*1e-3, 'r*--');
scatter(0, h0min*1e-3, 'ko');
scatter(0, h0max*1e-3, 'k^');
scatter(t(end), hfmin*1e-3, 'bo');
scatter(t(end), hfmax*1e-3, 'b^');
legend('Guess', 'Minimum', 'Maximum', 'Location', 'Best');



