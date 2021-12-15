function output = tensor6plant_to_euler6control(input)
% Mapping function from plant to control models
% For tensor6 to Euler cart6 dynamics models

% Initialise output with original input
output = input;

X = input.initial.state;
U = input.initial.control;

% Extract state variables
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

% Map state inputs to cartesian state inputs
North   = 0;    % arbitrary starting point
East    = 0;    % arbitrary starting point
Down    = dist; 

T_BG = TM_BG(roll, pitch, yaw);
vBE_B = T_BG * [uD; vD; wD];
u = vBE_B(1);
v = vBE_B(2);
w = vBE_B(3);

phi = roll;
theta = pitch;
psi = yaw;

mapped_state(1) = North;
mapped_state(2) = East;
mapped_state(3) = Down;
mapped_state(4) = u;
mapped_state(5) = v;
mapped_state(6) = w;
mapped_state(7) = p;
mapped_state(8) = q;
mapped_state(9) = r;
mapped_state(10) = phi;
mapped_state(11) = theta;
mapped_state(12) = psi;

mapped_state(13) = m;
mapped_state(14) = fda;
mapped_state(15) = thr;

% Update state
output.initial.state = mapped_state;

% Update models
output.control_model.dynamics = @cart6_euler;
output.control_model.output = @cart6_output;

