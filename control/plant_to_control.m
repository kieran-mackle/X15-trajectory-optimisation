function output = plant_to_control(input)
% Mapping function from plant to control models
% For tensor6 to cart3 dynamics models

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
vNorth = uD;
vEast = vD;
vDown = wD;


mapped_state(1) = North;
mapped_state(2) = East;
mapped_state(3) = Down;
mapped_state(4) = vNorth;
mapped_state(5) = vEast;
mapped_state(6) = vDown;
mapped_state(7) = m;
mapped_state(8) = fda;
mapped_state(9) = thr;

output.initial.state = mapped_state;
