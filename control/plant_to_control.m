function output = plant_to_control(input)
% Mapping function from plant to control models

% For tensor6 to cart3 dynamics models

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



T_DG        = TM_DG(delta(i));
T_BG        = TM_BG(roll(i),pitch(i),yaw(i));
T_BD        = T_BG * T_DG';
vBEB        = T_BD * vBED(i,:)';
beta        = asin(vBEB(2)/norm(vBEB));
aoa         = atan(vBEB(3)/vBEB(1));

output = input;