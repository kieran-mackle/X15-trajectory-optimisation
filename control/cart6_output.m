function output = cart6_output(input)


sBE_L = input(:,1:3);
vBE_B = input(:,4:6);

% Euler angles
phi = input(:,10);
theta = input(:,11);
psi = input(:,12);

fda = input(:,14);
thr = input(:,15);

Re0 = geocradius(0);
h = -sBE_L(:,3) - Re0;

R = 287.053;
gamma = 1.4;
[T,~,~] = GetAtmo(h);
a = sqrt(gamma.*R.*T);
V = sqrt(sum(vBE_B.^2,2));
Ma = V./a;

vBE_G = TM_BG(phi, theta, psi)' * vBE_B';
[~,~,fpa] = car2pol(vBE_G);

output = [h, fda, thr, Ma, fpa];