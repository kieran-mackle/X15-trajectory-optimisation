function output = cart3_output(input)

fda = input(8);
thr = input(9);

% Constants
Re = geocradius(0); 
R = 287.053;
gamma = 1.4;

sBE_L   = input(1:3);
vBE_L   = input(4:6);

h           = -sBE_L(:,3) - Re;
[T,~,~]   = GetAtmo(h);
a           = sqrt(gamma.*R.*T);

[V,~,fpa] = car2pol(vBE_L);
Ma          = V/a;

output = [h, fda, thr, Ma, fpa];
