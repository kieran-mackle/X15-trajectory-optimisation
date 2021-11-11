
auxdata = get_config('deck1.csv'); 

Ma = 4:1:8;
fda = -40:5:40;

[MACH, FDA] = meshgrid(Ma, fda);
AOA = zeros(size(MACH));

for i = 1:size(MACH,1)
    for j = 1:size(MACH,2)
        AOA(i,j) = trim_aero(auxdata, MACH(i,j), FDA(i,j));
    end
end

mach = reshape(MACH, size(AOA,1)*size(AOA, 2),1);
fda = reshape(FDA, size(AOA,1)*size(AOA, 2),1);
aoa = reshape(AOA, size(AOA,1)*size(AOA, 2),1);

writematrix([mach,fda,aoa], 'trimmed-aero.txt');
