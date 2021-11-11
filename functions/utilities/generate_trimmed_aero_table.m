
auxdata = get_config('deck1.csv'); 

Ma = 4:1:8;
fda = -40:5:40;

[MACH, FDA] = meshgrid(Ma, fda);
AOA = zeros(size(MACH));
CL = zeros(size(MACH));
CD = zeros(size(MACH));
Cm = zeros(size(MACH));

for i = 1:size(MACH,1)
    for j = 1:size(MACH,2)
        AOA(i,j) = trim_aero(auxdata, MACH(i,j), FDA(i,j));
        [cl,cd,cm] = GetAero(auxdata, AOA(i,j), MACH(i,j), FDA(i,j));
        CL(i,j) = cl;
        CD(i,j) = cd;
        Cm(i,j) = cm;
    end
end

mach = reshape(MACH, size(AOA,1)*size(AOA, 2),1);
fda = reshape(FDA, size(AOA,1)*size(AOA, 2),1);
aoa = reshape(AOA, size(AOA,1)*size(AOA, 2),1);
cls = reshape(CL, size(AOA,1)*size(AOA, 2),1);
cds = reshape(CD, size(AOA,1)*size(AOA, 2),1);
cms = reshape(Cm, size(AOA,1)*size(AOA, 2),1);

writematrix([mach,fda,aoa,cls,cds,cms], 'trimmed-aero.txt');
