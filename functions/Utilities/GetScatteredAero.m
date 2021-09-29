function [CL,CD,Cm] = GetScatteredAero(aoa,Ma,fda)

temp            = importdata('C1AeroDeck.csv');
dat             = sortrows(temp.data,2);
data            = sortrows(dat,3);

F1 = scatteredInterpolant(data(:,1),data(:,2),data(:,3),data(:,4));
F2 = scatteredInterpolant(data(:,1),data(:,2),data(:,3),data(:,5));
F3 = scatteredInterpolant(data(:,1),data(:,2),data(:,3),data(:,6));

CL = F1(aoa,Ma,fda);
CD = F2(aoa,Ma,fda);
Cm = F3(aoa,Ma,fda);

end