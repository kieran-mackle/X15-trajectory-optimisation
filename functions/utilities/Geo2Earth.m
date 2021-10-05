function [x,y,z] = Geo2Earth(sB_G)
h       = sB_G(:,1);
lat     = sB_G(:,2);
lon     = sB_G(:,3);

Re      = 6378145;
% will need to be modified for oblate earth model
% see http://journals.pan.pl/Content/98324/PDF/art05.pdf?handler=pdf

x       = (Re + h)*cos(lat)*cos(lon);
y       = (Re + h)*cos(lat)*sin(lon);
z       = (Re + h)*sin(lat);

end