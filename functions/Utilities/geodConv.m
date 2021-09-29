function [lond,lat,alt] = geodConv(sBII,t,l_G0)
% CADGEO84 equivalent
% Determines the geodetic latitude and longitude and altitude given sBII
a       = 6378137.0;
f       = 3.33528106e-3;
we      = 7.29211585e-5;

% Iterate to solve geodetic latitude
count   = 0;
lat0    = 1;
dbi     = norm(sBII);
latg    = asin(sBII(3)/dbi);
lat     = latg;

while abs(lat-lat0) > 1e-7
    lat0    = lat;
    R0      = a*(1 - (f/2)*(1-cos(2*lat0)) + (5*(f^2)/16)*(1-cos(4*lat0)));
    alt     = dbi - R0;
    dd      = f*sin(2*lat0)*(1 - f/2 - alt/R0);
    lat     = latg+dd;
    count   = count+1;
    if count > 100
        break
    end
end

% Calculate the geodetic longitude
x       = sBII(1);
y       = sBII(2);
temp    = asin(sBII(2)/sqrt(x^2 + y^2));

% Resolve multi-valued asin function for each quadrant
Q1 = x >= 0 & y >= 0;
Q2 = x < 0  & y >= 0;
Q3 = x < 0  & y < 0;
Q4 = x >= 0 & y < 0;

if any(Q1)
    temp(Q1) = temp(Q1);
end
if any(Q2)
    temp(Q2) = pi - temp(Q2);
end
if any(Q3)
    temp(Q3) = pi - temp(Q3);
end
if any(Q4)
    temp(Q4) = 2*pi + temp(Q4);
end

lond = temp - l_G0 - we*t;


% modify sign (east positive, west negative)
if lond > pi
    lond = -(2*pi - lond);
end


end
