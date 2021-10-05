function [h,lat,lon] = Earth2Geo(sBI_E)
x       = sBI_E(:,1);
y       = sBI_E(:,2);
z       = sBI_E(:,3);

dbi     = sqrt(x.^2 + y.^2 + z.^2);

lat     = asin(z./dbi);
h       = dbi - 6378145;
lon     = asin(y./sqrt(x.^2 + y.^2));

% resolve multi-valued asin function for each quadrant
Q1 = x >= 0 & y >= 0;
Q2 = x < 0 & y >= 0;
Q3 = x < 0 & y < 0;
Q4 = x >= 0 & y < 0;

if any(Q1)
    lon(Q1) = lon(Q1);
end
if any(Q2)
    lon(Q2) = pi - lon(Q2);
end
if any(Q3)
    lon(Q3) = pi - lon(Q3);
end
if any(Q4)
    lon(Q4) = 2*pi + lon(Q4);
end


% modify sign (east positive, west negative)
if lon > pi
    lon = -(2*pi - lon);
end


end