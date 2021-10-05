function distance = haversine(lat1, lon1, lat2, lon2)
%HAVERSINE An implementation of the Haversine forumla to calculate
% the distance between two points specified by latitude and longitude.
% Angles must be provided in radians.
% Distance returned in meters.
a = 0.5 - cos(lat2-lat1)/2 + cos(lat1) * cos(lat2) * ...
             (1-cos(lon2-lon1))/2;
distance = 12742e3 * asin(sqrt(a));
