function r = h2r(h,lat)
    r = h + geocradius(lat*180/pi);
end