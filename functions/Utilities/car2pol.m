function [V,hda,fpa] = car2pol(VBEG)
% returns angles in radians

V       = norm(VBEG);
hda     = atan2(VBEG(2), VBEG(1));
fpa     = atan2(-VBEG(3), sqrt(VBEG(1)^2 + VBEG(2)^2));

end
