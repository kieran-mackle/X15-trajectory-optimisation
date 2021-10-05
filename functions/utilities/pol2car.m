function vBE_G = pol2car(V,hda,fpa)

vBE_G   = V * [cos(fpa)*cos(hda);
               cos(fpa)*sin(hda);
               -sin(fpa)];

end
