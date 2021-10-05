function Re = GetRe(latitude, model)
if model == 1
    Re = 6378137.0 * ones(size(latitude));
elseif model == 2
    a = 6378137.0;
    f = 3.33528106e-3;
    Re = a.*(1-(f/2).*(1-cos(2.*latitude)) + ...
              (5.*f.^2./16).*(1-cos(4.*latitude)));
else
end