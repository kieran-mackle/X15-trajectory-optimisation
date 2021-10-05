function g = GetGravity(s_BE,latc,method)
% returns gravity vector expressed in geocentric coordinates ]G

Re          = 6371005;
GM          = 3.986005e14;
we          = 7.292115e-5;
mu          = 3.9860e+14;
R_EI        = zeros(3,3);
R_EI(1,2)   = -we;
R_EI(2,1)   = we;

if method == 1
    % constant gravity model
    g = [0; 0; 9.81];
    
elseif method == 2
    % flat earth gravity model
    r = norm(s_BE);
    g = [0; 0; GM/r^2];
    
elseif method == 3
    % perfect sphere gravity model - incomplete
    g = GM*s_BE/(norm(s_BE)^3) - R_EI*R_EI*s_BE;
    
elseif method == 4
    % ellipsoid gravity model
    p = GM/(norm(s_BE)^2);
    a = 6378137;            % semi-major axis of Earth
    C = -4.841668e-4;       % second-degree zonal coefficient
    g = p*[-3*sqrt(5)*C*(a/norm(s_BE))^2 * sin(latc)*cos(latc);
           0;
           1 + (3/2)*sqrt(5)*C*(a/norm(s_BE))^2 * (3*sin(latc)^2 -1)];

elseif method == 5
    % fourth order zonal harmonics
    phi     = pi/2 - latc;
    J2      = 1.08263e-3;
    J3      = 2.532153e-7;
    J4      = 1.6109876e-7;
    v       = cos(phi);
    r       = norm(s_BE);
    P2      = (1/2).*(3.*v.^2 - 1);
    P3      = (1/2).*(5.*v.^3 - 3.*v);
    P4      = (1/8).*(35.*v.^4 - 30.*v.^2 + 3);

    gr      = -(mu./r.^2).*(1 - 3.*J2.*(Re./r).^2 .*P2 - 4.*J3  ...
              .*(Re./r).^3 .*P3 - 5.*J4.*(Re./r).^4 .*P4);

    gt      = 3.*(mu./r.^2).*(Re./r).^2 .*sin(phi).*cos(phi).*  ...
              (J2 + 0.5.*J3.*(Re./r).*sec(phi).*(5.*cos(phi).^2 ...
              - 1) + (5/6).*J4.*(Re./r).^2 .*(7.*cos(phi).^2    ...
              - 1));
    
    g       = [gt;0;-gr];
end

end

