function [T,P,rho] = GetAtmo(h)
% GetAtmo retrieves atmospheric data based on the US 1976 Standard
% Atmosphere.
% 
% [T, P, rho] = GetAtmo(h)
% 
% in:     h       geopotential altitude (m)
% 
% out:    T       atmospheric temperature (K)
%         P       atmospheric pressure (Pa)
%         rho     atmospheric density (kg/m^3)
% 
% Limits: 
%   - Geopotential altitude (h) = 864 km
%   - Geocentric altitude (z) = 1000 km

R_e = 6378145;
R = 287.053;

z = (R_e .* h./(R_e - h))./1000;

% Pre-allocate memory
T = zeros(size(z));
P = zeros(size(z));
rho = zeros(size(z));
zeta = zeros(size(z));

P_cs = [86-91	0.000000	2.159582E-06	-4.836957E-04	-0.1425192	13.47530
91-100	0.000000	3.304895E-05	-0.009062730	0.6516698	-11.03037
100-110	0.000000	6.693926E-05	-0.01945388	1.719080	-47.75030
110-120	0.000000	-6.539316E-05	0.02485568	-3.223620	135.9355
120-150	2.283506E-07	-1.343221E-04	0.02999016	-3.055446	113.5764
150-200	1.209434E-08	-9.692458E-06	0.003002041	-0.4523015	19.19151
200-300	8.113942E-10	-9.822568E-07	4.687616E-04	-0.1231710	3.067409
300-500	9.814674E-11	-1.654439E-07	1.148115E-04	-0.05431334	-2.011365
500-750	-7.835161E-11	1.964589E-07	-1.657213E-04	0.04305869	-14.77132
750-1000	2.813255E-11	-1.120689E-07	1.695568E-04	-0.1188941	14.56718];

r_cs = [86-91	0.000000	-3.322622E-06	9.111460E-04	-0.2609971	5.944694
91-100	0.000000	2.873405E-05	-0.008492037	0.6541179	-23.62010
100-110	-1.240774E-05	0.005162063	-0.8048342	55.55996	-1443.338
110-120	0.00000	-8.854164E-05	0.03373254	-4.390837	176.5294
120-150	3.661771E-07	-2.154344E-04	0.04809214	-4.884744	172.3597
150-200	1.906032E-08	-1.527799E-05	0.004724294	-0.6992340	20.50921
200-300	1.199282E-09	-1.451051E-06	6.910474E-04	-0.1736220	-5.321644
300-500	1.140564E-10	-2.130756E-07	1.570762E-04	-0.07029296	-12.89844
500-750	8.105631E-12	-2.358417E-09	-2.635110E-06	-0.01562608	-20.02246
750-1000	-3.701195E-12	-8.608611E-09	5.118829E-05	-0.06600998	-6.137674];

% Define Conditions
% Lower Atmosphere - the values here are for h, need to do a conversion
a = z>=0 & z<=11.0190;
b = z>11.0190 & z<=20.0629;
c = z>20.0629 & z<=32.1614;
d = z>32.1614 & z<=47.3489;
e = z>47.3489 & z<=51.4111;
f = z>51.4111 & z<=71.7993;
g = z>71.7993 & z<=86;

% Upper Atmosphere
aa = z>86 & z<=91;
bb = z>91 & z<=100;
cc = z>100 & z<=110;
dd = z>110 & z<=120;
ee = z>120 & z<=150;
ff = z>150 & z<=200;
gg = z>200 & z<=300;
hh = z>300 & z<=500;
ii = z>500 & z<=750;
jj = z>750 & z<=1000;

if any(a)
    T(a) = 288.15 - 6.5.*h(a)./1000;
    P(a) = 101325.0 .* (288.15 ./ (288.15 - 6.5 .* h(a)./1000)).^(34.1632/-6.5);
    rho(a) = P(a)./(R.*T(a));
end

if any(b)
    T(b) = 216.65;
    P(b) = 22632.06 * exp(-34.1632 * (h(b)./1000 - 11) / 216.65);
    rho(b) = P(b)./(R.*T(b));
end

if any(c)
    T(c) = 196.65 + h(c)./1000;
    P(c) = 5474.889 * (216.65 ./ (216.65 + (h(c)./1000 - 20))).^(34.1632);
    rho(c) = P(c)./(R.*T(c));
end

if any(d)
    T(d) = 139.05 + 2.8 .* h(d)./1000;
    P(d) = 868.0187 .* (228.65 ./ (228.65 + 2.8 .* (h(d)./1000 - 32))).^(34.1632 / 2.8);
    rho(d) = P(d)./(R.*T(d));
end

if any(e)
    T(e) = 270.65;
    P(e) = 110.9063 * exp(-34.1632 .* (h(e)./1000 - 47) ./ 270.65);
    rho(e) = P(e)./(R.*T(e));
end

if any(f)
    T(f) = 413.45 - 2.8 .* h(f)./1000;
    P(f) = 66.93887 * (270.65 ./ (270.65 - 2.8 * (h(f)./1000 - 51))) .^ (34.1632 / -2.8);
    rho(f) = P(f)./(R.*T(f));
end

if any(g)
    T(g) = 356.65 - 2.0 .* h(g)./1000;
    P(g) = 3.956420 .* (214.65 ./ (214.65 - 2 .* (h(g)./1000 - 71))) .^ (34.1632 / -2);
    rho(g) = P(g)./(R.*T(g));
end

if any(aa)
    ix = 1;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    T(aa) = 186.8673;
    rho(aa) = exp(Ar.*z(aa).^4+Br.*z(aa).^3+Cr.*z(aa).^2+Dr.*z(aa)+Er);
    P(aa) = exp(Ap.*z(aa).^4+Bp.*z(aa).^3+Cp.*z(aa).^2+Dp.*z(aa)+Ep);
end

if any(bb)
    ix = 2;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    T(bb) = 263.1905 - 76.3232.*sqrt(1 - ((z(bb) - 91)/-19.9429).^2);
    rho(bb) = exp(Ar.*z(bb).^4+Br.*z(bb).^3+Cr.*z(bb).^2+Dr.*z(bb)+Er);
    P(bb) = exp(Ap.*z(bb).^4+Bp.*z(bb).^3+Cp.*z(bb).^2+Dp.*z(bb)+Ep);
end

if any(cc)
    ix = 3;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    T(cc) = 263.1905 - 76.3232*sqrt(1 - ((z(cc) - 91)/-19.9429).^2);
    rho(cc) = exp(Ar.*z(cc).^4+Br.*z(cc).^3+Cr.*z(cc).^2+Dr.*z(cc)+Er);
    P(cc) = exp(Ap.*z(cc).^4+Bp.*z(cc).^3+Cp.*z(cc).^2+Dp.*z(cc)+Ep);
end

if any(dd)
    ix = 4;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    T(dd) = 240 + 12.*(z(dd) - 110);
    rho(dd) = exp(Ar.*z(dd).^4+Br.*z(dd).^3+Cr.*z(dd).^2+Dr.*z(dd)+Er);
    P(dd) = exp(Ap.*z(dd).^4+Bp.*z(dd).^3+Cp.*z(dd).^2+Dp.*z(dd)+Ep);
end

if any(ee)
    ix = 5;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    zeta(ee) = (z(ee) - 120) .* (6356.766 + 120)./(6356.766 + z(ee));
    T(ee) = 1000 - 640 .* exp(-0.01875 .* zeta(ee));
    rho(ee) = exp(Ar.*z(ee).^4+Br.*z(ee).^3+Cr.*z(ee).^2+Dr.*z(ee)+Er);
    P(ee) = exp(Ap.*z(ee).^4+Bp.*z(ee).^3+Cp.*z(ee).^2+Dp.*z(ee)+Ep);
end

if any(ff)
    ix = 6;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    zeta(ff) = (z(ff) - 120) .* (6356.766 + 120)./(6356.766 + z(ff));
    T(ff) = 1000 - 640 .* exp(-0.01875 .* zeta(ff));
    rho(ff) = exp(Ar.*z(ff).^4+Br.*z(ff).^3+Cr.*z(ff).^2+Dr.*z(ff)+Er);
    P(ff) = exp(Ap.*z(ff).^4+Bp.*z(ff).^3+Cp.*z(ff).^2+Dp.*z(ff)+Ep);
end

if any(gg)
    ix = 7;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    zeta(gg) = (z(gg) - 120) .* (6356.766 + 120)./(6356.766 + z(gg));
    T(gg) = 1000 - 640 .* exp(-0.01875 .* zeta(gg));
    rho(gg) = exp(Ar.*z(gg).^4+Br.*z(gg).^3+Cr.*z(gg).^2+Dr.*z(gg)+Er);
    P(gg) = exp(Ap.*z(gg).^4+Bp.*z(gg).^3+Cp.*z(gg).^2+Dp.*z(gg)+Ep);
end

if any(hh)
    ix = 8;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    zeta(hh) = (z(hh) - 120) .* (6356.766 + 120)./(6356.766 + z(hh));
    T(hh) = 1000 - 640 .* exp(-0.01875 .* zeta(hh));
    rho(hh) = exp(Ar.*z(hh).^4+Br.*z(hh).^3+Cr.*z(hh).^2+Dr.*z(hh)+Er);
    P(hh) = exp(Ap.*z(hh).^4+Bp.*z(hh).^3+Cp.*z(hh).^2+Dp.*z(hh)+Ep);
end

if any(ii)
    ix = 9;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    zeta(ii) = (z(ii) - 120) .* (6356.766 + 120)./(6356.766 + z(ii));
    T(ii) = 1000 - 640 .* exp(-0.01875 .* zeta(ii));
    rho(ii) = exp(Ar.*z(ii).^4+Br.*z(ii).^3+Cr.*z(ii).^2+Dr.*z(ii)+Er);
    P(ii) = exp(Ap.*z(ii).^4+Bp.*z(ii).^3+Cp.*z(ii).^2+Dp.*z(ii)+Ep);
end

if any(jj)
    ix = 10;
    Ar = r_cs(ix,2); 
    Br = r_cs(ix,3); 
    Cr = r_cs(ix,4); 
    Dr = r_cs(ix,5); 
    Er = r_cs(ix,6);
    Ap = P_cs(ix,2); 
    Bp = P_cs(ix,3); 
    Cp = P_cs(ix,4); 
    Dp = P_cs(ix,5); 
    Ep = P_cs(ix,6);
    
    zeta(jj) = (z(jj) - 120) .* (6356.766 + 120)./(6356.766 + z(jj));
    T(jj) = 1000 - 640 .* exp(-0.01875 .* zeta(jj));
    rho(jj) = exp(Ar.*z(jj).^4+Br.*z(jj).^3+Cr.*z(jj).^2+Dr.*z(jj)+Er);
    P(jj) = exp(Ap.*z(jj).^4+Bp.*z(jj).^3+Cp.*z(jj).^2+Dp.*z(jj)+Ep);
end

end