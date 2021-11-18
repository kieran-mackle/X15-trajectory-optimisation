function [bounds, guess, auxdata] = cart6_manoeuvre(specification, auxdata)
% ===========  Cartesian 6 DoF Manouevre Initialisation ============
%  Dynamics initialisation for Cartesian 6 DoF flight dynamics.
% ==================================================================


% Quaternions - these can just be solved as states?
% These equations are for initialisation only.
% q0 = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
% q1 = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
% q2 = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
% q3 = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);

end