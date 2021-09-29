% ================ Minimum time objective function ================ %
% Objective function script for minimising time of mission whilst   %
% trimming the vehicle at the bounds.
% ================================================================= %

function output = MinTime(input)

tf      = input.phase.finaltime;
J       = tf;

output.objective = J;
