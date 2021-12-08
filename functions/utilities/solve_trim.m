% ================== Dynamics Trim Optimisation ====================
%  Optimisation wrapper to find steady-state solution to dynamics.
% ==================================================================
function x = solve_trim(auxdata, dynamics_func, initial, lb, ub)

function x_dot = dynamics_wrapper(x0)
    input.phase.time = 0;
    input.phase.state = x0(1:15);
    input.phase.control = x0(16:end);
    input.auxdata = auxdata;

    output = dynamics_func(input);

    x_dot = norm(output.dynamics);
end

% Solve for steady-state solution
x = fmincon(@dynamics_wrapper, initial, [], [], [], [], lb, ub);

end

