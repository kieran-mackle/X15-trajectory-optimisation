% ================== Dynamics Trim Optimisation ====================
%  Optimisation wrapper to find steady-state solution to dynamics.
% ==================================================================
function x = solve_trim(auxdata, dynamics_func, initial, lb, ub)

function J = dynamics_wrapper(x0)
    input.phase.time = 0;
    input.phase.state = x0(1:15);
    input.phase.control = x0(16:end);
    input.auxdata = auxdata;

    output = dynamics_func(input);
    
    % Rates
    % Only want to minimise accelerations
    % State vector: [N, E, D, u, v, w, p, q, r, phi, theta, psi, m, fda, thr]
    %               [1, 2, 3, 4, 5, 6, 7, 8, 9, 10,   11,   12, 13, 14,  15 ]
    dN = output.dynamics(1);
    dD = output.dynamics(3);
    du = output.dynamics(4);
    dv = output.dynamics(5);
    dw = output.dynamics(6);
    dq = output.dynamics(8);
    dphi = output.dynamics(10);
    dtheta = output.dynamics(11);
    dpsi = output.dynamics(12);
    dfda = output.dynamics(14);
    dthr = output.dynamics(15);
    
%     rates = norm(output.dynamics([1, 3, 4:6, 10:12, 14:15]));
    rates = dN^2 + dD^2 + du^2 + dv^2 + dw^2 + dq^2 + ...
            dphi^2 + dtheta^2 + dpsi^2 + dfda^2 + dthr^2;
    
    % Target deviations
    [Ma,~] = calculate_outputs(auxdata, x0);
    Ma_deviation = (6-Ma)^2;
    deviations = Ma_deviation;
    
    J = rates + 100000*deviations;
end

% Solve for steady-state solution
x = fmincon(@dynamics_wrapper, initial, [], [], [], [], lb, ub);

end

