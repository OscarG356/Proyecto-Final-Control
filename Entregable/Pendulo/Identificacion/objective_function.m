function negR2 = objective_function(params, t, y_real, u)
    % Initialize state
    x0 = [0; 0; 0; 0];  % Initial state (theta, dtheta, xpos, dxpos)
    
    % System parameters
    paramStruct = struct('g', 9.81, 'L', params(1), 'm', params(2), 'K_theta', params(3), 'K_x', params(4), 'b', params(5));

    % Create an interpolation function for u
    u_interp = @(ti) interp1(t, u, ti, 'previous', 'extrap');

    % Solve system dynamics
    [~, y_sim] = ode45(@(t, x) robot_dynamics_unified(t, x, u_interp, paramStruct), t, x0);

    % Interpolate simulation results to match the measurement timestamps
    y_sim_interp = interp1(t, y_sim(:, 1), t);  % Assuming theta is the first state variable

    % Compute residuals
    residuals = y_real - y_sim_interp;

    % Calculate R² score
    ss_res = sum(residuals.^2);
    ss_tot = sum((y_real - mean(y_real)).^2);
    R2 = 1 - (ss_res / ss_tot);

    % Return negative R² for maximization
    negR2 = -R2;
end