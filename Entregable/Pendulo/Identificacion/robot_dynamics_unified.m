function dx = robot_dynamics_unified(t, x, u_interp, params)
    u_current = u_interp(t);  % Interpolate the input u at time t
    % Example dynamics (simplified)
    g = params.g;
    L = params.L;
    m = params.m;
    K_theta = params.K_theta;
    K_x = params.K_x;
    b = params.b;

    theta = x(1);
    dtheta = x(2);
    xpos = x(3);
    dxpos = x(4);

    % Simple physics-based model calculations
    ddtheta = (g/L) * sin(theta) - (K_theta/(m*L^2)) * dtheta + (K_theta/(m*L^2)) * u_current;
    ddxpos = (K_x/m) * u_current - (b/m) * dxpos;

    dx = [dtheta; ddtheta; xpos; ddxpos];
end