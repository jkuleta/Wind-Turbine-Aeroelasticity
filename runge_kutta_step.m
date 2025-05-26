function [x_next, dx_next, ddx_next] = runge_kutta_step(x, dx, ddx, dt, V, omega, pitch, M, C, K, AeroParameters, StructuralParameters)
    % One time step of Runge-Kutta integration for 2nd-order ODE
    %
    % Inputs:
    %   x     - current displacement state (2x1)
    %   dx    - current velocity state (2x1)
    %   dt    - time step
    %   g_func - handle to function that returns acceleration ddx = g(dx, x, V, omega, pitch)
    %   V     - local wind speed vector
    %   omega - rotational speed vector
    %   pitch - pitch angle (scalar)
    %
    % Outputs:
    %   x_next    - next displacement state (2x1)
    %   dx_next   - next velocity state (2x1)
    %   ddx_next  - next acceleration state (2x1)
    F = compute_aero_force(x, dx, V, omega, pitch, AeroParameters.radius_aero, AeroParameters.twist_aero, StructuralParameters);
    
    A_rk = 0.5* dt * ddx;
    b_rk = 0.5* dt * (dx + 0.5 * A_rk);

    B_rk = 0.5 * dt * g(dx + A_rk, x + b_rk, M, C, K, F);
    C_rk = 0.5 * dt * g(dx + B_rk, x + b_rk, M, C, K, F);
    d_rk = dt * (dx + C_rk);
    D_rk = dt/2 * g(dx + 2*C_rk, x + d_rk, M, C, K, F);

    x_next = x + dt * (dx + 1/3 * (A_rk + B_rk + C_rk));
    dx_next = dx + 1/3 * (A_rk + 2*B_rk + 2*C_rk + D_rk);
    ddx_next = g(dx, x, M, C, K, F);
end

function ddx = g(dx, x, M, C, K, F)
    % Mass, damping, stiffness must be accessible globally or passed

    % Compute force based on current state

    % Return acceleration    
    ddx = linsolve(M, F - C*dx - K*x);

end
