function [x_next, dx_next, ddx_next] = runge_kutta_step(x, dx, ddx, dt, V, omega, pitch, M, C, K, AeroParameters)
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
    F = compute_aero_force(x, dx, V, omega, pitch, AeroParameters.radius_aero, AeroParameters.twist_aero, AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero);
    
    A_rk = dt/2 * ddx;
    b_rk = dt/2 * (dx + 0.5 * A_rk);

    B_rk = dt/2 * g(dx + A_rk, x + b_rk, V, omega, pitch, M, C, K, F,AeroParameters);
    C_rk = dt/2 * g(dx + B_rk, x + b_rk, V, omega, pitch, M, C, K, F,AeroParameters);
    d_rk = dt * (dx + C_rk);
    D_rk = dt/2 * g(dx + 2*C_rk, x + d_rk, V, omega, pitch, M, C, K, F,AeroParameters);

    x_next = x + dt * (dx + 1/3 * (A_rk + B_rk + C_rk));
    dx_next = dx + 1/3 * (A_rk + 2*B_rk + 2*C_rk + D_rk);
    ddx_next = g(dx, x, V, omega, pitch, M, C, K, F, AeroParameters);
end
