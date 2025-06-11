function dYdt = odefun_blade_dynamic(t, Y, V_0, omega, f, M, C, K_CG, StructuralParameters, AeroParameters, coupling, dynamic_inflow, dt)
    N = length(AeroParameters.radius_aero);
    x   = Y(1:2);
    dx  = Y(3:4);
    psi = Y(5);
    a_prev = Y(6:5+N);
    a_prime_prev = Y(6+N:5+2*N);

    pitch = 10.45 + 5 * sin(2 * pi * f * t);

    if K_CG
        K = get_total_K(StructuralParameters, omega(1), psi);
    else
        K = StructuralParameters.K;
    end

    [F_modal, ~, ~, ~, a_next, a_prime_next, ~] = compute_aero_force_dynamic(x, dx, V_0, omega, pitch, ...
        AeroParameters.radius_aero, AeroParameters.twist_aero, ...
        AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
        coupling, dynamic_inflow, a_prev, a_prime_prev, dt);

    ddx = M \ (F_modal - C*dx - K*x);
    dpsi = omega(1);

    if dynamic_inflow
        R = 63;
        Vwake = max((1 - 2 * a_prev) .* V_0, 1e-2);  % Prevent division by 0
        tau_nw = 0.5 * R ./ Vwake;
        tau_fw = 2.0 * R ./ Vwake;

        da = 0.6 * (a_next - a_prev) ./ tau_nw + 0.4 * (a_next - a_prev) ./ tau_fw;
        da_prime = 0.6 * (a_prime_next - a_prime_prev) ./ tau_nw + ...
                   0.4 * (a_prime_next - a_prime_prev) ./ tau_fw;
    else
        da = zeros(size(a_prev));
        da_prime = zeros(size(a_prime_prev));
    end

    dYdt = [dx; ddx; dpsi; da; da_prime];
end
