function dYdt = odefun_blade_dynamic(t, Y, V_0, omega, f, M, C, K_CG, StructuralParameters, AeroParameters, coupling, dynamic_inflow, PREVIOUS, dt)
    N = length(AeroParameters.radius_aero);
    x   = Y(1:2);
    dx  = Y(3:4);
    psi = Y(5);

    pitch = 10.45 + 5 * sin(2 * pi * f * t);

    if K_CG
        K = get_total_K(StructuralParameters, omega(1), psi);
    else
        K = StructuralParameters.K;
    end

    [F_modal, ~, ~, ~, PREVIOUS] = compute_aero_force_dynamic(x, dx, V_0, omega, pitch, ...
        AeroParameters.radius_aero, AeroParameters.twist_aero, ...
        AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
        coupling, dynamic_inflow, PREVIOUS, dt);

    ddx = M \ (F_modal - C*dx - K*x);
    dpsi = omega(1);


    dYdt = [dx; ddx; dpsi];
end
