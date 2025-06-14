function [ddx] = g_dynamic(dx, x, v0, omega, pitch, M, C, K, AeroParameters, coupling, dynamic_inflow, PREVIOUS, dt)

    [F_modal, ~, ~, ~, PREVIOUS] = compute_aero_force_dynamic( ...
        x, dx, v0, omega, pitch, ...
        AeroParameters.radius_aero, AeroParameters.twist_aero, ...
        AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
        coupling, dynamic_inflow, PREVIOUS, dt);

    ddx = M \ (F_modal - C * dx - K * x);
end
