function [F_modal, FF, FE, RotorThrust, a_out, a_prime_out] = compute_aero_force_dynamic(x, dx, v0, omega, pitch, radius, twist, phi_1flap_aero, phi_1edge_aero, coupling, dynamic_inflow, dt, a_prev, a_prime_prev)

    velocity = [dx(1) * phi_1flap_aero, dx(2) * phi_1edge_aero];
    V_outplane = velocity(:,1) .* cos(deg2rad(pitch + twist)) + velocity(:,2) .* sin(deg2rad(pitch + twist));
    V_inplane = velocity(:,1) .* sin(deg2rad(pitch + twist)) - velocity(:,2) .* cos(deg2rad(pitch + twist));

    if dynamic_inflow
        [Rx,FN,FT,~,~,~,a_out,a_prime_out] = BEM_dynamic(v0, omega, V_inplane, V_outplane, pitch, coupling, true, dt, a_prev, a_prime_prev);
    else
        [Rx,FN,FT,~,~,~] = BEM2(v0, omega, V_inplane, V_outplane, pitch, coupling);
        a_out = [];
        a_prime_out = [];
    end

    FF = FN .* cos(deg2rad(pitch + twist)) + FT .* sin(deg2rad(pitch + twist));
    FE = FN .* sin(deg2rad(pitch + twist)) - FT .* cos(deg2rad(pitch + twist));
    F_flap = trapz(radius, FF .* phi_1flap_aero);
    F_edge = trapz(radius, FE .* phi_1edge_aero);

    F_modal = [F_flap; F_edge];
    Nb = 3; % Number of blades (update if needed)
    RotorThrust = trapz(radius, FN) * Nb; % Total rotor thrust
end