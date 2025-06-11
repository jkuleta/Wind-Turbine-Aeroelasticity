% =============================
% compute_aero_force_dynamic.m
% =============================
function [F_modal, FF, FE, T, a_next, a_prime_next, a_steady] = compute_aero_force_dynamic( ...
    x, dx, v0, omega, pitch, radius, twist, ...
    phi_1flap_aero, phi_1edge_aero, ...
    coupling, dynamic_inflow, a_prev, a_prime_prev, dt)

    velocity = [dx(1) * phi_1flap_aero, dx(2) * phi_1edge_aero];
    V_outplane = velocity(:,1) .* cosd(pitch + twist) + velocity(:,2) .* sind(pitch + twist);
    V_inplane  = velocity(:,1) .* sind(pitch + twist) - velocity(:,2) .* cosd(pitch + twist);

    [Rx, FN, FT, T, a_next, a_prime_next, a_steady] = BEM_dynamic( ...
        v0, omega, V_inplane, V_outplane, pitch, coupling, dynamic_inflow, ...
        a_prev, a_prime_prev, dt);

    FF = FN .* cosd(pitch + twist) + FT .* sind(pitch + twist);
    FE = -FN .* sind(pitch + twist) + FT .* cosd(pitch + twist);

    F_flap = trapz(radius, FF .* phi_1flap_aero);
    F_edge = trapz(radius, FE .* phi_1edge_aero);
    T = 3 * trapz(radius, FN);

    F_modal = [F_flap; F_edge];
end
