% =============================
% compute_aero_force_dynamic.m
% =============================
function [F_modal, FF, FE, T, PREVIOUS] = compute_aero_force_dynamic( ...
    x, dx, v0, omega, pitch, radius, twist, ...
    phi_1flap_aero, phi_1edge_aero, ...
    coupling, dynamic_inflow, PREVIOUS, dt)

    if ~dynamic_inflow
        % zero out motion-induced velocities
        V_inplane  = zeros(size(radius));
        V_outplane = zeros(size(radius));

        velocity = [dx(1) * phi_1flap_aero, dx(2) * phi_1edge_aero];
        V_outplane = velocity(:,1) .* cosd(pitch + twist) + velocity(:,2) .* sind(pitch + twist);
        V_inplane  = velocity(:,1) .* sind(pitch + twist) - velocity(:,2) .* cosd(pitch + twist);

        [~, FN, ~, ~] = BEM2(v0, omega, V_inplane, V_outplane, pitch, coupling);

        %Not used for anything
        FF = FN;                     % normal force already in flapwise direction
        FE = zeros(size(FN));        % no edge component when velocities are zero

        F_flap  = trapz(radius, FF .* phi_1flap_aero);
        F_edge  = 0;                 % edge load is zero in this simplified call
        F_modal = [F_flap; F_edge];

        T = 3 * trapz(radius, FN);   % thrust, 3 blades
        PREVIOUS = [];                      % dummy so output list has 5 elements
        return
    end
    
    velocity = [dx(1) * phi_1flap_aero, dx(2) * phi_1edge_aero];
    V_outplane = velocity(:,1) .* cosd(pitch + twist) + velocity(:,2) .* sind(pitch + twist);
    V_inplane  = velocity(:,1) .* sind(pitch + twist) - velocity(:,2) .* cosd(pitch + twist);

    [Rx, FN, FT, T, PREVIOUS.a, PREVIOUS.a_prime] = BEM_dynamic( ...
        v0, omega, V_inplane, V_outplane, pitch, coupling, dynamic_inflow, ...
        PREVIOUS.a, PREVIOUS.a_prime, dt);

    fprintf('BEM_dynamic: a_next=%.2f, a_prime_next=%.2f\n', PREVIOUS.a, PREVIOUS.a_prime);	

    FF = FN .* cosd(pitch + twist) + FT .* sind(pitch + twist);
    FE = -FN .* sind(pitch + twist) + FT .* cosd(pitch + twist);

    F_flap = trapz(radius, FF .* phi_1flap_aero);
    F_edge = trapz(radius, FE .* phi_1edge_aero);
    T = 3 * trapz(radius, FN);

    F_modal = [F_flap; F_edge];
end