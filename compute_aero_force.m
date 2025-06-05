function F_modal = compute_aero_force(x, dx, v0, omega, pitch, radius, twist, phi_1flap_aero, phi_1edge_aero)


    velocity = [dx(1) * phi_1flap_aero, dx(2) * phi_1edge_aero];
    V_inplane = velocity(1) .* cos(deg2rad(pitch + twist)) + velocity(2) .* sin(deg2rad(pitch + twist));
    V_outplane = -velocity(1) .* sin(deg2rad(pitch + twist)) + velocity(2) .* cos(deg2rad(pitch + twist));

    [Rx,FN,FT,~] = BEM2(v0, omega,V_inplane,V_outplane, pitch);  % Call your existing BEM

    % Rotate loads to flap and edge directions

    FF = FN .* cos(deg2rad(pitch + twist)) + FT .* sin(deg2rad(pitch + twist));
    FE = FN .* -sin(deg2rad(pitch + twist)) + FT .* cos(deg2rad(pitch + twist));

    % Project to modal coordinates
    F_flap = trapz(radius, FF .* phi_1flap_aero);
    F_edge = trapz(radius, FE .* phi_1edge_aero);

    

    F_modal = [F_flap; F_edge];
end
