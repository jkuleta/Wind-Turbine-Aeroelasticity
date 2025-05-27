function F_modal = compute_aero_force(x, dx, v0, omega, pitch, radius, twist, structural, aero)

    % Convert modal deflection to blade deflection shape
    % (You could use phi*modal amplitude here, shape already given)

    [Rx,FN,FT,~] = BEM(v0, omega, pitch, aero.phi_1edge_aero, aero.phi_1flap_aero,dx);  % Call your existing BEM

    % Rotate loads to flap and edge directions

    FF = FN .* cosd(pitch + twist) + FT .* sind(pitch + twist);
    FE = FN .* -sind(pitch + twist) + FT .* cosd(pitch + twist);

    %F_flap = trapz(radius, FF .* phi_1flap_aero);
    %F_edge = trapz(radius, FE .* phi_1edge_aero);

    % Interpolate forces for structural points
    FF_structural = interp1(radius, FF, structural.radius, 'linear', 'extrap');
    FE_structural = interp1(radius, FE, structural.radius, 'linear', 'extrap');

    F_flap = trapz(structural.radius, FF_structural .* structural.phi_1flap);
    F_edge = trapz(structural.radius, FE_structural .* structural.phi_1edge);

    F_modal = [F_flap; F_edge];
end
