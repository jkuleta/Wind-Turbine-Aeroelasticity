function [F_modal, Moments] = compute_aero_force(x, dx, v0, omega, pitch, radius, twist, phi_1flap_aero, phi_1edge_aero)

    % Convert modal deflection to blade deflection shape
    % (You could use phi*modal amplitude here, shape already given)
    global Moments;

    [Rx,FN,FT,~] = BEM(v0, omega, pitch);  % Call your existing BEM

    % Rotate loads to flap and edge directions

    FF = FN .* cos(deg2rad(pitch + twist)) + FT .* sin(deg2rad(pitch + twist));
    FE = FN .* -sin(deg2rad(pitch + twist)) + FT .* cos(deg2rad(pitch + twist));

    % Project to modal coordinates
    F_flap = trapz(radius, FF .* phi_1flap_aero);
    F_edge = trapz(radius, FE .* phi_1edge_aero);
    
    M_flap = trapz(radius, FF .*radius);
    M_edge = trapz(radius, FE .*radius);

    F_modal = [F_flap; F_edge];
    Moment =[M_flap; M_edge];

    Moments(:, end+1) = Moment;
end
