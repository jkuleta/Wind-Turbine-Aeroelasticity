function dYdt = odefun_blade_periodic(t, Y, V_0, omega, pitch, M, C, K_CG, StructuralParameters, AeroParameters, coupling, dynamic_inflow)
    x   = Y(1:2);
    dx  = Y(3:4);
    psi = Y(5);
    disp(t);
    
    V_0 = (15 + 0.5*cos(1.267*t) + 0.085*cos(2.534*t) + 0.015*cos(3.801*t)) * ones(size(AeroParameters.radius_aero));
    %V_0 = (20) * ones(size(AeroParameters.radius_aero));

    % Optionally use psi-dependent stiffness
    if K_CG
        K = get_total_K(StructuralParameters, omega(1), psi);
    else
        K = StructuralParameters.K;
    end

    ddx = g(dx, x, V_0, omega, pitch, M, C, K, AeroParameters, coupling);
    dpsi = omega(1);

    dYdt = [dx; ddx; dpsi];
end
