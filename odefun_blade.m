function dYdt = odefun_blade(t, Y, V, omega, pitch, M, C, K_CG, StructuralParameters, AeroParameters)
    x   = Y(1:2);
    dx  = Y(3:4);
    psi = Y(5);


    if K_CG
        K = get_total_K(StructuralParameters, omega(1), psi);
    else
        K = StructuralParameters.K;
    end

    %disp("V = " + num2str(V(1)) + ", omega = " + num2str(omega(1)) + ", pitch = " + num2str(pitch));
    ddx = g(dx, x, V, omega, pitch, M, C, K, AeroParameters);
    dpsi = omega(1);

    dYdt = [dx; ddx; dpsi];
end
