function dYdt = odefun_blade(t, Y, V, omega, pitch, M, C, K, AeroParameters)
    % Y = [x; dx]
    x = Y(1:2);
    dx = Y(3:4);
    ddx = g(dx, x, V, omega, pitch, M, C, K, AeroParameters);
    dYdt = [dx; ddx];
end