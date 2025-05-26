function ddx = g(dx, x, v0, omega, pitch, M, C, K, F,AeroParameters)
    % Mass, damping, stiffness must be accessible globally or passed

    % Compute force based on current state

    % Return acceleration    
    ddx = M \ (F - C*dx - K*x);

end