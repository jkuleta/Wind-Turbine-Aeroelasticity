function ddx = g(dx, x, v0, omega, pitch, M, C, K, AeroParameters, coupling)
    % Mass, damping, stiffness must be accessible globally or passed


    % Compute force based on current state
    F = compute_aero_force(x, dx, v0, omega, pitch, AeroParameters.radius_aero, AeroParameters.twist_aero, AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, coupling);

    % Return acceleration    
    ddx = M \ (F - C*dx - K*x);

end