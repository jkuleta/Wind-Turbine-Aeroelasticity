% No Aero terms used? Not sure if we include the aeroterms.ddphi_1flap and ddphi_1edge or if they are purely structural terms
function total_K = get_total_K(StructuralParameters, omega, psi)
    K_1edgeS = StructuralParameters.K_1edge;
    K_1flapS = StructuralParameters.K_1flap;
    radius = StructuralParameters.radius;
    mass_distribution = StructuralParameters.mass_distribution;
    ddphi_1flap = StructuralParameters.ddphi_1flap;
    ddphi_1edge = StructuralParameters.ddphi_1edge;
    
    disp(['K_1edgeS = ', num2str(K_1edgeS)]);
    disp(['K_1flapS = ', num2str(K_1flapS)]);
    disp(['radius = ', mat2str(radius)]);
    disp(['mass_distribution = ', mat2str(mass_distribution)]);
    disp(['ddphi_1flap = ', mat2str(ddphi_1flap)]);
    disp(['ddphi_1edge = ', mat2str(ddphi_1edge)]);
    

    K_centrifugal_flap = centrifugal_stiffening(radius, mass_distribution, omega, ddphi_1flap);
    K_gravity_flap = gravity_stiffening(radius, mass_distribution, ddphi_1flap, psi); % or use twist if available

    K_1flap = K_1flapS + K_centrifugal_flap - K_gravity_flap;

    K_centrifugal_edge = centrifugal_stiffening(radius, mass_distribution, omega, ddphi_1edge);
    K_gravity_edge = gravity_stiffening(radius, mass_distribution, ddphi_1edge, psi); % or use twist if available

    K_1edge = K_1edgeS + K_centrifugal_edge - K_gravity_edge;

    disp(['omega = ', num2str(omega)]);
    disp(['K_centrifugal_flap = ', num2str(K_centrifugal_flap), ...
        ', K_gravity_flap = ', num2str(K_gravity_flap), ...
        ', K_1flapS = ', num2str(K_1flapS)]);
    disp(['K_centrifugal_edge = ', num2str(K_centrifugal_edge), ...
        ', K_gravity_edge = ', num2str(K_gravity_edge), ...
        ', K_1edgeS = ', num2str(K_1edgeS)]);

    total_K = diag([K_1flap, K_1edge]);

    
end

% filepath: c:\Users\Marcos\Documents\GitHub\Wind-Turbine-Aeroelasticity\centrifugal_stiffening.m
function K_c = centrifugal_stiffening(radius, mass_distribution, omega, phi_prime)
    % Calculates centrifugal stiffening term for a blade mode
    % radius: vector of blade radii
    % mass_distribution: mass per unit length along blade
    % omega: rotational speed [rad/s]
    % phi_prime: derivative of mode shape (same length as radius)

    K_c_integral = zeros(size(radius));
    for idx = 1:length(radius)
        xi = radius(idx:end);
        m_xi = mass_distribution(idx:end);

        if numel(xi) < 2
            K_c_integral(idx) = 0;
            continue
        end

        inner = trapz(xi, m_xi .* xi) * (omega^2);
        K_c_integral(idx) = inner * (phi_prime(idx))^2;
    end
    K_c = trapz(radius, K_c_integral);

end

% filepath: c:\Users\Marcos\Documents\GitHub\Wind-Turbine-Aeroelasticity\gravity_stiffening.m
function K_g = gravity_stiffening(radius, mass_distribution, phi_prime, psi)
    % Calculates gravity stiffening term for a blade mode
    % radius: vector of blade radii
    % mass_distribution: mass per unit length along blade
    % phi_prime: derivative of mode shape (same length as radius)
    % twist: local blade angle (same length as radius)

    g_const = 9.81; % m/s^2
    K_g_integral = zeros(size(radius));
    for idx = 1:length(radius)
        xi = radius(idx:end);
        m_xi = mass_distribution(idx:end);

        if numel(xi) < 2
            K_c_integral(idx) = 0;
            continue
        end
        
        inner = trapz(xi, m_xi * g_const .* cos(psi));
        K_g_integral(idx) = inner * (phi_prime(idx))^2;
    end
    K_g = trapz(radius, K_g_integral);
end







%    StructuralParameters.K = diag([StructuralParameters.K_1flap, StructuralParameters.K_1edge]);



% % After your current K_1flap and K_1edge calculation:
% omega = omega_values(i); % [rad/s]

% K_centrifugal_flap = centrifugal_stiffening(radius, mass_distribution, omega, ddphi_1flap);
% K_gravity_flap = gravity_stiffening(radius, mass_distribution, ddphi_1flap, zeros(size(radius))); % or use twist if available

% K_1flap = K_1flap + K_centrifugal_flap - K_gravity_flap;

% K_centrifugal_edge = centrifugal_stiffening(radius, mass_distribution, omega, ddphi_1edge);
% K_gravity_edge = gravity_stiffening(radius, mass_distribution, ddphi_1edge, zeros(size(radius))); % or use twist if available

% K_1edge = K_1edge + K_centrifugal_edge - K_gravity_edge;
% %%%%% 
% K_g = gravity_stiffening(radius, mass_distribution, phi_prime, psi);
% K_c = centrifugal_stiffening(radius, mass_distribution, omega, phi_prime);

% total_K = StructuralParameters.K + K_c - K_g;