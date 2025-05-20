% filepath: c:\Users\Marcos\Documents\GitHub\Wind-Turbine-Aeroelasticity\centrifugal_stiffening.m
function K_c = centrifugal_stiffening(radius, mass_distribution, Omega, phi_prime)
% Calculates centrifugal stiffening term for a blade mode
% radius: vector of blade radii
% mass_distribution: mass per unit length along blade
% Omega: rotational speed [rad/s]
% phi_prime: derivative of mode shape (same length as radius)

K_c_integral = zeros(size(radius));
for idx = 1:length(radius)
    xi = radius(idx:end);
    m_xi = mass_distribution(idx:end);
    inner = trapz(xi, m_xi .* xi * Omega^2);
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
    psi = twist(idx:end);
    inner = trapz(xi, m_xi * g_const .* cos(psi));
    K_g_integral(idx) = inner * (phi_prime(idx))^2;
end
K_g = trapz(radius, K_g_integral);
end





% After your current K_1flap and K_1edge calculation:
Omega = omega_values(i); % [rad/s]

K_centrifugal_flap = centrifugal_stiffening(radius, mass_distribution, Omega, ddphi_1flap);
K_gravity_flap = gravity_stiffening(radius, mass_distribution, ddphi_1flap, zeros(size(radius))); % or use twist if available

K_1flap = K_1flap + K_centrifugal_flap - K_gravity_flap;

K_centrifugal_edge = centrifugal_stiffening(radius, mass_distribution, Omega, ddphi_1edge);
K_gravity_edge = gravity_stiffening(radius, mass_distribution, ddphi_1edge, zeros(size(radius))); % or use twist if available

K_1edge = K_1edge + K_centrifugal_edge - K_gravity_edge;

