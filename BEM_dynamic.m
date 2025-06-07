function [Rx, FN, FT, P, a_list, a_prime_list, a_out, a_prime_out] = BEM_dynamic(v0_array, omega0, V_inplane, V_outplane, pitch, coupling, dynamic_inflow, dt, a_prev, a_prime_prev)

%------------------------------------------------
% Blade Element Momentum (BEM) with coupling and dynamic inflow memory
%------------------------------------------------

% Fixed parameters
B = 3;              % Number of blades
R = 63;             % Rotor radius [m]
hubrad = 1.5;       % Hub radius [m]
rho = 1.225;        % Air density [kg/m^3]
EPS = 1e-5;         % Iteration tolerance
REG = 1e-8;         % Regularization constant

% Simulation options
taustar_nw = 0.5;   % Constants for dynamic inflow model 
taustar_fw = 2;

% Load blade structure and aerofoil data
BS = importdata('Blade/Blade section/Blade section.dat').data;
Readfiles = dir(fullfile('Blade/Aero data', '*.dat'));
for j = 1:length(Readfiles)
    AD{j} = importdata(fullfile('Blade/Aero data', Readfiles(j).name));
end

NBS = size(BS,1);
Rx = zeros(NBS,1); FN = zeros(NBS,1); FT = zeros(NBS,1); Mx = zeros(NBS,1);
a_list = zeros(NBS,1); a_prime_list = zeros(NBS,1);
a_out = zeros(NBS,1); a_prime_out = zeros(NBS,1);

for i = 1:NBS
    % Section properties
    ADofBS = BS(i,2);
    r      = BS(i,3); Rx(i) = r;
    dr     = BS(i,4);
    theta  = BS(i,5);     % [deg]
    chord  = BS(i,6);     % [m]
    
    alpha = AD{ADofBS}(:,1);
    Cl    = AD{ADofBS}(:,2);
    Cd    = AD{ADofBS}(:,3);

    v0    = v0_array(i);
    omega = omega0(i);
    if coupling
        V_oop = V_outplane(i);
        V_ip  = V_inplane(i);
    else
        V_oop = 0;
        V_ip  = 0;
    end

    Sigma = chord * B / (2 * pi * r);

    % Initial induction guesses (use memory if available)
    if dynamic_inflow && nargin >= 10 && ~isempty(a_prev) && ~isempty(a_prime_prev)
        a = a_prev(i);
        a_prime = a_prime_prev(i);
    else
        a = 0.0;
        a_prime = 0.0;
    end
    ax = a; ax_prime = a_prime;
    numite = 0;
    
    % Iteration loop (no dynamic inflow update inside)
    while abs(ax - a) >= EPS || abs(ax_prime - a_prime) >= EPS
        numite = numite + 1;
        a = ax; a_prime = ax_prime;

        denom = (1 + a_prime) * r * omega - V_ip;
        if abs(denom) < REG
            denom = sign(denom + REG) * REG;
        end
        Phi = atan(((1 - a) * v0 - V_oop) / denom);
        Phi_deg = rad2deg(Phi);
        Alpha = Phi_deg - theta - pitch;

        % Interpolate Cl and Cd
        Cla = interp1(alpha, Cl, Alpha, 'linear', 'extrap');
        Cda = interp1(alpha, Cd, Alpha, 'linear', 'extrap');

        % Aerodynamic force coefficients
        Cn = Cla * cosd(Phi_deg) + Cda * sind(Phi_deg);
        Ct = Cla * sind(Phi_deg) - Cda * cosd(Phi_deg);

        % Prandtl losses
        if abs(sind(Phi_deg)) < REG
            F = 1.0;
        else
            f_tip = B/2 * (R - r) / (r * sind(Phi_deg));
            F_tip = (2/pi) * acos(exp(-abs(f_tip)));

            f_hub = B/2 * (r - hubrad) / (r * sind(Phi_deg));
            F_hub = (2/pi) * acos(exp(-abs(f_hub)));

            F = F_tip * F_hub;
        end

        % Glauert correction
        ac = 0.2;
        if a > ac
            K = 4 * F * sind(Phi_deg)^2 / (Sigma * Cn + REG);
            sqrt_term = sqrt((K * (1 - 2 * ac) + 2)^2 + 4 * (K * ac^2 - 1));
            ax = 0.5 * (2 + K * (1 - 2 * ac) - sqrt_term);
        else
            ax = 1 / (4 * F * sind(Phi_deg)^2 / (Sigma * Cn + REG) + 1);
        end

        denom_ct = 4 * F * sind(Phi_deg) * cosd(Phi_deg) / (Sigma * Ct + REG) - 1;
        if abs(denom_ct) > REG
            ax_prime = 1 / denom_ct;
        else
            ax_prime = 0;
        end

        % Force a stop if not converging
        if numite >= 100
            ax = 0.3;
            ax_prime = 0.1;
            break;
        end
    end

    % --- Apply dynamic inflow update AFTER convergence ---
    if dynamic_inflow
        Vwake = max((1-2*ax)*v0, 0.1); % Avoid zero or negative
        a_nw = a_prev(i).*exp(-dt./(taustar_nw*R./Vwake)) + ax.*(1-exp(-dt./(taustar_nw*R./Vwake)));
        a_fw = a_prev(i).*exp(-dt./(taustar_fw*R./Vwake)) + ax.*(1-exp(-dt./(taustar_fw*R./Vwake)));
        ap_nw = a_prime_prev(i).*exp(-dt./(taustar_nw*R./Vwake)) + ax_prime.*(1-exp(-dt./(taustar_nw*R./Vwake)));
        ap_fw = a_prime_prev(i).*exp(-dt./(taustar_fw*R./Vwake)) + ax_prime.*(1-exp(-dt./(taustar_fw*R./Vwake)));
        ax = 0.6*a_nw+0.4*a_fw;
        ax_prime = 0.6*ap_nw+0.4*ap_fw;
    end

    % Save final inductions
    a_list(i) = ax;
    a_prime_list(i) = ax_prime;
    a_out(i) = ax;
    a_prime_out(i) = ax_prime;

    % Recompute final Phi, Alpha, Cn, Ct using final a, a'
    Phi = atan(((1 - ax) * v0 - V_oop) / ((1 + ax_prime) * r * omega - V_ip));
    Phi_deg = rad2deg(Phi);
    Alpha = Phi_deg - theta - pitch;

    Cla = interp1(alpha, Cl, Alpha, 'linear', 'extrap');
    Cda = interp1(alpha, Cd, Alpha, 'linear', 'extrap');

    Cn = Cla * cosd(Phi_deg) + Cda * sind(Phi_deg);
    Ct = Cla * sind(Phi_deg) - Cda * cosd(Phi_deg);

    % Compute forces
    U_rel_sq = (r * omega * (1 + ax_prime) - V_ip)^2 + (v0 * (1 - ax) - V_oop)^2;
    FN(i) = 0.5 * rho * U_rel_sq * chord * Cn;
    FT(i) = 0.5 * rho * U_rel_sq * chord * Ct;

    % Root moment
    Mx(i) = FT(i) * r;
end

% Total torque and power (with 94.4% generator efficiency)
M = sum(Mx);
P = M * omega * 3 * 0.944;

end