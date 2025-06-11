% =============================
% BEM_dynamic.m (with diagnostics)
% =============================
function [Rx, FN, FT, P, a_next, a_prime_next, a_steady_out] = BEM_dynamic( ...
    v0_array, omega0, V_inplane, V_outplane, pitch, coupling, dynamic_inflow, ...
    a_prev, a_prime_prev, dt)

% Constants
B = 3;
R = 63;
hubrad = 1.5;
rho = 1.225;
EPS = 1e-8;
REG = 1e-8;

taustar_nw = 0.5;
taustar_fw = 2;

BS = importdata('Blade/Blade section/Blade section.dat').data;
Readfiles = dir(fullfile('Blade/Aero data', '*.dat'));
for j = 1:length(Readfiles)
    AD{j} = importdata(fullfile('Blade/Aero data', Readfiles(j).name));
end

NBS = size(BS,1);
Rx = zeros(NBS,1); FN = zeros(NBS,1); FT = zeros(NBS,1); Mx = zeros(NBS,1);
a_next = zeros(NBS,1); a_prime_next = zeros(NBS,1);
a_steady_out = zeros(NBS,1);

mid_idx = floor(NBS / 2);

for i = 1:NBS
    ADofBS = BS(i,2);
    r = BS(i,3); Rx(i) = r;
    dr = BS(i,4);
    theta = BS(i,5);
    chord = BS(i,6);

    alpha = AD{ADofBS}(:,1);
    Cl = AD{ADofBS}(:,2);
    Cd = AD{ADofBS}(:,3);

    v0 = v0_array(i);
    omega = omega0(i);
    V_oop = coupling * V_outplane(i);
    V_ip  = coupling * V_inplane(i);
    Sigma = chord * B / (2 * pi * r);

    a = 0.01; a_prime = 0.01;
    ax = a; ax_prime = a_prime;
    numite = 0;

    while abs(ax - a) >= EPS || abs(ax_prime - a_prime) >= EPS
        a = ax; a_prime = ax_prime;
        numite = numite + 1;

        denom = (1 + a_prime) * r * omega - V_ip;
        if abs(denom) < REG, denom = sign(denom + REG) * REG; end

        Phi = atan(((1 - a) * v0 - V_oop) / denom);
        Phi_deg = rad2deg(Phi);
        Alpha = Phi_deg - theta - pitch;

        Cla = interp1(alpha, Cl, Alpha, 'linear', 'extrap');
        Cda = interp1(alpha, Cd, Alpha, 'linear', 'extrap');

        if isnan(Cla) || isnan(Cda)
            warning('NaN Cl/Cd at Alpha = %.2f (section %d)', Alpha, i);
        end

        Cn = Cla * cosd(Phi_deg) + Cda * sind(Phi_deg);
        Ct = Cla * sind(Phi_deg) - Cda * cosd(Phi_deg);

        if abs(sind(Phi_deg)) < REG
            F = 1.0;
        else
            f_tip = B/2 * (R - r) / (r * sind(Phi_deg));
            f_hub = B/2 * (r - hubrad) / (r * sind(Phi_deg));
            F_tip = (2/pi) * acos(exp(-abs(f_tip)));
            F_hub = (2/pi) * acos(exp(-abs(f_hub)));
            F = F_tip * F_hub;
        end

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

        % Debug print for mid-span element
        if i == mid_idx && mod(numite, 5) == 0
            fprintf('mid-r=%.2f | Phi=%.2f | Alpha=%.2f | a=%.4f\n', r, Phi_deg, Alpha, ax);
        end
    end

    a_steady_out(i) = ax;

    if dynamic_inflow
        Vwake = max((1 - 2 * ax) * v0, 1e-3);
        tau_nw = taustar_nw * R / Vwake;
        tau_fw = taustar_fw * R / Vwake;

        a_nw = a_prev(i) * exp(-dt / tau_nw) + ax * (1 - exp(-dt / tau_nw));
        a_fw = a_prev(i) * exp(-dt / tau_fw) + ax * (1 - exp(-dt / tau_fw));
        ax_dyn = 0.6 * a_nw + 0.4 * a_fw;

        ap_nw = a_prime_prev(i) * exp(-dt / tau_nw) + ax_prime * (1 - exp(-dt / tau_nw));
        ap_fw = a_prime_prev(i) * exp(-dt / tau_fw) + ax_prime * (1 - exp(-dt / tau_fw));
        ax_prime_dyn = 0.6 * ap_nw + 0.4 * ap_fw;
    else
        ax_dyn = ax;
        ax_prime_dyn = ax_prime;
    end

    a_next(i) = ax_dyn;
    a_prime_next(i) = ax_prime_dyn;

    Phi = atan(((1 - ax_dyn) * v0 - V_oop) / ((1 + ax_prime_dyn) * r * omega - V_ip));
    Phi_deg = rad2deg(Phi);
    Alpha = Phi_deg - theta - pitch;

    Cla = interp1(alpha, Cl, Alpha, 'linear', 'extrap');
    Cda = interp1(alpha, Cd, Alpha, 'linear', 'extrap');

    Cn = Cla * cosd(Phi_deg) + Cda * sind(Phi_deg);
    Ct = Cla * sind(Phi_deg) - Cda * cosd(Phi_deg);

    U_rel_sq = (r * omega * (1 + ax_prime_dyn) - V_ip)^2 + (v0 * (1 - ax_dyn) - V_oop)^2;
    FN(i) = 0.5 * rho * U_rel_sq * chord * Cn;
    FT(i) = 0.5 * rho * U_rel_sq * chord * Ct;

    Mx(i) = FT(i) * r;
end

M = sum(Mx);
P = M * omega0(1) * B * 0.944;

end
