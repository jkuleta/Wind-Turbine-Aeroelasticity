% =============================
% Main Simulation Script (updated for dynamic a state)
% =============================
clc;
clear;
close all;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

%% Simulation setup
periodic = true;
vinduced = 0;
dt = 0.05;
tf = 10;
tspan = 0:dt:tf;

frequencies = [0.05, 0.2, 0.5];
dynamic_inflow_options = [true, false];
dynamic_inflow_labels = {'Dynamic Inflow ON', 'Dynamic Inflow OFF'};
colors_dynamic = {'b', 'r'};

i = 19; % index for 15 m/s

for f_idx = 1:length(frequencies)
    f = frequencies(f_idx);

    T_hist_all = cell(1,2);
    t_out_all = cell(1,2);
    pitch_all = cell(1,2);
    a_dyn_all = cell(1,2);
    a_steady_all = cell(1,2);

    for k = 1:2
        coupling = true;
        dynamic_inflow = dynamic_inflow_options(k);

        V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
        omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));

        N_blade_sections = length(AeroParameters.radius_aero);
        a_prev = zeros(N_blade_sections, 1);
        a_prime_prev = zeros(N_blade_sections, 1);

        Y0 = [0; 0; 0; 0; 0; a_prev; a_prime_prev];

        opts = odeset('RelTol',1e-3,'AbsTol',1e-5);
        [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_dynamic(tt, YY, V_org, omega_org, f, ...
            StructuralParameters.M, StructuralParameters.C, false, StructuralParameters, AeroParameters, ...
            coupling, dynamic_inflow, dt), tspan, Y0, opts);

        N_time = size(Y_out,1);
        T_hist = zeros(N_time, 1);
        pitch_profile = zeros(N_time, 1);
        a_mid_hist = zeros(N_time, 1);
        a_steady_hist = zeros(N_time, 1);
        mid_idx = floor(N_blade_sections / 2);

        for it = 1:N_time
            x_t = Y_out(it,1:2)';
            dx_t = Y_out(it,3:4)';
            pitch_t = 10.45 + 5 * sin(2 * pi * f * t_out(it));

            a_prev = Y_out(it,6:5+N_blade_sections)';
            a_prime_prev = Y_out(it,6+N_blade_sections:5+2*N_blade_sections)';

            [F_modal, FF, FE, T, a_next, a_prime_next, a_steady] = compute_aero_force_dynamic( ...
                x_t, dx_t, V_org, omega_org, pitch_t, ...
                AeroParameters.radius_aero, AeroParameters.twist_aero, ...
                AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
                coupling, dynamic_inflow, a_prev, a_prime_prev, dt);

            T_hist(it) = T;
            pitch_profile(it) = pitch_t;
            a_mid_hist(it) = a_next(mid_idx);
            a_steady_hist(it) = a_steady(mid_idx);
        end

        T_hist_all{k} = T_hist;
        t_out_all{k} = t_out;
        pitch_all{k} = pitch_profile;
        a_dyn_all{k} = a_mid_hist;
        a_steady_all{k} = a_steady_hist;
    end

    figure;
    subplot(2,1,1);
    for k = 1:2
        plot(t_out_all{k}, T_hist_all{k}, 'Color', colors_dynamic{k}, ...
            'LineWidth', 1.5, 'DisplayName', dynamic_inflow_labels{k}); hold on;
    end
    ylabel('Rotor Thrust [N]');
    title(['Rotor Thrust vs Time, f = ' num2str(f) ' Hz']);
    legend; grid on;

    subplot(2,1,2);
    for k = 1:2
        plot(pitch_all{k}, T_hist_all{k}, 'Color', colors_dynamic{k}, ...
            'LineWidth', 1.5, 'DisplayName', dynamic_inflow_labels{k}); hold on;
    end
    ylabel('Rotor Thrust [N]');
    title(['Rotor Thrust vs Pitch Angle, f = ' num2str(f) ' Hz']);
    legend; grid on;

    figure;
    plot(t_out_all{1}, a_dyn_all{1}, 'b', 'DisplayName', 'a_{dyn} (ON)'); hold on;
    plot(t_out_all{1}, a_steady_all{1}, 'k--', 'DisplayName', 'a_{steady} (ON)');
    plot(t_out_all{2}, a_dyn_all{2}, 'r', 'DisplayName', 'a_{dyn} (OFF)');
    plot(t_out_all{2}, a_steady_all{2}, 'm--', 'DisplayName', 'a_{steady} (OFF)');
    xlabel('Time [s]');
    ylabel('Axial Induction');
    title(['Dynamic vs Steady Induction (mid-span), f = ' num2str(f) ' Hz']);
    legend;
    grid on;
    sgtitle(['Frequency: ' num2str(f) ' Hz']);
end
