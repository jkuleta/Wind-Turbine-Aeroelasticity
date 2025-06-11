
% =============================
% Main Simulation Script (updated for dynamic a state with PREVIOUSTIME)
% =============================
clc;
clear;
close all;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

%% Simulation setup
periodic = true;
vinduced = 0;
dt = 0.2;
tf = 30;
tspan = 0:dt:tf;

frequencies = [0.05, 0.2, 0.5];
dynamic_inflow_options = [true, false];
dynamic_inflow_labels = {'Dynamic Inflow ON', 'Dynamic Inflow OFF'};
colors_dynamic = {'b', 'r'};

i = 19; % index for 15 m/s

for f_idx = 1:length(frequencies)
    f = frequencies(f_idx);
    fprintf('\n===== Starting simulation for f = %.2f Hz =====\n', f);

    T_hist_all = cell(1,2);
    t_out_all = cell(1,2);
    pitch_all = cell(1,2);

    for k = 1:2
        fprintf('-- Simulating with: %s\n', dynamic_inflow_labels{k});
        coupling = true;
        dynamic_inflow = dynamic_inflow_options(k);

        V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
        omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));

        N_blade_sections = length(AeroParameters.radius_aero);
        Y0 = [0; 0; 0; 0; 0];

        PREVIOUS.a = 0.15 * ones(N_blade_sections, 1);
        PREVIOUS.a_prime = 0.01 * ones(N_blade_sections, 1);

        opts = odeset('RelTol',1e-3,'AbsTol',1e-5);
        [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_dynamic(tt, YY, V_org, omega_org, f, ...
            StructuralParameters.M, StructuralParameters.C, false, StructuralParameters, AeroParameters, ...
            coupling, dynamic_inflow, PREVIOUS, dt), tspan, Y0, opts);


        N_time = size(Y_out,1);
        T_hist = zeros(N_time, 1);
        pitch_profile = zeros(N_time, 1);
        mid_idx = floor(N_blade_sections / 2);

        for it = 1:N_time
            x_t = Y_out(it,1:2)';
            dx_t = Y_out(it,3:4)';
            pitch_t = 10.45 + 5 * sin(2 * pi * f * t_out(it));

            if it == 1
                PREVIOUS.a = 0.15 * ones(N_blade_sections,1);
                PREVIOUS.a_prime = 0.01 * ones(N_blade_sections,1);
            else
                PREVIOUS.a = a_new;
                PREVIOUS.a_prime = a_prime_new;
            end

            % Call BEM solver
            [Rx, FN, FT, P, a_new, a_prime_new, a_steady] = BEM_dynamic( ...
                V_org, omega_org, zeros(N_blade_sections,1), zeros(N_blade_sections,1), ...
                pitch_t, coupling, dynamic_inflow, ...
                PREVIOUS.a, PREVIOUS.a_prime, dt);


            T_hist(it) = sum(FN) * 3;  % Total thrust over 3 blades
            pitch_profile(it) = pitch_t;

            if mod(it,5)==0 || it==1 || it==N_time
                fprintf('   Time step %3d/%3d | t = %.2f s | pitch = %.2f deg | T = %.1f N\n', ...
                        it, N_time, t_out(it), pitch_t, T_hist(it));
            end
        end

        T_hist_all{k} = T_hist;
        t_out_all{k} = t_out;
        pitch_all{k} = pitch_profile;
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

    fprintf('>> Plot generated for f = %.2f Hz\n', f);
end
