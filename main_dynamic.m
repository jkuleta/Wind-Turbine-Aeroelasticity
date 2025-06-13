
% % % =============================
% % % Main Simulation Script (updated for dynamic a state with PREVIOUSTIME)
% % % =============================
% % clc;
% % clear;
% % close all;

% % [StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% % %% Simulation setup
% % periodic = true;
% % vinduced = 0;
% % dt = 0.2;
% % tf = 10;
% % tspan = 0:dt:tf;

% % frequencies = [0.2]; %0.05, 0.2, 0.5];
% % dynamic_inflow_options = [true, false];
% % dynamic_inflow_labels = {'Dynamic Inflow ON', 'Dynamic Inflow OFF'};
% % colors_dynamic = {'b', 'r'};

% % i = 19; % index for 15 m/s

% % for f_idx = 1:length(frequencies)
% %     f = frequencies(f_idx);
% %     fprintf('\n===== Starting simulation for f = %.2f Hz =====\n', f);

% %     T_hist_all = cell(1,2);
% %     t_out_all = cell(1,2);
% %     pitch_all = cell(1,2);

% %     for k = 1:2
% %         fprintf('-- Simulating with: %s\n', dynamic_inflow_labels{k});
% %         coupling = true;
% %         dynamic_inflow = dynamic_inflow_options(k);

% %         V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
% %         omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));

% %         N_blade_sections = length(AeroParameters.radius_aero);
% %         Y0 = [0; 0; 0; 0; 0];

% %         PREVIOUS.a = 0.15 * ones(N_blade_sections, 1);
% %         PREVIOUS.a_prime = 0.01 * ones(N_blade_sections, 1);

% %         opts = odeset('RelTol',1e-3,'AbsTol',1e-5);
% %         [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_dynamic(tt, YY, V_org, omega_org, f, ...
% %             StructuralParameters.M, StructuralParameters.C, false, StructuralParameters, AeroParameters, ...
% %             coupling, dynamic_inflow, PREVIOUS, dt), tspan, Y0, opts);


% %         N_time = size(Y_out,1);
% %         T_hist = zeros(N_time, 1);
% %         pitch_profile = zeros(N_time, 1);
% %         mid_idx = floor(N_blade_sections / 2);

% %         for it = 1:N_time
% %             x_t = Y_out(it,1:2)';
% %             dx_t = Y_out(it,3:4)';
% %             pitch_t = 10.45 + 5 * sin(2 * pi * f * t_out(it));

% %             if it == 1
% %                 PREVIOUS.a = 0.15 * ones(N_blade_sections,1);
% %                 PREVIOUS.a_prime = 0.01 * ones(N_blade_sections,1);
% %             else
% %                 PREVIOUS.a = a_new;
% %                 PREVIOUS.a_prime = a_prime_new;
% %             end

% %             % Call BEM solver
% %             [Rx, FN, FT, P, a_new, a_prime_new, a_steady] = BEM_dynamic( ...
% %                 V_org, omega_org, zeros(N_blade_sections,1), zeros(N_blade_sections,1), ...
% %                 pitch_t, coupling, dynamic_inflow, ...
% %                 PREVIOUS.a, PREVIOUS.a_prime, dt);


% %             T_hist(it) = sum(FN) * 3;  % Total thrust over 3 blades
% %             pitch_profile(it) = pitch_t;

% %             if mod(it,5)==0 || it==1 || it==N_time
% %                 fprintf('   Time step %3d/%3d | t = %.2f s | pitch = %.2f deg | T = %.1f N\n', ...
% %                         it, N_time, t_out(it), pitch_t, T_hist(it));
% %             end
% %         end

% %         T_hist_all{k} = T_hist;
% %         t_out_all{k} = t_out;
% %         pitch_all{k} = pitch_profile;
% %     end

% %     figure;
% %     subplot(2,1,1);
% %     for k = 1:2
% %         plot(t_out_all{k}, T_hist_all{k}, 'Color', colors_dynamic{k}, ...
% %             'LineWidth', 1.5, 'DisplayName', dynamic_inflow_labels{k}); hold on;
% %     end
% %     ylabel('Rotor Thrust [N]');
% %     title(['Rotor Thrust vs Time, f = ' num2str(f) ' Hz']);
% %     legend; grid on;

% %     subplot(2,1,2);
% %     for k = 1:2
% %         plot(pitch_all{k}, T_hist_all{k}, 'Color', colors_dynamic{k}, ...
% %             'LineWidth', 1.5, 'DisplayName', dynamic_inflow_labels{k}); hold on;
% %     end
% %     ylabel('Rotor Thrust [N]');
% %     title(['Rotor Thrust vs Pitch Angle, f = ' num2str(f) ' Hz']);
% %     legend; grid on;

% %     fprintf('>> Plot generated for f = %.2f Hz\n', f);
% % end


% clc;
% clear;
% close all;

% [StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% %% Simulation setup
% dt = 0.2;
% tf = 10;
% tspan = 0:dt:tf;

% frequencies = [0.2];  % Frequency for periodic pitch
% dynamic_inflow_options = [true, false];
% dynamic_inflow_labels = {'Dynamic Inflow ON', 'Dynamic Inflow OFF'};

% i = 19;  % Wind index (e.g., 15 m/s)
% N_blade_sections = length(AeroParameters.radius_aero);

% for f_idx = 1:length(frequencies)
%     f = frequencies(f_idx);
%     fprintf('\n===== Starting simulation for f = %.2f Hz =====\n', f);

%     % Allocate storage
%     T_hist_all = cell(1, 2);
%     t_out_all  = cell(1, 2);
%     pitch_all  = cell(1, 2);

%     for k = 1:2
%         fprintf('-- Simulating with: %s\n', dynamic_inflow_labels{k});
%         dynamic_inflow = dynamic_inflow_options(k);
%         coupling = true;

%         V_org = OperationalParameters.v0_values(i) * ones(N_blade_sections, 1);
%         omega_org = OperationalParameters.omega_values(i) * ones(N_blade_sections, 1);

%         % Initial condition: [x1, x2, dx1, dx2, psi, a(1:N), a'(1:N)]
%         x0 = [0; 0];
%         dx0 = [0; 0];
%         psi0 = 0;
%         a0 = 0.15 * ones(N_blade_sections, 1);
%         ap0 = 0.01 * ones(N_blade_sections, 1);
%         Y0 = [x0; dx0; psi0; a0; ap0];

%         opts = odeset('RelTol',1e-3,'AbsTol',1e-5);
%         [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_dynamic(tt, YY, V_org, omega_org, f, ...
%             StructuralParameters.M, StructuralParameters.C, false, ...
%             StructuralParameters, AeroParameters, ...
%             coupling, dynamic_inflow, dt), ...
%             tspan, Y0, opts);

%         % Indices for a and a'
%         a_start = 6;
%         a_end = 5 + N_blade_sections;
%         ap_start = a_end + 1;
%         ap_end = ap_start + N_blade_sections - 1;

%         % Store pitch and thrust
%         T_hist_all{k} = zeros(length(t_out), 1);
%         pitch_all{k}  = zeros(length(t_out), 1);

%         for m = 1:length(t_out)
%             pitch_tmp = 10.45 + 5 * sin(2 * pi * f * t_out(m));
%             pitch_all{k}(m) = pitch_tmp;

%             PREVIOUS.a = Y_out(m, a_start:a_end)';
%             PREVIOUS.a_prime = Y_out(m, ap_start:ap_end)';

%             [~, ~, ~, T_tmp, ~] = compute_aero_force_dynamic( ...
%                 Y_out(m,1:2)', Y_out(m,3:4)', V_org, omega_org, pitch_tmp, ...
%                 AeroParameters.radius_aero, AeroParameters.twist_aero, ...
%                 AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
%                 coupling, dynamic_inflow, PREVIOUS, dt);

%             T_hist_all{k}(m) = T_tmp;
%         end

%         t_out_all{k} = t_out;
%     end

%     % Plot Thrust vs Pitch
%     % --- Thrust vs Pitch ---
%     figure;
%     for k = 1:2
%         plot(pitch_all{k}, T_hist_all{k}, 'LineWidth', 1.5, 'DisplayName', dynamic_inflow_labels{k}); hold on;
%     end
%     xlabel('Pitch [deg]');
%     ylabel('Thrust [N]');
%     title(sprintf('Thrust vs Pitch (f = %.2f Hz)', f));
%     legend('Location', 'best');
%     grid on;

%     % --- Thrust vs Time ---
%     figure;
%     for k = 1:2
%         plot(t_out_all{k}, T_hist_all{k}, 'LineWidth', 1.5, 'DisplayName', dynamic_inflow_labels{k}); hold on;
%     end
%     xlabel('Time [s]');
%     ylabel('Thrust [N]');
%     title(sprintf('Thrust vs Time (f = %.2f Hz)', f));
%     legend('Location', 'best');
%     grid on;

% end

% ==============================================================
% main_dynamic_inflow.m
% ==============================================================

clc; clear; close all;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% --- simulation controls -----------------------------------------------
dt   = 0.2;        % [s]
tf   = 30;         % total time
tspan = 0:dt:tf;

frequencies            = [0.2];            % Hz
dynamic_inflow_options = [true, false];
dynamic_inflow_labels  = {'Dynamic Inflow ON', 'Dynamic Inflow OFF'};
colors_dynamic         = {'b', 'r'};

i_wind = 19;                                 % index → 15 m/s
N_sec  = numel(AeroParameters.radius_aero);  % blade sections

for f_idx = 1:numel(frequencies)
    f = frequencies(f_idx);
    fprintf('\n=====  f = %.2f Hz  =====\n', f);

    T_hist_all = cell(1,2);
    t_out_all  = cell(1,2);
    pitch_all  = cell(1,2);

    for k = 1:2
        dynamic_inflow = dynamic_inflow_options(k);
        fprintf('-- %s\n', dynamic_inflow_labels{k});

        % operating point ------------------------------------------------
        V_org     = OperationalParameters.v0_values(i_wind)   * ones(N_sec,1);
        omega_org = OperationalParameters.omega_values(i_wind)* ones(N_sec,1);
        coupling  = true;

        % initial state --------------------------------------------------
        x0   = [0;0];  dx0  = [0;0];  psi0 = 0;
        a0   = 0.15 * ones(N_sec,1);
        ap0  = 0.01 * ones(N_sec,1);

        Y0  = [x0; dx0; psi0; a0; ap0];   % full state

        % integrate ------------------------------------------------------
        opts = odeset('RelTol',1e-3,'AbsTol',1e-5);
        [t_out, Y_out] = ode45(@(tt,YY) odefun_blade_dynamic( ...
                                   tt, YY, V_org, omega_org, f, ...
                                   StructuralParameters.M, StructuralParameters.C, ...
                                   false, StructuralParameters, AeroParameters, ...
                                   coupling, dynamic_inflow, dt), ...
                               tspan, Y0, opts);

        % thrust & pitch history ----------------------------------------
        a_start    = 6;              a_end = 5+N_sec;
        ap_start   = a_end+1;        ap_end= ap_start+N_sec-1;

        T_hist  = zeros(numel(t_out),1);
        pitch_h = zeros(numel(t_out),1);

        for m = 1:numel(t_out)
            pitch_m = 10.45 + 5*sin(2*pi*f*t_out(m));
            pitch_h(m) = pitch_m;

            if dynamic_inflow
                PREV.a       = Y_out(m,a_start:a_end).';
                PREV.a_prime = Y_out(m,ap_start:ap_end).';
                [~,~,~,T_m,~] = compute_aero_force_dynamic( ...
                       Y_out(m,1:2).', Y_out(m,3:4).', V_org, omega_org, pitch_m, ...
                       AeroParameters.radius_aero, AeroParameters.twist_aero, ...
                       AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
                       coupling, true, PREV, dt);
            else
                [~,~,~,T_m,~] = compute_aero_force_dynamic( ...
                    Y_out(m,1:2).', Y_out(m,3:4).', V_org, omega_org, pitch_m, ...
                    AeroParameters.radius_aero, AeroParameters.twist_aero, ...
                    AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
                    coupling, false, [], dt);

            end
            T_hist(m) = T_m;
        end

        % store ----------------------------------------------------------
        T_hist_all{k} = T_hist;
        t_out_all{k}  = t_out;
        pitch_all{k}  = pitch_h;
    end

    % ------------ plots -------------------------------------------------
    figure; hold on;
    for k = 1:2
        plot(pitch_all{k}, T_hist_all{k}, colors_dynamic{k}, 'LineWidth',1.5, ...
             'DisplayName', dynamic_inflow_labels{k});
    end
    xlabel('Pitch [deg]'); ylabel('Thrust [N]');
    title(sprintf('Thrust vs Pitch (f = %.2f Hz)',f));
    legend show; grid on;

    figure; hold on;
    for k = 1:2
        plot(t_out_all{k}, T_hist_all{k}, colors_dynamic{k}, 'LineWidth',1.5, ...
             'DisplayName', dynamic_inflow_labels{k});
    end
    xlabel('Time [s]'); ylabel('Thrust [N]');
    title(sprintf('Thrust vs Time (f = %.2f Hz)',f));
    legend show; grid on;
end
