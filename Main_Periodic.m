% Wind Turbine Aeroelasticity Simulation
clc;
close all;
clear;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

%% Simulation setup
dynamic_inflow = false; % Enable if needed
coupling = true;
periodic = true; % Enable periodic boundary conditions
% V0 = 15 + 0.5*cos(1.267*t) + 0.085*cos(2.534*t) + 0.015*cos(3.801*t);

vinduced = 0;
dt = 0.2;
tf = 10;
tspan = [0 tf];

% Output storage
K_CG_options = [false, false];
Coupling_options = [true, false];
tip_deflection_all = cell(1, 2);
time_hist = cell(1, 2);

for k = 1:2
    K_CG = K_CG_options(k);
    coupling = Coupling_options(k);
    tip_deflection = zeros(2, length(OperationalParameters.v0_values));


    % Inputs
    if periodic
        i = 14; % index for 15 m/s wind speed
        V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
        omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
        pitch = OperationalParameters.pitch_values(i);
        Y0 = [0; 0; 0; 0; 0];  % [x1; x2; dx1; dx2; psi]
    else
        disp('this is not periodic');
    end

    opts = odeset('RelTol',1e-3,'AbsTol',1e-5);

    % Solve ODE
        [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_periodic(tt, YY, V_org, omega_org, pitch, ...
            StructuralParameters.M, StructuralParameters.C, K_CG, StructuralParameters, AeroParameters, coupling), ...
            tspan, Y0, opts);

    % Extract results
    x = Y_out(:,1:2)';
    t_out = t_out(:);

    % Tip deflection time history
    tip_flap = x(1,:) * StructuralParameters.phi_1flap(end);
    tip_edge = x(2,:) * StructuralParameters.phi_1edge(end);

    % Use findpeaks to estimate period of last cycle
    [~, locs] = findpeaks(tip_flap, t_out, 'MinPeakProminence', 0.01);

    % Require at least 2 peaks
    if length(locs) >= 2
        T_est = locs(end) - locs(end-1);  % Time between last two peaks
    else
        T_est = 1;  % fallback (1 second)
    end

    % Get indices of final cycle
    t_final = t_out(end);
    idx_last_cycle = find(t_out >= t_final - T_est);

    % Compute average absolute deflection in that cycle
    max_flap_tip = mean(abs(tip_flap(idx_last_cycle)));
    max_edge_tip = mean(abs(tip_edge(idx_last_cycle)));

    tip_deflection(:, i) = [max_flap_tip; max_edge_tip];


    M_flap_root = StructuralParameters.flap_stiffness_distribution(1) * StructuralParameters.ddphi_1flap(1) * x(1,:);
    M_edge_root = StructuralParameters.edge_stiffness_distribution(1) * StructuralParameters.ddphi_1edge(1) * x(2,:);

    % Store time-domain data from the **last** wind speed for this K_CG
    time_hist{k}.t_out = t_out;
    time_hist{k}.x = Y_out(:,1:2)';
    time_hist{k}.dx = Y_out(:,3:4)';
    time_hist{k}.psi = Y_out(:,5);
    time_hist{k}.M_flap_root = M_flap_root;
    time_hist{k}.M_edge_root = M_edge_root;

    tip_deflection_all{k} = tip_deflection;
end

% Plot: Time-Domain Comparison for Coupling = true and false
figure;
titles = {'Coupling = true', 'Coupling = false'};

for k = 1:2
    t_out = time_hist{k}.t_out;
    x     = time_hist{k}.x;
    dx    = time_hist{k}.dx;
    psi   = time_hist{k}.psi;

    subplot(2,2,1 + (k-1));
    plot(t_out, x(1,:), 'LineWidth', 2); hold on;
    plot(t_out, dx(1,:), '--', 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('Flapwise');
    title(['Flapwise - ' titles{k}]);
    legend('Displacement', 'Velocity');
    grid on;

    subplot(2,2,3 + (k-1));
    plot(t_out, x(2,:), 'LineWidth', 2); hold on;
    plot(t_out, dx(2,:), '--', 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('Edgewise');
    title(['Edgewise - ' titles{k}]);
    legend('Displacement', 'Velocity');
    grid on;
end

% Plot: Tip Deflection vs Time for Coupling = true and false
figure;
plot(time_hist{1}.t_out, time_hist{1}.x(1,:), 'b-', 'LineWidth', 2); hold on;
plot(time_hist{2}.t_out, time_hist{2}.x(1,:), 'r--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Flapwise Tip Deflection [m]');
title('Flapwise Tip Deflection: Coupling vs No Coupling');
legend('Coupling', 'No Coupling');
grid on;

figure;
plot(time_hist{1}.t_out, time_hist{1}.x(2,:), 'b-', 'LineWidth', 2); hold on;
plot(time_hist{2}.t_out, time_hist{2}.x(2,:), 'r--', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Edgewise Tip Deflection [m]');
title('Edgewise Tip Deflection: Coupling vs No Coupling');
legend('Coupling', 'No Coupling');
grid on;

% figure;
% plot(t_out, M_flap_root, 'b-', 'LineWidth', 2); hold on;
% plot(t_out, M_edge_root, 'r-', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Bending Moment [Nm]');
% title('Bending Moment vs Time');
% legend('Flapwise Root Moment', 'Edgewise Root Moment');
% grid on;

% figure;
% plot(t_out, tip_flap, 'b-', 'LineWidth', 2); hold on;
% plot(t_out, tip_edge, 'r-', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Tip Deflection [m]');
% title('Tip Deflection vs Time');
% legend('Flapwise Tip', 'Edgewise Tip');
% grid on;


% % FFT of bending moments
% L = length(t_out);
% Fs = 1 / mean(diff(t_out));  % Sampling frequency

% f = Fs * (0:(L/2)) / L;

% Y_flap = fft(M_flap_root);
% Y_edge = fft(M_edge_root);

% P_flap = abs(Y_flap / L);
% P_edge = abs(Y_edge / L);

% P_flap = P_flap(1:L/2+1);
% P_edge = P_edge(1:L/2+1);

% P_flap(2:end-1) = 2*P_flap(2:end-1);
% P_edge(2:end-1) = 2*P_edge(2:end-1);

% figure;
% loglog(f, P_flap, 'b-', 'LineWidth', 2); hold on;
% loglog(f, P_edge, 'r-', 'LineWidth', 2);
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% title('Bending Moment Frequency Spectrum');
% legend('Flapwise Moment', 'Edgewise Moment');
% grid on;


% % FFT of tip deflections
% Y_tip_flap = fft(tip_flap);
% Y_tip_edge = fft(tip_edge);

% P_tip_flap = abs(Y_tip_flap / L);
% P_tip_edge = abs(Y_tip_edge / L);

% P_tip_flap = P_tip_flap(1:L/2+1);
% P_tip_edge = P_tip_edge(1:L/2+1);

% P_tip_flap(2:end-1) = 2*P_tip_flap(2:end-1);
% P_tip_edge(2:end-1) = 2*P_tip_edge(2:end-1);

% figure;
% loglog(f, P_tip_flap, 'b-', 'LineWidth', 2); hold on;
% loglog(f, P_tip_edge, 'r-', 'LineWidth', 2);
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% title('Tip Deflection Frequency Spectrum');
% legend('Flapwise Tip', 'Edgewise Tip');
% grid on;


% % %% Plot: Tip Deflection vs Wind Speed
% % figure;
% % plot(OperationalParameters.v0_values, tip_deflection_all{1}(1,:), 'b-x', 'DisplayName', 'Flapwise w/ K_{CG}'); hold on;
% % plot(OperationalParameters.v0_values, tip_deflection_all{2}(1,:), 'b--o', 'DisplayName', 'Flapwise w/o K_{CG}');
% % plot(OperationalParameters.v0_values, tip_deflection_all{1}(2,:), 'r-x', 'DisplayName', 'Edgewise w/ K_{CG}');
% % plot(OperationalParameters.v0_values, tip_deflection_all{2}(2,:), 'r--o', 'DisplayName', 'Edgewise w/o K_{CG}');
% % xlabel('Wind Speed [m/s]');
% % ylabel('Tip Deflection [m]');
% % title('Tip Deflection vs Wind Speed with/without K_{CG}');
% % legend('Location', 'best');
% % grid on;

% % %% Plot: Time-Domain Comparison for K_CG = true and false
% % figure;
% % titles = {'K_{CG} = true', 'K_{CG} = false'};

% % for k = 1:2
% %     t_out = time_hist{k}.t_out;
% %     x     = time_hist{k}.x;
% %     dx    = time_hist{k}.dx;
% %     psi   = time_hist{k}.psi;

% %     avg_flap = mean(x(1,:));
% %     avg_edge = mean(x(2,:));

% %     subplot(2,2,1 + (k-1));
% %     plot(t_out, x(1,:), 'LineWidth', 2); hold on;
% %     plot(t_out, dx(1,:), '--', 'LineWidth', 2);
% %     yline(avg_flap, ':k', 'LineWidth', 1.5);
% %     xlabel('Time [s]'); ylabel('Flapwise');
% %     title(['Flapwise - ' titles{k}]);
% %     legend('Displacement', 'Velocity');
% %     grid on;

% %     subplot(2,2,3 + (k-1));
% %     plot(t_out, x(2,:), 'LineWidth', 2); hold on;
% %     plot(t_out, dx(2,:), '--', 'LineWidth', 2);
% %     yline(avg_edge, ':k', 'LineWidth', 1.5);
% %     xlabel('Time [s]'); ylabel('Edgewise');
% %     title(['Edgewise - ' titles{k}]);
% %     legend('Displacement', 'Velocity');
% %     grid on;

% % end
