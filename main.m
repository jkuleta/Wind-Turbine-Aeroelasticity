% Wind Turbine Aeroelasticity Simulation
clc;
close all;
clear;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

%% Simulation setup
dynamic_inflow = false; % Enable if needed
coupling = true;
vinduced = 0;
dt = 0.2;
tf = 100;
tspan = [0 tf];

% Output storage
K_CG_options = [true, false];
tip_deflection_all = cell(1, 2);
time_hist = cell(1, 2);

for k = 1:2
    K_CG = K_CG_options(k);
    tip_deflection = zeros(2, length(OperationalParameters.v0_values));

    for i = 1:length(OperationalParameters.v0_values)
        if i == 1
            hWait = waitbar(0, sprintf('Running simulation (K\\_CG = %d)...', K_CG));
        end

        if mod(i, max(1, floor((length(OperationalParameters.v0_values) - 1) / 100))) == 0 || i == length(OperationalParameters.v0_values)
            waitbar(i / length(OperationalParameters.v0_values), hWait, ...
                sprintf('Running simulation (K\\_CG = %d)... %5.1f%%', K_CG, (i / length(OperationalParameters.v0_values)) * 100));
        end

        if i == length(OperationalParameters.v0_values)
            close(hWait);
        end

        % Inputs
        V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
        omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
        pitch = OperationalParameters.pitch_values(i);
        Y0 = [0; 0; 0; 0; 0];  % [x1; x2; dx1; dx2; psi]

        opts = odeset('RelTol',1e-3,'AbsTol',1e-5);

        % Solve ODE
        [t_out, Y_out] = ode45(@(tt, YY) odefun_blade(tt, YY, V_org, omega_org, pitch, ...
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




        % Store time-domain data from the **last** wind speed for this K_CG
        if i == length(OperationalParameters.v0_values)
            time_hist{k}.t_out = t_out;
            time_hist{k}.x = Y_out(:,1:2)';
            time_hist{k}.dx = Y_out(:,3:4)';
            time_hist{k}.psi = Y_out(:,5);
        end
    end

    tip_deflection_all{k} = tip_deflection;
end

%% Plot: Tip Deflection vs Wind Speed
figure;
plot(OperationalParameters.v0_values, tip_deflection_all{1}(1,:), 'b-x', 'DisplayName', 'Flapwise w/ K_{CG}'); hold on;
plot(OperationalParameters.v0_values, tip_deflection_all{2}(1,:), 'b--o', 'DisplayName', 'Flapwise w/o K_{CG}');
plot(OperationalParameters.v0_values, tip_deflection_all{1}(2,:), 'r-x', 'DisplayName', 'Edgewise w/ K_{CG}');
plot(OperationalParameters.v0_values, tip_deflection_all{2}(2,:), 'r--o', 'DisplayName', 'Edgewise w/o K_{CG}');
xlabel('Wind Speed [m/s]');
ylabel('Tip Deflection [m]');
title('Tip Deflection vs Wind Speed with/without K_{CG}');
legend('Location', 'best');
grid on;

%% Plot: Time-Domain Comparison for K_CG = true and false
figure;
titles = {'K_{CG} = true', 'K_{CG} = false'};

for k = 1:2
    t_out = time_hist{k}.t_out;
    x     = time_hist{k}.x;
    dx    = time_hist{k}.dx;
    psi   = time_hist{k}.psi;

    avg_flap = mean(x(1,:));
    avg_edge = mean(x(2,:));

    subplot(3,2,1 + (k-1));
    plot(t_out, x(1,:), 'LineWidth', 2); hold on;
    plot(t_out, dx(1,:), '--', 'LineWidth', 2);
    yline(avg_flap, ':k', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Flapwise');
    title(['Flapwise - ' titles{k}]);
    legend('Displacement', 'Velocity');
    grid on;

    subplot(3,2,3 + (k-1));
    plot(t_out, x(2,:), 'LineWidth', 2); hold on;
    plot(t_out, dx(2,:), '--', 'LineWidth', 2);
    yline(avg_edge, ':k', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Edgewise');
    title(['Edgewise - ' titles{k}]);
    legend('Displacement', 'Velocity');
    grid on;

    subplot(3,2,5 + (k-1));
    plot(t_out, psi, 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('\psi [rad]');
    title(['Azimuth Angle (\psi) - ' titles{k}]);
    grid on;
end
