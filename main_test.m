% Wind Turbine Aeroelasticity Simulation
clc
close all;
clear all;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

%% Simulation setup
sinusoidal = true;
dynamic_inflow = false; % Set to true if you want to include dynamic inflow
vinduced = 0; % Initial induced velocity
K_CG = false; % Set to true if you want to include centrifugal and gravity stiffening
dt = 0.02;
tf = 50;
t = 0:dt:tf;
psi = 0; % Assuming blade starts vertical, 0 radians

if sinusoidal
    OperationalParameters.V_sin = 15+0.5*cos(1.267*t) + 0.085*cos(2.534*t)+ 0.015*cos(3.801*t);

    % Find the index of a specific wind speed in OperationalParameters.v0_values
    target_windspeed = 15; % example wind speed to find
    [~, windspeed_idx] = min(abs(OperationalParameters.v0_values - target_windspeed));

    pitch = OperationalParameters.pitch_values(windspeed_idx);
    omega = OperationalParameters.omega_values(windspeed_idx);

    V = OperationalParameters.V_sin(1)*ones(size(AeroParameters.radius_aero));
    omega = omega*ones(size(AeroParameters.radius_aero));
    omega_org = omega;
    V_org = V(1);

    % Initial conditions
    Y0 = [0; 0; 0; 0]; % [x1; x2; dx1; dx2]
    if K_CG
        psi = 0;
        total_K = get_total_K(StructuralParameters, omega(1), psi);
    else
        total_K = StructuralParameters.K;
    end

    % ODE45 integration
    [t_out, Y_out] = ode45(@(tt, YY) odefun_blade(tt, YY, V, omega, pitch, StructuralParameters.M, StructuralParameters.C, total_K, AeroParameters), t, Y0);

    x = Y_out(:,1:2)';
    dx = Y_out(:,3:4)';

    % For plotting tip deflection over time (optional)
    tip_deflection = [x(1,:)*StructuralParameters.phi_1flap(end); x(2,:)*StructuralParameters.phi_1edge(end)];

else
    % Only simulate for wind speeds 3, 12, 22 m/s
    selected_speeds = [3, 12, 22];
    [~, selected_indices] = arrayfun(@(v) min(abs(OperationalParameters.v0_values - v)), selected_speeds);

    tip_deflection = zeros(2, length(selected_indices));

    for idx = 1:length(selected_indices)
        i = selected_indices(idx);

        if idx == 1
            hWait = waitbar(0, 'Running simulation...');
        end

        waitbar(idx / length(selected_indices), hWait, ...
            sprintf('Running simulation... %5.1f%%', (idx / length(selected_indices)) * 100));

        if idx == length(selected_indices)
            close(hWait);
        end

        V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
        omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
        pitch = OperationalParameters.pitch_values(i);

        if K_CG
            psi = 0;
            total_K = get_total_K(StructuralParameters, omega_org(1), psi);
        else
            total_K = StructuralParameters.K;
        end

        Y0 = [0; 0; 0; 0]; % [x1; x2; dx1; dx2]
        [t_out, Y_out] = ode45(@(tt, YY) odefun_blade(tt, YY, V_org, omega_org, pitch, StructuralParameters.M, StructuralParameters.C, total_K, AeroParameters), t, Y0);

        x = Y_out(:,1:2)';
        dx = Y_out(:,3:4)';

        tip_deflection(:, idx) = [x(1,end)*StructuralParameters.phi_1flap(end); x(2,end)*StructuralParameters.phi_1edge(end)];
    end

    figure;
    plot(selected_speeds, tip_deflection(1,:), 'LineWidth', 1.5,'Marker','x'); hold on;
    plot(selected_speeds, tip_deflection(2,:), 'LineWidth', 1.5, 'Marker', 'x'); 
    grid on;
    legend('Flapwise', 'Edgewise');
    xlabel('Wind Speed [m/s]');
    ylabel('Tip Deflection [m]');
    title('Tip Deflection at Selected Wind Speeds');
end

%% Plotting (for time simulation, not for the 3-speed plot)
if sinusoidal
    figure;

    % Use default MATLAB color order
    co = get(gca, 'ColorOrder');

    % Compute average displacement in each direction
    avg_flap = mean(x(1,:));
    avg_edge = mean(x(2,:));

    % Flapwise displacement and velocity
    subplot(3,1,1);
    plot(t_out, x(1,:), 'LineWidth', 2, 'Color', co(1,:)); hold on;
    plot(t_out, dx(1,:), '--', 'LineWidth', 2, 'Color', co(1,:));
    plot(t_out, avg_flap*ones(size(t_out)), ':', 'LineWidth', 1.5, 'Color', 'k'); % average as dotted line
    hold off;
    xlabel('Time [s]');
    ylabel('Flapwise');
    legend('Displacement', 'Velocity');
    title('Flapwise Displacement and Velocity vs Time');
    grid on;

    % Edgewise displacement and velocity
    subplot(3,1,2);
    plot(t_out, x(2,:), 'LineWidth', 2, 'Color', co(2,:)); hold on;
    plot(t_out, dx(2,:), '--', 'LineWidth', 2, 'Color', co(2,:));
    plot(t_out, avg_edge*ones(size(t_out)), ':', 'LineWidth', 1.5, 'Color', 'k'); % average as dotted line
    hold off;
    xlabel('Time [s]');
    ylabel('Edgewise');
    legend('Displacement', 'Velocity');
    title('Edgewise Displacement and Velocity vs Time');
    grid on;

    subplot(3,1,3);
    plot(t, OperationalParameters.V_sin, 'LineWidth', 2, 'Color', co(3,:));
    xlabel('Time [s]');
    ylabel('Wind Speed [m/s]');
    grid on;
    title('Wind Speed');
end