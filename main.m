% Wind Turbine Aeroelasticity Simulation
clc
close all;
clear all;


[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

%% Simulation setup

dt = 0.25;
tf = 10;
t = 0:dt:tf;

tip_deflection = zeros(2, length(OperationalParameters.v0_values));

for i = 1:length(OperationalParameters.v0_values)
    if i == 1
        hWait = waitbar(0, 'Running simulation...');
    end

    if mod(i, max(1, floor((length(OperationalParameters.v0_values) - 1) / 100))) == 0 || i == length(OperationalParameters.v0_values)
        waitbar(i / length(OperationalParameters.v0_values), hWait, ...
            sprintf('Running simulation... %5.1f%%', (i / length(OperationalParameters.v0_values)) * 100));
    end

    if i == length(OperationalParameters.v0_values)
        close(hWait);
    end

    
    % defitnition of arrays
    x = zeros(2, length(t));
    dx = zeros(2, length(t));
    ddx = zeros(2, length(t));

    V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
    omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
    pitch = OperationalParameters.pitch_values(i);

    V = V_org;
    omega = omega_org;
    time_loop = false;

    for j = 1:length(t)-1
        % Use MATLAB's built-in waitbar for progress indication
        %if j == 1 && time_loop == true
        %    hWait = waitbar(0, 'Running simulation...');
        %end
        %if mod(j, max(1, floor((length(t)-1)/100))) == 0 || j == length(t)-1 && time_loop == true
        %    waitbar(j/(length(t)-1), hWait, sprintf('Running simulation... %5.1f%%', (j/(length(t)-1))*100));
        %end
        %if j == length(t)-1 && time_loop == true
        %    close(hWait);
        %end
        
        % Runge-Kutta integration
        [x(:,j+1), dx(:,j+1), ddx(:,j+1)] = runge_kutta_step(x(:,j), dx(:,j), ddx(:,j), dt, V, omega, pitch, StructuralParameters.M, StructuralParameters.C, StructuralParameters.K, AeroParameters);

        
        % Velocity coupling
        velocity = [dx(1,j) * AeroParameters.phi_1flap_aero; dx(2,j) * AeroParameters.phi_1edge_aero];
        V_inplane = velocity(1,:) .* cos(pitch + AeroParameters.twist_aero') - velocity(2,:) .* sin(pitch + AeroParameters.twist_aero');
        V_outplane = velocity(1,:) .* sin(pitch + AeroParameters.twist_aero') + velocity(2,:) .* cos(pitch + AeroParameters.twist_aero');
        
        %V = V_org - V_outplane;
        %omega = omega_org - V_inplane ./ AeroParameters.radius_aero;

    end
    tip_deflection(:, i) = x(:, end);
    
end

figure;

plot(OperationalParameters.v0_values, tip_deflection(1,:), 'LineWidth', 1.5,'Marker','*'); hold on;
plot(OperationalParameters.v0_values, tip_deflection(2,:), 'LineWidth', 1.5, 'Marker', '*'); 
grid on;
legend('Flapwise', 'Edgewise');
xlabel('Wind Speed [m/s]');
ylabel('Tip Deflection [m]');

% Calculate oscillating frequencies from displacement signals
% Remove initial transient (e.g., first 1 second)
transient_time = 1; % seconds
transient_idx = find(t >= transient_time, 1);


%% Plotting
figure;

% Use default MATLAB color order
co = get(gca, 'ColorOrder');

% Compute average displacement in each direction
avg_flap = mean(x(1,:));
avg_edge = mean(x(2,:));

% Flapwise displacement and velocity
subplot(2,1,1);
plot(t, x(1,:), 'LineWidth', 2, 'Color', co(1,:)); hold on;
plot(t, dx(1,:), '--', 'LineWidth', 2, 'Color', co(1,:));
plot(t, avg_flap*ones(size(t)), ':', 'LineWidth', 1.5, 'Color', 'k'); % average as dotted line
hold off;
xlabel('Time [s]');
ylabel('Flapwise');
legend('Displacement', 'Velocity');
title('Flapwise Displacement and Velocity vs Time');
grid on;

% Edgewise displacement and velocity
subplot(2,1,2);
plot(t, x(2,:), 'LineWidth', 2, 'Color', co(2,:)); hold on;
plot(t, dx(2,:), '--', 'LineWidth', 2, 'Color', co(2,:));
plot(t, avg_edge*ones(size(t)), ':', 'LineWidth', 1.5, 'Color', 'k'); % average as dotted line
hold off;
xlabel('Time [s]');
ylabel('Edgewise');
legend('Displacement', 'Velocity');
title('Edgewise Displacement and Velocity vs Time');
grid on;
