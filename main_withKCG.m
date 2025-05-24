% Wind Turbine Aeroelasticity Simulation: Compare with and without K_CG
clc
close all;
clear all;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

dt = 0.25;
tf = 10;
t = 0:dt:tf;

tip_deflection_KCG = zeros(2, length(OperationalParameters.v0_values));
tip_deflection_struct = zeros(2, length(OperationalParameters.v0_values));

hWait = waitbar(0, 'Running simulation...');

for i = 1:length(OperationalParameters.v0_values)
    % Update progress bar
    waitbar(i / length(OperationalParameters.v0_values), hWait, ...
        sprintf('Running simulation... %5.1f%%', (i / length(OperationalParameters.v0_values)) * 100));
        
    % --- Common setup ---
    x_KCG = zeros(2, length(t));
    dx_KCG = zeros(2, length(t));
    ddx_KCG = zeros(2, length(t));
    x_struct = zeros(2, length(t));
    dx_struct = zeros(2, length(t));
    ddx_struct = zeros(2, length(t));
    V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
    omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
    pitch = OperationalParameters.pitch_values(i);
    V = V_org;
    omega = omega_org;
    psi = 0;

    % --- With centrifugal & gravity stiffening ---
    for j = 1:length(t)-1
        psi = psi + omega_org(1)*dt;
        total_K = get_total_K(StructuralParameters, omega_org(1), psi);
        [x_KCG(:,j+1), dx_KCG(:,j+1), ddx_KCG(:,j+1)] = runge_kutta_step(x_KCG(:,j), dx_KCG(:,j), ddx_KCG(:,j), dt, V, omega, pitch, StructuralParameters.M, StructuralParameters.C, total_K, AeroParameters);
        velocity = [dx_KCG(1,j) * AeroParameters.phi_1flap_aero; dx_KCG(2,j) * AeroParameters.phi_1edge_aero];
        V_inplane = velocity(1,:) .* cos(deg2rad(pitch + AeroParameters.twist_aero)) - velocity(2,:) .* sin(deg2rad(pitch + AeroParameters.twist_aero));
        V_outplane = velocity(1,:) .* sin(deg2rad(pitch + AeroParameters.twist_aero)) + velocity(2,:) .* cos(deg2rad(pitch + AeroParameters.twist_aero));
        V = V_org - V_outplane;
        omega = omega_org - V_inplane ./ AeroParameters.radius_aero;
    end
    tip_deflection_KCG(:, i) = x_KCG(:, end);

    % --- Structural only (no centrifugal/gravity stiffening) ---
    V = V_org;
    omega = omega_org;
    psi = 0;
    for j = 1:length(t)-1
        total_K = StructuralParameters.K;
        [x_struct(:,j+1), dx_struct(:,j+1), ddx_struct(:,j+1)] = runge_kutta_step(x_struct(:,j), dx_struct(:,j), ddx_struct(:,j), dt, V, omega, pitch, StructuralParameters.M, StructuralParameters.C, total_K, AeroParameters);
        velocity = [dx_struct(1,j) * AeroParameters.phi_1flap_aero; dx_struct(2,j) * AeroParameters.phi_1edge_aero];
        V_inplane = velocity(1,:) .* cos(pitch + AeroParameters.twist_aero') - velocity(2,:) .* sin(pitch + AeroParameters.twist_aero');
        V_outplane = velocity(1,:) .* sin(pitch + AeroParameters.twist_aero') + velocity(2,:) .* cos(pitch + AeroParameters.twist_aero');
        V = V_org - V_outplane;
        omega = omega_org - V_inplane ./ AeroParameters.radius_aero;
    end
    tip_deflection_struct(:, i) = x_struct(:, end);
end



close(hWait);

% --- Plot comparison ---
figure;
plot(OperationalParameters.v0_values, tip_deflection_KCG(1,:), 'b-*', 'LineWidth', 1.5); hold on;
plot(OperationalParameters.v0_values, tip_deflection_struct(1,:), 'r--o', 'LineWidth', 1.5);
plot(OperationalParameters.v0_values, tip_deflection_KCG(2,:), 'g-*', 'LineWidth', 1.5);
plot(OperationalParameters.v0_values, tip_deflection_struct(2,:), 'm--o', 'LineWidth', 1.5);
grid on;
legend('Flapwise (K_{CG})', 'Flapwise (struct)', 'Edgewise (K_{CG})', 'Edgewise (struct)');
xlabel('Wind Speed [m/s]');
ylabel('Tip Deflection [m]');
title('Tip Deflection Comparison: With and Without Centrifugal/Gravity Stiffening');
