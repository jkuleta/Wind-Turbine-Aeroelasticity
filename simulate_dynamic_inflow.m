% Simulate unsteady loads due to dynamic inflow for harmonic pitch

clc; clear; close all;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

V0 = 15 * ones(size(AeroParameters.radius_aero));
omega = interp1(OperationalParameters.v0_values, OperationalParameters.omega_values, 15, 'linear', 'extrap') * ones(size(AeroParameters.radius_aero));
dt = 0.2;
tf = 10;
t = 0:dt:tf;

f_list = [0.05, 0.2, 0.5]; % [Hz]
pitch_mean = 10.45;
pitch_amp = 5;

for ff = 1:length(f_list)
    f = f_list(ff);
    pitch_time = pitch_mean + pitch_amp * sin(2*pi*f*t);

    RotorThrust_dyn = zeros(size(t));
    RotorThrust_steady = zeros(size(t));

    % Initialize induction factors
    BS = importdata('Blade/Blade section/Blade section.dat').data;
    N_sections = size(BS,1); % use same number of sections as BEM_dynamic expects
    a_prev = zeros(N_sections, 1);
    a_prime_prev = zeros(N_sections, 1);

    for k = 1:length(t)
        display(['Time: ' num2str(t(k)) ' s, Pitch: ' num2str(pitch_time(k)) ' deg']);
        pitch_dyn = pitch_time(k);

        % Dynamic inflow
        [~, ~, ~, thrust_dyn, a_out, a_prime_out] = compute_aero_force_dynamic(...
            [0;0], [0;0], V0, omega, pitch_dyn, ...
            AeroParameters.radius_aero, AeroParameters.twist_aero, ...
            AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
            true, true, a_prev, a_prime_prev);

        a_prev = a_out;
        a_prime_prev = a_prime_out;
        RotorThrust_dyn(k) = thrust_dyn;

        % Quasi-steady inflow
        [~, ~, ~, thrust_steady, ~, ~] = compute_aero_force_dynamic([0;0], [0;0], V0, omega, pitch_dyn, ...
            AeroParameters.radius_aero, AeroParameters.twist_aero, ...
            AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, true, false, a_prev, a_prime_prev);
        RotorThrust_steady(k) = thrust_steady;
    end


    % Plot thrust and pitch vs time using subplots
    figure;
    subplot(3,1,1);
    plot(t, RotorThrust_dyn, 'b', 'LineWidth', 1.5); hold on;
    plot(t, RotorThrust_steady, 'r--', 'LineWidth', 1.5);
    ylabel('Rotor Thrust [N]');
    title(['Rotor Thrust vs Time, f = ' num2str(f) ' Hz']);
    legend('Dynamic Inflow', 'Quasi-Steady');
    grid on;

    subplot(3,1,2);
    plot(t, pitch_time, 'k', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Pitch Angle [deg]');
    title('Pitch Angle vs Time');
    grid on;

    subplot(3,1,3);
    plot(pitch_time, RotorThrust_dyn, 'b', 'LineWidth', 1.5); hold on;
    plot(pitch_time, RotorThrust_steady, 'r--', 'LineWidth', 1.5);
    xlabel('Pitch Angle [deg]');
    ylabel('Rotor Thrust [N]');
    title('Rotor Thrust vs Pitch Angle');
    legend('Dynamic Inflow', 'Quasi-Steady');
    grid on;

end