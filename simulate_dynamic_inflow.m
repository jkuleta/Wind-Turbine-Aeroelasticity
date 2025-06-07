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

    % Initialize induction memory for dynamic inflow
    a_prev = zeros(size(AeroParameters.radius_aero));
    a_prime_prev = zeros(size(AeroParameters.radius_aero));

    % Get the steady pitch angle at 15 m/s
    [~, idx_steady] = min(abs(OperationalParameters.v0_values - 15));
    steady_pitch = OperationalParameters.pitch_values(idx_steady);

    for k = 1:length(t)
        % Dynamic inflow ON: use time-varying pitch
        display(['Time: ' num2str(t(k)) ' s, Pitch: ' num2str(pitch_time(k)) ' deg']);
        pitch_dyn = pitch_time(k);

        [~, ~, ~, thrust_dyn, a_out, a_prime_out] = compute_aero_force_dynamic([0;0], [0;0], V0, omega, pitch_dyn, ...
            AeroParameters.radius_aero, AeroParameters.twist_aero, ...
            AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, true, true, dt, a_prev, a_prime_prev);
        RotorThrust_dyn(k) = thrust_dyn;
        a_prev = a_out;
        a_prime_prev = a_prime_out;

        % Quasi-steady: use constant pitch at 15 m/s
        [~, ~, ~, thrust_steady] = compute_aero_force_dynamic([0;0], [0;0], V0, omega, steady_pitch, ...
            AeroParameters.radius_aero, AeroParameters.twist_aero, ...
            AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, true, false, dt);
        RotorThrust_steady(k) = thrust_steady;
    end

    % Plot thrust and pitch vs time using subplots
    figure;
    subplot(2,1,1);
    plot(t, RotorThrust_dyn, 'b', 'LineWidth', 1.5); hold on;
    plot(t, RotorThrust_steady, 'r--', 'LineWidth', 1.5);
    ylabel('Rotor Thrust [N]');
    title(['Rotor Thrust vs Time, f = ' num2str(f) ' Hz']);
    legend('Dynamic Inflow', 'Quasi-Steady');
    grid on;

    subplot(2,1,2);
    plot(t, pitch_time, 'k', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Pitch Angle [deg]');
    title('Pitch Angle vs Time');
    grid on;
end