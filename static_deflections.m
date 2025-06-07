% Wind Turbine Aeroelasticity Simulation: Static Tip Deflection vs Wind Speed (K_CG ON/OFF)
clc;
close all;
clear;

[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

dt = 0.2;
tf = 10;
tspan = [0 tf];
Y0 = [0; 0; 0; 0; 0];

wind_speeds = 3:1:25;
tip_flap_static_KCG = zeros(size(wind_speeds));
tip_edge_static_KCG = zeros(size(wind_speeds));
tip_flap_static_noKCG = zeros(size(wind_speeds));
tip_edge_static_noKCG = zeros(size(wind_speeds));

for i = 1:length(wind_speeds)
    fprintf('Simulating for wind speed: %.1f m/s\n', wind_speeds(i)); % Print current wind speed
    [~, idx] = min(abs(OperationalParameters.v0_values - wind_speeds(i)));
    V0 = OperationalParameters.v0_values(idx) * ones(size(AeroParameters.radius_aero));
    omega = OperationalParameters.omega_values(idx) * ones(size(AeroParameters.radius_aero));
    pitch = OperationalParameters.pitch_values(idx);

    % With K_CG
    [~, Y_out] = ode45(@(tt, YY) odefun_blade_periodic(tt, YY, V0, omega, pitch, ...
        StructuralParameters.M, StructuralParameters.C, true, StructuralParameters, AeroParameters, true), ...
        tspan, Y0);
    x = Y_out(end,1:2);
    tip_flap_static_KCG(i) = x(1) * StructuralParameters.phi_1flap(end);
    tip_edge_static_KCG(i) = x(2) * StructuralParameters.phi_1edge(end);

    % Without K_CG
    [~, Y_out] = ode45(@(tt, YY) odefun_blade_periodic(tt, YY, V0, omega, pitch, ...
        StructuralParameters.M, StructuralParameters.C, false, StructuralParameters, AeroParameters, true), ...
        tspan, Y0);
    x = Y_out(end,1:2);
    tip_flap_static_noKCG(i) = x(1) * StructuralParameters.phi_1flap(end);
    tip_edge_static_noKCG(i) = x(2) * StructuralParameters.phi_1edge(end);
end

figure;
plot(wind_speeds, tip_flap_static_KCG, 'b-o', 'LineWidth', 2, 'DisplayName', 'Flapwise (K_{CG} ON)');
hold on;
plot(wind_speeds, tip_flap_static_noKCG, 'b--s', 'LineWidth', 2, 'DisplayName', 'Flapwise (K_{CG} OFF)');
plot(wind_speeds, tip_edge_static_KCG, 'r-o', 'LineWidth', 2, 'DisplayName', 'Edgewise (K_{CG} ON)');
plot(wind_speeds, tip_edge_static_noKCG, 'r--s', 'LineWidth', 2, 'DisplayName', 'Edgewise (K_{CG} OFF)');
xlabel('Wind Speed [m/s]');
ylabel('Tip Deflection [m]');
%title('Static Tip Deflection vs Wind Speed (K_{CG} ON/OFF)');
legend('show');
grid on;