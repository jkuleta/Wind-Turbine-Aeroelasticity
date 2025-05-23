
function compare_K(StructuralParameters, OperationalParameters)
    % Compare total_K (with centrifugal/gravity) vs. structural K
    wind_speeds = OperationalParameters.v0_values;
    omegas = OperationalParameters.omega_values;
    psi = 0; % or try a range if you want to see azimuth effect

    K_struct = StructuralParameters.K;
    K_flap_diff = zeros(size(wind_speeds));
    K_edge_diff = zeros(size(wind_speeds));
    K_flap_total = zeros(size(wind_speeds));
    K_edge_total = zeros(size(wind_speeds));
    K_flap_struct = K_struct(1,1) * ones(size(wind_speeds));
    K_edge_struct = K_struct(2,2) * ones(size(wind_speeds));

    for i = 1:length(wind_speeds)
        total_K = get_total_K(StructuralParameters, omegas(i), psi);
        K_flap_total(i) = total_K(1,1);
        K_edge_total(i) = total_K(2,2);
        K_flap_diff(i) = total_K(1,1) - K_struct(1,1);
        K_edge_diff(i) = total_K(2,2) - K_struct(2,2);
    end

    figure;
    subplot(2,1,1);
    plot(wind_speeds, K_flap_total, 'b-', 'LineWidth', 1.5); hold on;
    plot(wind_speeds, K_flap_struct, 'r--', 'LineWidth', 1.5);
    ylabel('Flapwise K');
    legend('With K_{CG}', 'Structural Only');
    title('Flapwise Stiffness vs Wind Speed');
    grid on;

    subplot(2,1,2);
    plot(wind_speeds, K_edge_total, 'g-', 'LineWidth', 1.5); hold on;
    plot(wind_speeds, K_edge_struct, 'm--', 'LineWidth', 1.5);
    ylabel('Edgewise K');
    xlabel('Wind Speed [m/s]');
    legend('With K_{CG}', 'Structural Only');
    title('Edgewise Stiffness vs Wind Speed');
    grid on;

    figure;
    plot(wind_speeds, K_flap_diff, 'b-', 'LineWidth', 1.5); hold on;
    plot(wind_speeds, K_edge_diff, 'g-', 'LineWidth', 1.5);
    xlabel('Wind Speed [m/s]');
    ylabel('\Delta K (With K_{CG} - Structural)');
    legend('Flapwise', 'Edgewise');
    title('Difference in Stiffness vs Wind Speed');
    grid on;
end

compare_K(StructuralParameters, OperationalParameters);

