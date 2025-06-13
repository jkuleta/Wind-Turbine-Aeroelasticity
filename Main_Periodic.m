% % Wind Turbine Aeroelasticity Simulation
% clc;
% close all;
% clear;

% [StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% %% Simulation setup
% dynamic_inflow = false; % Enable if needed
% coupling = true;
% periodic = true; % Enable periodic boundary conditions
% % V0 = 15 + 0.5*cos(1.267*t) + 0.085*cos(2.534*t) + 0.015*cos(3.801*t);

% vinduced = 0;
% dt = 0.2;
% tf = 200;
% tspan = [0 tf];

% % Output storage
% K_CG_options = [false, false];
% Coupling_options = [true, false];
% tip_deflection_all = cell(1, 2);
% time_hist = cell(1, 2);

% for k = 1:2
%     K_CG = K_CG_options(k);
%     coupling = Coupling_options(k);

%     tip_deflection = zeros(2, length(OperationalParameters.v0_values));


%     % Inputs
%     if periodic
%         i = 14; % index for 15 m/s wind speed
%         V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
%         omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
%         pitch = OperationalParameters.pitch_values(i);
%         Y0 = [0; 0; 0; 0; 0];  % [x1; x2; dx1; dx2; psi]
%     else
%         disp('this is not periodic');
%     end

%     opts = odeset('RelTol',1e-3,'AbsTol',1e-5);

%     % Solve ODE
%         [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_periodic(tt, YY, V_org, omega_org, pitch, ...
%             StructuralParameters.M, StructuralParameters.C, K_CG, StructuralParameters, AeroParameters, coupling), ...
%             tspan, Y0, opts);

%     % Extract results
%     x = Y_out(:,1:2)';
%     t_out = t_out(:);

%     % Tip deflection time history
%     tip_flap = x(1,:) * StructuralParameters.phi_1flap(end);
%     tip_edge = x(2,:) * StructuralParameters.phi_1edge(end);

%     % Use findpeaks to estimate period of last cycle
%     [~, locs] = findpeaks(tip_flap, t_out, 'MinPeakProminence', 0.01);

%     % Require at least 2 peaks
%     if length(locs) >= 2
%         T_est = locs(end) - locs(end-1);  % Time between last two peaks
%     else
%         T_est = 1;  % fallback (1 second)
%     end

%     % Get indices of final cycle
%     t_final = t_out(end);
%     idx_last_cycle = find(t_out >= t_final - T_est);

%     % Compute average absolute deflection in that cycle
%     max_flap_tip = mean(abs(tip_flap(idx_last_cycle)));
%     max_edge_tip = mean(abs(tip_edge(idx_last_cycle)));

%     tip_deflection(:, i) = [max_flap_tip; max_edge_tip];


%     M_flap_root = StructuralParameters.flap_stiffness_distribution(1) * StructuralParameters.ddphi_1flap(1) * x(1,:);
%     M_edge_root = StructuralParameters.edge_stiffness_distribution(1) * StructuralParameters.ddphi_1edge(1) * x(2,:);

%     % Store time-domain data from the **last** wind speed for this K_CG
%     time_hist{k}.t_out = t_out;
%     time_hist{k}.x = Y_out(:,1:2)';
%     time_hist{k}.dx = Y_out(:,3:4)';
%     time_hist{k}.psi = Y_out(:,5);
%     time_hist{k}.M_flap_root = M_flap_root;
%     time_hist{k}.M_edge_root = M_edge_root;

%     tip_deflection_all{k} = tip_deflection;
% end

% % Plot: Time-Domain Comparison for Coupling = true and false
% figure;
% titles = {'Coupling = true', 'Coupling = false'};

% for k = 1:2
%     t_out = time_hist{k}.t_out;
%     x     = time_hist{k}.x;
%     dx    = time_hist{k}.dx;
%     psi   = time_hist{k}.psi;

%     subplot(2,2,1 + (k-1));
%     plot(t_out, x(1,:), 'LineWidth', 2); hold on;
%     plot(t_out, dx(1,:), '--', 'LineWidth', 2);
%     xlabel('Time [s]'); ylabel('Flapwise');
%     title(['Flapwise - ' titles{k}]);
%     legend('Displacement', 'Velocity');
%     grid on;

%     subplot(2,2,3 + (k-1));
%     plot(t_out, x(2,:), 'LineWidth', 2); hold on;
%     plot(t_out, dx(2,:), '--', 'LineWidth', 2);
%     xlabel('Time [s]'); ylabel('Edgewise');
%     title(['Edgewise - ' titles{k}]);
%     legend('Displacement', 'Velocity');
%     grid on;
% end

% % Plot: Tip Deflection vs Time for Coupling = true and false
% figure;
% plot(time_hist{1}.t_out, time_hist{1}.x(1,:), 'b-', 'LineWidth', 2); hold on;
% plot(time_hist{2}.t_out, time_hist{2}.x(1,:), 'r--', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Flapwise Tip Deflection [m]');
% title('Flapwise Tip Deflection: Coupling vs No Coupling');
% legend('Coupling', 'No Coupling');
% grid on;

% figure;
% plot(time_hist{1}.t_out, time_hist{1}.x(2,:), 'b-', 'LineWidth', 2); hold on;
% plot(time_hist{2}.t_out, time_hist{2}.x(2,:), 'r--', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Edgewise Tip Deflection [m]');
% title('Edgewise Tip Deflection: Coupling vs No Coupling');
% legend('Coupling', 'No Coupling');
% grid on;


%Wind Turbine Aeroelasticity Simulation
% clc;
% close all;
% clear;

% [StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% %% Simulation setup
% dynamic_inflow = false; % Enable if needed
% periodic = true; % Enable periodic boundary conditions
% vinduced = 0;
% dt = 0.2;
% tf = 200;
% tspan = [0 tf];

% % Output storage
% K_CG = false; % Keep as in your original code
% tip_deflection_all = cell(1, 2);
% time_hist = cell(1, 2);

% % Run for both coupling ON and OFF
% coupling_options = [true, false];
% labels = {'Coupling ON', 'Coupling OFF'};
% colors = {'b', 'r'};

% M_flap_root_all = cell(1,2);
% M_edge_root_all = cell(1,2);
% t_out_all = cell(1,2);

% for k = 1:2
%     coupling = coupling_options(k);

%     % Inputs
%     if periodic
%         i = 14; % index for 15 m/s wind speed
%         V_org = OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
%         omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
%         pitch = OperationalParameters.pitch_values(i);
%         Y0 = [0; 0; 0; 0; 0];  % [x1; x2; dx1; dx2; psi]
%     else
%         disp('this is not periodic');
%     end

%     opts = odeset('RelTol',1e-3,'AbsTol',1e-5);

%     % Solve ODE
%     [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_periodic(tt, YY, V_org, omega_org, pitch, ...
%         StructuralParameters.M, StructuralParameters.C, K_CG, StructuralParameters, AeroParameters, coupling), ...
%         tspan, Y0, opts);

%     % Extract results
%     x = Y_out(:,1:2)';
%     t_out = t_out(:);

%     % Tip deflection time history
%     tip_flap = x(1,:) * StructuralParameters.phi_1flap(end);
%     tip_edge = x(2,:) * StructuralParameters.phi_1edge(end);

%     % Use findpeaks to estimate period of last cycle
%     [~, locs] = findpeaks(tip_flap, t_out, 'MinPeakProminence', 0.01);

%     % Require at least 2 peaks
%     if length(locs) >= 2
%         T_est = locs(end) - locs(end-1);  % Time between last two peaks
%     else
%         T_est = 1;  % fallback (1 second)
%     end

%     % Get indices of final cycle
%     t_final = t_out(end);
%     idx_last_cycle = find(t_out >= t_final - T_est);

%     % Compute average absolute deflection in that cycle
%     max_flap_tip = mean(abs(tip_flap(idx_last_cycle)));
%     max_edge_tip = mean(abs(tip_edge(idx_last_cycle)));

%     tip_deflection = zeros(2, length(OperationalParameters.v0_values));
%     tip_deflection(:, i) = [max_flap_tip; max_edge_tip];

%     M_flap_root = StructuralParameters.flap_stiffness_distribution(1) * StructuralParameters.ddphi_1flap(1) * x(1,:);
%     M_edge_root = StructuralParameters.edge_stiffness_distribution(1) * StructuralParameters.ddphi_1edge(1) * x(2,:);

%     % Store time-domain data
%     time_hist{k}.t_out = t_out;
%     time_hist{k}.x = Y_out(:,1:2)';
%     time_hist{k}.dx = Y_out(:,3:4)';
%     time_hist{k}.psi = Y_out(:,5);
%     time_hist{k}.M_flap_root = M_flap_root;
%     time_hist{k}.M_edge_root = M_edge_root;
%     tip_deflection_all{k} = tip_deflection;

%     M_flap_root_all{k} = M_flap_root;
%     M_edge_root_all{k} = M_edge_root;
%     t_out_all{k} = t_out;

%     % Store tip deflections for FFT later
%     tip_flap_all{k} = tip_flap;
%     tip_edge_all{k} = tip_edge;

%     %forces 
%     % Preallocate for aerodynamic loads
%     N_time = size(Y_out,1);
%     N_radial = length(AeroParameters.radius_aero);
%     FF_hist = zeros(N_time, N_radial);
%     FE_hist = zeros(N_time, N_radial);

%     for it = 1:N_time
%         x_t = Y_out(it,1:2)';
%         dx_t = Y_out(it,3:4)';
%         % Call compute_aero_force for each time step
%         [~, FF, FE] = compute_aero_force(x_t, dx_t, V_org, omega_org, pitch, ...
%         AeroParameters.radius_aero, AeroParameters.twist_aero, ...
%         AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, coupling);
%         FF_hist(it,:) = FF;
%         FE_hist(it,:) = FE;
%     end

%     % Store for later plotting
%     FF_hist_all{k} = FF_hist;
%     FE_hist_all{k} = FE_hist;
% end

% % Plot maximum values along the blade for both coupling cases
% figure;
% hold on;
% for k = 1:2
%     max_FF = max(abs(FF_hist_all{k}), [], 1); % max over time, for each radius
%     max_FE = max(abs(FE_hist_all{k}), [], 1);
%     plot(AeroParameters.radius_aero, max_FF, [colors{k} '-'], 'LineWidth', 2, 'DisplayName', ['Flapwise ' labels{k}]);
%     plot(AeroParameters.radius_aero, max_FE, [colors{k} '--'], 'LineWidth', 2, 'DisplayName', ['Edgewise ' labels{k}]);
% end
% xlabel('Blade Radius [m]');
% ylabel('Maximum Loads [N/m]');
% %title('Maximal Radial Distribution of Aero Loads (Flapwise/Edgewise, Coupling ON/OFF)');
% legend('show');
% grid on;


% % Plot root moments for both cases
% figure;
% hold on;
% for k = 1:2
%     plot(t_out_all{k}, M_flap_root_all{k}, [colors{k} '-'], 'LineWidth', 2, 'DisplayName', ['Flapwise ' labels{k}]);
%     plot(t_out_all{k}, M_edge_root_all{k},  [colors{k} '--'], 'LineWidth', 2, 'DisplayName', ['Edgewise ' labels{k}]);
% end
% xlabel('Time [s]');
% ylabel('Bending Moment [Nm]');
% %title('Root Bending Moments: Flapwise and Edgewise (Coupling ON/OFF)');
% legend('show');
% grid on;

% % Plot tip deflections for both cases
% figure;
% hold on;
% for k = 1:2
%     plot(t_out_all{k}, tip_flap_all{k}, [colors{k} '-'], 'LineWidth', 2, 'DisplayName', ['Flapwise ' labels{k}]);
%     plot(t_out_all{k}, tip_edge_all{k},  [colors{k} '--'], 'LineWidth', 2, 'DisplayName', ['Edgewise ' labels{k}]);
% end
% xlabel('Time [s]');
% ylabel('Tip Deflection [m]');
% %title('Tip Deflection vs Time (Coupling ON/OFF)');
% legend('show');
% grid on;

% figure(100); clf; hold on;
% for k = 1:2
%     t_out = t_out_all{k};
%     M_flap_root = M_flap_root_all{k};
%     M_edge_root = M_edge_root_all{k};
%     L = length(t_out);
%     Fs = 1 / mean(diff(t_out));  % Sampling frequency
%     f = Fs * (0:(L/2)) / L;

%     Y_flap = fft(M_flap_root);
%     Y_edge = fft(M_edge_root);

%     P_flap = abs(Y_flap / L);
%     P_edge = abs(Y_edge / L);

%     P_flap = P_flap(1:L/2+1);
%     P_edge = P_edge(1:L/2+1);

%     P_flap(2:end-1) = 2*P_flap(2:end-1);
%     P_edge(2:end-1) = 2*P_edge(2:end-1);

%     % Remove zero or negative frequencies and magnitudes
%     idx_flap = f(2:end) > 0 & P_flap(2:end) > 0;
%     idx_edge = f(2:end) > 0 & P_edge(2:end) > 0;
%     loglog(f(2:end), P_flap(2:end), [colors{k} '-'], 'LineWidth', 2, 'DisplayName', ['Flapwise ' labels{k}]);
%     loglog(f(2:end), P_edge(2:end),  [colors{k} '--'], 'LineWidth', 2, 'DisplayName', ['Edgewise ' labels{k}]);
% end
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% %title('Bending Moment Frequency Spectrum (Coupling ON/OFF)');
% legend('show');
% xlim([1e-1, 1.1*max(f)]);
% %ylim([1e-6, 1.1*max([P_flap(:); P_edge(:)])]);
% set(gca, 'XScale', 'log', 'YScale', 'log'); % Force log-log axes
% grid on;

% figure(200); clf; hold on;
% for k = 1:2
%     t_out = t_out_all{k};
%     tip_flap = tip_flap_all{k};
%     tip_edge = tip_edge_all{k};
%     L = length(t_out);
%     Fs = 1 / mean(diff(t_out));
%     f = Fs * (0:(L/2)) / L;

%     Y_tip_flap = fft(tip_flap);
%     Y_tip_edge = fft(tip_edge);

%     P_tip_flap = abs(Y_tip_flap / L);
%     P_tip_edge = abs(Y_tip_edge / L);

%     P_tip_flap = P_tip_flap(1:L/2+1);
%     P_tip_edge = P_tip_edge(1:L/2+1);

%     P_tip_flap(2:end-1) = 2*P_tip_flap(2:end-1);
%     P_tip_edge(2:end-1) = 2*P_tip_edge(2:end-1);

%     % Remove zero or negative frequencies and magnitudes
%     idx = f(2:end) > 0 & P_tip_flap(2:end) > 0;
%     loglog(f(2:end), P_tip_flap(2:end), [colors{k} '-'], 'LineWidth', 2, 'DisplayName', ['Flapwise ' labels{k}]);
%     idx = f(2:end) > 0 & P_tip_edge(2:end) > 0;
%     loglog(f(2:end), P_tip_edge(2:end),  [colors{k} '--'], 'LineWidth', 2, 'DisplayName', ['Edgewise ' labels{k}]);
% end
% xlabel('Frequency [Hz]');
% ylabel('Magnitude');
% %title('Tip Deflection Frequency Spectrum (Coupling ON/OFF)');
% legend('show');
% xlim([1e-1, 1.1*max(f)]);
% %ylim([1e-6, 1.1*max([P_tip_flap(:); P_tip_edge(:)])]);
% set(gca, 'XScale', 'log', 'YScale', 'log'); % Force log-log axes
% grid on;



% % % Wind Turbine Aeroelasticity Simulation - RK4 Version with Time-Domain Plots
% % clc;
% % close all;
% % clear;

% % [StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% % %% Simulation setup
% % dynamic_inflow = false;
% % coupling = true;
% % periodic = true;

% % vinduced = 0;
% % dt = 0.01;
% % tf = 10;
% % t = 0:dt:tf;

% % % Output storage
% % K_CG_options = [false, false];
% % Coupling_options = [true, false];
% % time_hist = cell(1, 2);
% % t_out_all = cell(1, 2);

% % for k = 1:2
% %     K_CG = K_CG_options(k);
% %     coupling = Coupling_options(k);

% %     if periodic
% %         i = 14; % index for 15 m/s wind
% %         omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
% %         pitch_fn = @(tt) 10.45 * ones(size(AeroParameters.radius_aero));
% %         V_fun = @(t) 15 + 0.5*cos(1.267*t) + 0.085*cos(2.534*t) + 0.015*cos(3.801*t);
% %     else
% %         i = 10;
% %         omega_org = OperationalParameters.omega_values(i) * ones(size(AeroParameters.radius_aero));
% %         pitch_fn = @(tt) OperationalParameters.pitch_values(i) * ones(size(AeroParameters.radius_aero));
% %         V_fun = @(t) OperationalParameters.v0_values(i) * ones(size(AeroParameters.radius_aero));
% %     end

% %     % Initial state
% %     Y0 = zeros(5,1);
% %     Y = zeros(length(t), length(Y0));
% %     Y(1,:) = Y0;

% %     % RK4 Time Integration
% %     for n = 1:length(t)-1
% %         tn = t(n);
% %         V_now = V_fun(tn);
% %         pitch = pitch_fn(tn);

% %         k1 = odefun_blade(tn,         Y(n,:)', V_now, omega_org, pitch, ...
% %                          StructuralParameters.M, StructuralParameters.C, ...
% %                          K_CG, StructuralParameters, AeroParameters, coupling);
% %         k2 = odefun_blade(tn+dt/2,   Y(n,:)' + dt/2*k1, V_now, omega_org, pitch, ...
% %                          StructuralParameters.M, StructuralParameters.C, ...
% %                          K_CG, StructuralParameters, AeroParameters, coupling);
% %         k3 = odefun_blade(tn+dt/2,   Y(n,:)' + dt/2*k2, V_now, omega_org, pitch, ...
% %                          StructuralParameters.M, StructuralParameters.C, ...
% %                          K_CG, StructuralParameters, AeroParameters, coupling);
% %         k4 = odefun_blade(tn+dt,     Y(n,:)' + dt*k3,   V_now, omega_org, pitch, ...
% %                          StructuralParameters.M, StructuralParameters.C, ...
% %                          K_CG, StructuralParameters, AeroParameters, coupling);
% %         Y(n+1,:) = Y(n,:) + dt/6 * (k1' + 2*k2' + 2*k3' + k4');
% %     end

% %     % Store output
% %     t_out_all{k} = t;
% %     Y_full_struct.t_out = t;
% %     Y_full_struct.x  = zeros(2, length(t));
% %     Y_full_struct.dx = zeros(2, length(t));
% %     Y_full_struct.psi = zeros(1, length(t));

% %     for m = 1:length(t)
% %         Y_full_struct.x(:,m)  = Y(m, 1:2)';
% %         Y_full_struct.dx(:,m) = Y(m, 3:4)';
% %         Y_full_struct.psi(m)  = Y(m, 5);
% %     end

% %     time_hist{k} = Y_full_struct;
% % end

% % % Plot: Time-Domain Comparison for Coupling = true and false
% % figure;
% % titles = {'Coupling = true', 'Coupling = false'};

% % for k = 1:2
% %     t_out = time_hist{k}.t_out;
% %     x     = time_hist{k}.x;
% %     dx    = time_hist{k}.dx;
% %     psi   = time_hist{k}.psi;

% %     subplot(2,2,1 + (k-1));
% %     plot(t_out, x(1,:), 'LineWidth', 2); hold on;
% %     plot(t_out, dx(1,:), '--', 'LineWidth', 2);
% %     xlabel('Time [s]'); ylabel('Flapwise');
% %     title(['Flapwise - ' titles{k}]);
% %     legend('Displacement', 'Velocity');
% %     grid on;

% %     subplot(2,2,3 + (k-1));
% %     plot(t_out, x(2,:), 'LineWidth', 2); hold on;
% %     plot(t_out, dx(2,:), '--', 'LineWidth', 2);
% %     xlabel('Time [s]'); ylabel('Edgewise');
% %     title(['Edgewise - ' titles{k}]);
% %     legend('Displacement', 'Velocity');
% %     grid on;
% % end

% % % Plot: Tip Deflection vs Time for Coupling = true and false
% % figure;
% % plot(time_hist{1}.t_out, time_hist{1}.x(1,:), 'b-', 'LineWidth', 2); hold on;
% % plot(time_hist{2}.t_out, time_hist{2}.x(1,:), 'r--', 'LineWidth', 2);
% % xlabel('Time [s]');
% % ylabel('Flapwise Tip Deflection [m]');
% % title('Flapwise Tip Deflection: Coupling vs No Coupling');
% % legend('Coupling', 'No Coupling');
% % grid on;

% % figure;
% % plot(time_hist{1}.t_out, time_hist{1}.x(2,:), 'b-', 'LineWidth', 2); hold on;
% % plot(time_hist{2}.t_out, time_hist{2}.x(2,:), 'r--', 'LineWidth', 2);
% % xlabel('Time [s]');
% % ylabel('Edgewise Tip Deflection [m]');
% % title('Edgewise Tip Deflection: Coupling vs No Coupling');
% % legend('Coupling', 'No Coupling');
% % grid on;



% % Wind Turbine Aeroelasticity Simulation using BEM (scalar pitch, scalar v0)
% clc;
% close all;
% clear;

% [StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% %% Setup
% dynamic_inflow = false;
% coupling = true;
% periodic = true;

% vinduced = 0;
% dt = 0.2;
% tf = 10;
% tspan = [0 tf];

% K_CG = false;
% tip_deflection_all = cell(1, 2);
% time_hist = cell(1, 2);

% for k = 2
%     coupling = false;  % BEM used here does not support induced velocities from blade motion

%     % Inputs
%     if periodic
%         i = 14; % 15 m/s wind
%         V_scalar = OperationalParameters.v0_values(i);
%         omega_scalar = OperationalParameters.omega_values(i);
%         pitch_scalar = OperationalParameters.pitch_values(i);
%         Y0 = [0; 0; 0; 0; 0];
%     else
%         error('Non-periodic case not supported in this version.');
%     end

%     opts = odeset('RelTol',1e-3,'AbsTol',1e-5);

%     [t_out, Y_out] = ode45(@(tt, YY) odefun_blade_periodic(tt, YY, ...
%         V_scalar, omega_scalar, pitch_scalar, ...
%         StructuralParameters.M, StructuralParameters.C, ...
%         K_CG, StructuralParameters, AeroParameters, coupling), ...
%         tspan, Y0, opts);

%     x = Y_out(:,1:2)';
%     t_out = t_out(:);

%     tip_flap = x(1,:) * StructuralParameters.phi_1flap(end);
%     tip_edge = x(2,:) * StructuralParameters.phi_1edge(end);

%     [~, locs] = findpeaks(tip_flap, t_out, 'MinPeakProminence', 0.01);
%     if length(locs) >= 2
%         T_est = locs(end) - locs(end-1);
%     else
%         T_est = 1;
%     end
%     t_final = t_out(end);
%     idx_last_cycle = find(t_out >= t_final - T_est);
%     max_flap_tip = mean(abs(tip_flap(idx_last_cycle)));
%     max_edge_tip = mean(abs(tip_edge(idx_last_cycle)));

%     tip_deflection = zeros(2, length(OperationalParameters.v0_values));
%     tip_deflection(:, i) = [max_flap_tip; max_edge_tip];

%     M_flap_root = StructuralParameters.flap_stiffness_distribution(1) * StructuralParameters.ddphi_1flap(1) * x(1,:);
%     M_edge_root = StructuralParameters.edge_stiffness_distribution(1) * StructuralParameters.ddphi_1edge(1) * x(2,:);

%     time_hist{k}.t_out = t_out;
%     time_hist{k}.x = Y_out(:,1:2)';
%     time_hist{k}.dx = Y_out(:,3:4)';
%     time_hist{k}.psi = Y_out(:,5);
%     time_hist{k}.M_flap_root = M_flap_root;
%     time_hist{k}.M_edge_root = M_edge_root;

%     tip_deflection_all{k} = tip_deflection;
% end

% % Plot: Time-Domain Comparison
% figure;
% titles = {'Coupling = true', 'Coupling = false'};

% for k = 2
%     t_out = time_hist{k}.t_out;
%     x     = time_hist{k}.x;
%     dx    = time_hist{k}.dx;

%     subplot(2,2,1 + (k-1));
%     plot(t_out, x(1,:), 'LineWidth', 2); hold on;
%     plot(t_out, dx(1,:), '--', 'LineWidth', 2);
%     xlabel('Time [s]'); ylabel('Flapwise');
%     title(['Flapwise - ' titles{k}]);
%     legend('Displacement', 'Velocity');
%     grid on;

%     subplot(2,2,3 + (k-1));
%     plot(t_out, x(2,:), 'LineWidth', 2); hold on;
%     plot(t_out, dx(2,:), '--', 'LineWidth', 2);
%     xlabel('Time [s]'); ylabel('Edgewise');
%     title(['Edgewise - ' titles{k}]);
%     legend('Displacement', 'Velocity');
%     grid on;
% end

% % Tip deflection vs time
% figure;
% plot(time_hist{2}.t_out, time_hist{2}.x(1,:), 'r--', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Flapwise Tip Deflection [m]');
% title('Flapwise Tip Deflection (Coupling OFF, scalar BEM)');
% grid on;

% figure;
% plot(time_hist{2}.t_out, time_hist{2}.x(2,:), 'r--', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Edgewise Tip Deflection [m]');
% title('Edgewise Tip Deflection (Coupling OFF, scalar BEM)');
% grid on;

% =======================================================================
% main_blade_sim.m
% =======================================================================
% High-level driver:
%   • integrates the 1-DOF-per-direction bending model with ode45
%   • reconstructs distributed aerodynamic loads at every solver step
%   • computes flapwise/edgewise root moments from those loads
%   • produces time histories, radial load plots and spectra
%
% Author:  <your name>   Date: <today>
% =======================================================================

clc; close all; clear;

% -----------------------------------------------------------------------
% 1.  Model & operating data
% -----------------------------------------------------------------------
[StructuralParameters, OperationalParameters, AeroParameters] = load_data();

% -----------------------------------------------------------------------
% 2.  Simulation controls
% -----------------------------------------------------------------------
dynamic_inflow = false;        %#ok<NASGU>  (placeholder for future use)
periodic       = true;         % use periodic operating point
vinduced       = 0;            %#ok<NASGU>  (not used here)
dt             = 0.2;          %#ok<NASGU>  (not used – variable step)
tf             = 30;
tspan          = [0 tf];

K_CG           = false;        % keep baseline stiffness matrix

% -----------------------------------------------------------------------
% 3.  Containers for results
% -----------------------------------------------------------------------
coupling_options   = [true, false];
labels             = {'Coupling ON', 'Coupling OFF'};
colors             = {'b', 'r'};

time_hist          = cell(1,2);
tip_deflection_all = cell(1,2);

M_flap_root_all    = cell(1,2);
M_edge_root_all    = cell(1,2);
t_out_all          = cell(1,2);

FF_hist_all        = cell(1,2);
FE_hist_all        = cell(1,2);

tip_flap_all       = cell(1,2);
tip_edge_all       = cell(1,2);

% -----------------------------------------------------------------------
% 4.  Main loop – two runs: coupling ON / OFF
% -----------------------------------------------------------------------
for k = 1:2
    coupling = coupling_options(k);

    % ---- (a) operating point ------------------------------------------------
    if periodic
        i          = 14;            % index for 15 m/s wind speed
        V_org      = OperationalParameters.v0_values(i) * ...
                     ones(size(AeroParameters.radius_aero));
        omega_org  = OperationalParameters.omega_values(i) * ...
                     ones(size(AeroParameters.radius_aero));
        pitch      = OperationalParameters.pitch_values(i);
        Y0         = [0; 0; 0; 0; 0];   % [x1; x2; dx1; dx2; psi]
    else
        error('Non-periodic mode not implemented in this script.');
    end

    % ---- (b) integrate -------------------------------------------------------
    opts = odeset('RelTol',1e-3,'AbsTol',1e-5);
    [t_out, Y_out] = ode45(@(tt,YY) odefun_blade_periodic( ...
                                tt, YY, V_org, omega_org, pitch, ...
                                StructuralParameters.M, StructuralParameters.C, ...
                                K_CG, StructuralParameters, AeroParameters, coupling), ...
                           tspan, Y0, opts);

    % ---- (c) unpack states ---------------------------------------------------
    x      = Y_out(:,1:2).';     % rows = modes, cols = time
    dx     = Y_out(:,3:4).';
    psi    = Y_out(:,5).';
    t_out  = t_out(:);

    % ---- (d) tip deflection & “last cycle” metric ---------------------------
    tip_flap = x(1,:) * StructuralParameters.phi_1flap(end);
    tip_edge = x(2,:) * StructuralParameters.phi_1edge(end);

    [~, locs] = findpeaks(tip_flap, t_out, 'MinPeakProminence',0.01);
    % --- robust estimate of the last cycle period ---------------------
    if numel(locs) >= 2
        T_est = locs(end) - locs(end-1);   % time between last two peaks
    else
        T_est = 1;                         % fallback (1 second)
    end


    idx_last  = t_out >= t_out(end) - T_est;
    max_flap_tip = mean(abs(tip_flap(idx_last)));
    max_edge_tip = mean(abs(tip_edge(idx_last)));

    tip_deflection          = zeros(2, numel(OperationalParameters.v0_values));
    tip_deflection(:, i)    = [max_flap_tip; max_edge_tip];
    tip_deflection_all{k}   = tip_deflection;

    % ------------------------------------------------------------------------
    % ------------------------------------------------------------------------
% ------------------------------------------------------------------------
    % (e) **Option-1** post-processing – moments from f_flap / f_edge
    % ------------------------------------------------------------------------
    N_time   = size(Y_out,1);
    r        = AeroParameters.radius_aero(:);      % column (N×1)
    N_radial = numel(r);

    FF_hist     = zeros(N_time, N_radial);         % flapwise section loads
    FE_hist     = zeros(N_time, N_radial);         % edgewise  section loads
    M_flap_root = zeros(N_time, 1);                % flapwise root moment
    M_edge_root = zeros(N_time, 1);                % edgewise  root moment

    for it = 1:N_time
        x_t  = Y_out(it,1:2).';
        dx_t = Y_out(it,3:4).';

        t_curr     = t_out(it);                           % current time [s]
        V_scalar   = 15 + 0.5*cos(1.267*t_curr) ...       % m/s
                    + 0.085*cos(2.534*t_curr) ...
                    + 0.015*cos(3.801*t_curr);
        V_inst     = V_scalar * ones(size(r)); 
        % ---------- aerodynamic outputs ------------------------------------
        [F_modal, FF, FE] = compute_aero_force( ...
                        x_t, dx_t, V_inst, omega_org, pitch, ...
                        AeroParameters.radius_aero, AeroParameters.twist_aero, ...
                        AeroParameters.phi_1flap_aero, ...
                        AeroParameters.phi_1edge_aero, coupling);

        % ---------- modal forces ------------------------------------------
        f_flap = F_modal(1);          % <- first  element of F_modal
        f_edge = F_modal(2);          % <- second element of F_modal

        %M_flap_root(it) = f_flap * r(1);     % use directly for “moment” history
        %M_edge_root(it) = f_edge * r(1);

        M_flap_root(it) = trapz(r, FF .* r);   % Nm
        M_edge_root(it) = trapz(r, FE .* r);
    end
% ------------------------------------------------------------------------

% ------------------------------------------------------------------------


    % ---- (f) store everything ----------------------------------------------
    time_hist{k}.t_out       = t_out;
    time_hist{k}.x           = x;
    time_hist{k}.dx          = dx;
    time_hist{k}.psi         = psi;
    time_hist{k}.M_flap_root = M_flap_root;
    time_hist{k}.M_edge_root = M_edge_root;

    M_flap_root_all{k} = M_flap_root;
    M_edge_root_all{k} = M_edge_root;
    t_out_all{k}       = t_out;

    FF_hist_all{k}     = FF_hist;
    FE_hist_all{k}     = FE_hist;

    tip_flap_all{k}    = tip_flap;
    tip_edge_all{k}    = tip_edge;
end

% -----------------------------------------------------------------------
% 5.  Plots
% -----------------------------------------------------------------------

% ---- (b) root bending moments ----------------------------------------------
figure; hold on;
for k = 1:2
    plot(t_out_all{k}, M_flap_root_all{k}, [colors{k} '-'], 'LineWidth',2, ...
         'DisplayName',['Flapwise ' labels{k}]);
    plot(t_out_all{k}, M_edge_root_all{k}, [colors{k} '--'], 'LineWidth',2, ...
         'DisplayName',['Edgewise ' labels{k}]);
end
xlabel('Time  [s]');
ylabel('Root bending moment  [Nm]');
legend show; grid on;

% ---- (c) tip deflections ----------------------------------------------------
figure; hold on;
for k = 1:2
    plot(t_out_all{k}, tip_flap_all{k}, [colors{k} '-'], 'LineWidth',2, ...
         'DisplayName',['Flapwise ' labels{k}]);
    plot(t_out_all{k}, tip_edge_all{k}, [colors{k} '--'], 'LineWidth',2, ...
         'DisplayName',['Edgewise ' labels{k}]);
end
xlabel('Time  [s]');
ylabel('Tip deflection  [m]');
legend show; grid on;

% ---- (d) FFT of root moments (log-log) -------------------------------------
figure(100); clf; hold on;
for k = 1:2
    t      = t_out_all{k};
    Mflap  = M_flap_root_all{k};
    Medge  = M_edge_root_all{k};
    L      = numel(t);
    Fs     = 1 / mean(diff(t));
    f      = Fs*(0:L/2)/L;

    Pf = abs(fft(Mflap)/L);  Pf = 2*Pf(2:L/2+1);  Pf = [Pf(1); Pf];  % single side
    Pe = abs(fft(Medge)/L);  Pe = 2*Pe(2:L/2+1);  Pe = [Pe(1); Pe];

    loglog(f(2:end), Pf(2:end), [colors{k} '-'],  'LineWidth',2, ...
           'DisplayName',['Flapwise ' labels{k}]);
    loglog(f(2:end), Pe(2:end), [colors{k} '--'], 'LineWidth',2, ...
           'DisplayName',['Edgewise ' labels{k}]);
end
xlabel('Frequency  [Hz]'); ylabel('|FFT|');
set(gca,'XScale','log','YScale','log'); grid on; legend show;

% ---- (e) FFT of tip deflection ---------------------------------------------
figure(200); clf; hold on;
for k = 1:2
    t   = t_out_all{k};
    tf  = tip_flap_all{k};
    te  = tip_edge_all{k};
    L   = numel(t);
    Fs  = 1 / mean(diff(t));
    f   = Fs*(0:L/2)/L;

    Pf = abs(fft(tf)/L);  Pf = 2*Pf(2:L/2+1);  Pf = [Pf(1); Pf];
    Pe = abs(fft(te)/L);  Pe = 2*Pe(2:L/2+1);  Pe = [Pe(1); Pe];

    loglog(f(2:end), Pf(2:end), [colors{k} '-'],  'LineWidth',2, ...
           'DisplayName',['Flapwise ' labels{k}]);
    loglog(f(2:end), Pe(2:end), [colors{k} '--'], 'LineWidth',2, ...
           'DisplayName',['Edgewise ' labels{k}]);
end
xlabel('Frequency  [Hz]'); ylabel('|FFT|');
set(gca,'XScale','log','YScale','log'); grid on; legend show;
