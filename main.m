% Wind Turbine Aeroelasticity Simulation
clc
close all;
clear all;


[StructuralParameters, OperationalParameters, AeroParameters] = load_data();


%% Simulation setup
sinusoidal = false;
dynamic_inflow = false; % Set to true if you want to include dynamic inflow
vinduced = 0; % Initial induced velocity
K_CG = false; % Set to true if you want to include centrifugal and gravity stiffening
dt = 0.01;
tf = 30;
t = 0:dt:tf;
psi = 0; % Assuming blade starts vertical, 0 radians

x = zeros(2,length(t));
dx = zeros(2,length(t));
ddx = zeros(2,length(t));

if sinusoidal
    OperationalParameters.V_sin = 15+0.5*cos(1.267*t) + 0.085*cos(2.534*t)+ 0.015*cos(3.801*t);

    % Find the index of a specific wind speed in OperationalParameters.v0_values
    target_windspeed = 15; % example wind speed to find
    [~, windspeed_idx] = min(abs(OperationalParameters.v0_values - target_windspeed));

    pitch = OperationalParameters.pitch_values(windspeed_idx);
    omega = OperationalParameters.omega_values(windspeed_idx);

    % defitnition of arrays
    x = zeros(2, length(t));
    dx = zeros(2, length(t));
    ddx = zeros(2, length(t));

    for j = 1:length(t)-1
            % Use MATLAB's built-in waitbar for progress indication
            if j == 1
                hWait = waitbar(0, 'Running simulation...');
            end
            if mod(j, max(1, floor((length(t)-1)/100))) == 0 || j == length(t)-1 
                waitbar(j/(length(t)-1), hWait, sprintf('Running simulation... %5.1f%%', (j/(length(t)-1))*100));
            end
            if j == length(t)-1 
                close(hWait);
            end

            V = OperationalParameters.V_sin(j);
            % Implement the centrifugal and gravity stiffening terms
            if K_CG
                % Use the first element of omega for the calculation
                psi = psi + omega*dt; % Update blade position
                total_K = get_total_K(StructuralParameters, omega, psi);
            else
                total_K = StructuralParameters.K; % Use the structural stiffness matrix
            end

            % Runge-Kutta integration
            [x(:,j+1), dx(:,j+1), ddx(:,j+1)] = runge_kutta_step(x(:,j), dx(:,j), ddx(:,j), dt, V, omega, pitch, StructuralParameters.M, StructuralParameters.C, total_K, AeroParameters);
            
%Implement dynamic inflow
            %if dynamic_inflow
            %    disp("Running dynamic inflow...");
            %    [Rx, FN, FT, P, a_list, prime_list] = BEM(V, omega, pitch);  % Call your existing BEM

            %    CT = FN ./ (0.5 * OperationalParameters.rho * V.^2 .* AeroParameters.radius_aero);
            %    vind = V .* (1-a_list);
            %    vinduced = pitt_peters(CT, vinduced, V_org, StructuralParameters.R, dt);
    
                
            %    V = OperationalParameters.V_sin(j+1)*ones(size(AeroParameters.radius_aero)) - -vinduced - V_outplane;
            %    omega = omega_org - V_inplane ./ AeroParameters.radius_aero;
            %else
            %    V = OperationalParameters.V_sin(j+1)*ones(size(AeroParameters.radius_aero)) - V_outplane;
            %    omega = omega_org - V_inplane ./ AeroParameters.radius_aero;
            %end
            tip_deflection(:, j) = x(:, j);
    
        end
    
else
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

        V = OperationalParameters.v0_values(i);
        disp("Wind Speed: " + num2str(V) + " m/s");
        omega = OperationalParameters.omega_values(i);
        pitch = OperationalParameters.pitch_values(i);
    
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
            
            %%%%%% Implement the centrifugal and gravity stiffening terms
            if K_CG
                % Use the first element of omega for the calculation
                psi = psi + omega*dt; % Update blade position
                total_K = get_total_K(StructuralParameters, omega, psi);
            else
                total_K = StructuralParameters.K; % Use the structural stiffness matrix
            end

            % Runge-Kutta integration
            [x(:,j+1), dx(:,j+1), ddx(:,j+1)] = runge_kutta_step(x(:,j), dx(:,j), ddx(:,j), dt, V, omega, pitch, StructuralParameters.M, StructuralParameters.C, total_K, AeroParameters, StructuralParameters);
    
            
            %Implement dynamic inflow
            %if dynamic_inflow
            %    disp("Running dynamic inflow...");
            %    [Rx, FN, FT, P, a_list, prime_list] = BEM(V, omega, pitch);  % Call your existing BEM
            %    CT = FN ./ (0.5 * OperationalParameters.rho * V.^2 * AeroParameters.radius_aero);
            %    vind = V .* (1-a_list);
            %    vinduced = pitt_peters(CT, vinduced, V, StructuralParameters.R, dt);
            %    V = V_org - V_outplane - vinduced;
            %    omega = omega_org - V_inplane./AeroParameters.radius_aero;
            %else
            %    V = V_org - V_outplane;
            %    omega = omega_org - V_inplane ./ AeroParameters.radius_aero;
            %end
    
        end
        tip_deflection(:, i) = [x(1, end)* StructuralParameters.phi_1flap(end); x(2, end)* StructuralParameters.phi_1edge(end)];
        
    end
    figure;

    plot(OperationalParameters.v0_values, tip_deflection(1,:), 'LineWidth', 1.5,'Marker','x'); hold on;
    plot(OperationalParameters.v0_values, tip_deflection(2,:), 'LineWidth', 1.5, 'Marker', 'x'); 
    grid on;
    legend('Flapwise', 'Edgewise');
    xlabel('Wind Speed [m/s]');
    ylabel('Tip Deflection [m]');
end




%% Plotting
figure;

% Use default MATLAB color order
co = get(gca, 'ColorOrder');

% Compute average displacement in each direction
avg_flap = mean(x(1,:));
avg_edge = mean(x(2,:));

% Flapwise displacement and velocity
subplot(3,1,1);
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
subplot(3,1,2);
plot(t, x(2,:), 'LineWidth', 2, 'Color', co(2,:)); hold on;
plot(t, dx(2,:), '--', 'LineWidth', 2, 'Color', co(2,:));
plot(t, avg_edge*ones(size(t)), ':', 'LineWidth', 1.5, 'Color', 'k'); % average as dotted line
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
