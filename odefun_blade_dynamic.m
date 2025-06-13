% % function dYdt = odefun_blade_dynamic(t, Y, V_0, omega, f, M, C, K_CG, StructuralParameters, AeroParameters, coupling, dynamic_inflow, PREVIOUS, dt)
% %     N = length(AeroParameters.radius_aero);
% %     x   = Y(1:2);
% %     dx  = Y(3:4);
% %     psi = Y(5);

% %     pitch = 10.45 + 5 * sin(2 * pi * f * t);

% %     if K_CG
% %         K = get_total_K(StructuralParameters, omega(1), psi);
% %     else
% %         K = StructuralParameters.K;
% %     end

% %     [F_modal, ~, ~, ~, PREVIOUS] = compute_aero_force_dynamic(x, dx, V_0, omega, pitch, ...
% %         AeroParameters.radius_aero, AeroParameters.twist_aero, ...
% %         AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
% %         coupling, dynamic_inflow, PREVIOUS, dt);

% %     ddx = M \ (F_modal - C*dx - K*x);
% %     dpsi = omega(1);


% %     dYdt = [dx; ddx; dpsi];
% % end

% function dYdt = odefun_blade_dynamic(t, Y, V_0, omega, f, M, C, K_CG, StructuralParameters, AeroParameters, coupling, dynamic_inflow, dt)
%     N = length(AeroParameters.radius_aero);

%     % Unpack state vector
%     x   = Y(1:2);
%     dx  = Y(3:4);
%     psi = Y(5);
%     a       = Y(6:5+N);
%     a_prime = Y(6+N:5+2*N);

%     pitch = 10.45 + 5 * sin(2 * pi * f * t);

%     if K_CG
%         K = get_total_K(StructuralParameters, omega(1), psi);
%     else
%         K = StructuralParameters.K;
%     end

%     if dynamic_inflow
%     % Build PREVIOUS struct from current state
%         PREVIOUS.a = a;
%         PREVIOUS.a_prime = a_prime;
%     else 
%         PREVIOUS.a = 0;
%         PREVIOUS.a_prime = 0;
%     end

%     [F_modal, ~, ~, ~, PREVIOUS] = compute_aero_force_dynamic(x, dx, V_0, omega, pitch, ...
%         AeroParameters.radius_aero, AeroParameters.twist_aero, ...
%         AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
%         coupling, dynamic_inflow, PREVIOUS, dt);

%     ddx = M \ (F_modal - C*dx - K*x);
%     dpsi = omega(1);

%     if dynamic_inflow
%     % -------- dynamic inflow ON -----------------------------------------
%     da       = (PREVIOUS.a       - a      ) / dt;
%     da_prime = (PREVIOUS.a_prime - a_prime) / dt;
%     else
%         % -------- dynamic inflow OFF ----------------------------------------
%         da       = zeros(size(a));        % <-- nothing happens
%         da_prime = zeros(size(a_prime));  % <-- nothing happens
%     end

%     % Return full derivative vector
%     dYdt = [dx; ddx; dpsi; da; da_prime];
% end

function dYdt = odefun_blade_dynamic(t, Y, V_0, omega, f, ...
                    M, C, K_CG, StructuralParameters, AeroParameters, ...
                    coupling, dynamic_inflow, dt)

    N = numel(AeroParameters.radius_aero);

    % unpack -------------------------------------------------------------
    x   = Y(1:2);           dx  = Y(3:4);      psi = Y(5);
    a       = Y(6:5+N);
    a_prime = Y(6+N:5+2*N);

    pitch = 10.45 + 5*sin(2*pi*f*t);

    % stiffness ----------------------------------------------------------
    if K_CG
        K = get_total_K(StructuralParameters, omega(1), psi);
    else
        K = StructuralParameters.K;
    end

    % aerodynamic forces -------------------------------------------------
    if dynamic_inflow
        PREV.a       = a;
        PREV.a_prime = a_prime;

        [F_modal, ~, ~, ~, PREV] = compute_aero_force_dynamic( ...
            x, dx, V_0, omega, pitch, ...
            AeroParameters.radius_aero, AeroParameters.twist_aero, ...
            AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
            coupling, true, PREV, dt);

        %da       = (PREV.a       - a)       / dt;
        %da_prime = (PREV.a_prime - a_prime) / dt;
        da       = zeros(size(a));          % frozen
        da_prime = zeros(size(a_prime));
    else
        [F_modal, ~, ~] = compute_aero_force_dynamic( ...
            x, dx, V_0, omega, pitch, ...
            AeroParameters.radius_aero, AeroParameters.twist_aero, ...
            AeroParameters.phi_1flap_aero, AeroParameters.phi_1edge_aero, ...
            coupling, false, [], dt);

        da       = zeros(size(a));          % frozen
        da_prime = zeros(size(a_prime));
    end

    % structural dynamics -----------------------------------------------
    ddx  = M \ (F_modal - C*dx - K*x);
    dpsi = omega(1);

    % return -------------------------------------------------------------
    dYdt = [dx; ddx; dpsi; da; da_prime];
end
