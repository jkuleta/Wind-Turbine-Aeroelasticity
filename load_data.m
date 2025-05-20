function [StructuralParameters, OperationalParameters, AeroParameters] = load_data()

  %% Load key parameters - AERODYNAMICS
    load 'STATE'

    OperationalParameters.v0_values = WindSpeeds;
    OperationalParameters.omega_values = RtSpeeds * 2 * pi / 60;
    OperationalParameters.pitch_values = PitchAngles;

    %% Load structural blade data
    structural_data = importdata("Structural data.txt");

    StructuralParameters.radius = structural_data.data(:,1);
    StructuralParameters.blade_position = structural_data.data(:,2);
    StructuralParameters.mass_distribution = structural_data.data(:, 5);
    StructuralParameters.flap_stiffness_distribution = structural_data.data(:, 6);
    StructuralParameters.edge_stiffness_distribution = structural_data.data(:, 7);
    StructuralParameters.R = StructuralParameters.radius(end);

    %% Damping
    StructuralParameters.damping_flapwise = 0.477465;
    StructuralParameters.damping_edgewise = StructuralParameters.damping_flapwise;

    %% Shape functions
    StructuralParameters.phi_1flap = 0.0622*StructuralParameters.blade_position.^2 + 1.7254*StructuralParameters.blade_position.^3 - ...
                3.2452*StructuralParameters.blade_position.^4 + 4.7131*StructuralParameters.blade_position.^5 - 2.2555*StructuralParameters.blade_position.^6;
    StructuralParameters.ddphi_1flap = 2*.0622/(StructuralParameters.R^2) + 2*3*1.7254*StructuralParameters.radius/(StructuralParameters.R^3) - ...
                3*4*3.2452*StructuralParameters.radius.^2/(StructuralParameters.R^4) + 4*5*4.7131*StructuralParameters.radius.^3/(StructuralParameters.R^5) - ...
                5*6*2.2555*StructuralParameters.radius.^4/(StructuralParameters.R^6);

    StructuralParameters.phi_1edge = 0.3627*StructuralParameters.blade_position.^2 + 2.5337*StructuralParameters.blade_position.^3 - ...
                3.5772*StructuralParameters.blade_position.^4 + 2.376*StructuralParameters.blade_position.^5 - 0.6952*StructuralParameters.blade_position.^6;
    StructuralParameters.ddphi_1edge = 2*0.3627/(StructuralParameters.R^2) + 2*3*2.5337*StructuralParameters.radius/(StructuralParameters.R^3) - ...
                3*4*3.5772*StructuralParameters.radius.^2/(StructuralParameters.R^4) + 4*5*2.376*StructuralParameters.radius.^3/(StructuralParameters.R^5) - ...
                5*6*0.6952*StructuralParameters.radius.^4/(StructuralParameters.R^6);

    %% Mass & Stiffness
    StructuralParameters.M_1flap = trapz(StructuralParameters.radius, StructuralParameters.mass_distribution.*(StructuralParameters.phi_1flap.^2));
    StructuralParameters.M_1edge = trapz(StructuralParameters.radius, StructuralParameters.mass_distribution.*(StructuralParameters.phi_1edge.^2));
    StructuralParameters.K_1flap = trapz(StructuralParameters.radius, StructuralParameters.flap_stiffness_distribution.*(StructuralParameters.ddphi_1flap.^2));
    StructuralParameters.K_1edge = trapz(StructuralParameters.radius, StructuralParameters.edge_stiffness_distribution.*(StructuralParameters.ddphi_1edge.^2));

    StructuralParameters.M = diag([StructuralParameters.M_1flap, StructuralParameters.M_1edge]);
    StructuralParameters.K = diag([StructuralParameters.K_1flap, StructuralParameters.K_1edge]);
    StructuralParameters.C = diag([2*StructuralParameters.damping_flapwise * sqrt(StructuralParameters.K_1flap*StructuralParameters.M_1flap), ...
            2*StructuralParameters.damping_edgewise * sqrt(StructuralParameters.K_1edge*StructuralParameters.M_1edge)]);

    %% Natural frequencies
    StructuralParameters.omega_1flap = sqrt(StructuralParameters.K_1flap/StructuralParameters.M_1flap);
    StructuralParameters.omega_1edge = sqrt(StructuralParameters.K_1edge/StructuralParameters.M_1edge);

    %% Aerodynamic properties
    blade_data = importdata('Blade/Blade section/Blade section.dat');
    AeroParameters.twist_aero = blade_data.data(:,5);
    AeroParameters.radius_aero = blade_data.data(:,3);
    AeroParameters.blade_position_aero = AeroParameters.radius_aero/StructuralParameters.R;

    AeroParameters.phi_1flap_aero = 0.0622*AeroParameters.blade_position_aero.^2 + 1.7254*AeroParameters.blade_position_aero.^3 - ...
                    3.2452*AeroParameters.blade_position_aero.^4 + 4.7131*AeroParameters.blade_position_aero.^5 - ...
                    2.2555*AeroParameters.blade_position_aero.^6;
    AeroParameters.phi_1edge_aero = 0.3627*AeroParameters.blade_position_aero.^2 + 2.5337*AeroParameters.blade_position_aero.^3 - ...
                    3.5772*AeroParameters.blade_position_aero.^4 + 2.376*AeroParameters.blade_position_aero.^5 - ...
                    0.6952*AeroParameters.blade_position_aero.^6;
end
