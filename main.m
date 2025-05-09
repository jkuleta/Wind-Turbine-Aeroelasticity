% this code is for the course: wind turbine aeroelasticity
% Delft university of technology

clc
close all;
clear all;

%% Load key parameters - AERODYNAMICS
load 'STATE'
v0=WindSpeeds;    % read wind speed array
omega=RtSpeeds*2*pi/60;  %rotation angular velocity
pitch=PitchAngles;   % pitch angle: collective pitch wind turbine

%% Load structural blade data
structural_data = importdata("Structural data.txt");

radius = structural_data.data(:,1); % R [m]
blade_position = structural_data.data(:,2); % r/R [-]
mass_distribution = structural_data.data(:, 5); % rhoA [kg/m]
flap_stiffness_distribution = structural_data.data(:, 6); % EI [Nm^2]
edge_stiffness_distribution = structural_data.data(:, 7); % EI [Nm^2]
R = radius(end);

%% Define damping parameters
damping_flapwise = 0.477465*10^-2; % damping = 0.477465%
damping_edgewise = damping_flapwise; 

%% Define shape functions and its derivative
phi_1flap = 0.0622*blade_position.^2 + 1.7254*blade_position.^3 - 3.2452 * blade_position.^4 + 4.7131*blade_position.^5 - 2.2555*blade_position.^6;
dphi_1flap = 2*.0622*radius/(R^2) + 3*1.7254*radius.^2/(R^3) - 4*3.2452*radius.^3/(R^4) + 5*4.7131*radius.^4/(R^5) - 6*2.2555*radius.^5/(R^6); %first derivative
ddphi_1flap = 2*.0622/(R^2) + 2*3*1.7254*radius/(R^3) - 3*4*3.2452*radius.^2/(R^4) + 4*5*4.7131*radius.^3/(R^5) - 5*6*2.2555*radius.^4/(R^6); %second derivative

phi_1edge = 0.3627*blade_position.^2 + 2.5337*blade_position.^3 - 3.5772*blade_position.^4 + 2.376*blade_position.^5 - 0.6952*blade_position.^6;
dphi_1edge = 2*0.3627*radius/(R^2) + 3*2.5337*radius.^2/(R^3) - 4*3.5772*radius.^3/(R^4) + 5*2.376*radius.^4/(R^5) - 6*0.6952*radius.^5/(R^6); %first derivative
ddphi_1edge = 2*0.3627/(R^2) + 2*3*2.5337*radius/(R^3) - 3*4*3.5772*radius.^2/(R^4) + 4*5*2.376*radius.^3/(R^5) - 5*6*0.6952*radius.^4/(R^6); %second derivative

%% Calculate masses and stiffnesses

M_1flap = trapz(radius, mass_distribution.*(phi_1flap.^2));
M_1edge = trapz(radius, mass_distribution.*(phi_1edge.^2));

K_1flap = trapz(radius, flap_stiffness_distribution.*(ddphi_1flap.^2));
K_1edge = trapz(radius, edge_stiffness_distribution.*(ddphi_1edge.^2));

%% Create matrices

M = [M_1flap, 0; 
     0, M_1edge;];
K = [K_1flap, 0;
    0,  K_1edge];

C = [2*damping_flapwise * sqrt(K_1flap*M_1flap), 0;
    0,  2*damping_edgewise * sqrt(K_1edge*M_1edge)];

%% Calculate natural frequencies

omega_1flap = sqrt(K_1flap/M_1flap);
omega_1edge = sqrt(K_1edge/M_1edge);

%% Read aerodynamic twist data and radius discretization, compute shape function

twist_aero = importdata('Blade\Blade section\Blade section.dat').data(:,5);
radius_aero = importdata('Blade\Blade section\Blade section.dat').data(:,3);
blade_position_aero = radius_aero/R;

phi_1flap_aero = 0.0622*blade_position_aero.^2 + 1.7254*blade_position_aero.^3 - 3.2452 * blade_position_aero.^4 + 4.7131*blade_position_aero.^5 - 2.2555*blade_position_aero.^6;
phi_1edge_aero = 0.3627*blade_position_aero.^2 + 2.5337*blade_position_aero.^3 - 3.5772*blade_position_aero.^4 + 2.376*blade_position_aero.^5 - 0.6952*blade_position_aero.^6;

%% Coupling with BEM - loop: wind speed from 3m/s to 25m/s
for i=1:length(v0)
    % call BEM, outputs: Radius, Loads, and power outputs
    [Rx,FN,FT,P(i)]=BEM(v0(i),omega(i),pitch(i));

    % translation of coordinates
    FF = FN .* cos(pitch(i)+twist_aero) + FT .* sin(pitch(i)+twist_aero);
    FE = FN .* -sin(pitch(i)+twist_aero) + FT .* cos(pitch(i)+twist_aero);

    % calculate resulting forces
    F_1flap = trapz(radius_aero, FF.*phi_1flap_aero);
    F_1edge = trapz(radius_aero,FE.*phi_1edge_aero);

    % plot FN and FT
    figure(1)
    plot(Rx,FN,'r-o');
    hold on;
    plot(Rx,FT,'b-o');
    hold on
    grid on
    xlabel('Radius(m)');
    ylabel('Loads(N)');
    legend('Fn','Ft');
end

%% plot power curve regarding wind speed
figure(2)
plot(v0,P,'b-o','linewidth',1.5);
xlabel('Wind speed(m/s)');
ylabel('Power(W)');