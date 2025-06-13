function [Rx, FN, FT, P, a_list, a_prime_list] = BEM2(v0_array, omega0, V_inplane, V_outplane, pitch, coupling)

%------------------------------------------------
% Blade Element Momentum (BEM) with coupling
%------------------------------------------------
% Delft University of Technology – Wind Turbine Aeroelasticity

% Fixed parameters
B = 3;              % Number of blades
R = 63;             % Rotor radius [m]
hubrad = 1.5;       % Hub radius [m]
rho = 1.225;        % Air density [kg/m^3]
EPS = 1e-5;         % Iteration tolerance
REG = 1e-8;         % Regularization constant

% Load blade structure and aerofoil data
BS = importdata('Blade/Blade section/Blade section.dat').data;
Readfiles = dir(fullfile('Blade/Aero data', '*.dat'));
for j = 1:length(Readfiles)
    AD{j} = importdata(fullfile('Blade/Aero data', Readfiles(j).name));
end

NBS = size(BS,1);
Rx = zeros(NBS,1); FN = zeros(NBS,1); FT = zeros(NBS,1); Mx = zeros(NBS,1);
a_list = zeros(NBS,1); a_prime_list = zeros(NBS,1);

for i = 1:NBS
    % Section properties
    ADofBS = BS(i,2);
    r      = BS(i,3); Rx(i) = r;
    dr     = BS(i,4);
    theta  = BS(i,5);     % [deg]
    chord  = BS(i,6);     % [m]
    
    alpha = AD{ADofBS}(:,1);
    Cl    = AD{ADofBS}(:,2);
    Cd    = AD{ADofBS}(:,3);

    v0    = v0_array(i);
    omega = omega0(i);
    if coupling
        V_oop = V_outplane(i);
        V_ip  = V_inplane(i);
    else
        V_oop = 0;
        V_ip  = 0;
    end

    Sigma = chord * B / (2 * pi * r);

    % Initial induction guesses
    a = 0.0; a_prime = 0.0;
    ax = a; ax_prime = a_prime;
    numite = 0;
    
    % Iteration loop
    while abs(ax - a) >= EPS || abs(ax_prime - a_prime) >= EPS
        numite = numite + 1;
        a = ax; a_prime = ax_prime;

        denom = (1 + a_prime) * r * omega - V_ip;
        if abs(denom) < REG
            denom = sign(denom + REG) * REG;
        end
        Phi = atan((((1 - a) * v0 - V_oop)/ denom));
        Phi_deg = rad2deg(Phi);
        Alpha = Phi_deg - theta - pitch;

        % Interpolate Cl and Cd
        Cla = interp1(alpha, Cl, Alpha, 'linear', 'extrap');
        Cda = interp1(alpha, Cd, Alpha, 'linear', 'extrap');

        % Aerodynamic force coefficients
        Cn = Cla * cosd(Phi_deg) + Cda * sind(Phi_deg);
        Ct = Cla * sind(Phi_deg) - Cda * cosd(Phi_deg);

        % Prandtl losses
        if abs(sind(Phi_deg)) < REG
            F = 1.0;
        else
            f_tip = B/2 * (R - r) / (r * sind(Phi_deg));
            F_tip = (2/pi) * acos(exp(-abs(f_tip)));

            f_hub = B/2 * (r - hubrad) / (r * sind(Phi_deg));
            F_hub = (2/pi) * acos(exp(-abs(f_hub)));

            F = F_tip * F_hub;
        end

        % Glauert correction
        ac = 0.2;
        if a > ac
            K = 4 * F * sind(Phi_deg)^2 / (Sigma * Cn + REG);
            sqrt_term = sqrt((K * (1 - 2 * ac) + 2)^2 + 4 * (K * ac^2 - 1));
            ax = 0.5 * (2 + K * (1 - 2 * ac) - sqrt_term);
        else
            ax = 1 / (4 * F * sind(Phi_deg)^2 / (Sigma * Cn + REG) + 1);
        end

        denom_ct = 4 * F * sind(Phi_deg) * cosd(Phi_deg) / (Sigma * Ct + REG) - 1;
        if abs(denom_ct) > REG
            ax_prime = 1 / denom_ct;
        else
            ax_prime = 0;
        end

        % Force a stop if not converging
        if numite >= 100
            ax = 0.3;
            ax_prime = 0.1;
            break;
        end
    end

    % Save final inductions
    a_list(i) = ax;
    a_prime_list(i) = ax_prime;

    % Recompute final Phi, Alpha, Cn, Ct using final a, a'
    %Phi = atan(((1 - ax) * v0 - V_oop) / ((1 + ax_prime) * r * omega - V_ip));
    Phi = atan(((1 - ax) * v0 - V_oop) / ((1 + ax_prime) * r * omega - V_ip));
    Phi_deg = rad2deg(Phi);
    Alpha = Phi_deg - theta - pitch;

    Cla = interp1(alpha, Cl, Alpha, 'linear', 'extrap');
    Cda = interp1(alpha, Cd, Alpha, 'linear', 'extrap');

    Cn = Cla * cosd(Phi_deg) + Cda * sind(Phi_deg);
    Ct = Cla * sind(Phi_deg) - Cda * cosd(Phi_deg);

    % Compute forces
    U_rel_sq = (r * omega * (1 + ax_prime) - V_ip)^2 + (v0 * (1 - ax) - V_oop)^2;
    FN(i) = 0.5 * rho * U_rel_sq * chord * Cn;
    FT(i) = 0.5 * rho * U_rel_sq * chord * Ct;

    % Root moment
    Mx(i) = FT(i) * r;
end

% Total torque and power (with 94.4% generator efficiency)
M = sum(Mx);
P = M * omega * 3 * 0.944;

end


% function [Rx,FN,FT,P]=BEM(v0,omega,pitch)
% %------------------------------------------------
% % Blade Element Momentum
% %------------------------------------------------
% % this code is for the course: wind turbine aeroelasticity
% % Delft university of technology

% %-------------------STATEMENT-------------------%
% % a: axial induction
% % a_prime: tangential induction
% % Phi: inflow angle ��
% % Alpha: local attack angle ��
% % Theta: twist angle
% % pitch: blade pitch angle
% % Sigma: solidity
% % Cl: lift coefficient
% % Cd: drag coefficient
% % Cn: components of n (along wind direction) in Cartesian coordinate system
% % Ct: components of t (perpendicular to wind direction) in Cartesian coordinate system
% % v0: inflow wind speed
% % omega: blade rotation angular speed
% % r: radius of blade
% %-----------------------------------------------%

% %---------------START SIMULATION----------------%

% %------------------------------------------------
% % fixed parameters
% %------------------------------------------------
% B=3;           %number of blades
% R=63;          %rotor radius
% hubrad=1.5;    %hub radius
% rou=1.225;     %density of air
% EPS=0.00001;    %iterative precision tolerance

% %------------------------------------------------
% % Initialization & Iteration
% %------------------------------------------------
% %initialization: initial value of inductions
% a=0;a_prime=0;

% %import Blade section file
% BS=importdata('Blade\Blade section\Blade section.dat').data;

% %import Aero data files
% Readfiles = dir(fullfile('Blade\Aero data\','*.dat'));
% for i=1:length(Readfiles)
%     AD{i}=importdata(strcat('Blade\Aero data\',Readfiles(i).name));
% end

% NBS=length(BS);    %Number of blade sections
% % define vectors for blade section locations and loads in two directions
% Rx=zeros(NBS,1);FN=zeros(NBS,1);FT=zeros(NBS,1);

% % LOOP: from the root section to the tip section
% for i=1:NBS
%     ADofBS=BS(i,2); % read airfoil number for each section
%     r=BS(i,3);      % read radius
%     Rx(i)=r;        % record radius
%     dr=BS(i,4);     % read segment length
%     Theta=BS(i,5);  % read twist angle
%     chord=BS(i,6);   % chord length
%     alpha=AD{ADofBS}(:,1);% coefficients table: AOA
%     Cl=AD{ADofBS}(:,2); % coefficients table: lift coe
%     Cd=AD{ADofBS}(:,3); % coefficients table: drag coe
%     Sigma=chord*B/(2*pi*r); % solidity
    
%     ax=a;                     %change value
%     ax_prime=a_prime;         %change value
%     a=ax-10*EPS;              %generate error, active iteration
%     a_prime=ax_prime-10*EPS;  %generate error, active iteration
    
%     numite=0; % iteration counter
%     %iteration, stop when error is smaller than EPS
%     while abs(ax-a)>=EPS || abs(ax_prime-a_prime)>=EPS
%         numite=numite+1;
        
%         % record results of last step
%         a=ax;
%         a_prime=ax_prime;
        
%         % inflow angle
%         Phi=atan((1-a)*v0/((1+a_prime)*r*omega));
%         Phi=rad2deg(Phi);
        
%         %AOA
%         Alpha=Phi-Theta-pitch;
        
%         % find Cl and Cd
%         Cla=interp1(alpha,Cl,Alpha);
%         Cda=interp1(alpha,Cd,Alpha);
        
%         %projection in and out of plane
%         Cn=Cla*cosd(Phi)+Cda*sind(Phi);
%         Ct=Cla*sind(Phi)-Cda*cosd(Phi);
        
%         %Prandtl Loss
%         f_tiploss = B/2*(R-r)/(r*sind(Phi));
%         F_tiploss = (2/pi)*acos(exp(-f_tiploss));
%         f_hubloss = B/2*(r-hubrad)/(r*sind(Phi));
%         F_hubloss = (2/pi)*acos(exp(-f_hubloss));
%         F = F_tiploss*F_hubloss;
        
%         %Glauert Correction
%         ac=0.2;
%         if ax>ac
%             K=4*F*sind(Phi)^2/(Sigma*Cn);
%             ax=0.5*(2+K*(1-2*ac)-sqrt((K*(1-2*ac)+2)^2+4*(K*ac^2-1)));
%         else
%             ax=1/(4*F*(sind(Phi))^2/(Sigma*Cn)+1);
%         end
%         ax_prime=1/(4*F*sind(Phi)*cosd(Phi)/(Sigma*Ct)-1);
        
%         % in case of iterative convergence failure
%         if numite>=100
%             ax=0.3;
%             ax_prime=0.1;
%         end
%     end
    
% %------------------------------------------------
% % Result
% %------------------------------------------------
%     % update value
%     a=ax;
%     a_prime=ax_prime;
    
%     % force in two directions
%     FN(i)=0.5*rou*((r*omega*(1+a_prime))^2+(v0*(1-a))^2)*chord*Cn;
%     FT(i)=0.5*rou*((r*omega*(1+a_prime))^2+(v0*(1-a))^2)*chord*Ct;
%     % bending moment
%     Mx(i)=FT(i)*r;
% end

% M=sum(Mx); % rotor torque from one blade
% P=M*omega*3*0.944;  % Power
% end