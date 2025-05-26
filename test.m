[StructuralParameters, OperationalParameters, AerodynamicParameters] = load_data();

v0 = 15*ones(size(AerodynamicParameters.blade_position_aero));
omega = 9*2*pi/60*ones(size(AerodynamicParameters.blade_position_aero));

[Rx, FN, FT, a_list, a_prime_list] = BEM(v0, omega, 0);

plot(Rx, FN, "ro-", "LineWidth", 2);
hold on;
plot(Rx, FT, "ko-", "LineWidth", 2);
xlabel("Radius [m]");
ylabel("Force [N]");
grid on;
legend("Normal Force", "Tangential Force");