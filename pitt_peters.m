function [vindnew, dvind_dt] = pitt_peters(Ct, vind, Uinf, R, dt)
% Pitt-Peters dynamic inflow model
% Ct: thrust coefficient on the actuator
% vind: induced velocity
% Uinf: unperturbed velocity
% R: radial scale of the flow
% dt: time step
% glauert: (optional) apply Glauert correction

a = -vind ./ Uinf; % induction coefficient for the time step {i-1}
Ctn = -CTfunction(a); % thrust coefficient from induction for the time step {i-1}

dvind_dt = (Ct - Ctn) / (16 / (3 * pi)) * (Uinf^2 / R); % time derivative of induction velocity
vindnew = vind + dvind_dt * dt; % induction at time {i} by time integration

end


function CT = CTfunction(a)
% Calculates the thrust coefficient as a function of induction factor 'a'

CT1 = 1.816;
if a < 1 - sqrt(CT1)/2
    CT = 4 .* a .* (1 - a);
else
    CT = CT1 - 4 * (sqrt(CT1) - 1) .* (1 - a);
end

end