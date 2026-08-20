function [plant_voltage_to_angle, voltageToTorqueGain] = nominal_plant_tf(sizing, actuator, s)
% Nominal voltage-to-mirror-angle plant of the sized second-order mechanism:
%
%   P(s) = (r_arm*Kf/R) / (J*s^2 + d*s + k)
%
% sizing needs the fields r_arm_m, J_kgm2, d_Nms_per_rad and k_Nm_per_rad
% (as produced by nominal_plant_sizing), actuator the fields Kf_N_per_A and
% R25_ohm. voltageToTorqueGain is the numerator gain, i.e. J/gain is the
% equivalent inertia seen by the controller.

if nargin < 3 || isempty(s)
    s = tf('s');
end

voltageToTorqueGain = sizing.r_arm_m * actuator.Kf_N_per_A / actuator.R25_ohm;

plant_voltage_to_angle = voltageToTorqueGain / ...
    (sizing.J_kgm2*s^2 + sizing.d_Nms_per_rad*s + sizing.k_Nm_per_rad);

end
