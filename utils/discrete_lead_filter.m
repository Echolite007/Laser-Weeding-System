function [C_lead_z, C_lead_c] = discrete_lead_filter(tau_z, tau_p, Ts)
% Continuous lead filter (tau_z*s + 1)/(tau_p*s + 1) and its Tustin
% discretisation at sample time Ts.

C_lead_c = tf([tau_z 1], [tau_p 1]);
C_lead_z = c2d(C_lead_c, Ts, 'tustin');

end
