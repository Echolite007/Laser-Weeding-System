function [tau_z, tau_p] = lead_time_constants(alpha, fc_lead_hz)
% Zero/pole time constants of a lead filter centred at fc_lead_hz with
% ratio alpha, so the maximum phase lead occurs at fc_lead_hz.

wc_lead_rad_s = 2*pi*fc_lead_hz;

tau_z = 1/(wc_lead_rad_s*sqrt(alpha));
tau_p = alpha*tau_z;

end
