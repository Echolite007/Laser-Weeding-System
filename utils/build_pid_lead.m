function [controller_tf, gains] = build_pid_lead(m_eq, wc_rad_s, alpha, beta, s)
% PID controller with lead filter:
% C(s) = kp*(tau_z*s + 1)*(tau_i*s + 1) / ((tau_p*s + 1)*tau_i*s)
%
% gains returns the individual time constants and the proportional gain.

if nargin < 5 || isempty(s)
    s = tf('s');
end

leadRatio = sqrt(1/alpha);

gains = struct();
gains.tau_z = leadRatio/wc_rad_s;
gains.tau_i = beta*gains.tau_z;
gains.tau_p = 1/(wc_rad_s*leadRatio);
gains.kp    = m_eq*wc_rad_s^2/leadRatio;

controller_tf = gains.kp * ...
    (gains.tau_z*s + 1) * ...
    (gains.tau_i*s + 1) / ...
    ((gains.tau_p*s + 1)*gains.tau_i*s);

end
