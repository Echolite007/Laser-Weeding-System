function p = second_order_params(sys)
% Equivalent second-order parameters of a transfer function written in the
% standard form K/(m*s^2 + d*s + k).

[numerator, denominator] = tfdata(sys, 'v');

p = struct();
p.m_eq     = abs(denominator(end-2)/numerator(end));
p.d_eq     = abs(denominator(end-1)/numerator(end));
p.k_eq     = abs(denominator(end)/numerator(end));
p.wn_rad_s = sqrt(denominator(end));
p.zeta     = denominator(end-1)/(2*p.wn_rad_s);

end
