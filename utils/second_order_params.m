function p = second_order_params(sys)
% Equivalent second-order parameters of a transfer function written in the
% standard form K/(m*s^2 + d*s + k).

[numerator, denominator] = tfdata(sys, 'v');

% Indexing three denominator coefficients and dividing by the numerator DC
% gain both silently return Inf/NaN parameters for a lower-order or gain-free
% model, which then propagate through the whole design chain.
if numel(denominator) < 3
    error('secondOrderParams:order', ...
        ['Transfer function has denominator order %d; a second-order model is ', ...
         'required to extract m_eq, d_eq and k_eq.'], numel(denominator) - 1);
end

if ~isfinite(numerator(end)) || numerator(end) == 0
    error('secondOrderParams:gain', ...
        'Transfer function has a zero or non-finite DC numerator coefficient (%g).', ...
        numerator(end));
end

p = struct();
p.m_eq     = abs(denominator(end-2)/numerator(end));
p.d_eq     = abs(denominator(end-1)/numerator(end));
p.k_eq     = abs(denominator(end)/numerator(end));
p.wn_rad_s = sqrt(denominator(end));
p.zeta     = denominator(end-1)/(2*p.wn_rad_s);

end
