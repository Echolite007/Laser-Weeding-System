function check_positive_finite(name, value)
% Reject inputs that would silently turn a sizing or design calculation into
% Inf/NaN (e.g. the NaN placeholders in parameters.m used as divisors).

if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('checkPositiveFinite:invalidInput', ...
        '%s must be a finite positive scalar, got %s.', name, mat2str(value));
end

end
