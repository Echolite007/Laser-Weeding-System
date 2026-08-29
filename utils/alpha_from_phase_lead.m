function alpha = alpha_from_phase_lead(phase_lead_deg, alpha_limits)
    % Lead-filter ratio alpha that provides phase_lead_deg of phase lead:
    %   alpha = (1 - sin(phi)) / (1 + sin(phi))
    %
    % alpha_limits (optional) is a [lower upper] pair the result is clamped to.

    alpha = (1 - sind(phase_lead_deg)) / (1 + sind(phase_lead_deg));

    if nargin > 1 && ~isempty(alpha_limits)
        alpha = max(alpha_limits(1), min(alpha_limits(2), alpha));
    end

end
