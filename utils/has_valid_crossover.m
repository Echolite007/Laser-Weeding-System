function ok = has_valid_crossover(m)
% True when loop_margins returned a usable gain crossover / phase margin.

ok = ~isempty(m.PM_deg) && ~isempty(m.wcp_rad_s) && ...
     ~any(isnan(m.PM_deg)) && ~any(isnan(m.wcp_rad_s)) && ...
     all(m.wcp_rad_s > 0);

end
