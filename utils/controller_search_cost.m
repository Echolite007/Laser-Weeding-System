function [J, feasible] = controller_search_cost(PM_deg, fc_hz, fc_target_hz, PM_min_deg, PM_max_deg)
    % Cost used by the retuning search: quadratic penalty on the log-distance to
    % the target crossover plus quadratic penalties for leaving the phase-margin
    % window. Feasible candidates get a fixed bonus so they always win over
    % infeasible ones with a similar crossover.

    fc_penalty      = (log(fc_hz / fc_target_hz))^2;
    PM_low_penalty  = max(0, PM_min_deg - PM_deg)^2;
    PM_high_penalty = max(0, PM_deg - PM_max_deg)^2;

    J = 100 * PM_low_penalty + 100 * PM_high_penalty + 50 * fc_penalty;

    feasible = (PM_deg >= PM_min_deg) && (PM_deg <= PM_max_deg);

    if feasible
        J = J - 10;
    end

end
