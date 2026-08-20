function m = check_margins(label, m)
% Report open loops without a usable gain crossover instead of letting the
% empty output of margin() propagate into summary tables and result structs,
% and normalise missing margins to NaN.

if ~has_valid_crossover(m)
    warning('checkMargins:noCrossover', ...
        ['No valid gain crossover found for %s; the reported margins for this ', ...
         'configuration are meaningless.'], label);

    names = fieldnames(m);
    for i = 1:numel(names)
        if isempty(m.(names{i}))
            m.(names{i}) = NaN;
        end
    end
elseif any(m.PM_deg <= 0)
    warning('checkMargins:nonPositivePhaseMargin', ...
        'Phase margin for %s is %.2f deg, i.e. the open loop is not stable.', ...
        label, m.PM_deg(1));
end

end
