function print_margin_table(headerText, labelWidth, rows)
% Print a crossover / phase-margin / gain-margin summary table.
%
% rows is an n-by-4 cell array: {label, wc_rad_s, PM_deg, GM_dB}.

fprintf('\n%s\n', headerText);

fprintf('%-*s  %8s  %8s  %8s\n', labelWidth, ...
    'Configuration', 'wc(rad/s)', 'PM(deg)', 'GM(dB)');

for i = 1:size(rows, 1)
    fprintf('%-*s  %8.1f  %8.1f  %8.1f\n', labelWidth, ...
        rows{i,1}, rows{i,2}, rows{i,3}, rows{i,4});
end

end
