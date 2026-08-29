function style_plot(titleText)
    % Common styling for the report figures: optional title, grid, thicker lines.

    if nargin > 0 && ~isempty(titleText)
        title(titleText);
    end

    grid on;
    set(findall(gcf, 'Type', 'line'), 'LineWidth', 1.3);

end
