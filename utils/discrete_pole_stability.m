function [numUnstablePoles, isStable] = discrete_pole_stability(sys)
    % Count closed-loop poles outside the unit circle of a discrete-time system.

    numUnstablePoles = sum(abs(pole(sys)) > 1);
    isStable = (numUnstablePoles == 0);

end
