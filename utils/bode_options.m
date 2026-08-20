function opts = bode_options(varargin)
% bodeoptions with unwrapped phase by default. Extra name/value pairs are
% applied on top, e.g. bode_options('XLim', [1 6283], 'FreqUnits', 'Hz').

opts = bodeoptions;
opts.PhaseWrapping = 'off';

for k = 1:2:numel(varargin)
    opts.(varargin{k}) = varargin{k+1};
end

end
