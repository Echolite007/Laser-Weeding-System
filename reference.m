function ref = reference(vw, tw, tr, ts)

refgenName = 'ReferenceGenerator_2023a';
harness = 'refgen_harness_tmp';

if isempty(which([refgenName '.slx']))
    error('reference:missingModel', ...
        'Reference generator model %s.slx was not found on the MATLAB path.', refgenName);
end

if bdIsLoaded(refgenName); close_system(refgenName, 0); end
if bdIsLoaded(harness);    close_system(harness, 0);    end

load_system([refgenName '.slx']);

new_system(harness);
load_system(harness);

% Close both models even if the harness build or the simulation fails, so a
% failed run cannot leave stale models loaded and break the next call.
cleanup = onCleanup(@() closeModels({harness, refgenName})); %#ok<NASGU>

add_block('simulink/Sources/Constant', [harness '/vw_const'], 'Value', num2str(vw, 16));
add_block('simulink/Sources/Constant', [harness '/tw_const'], 'Value', num2str(tw, 16));
add_block('simulink/Sources/Constant', [harness '/tr_const'], 'Value', num2str(tr, 16));
add_block('simulink/Sources/Constant', [harness '/ts_const'], 'Value', num2str(ts, 16));

add_block([refgenName '/Mirror angle for weeding'], [harness '/RefGen']);

outs = {'r','dr','ddr','dddr','a2p'};
for i = 1:numel(outs)
    add_block('simulink/Sinks/To Workspace', [harness '/' outs{i} '_out']);
    set_param([harness '/' outs{i} '_out'], 'VariableName', outs{i});
    set_param([harness '/' outs{i} '_out'], 'SaveFormat', 'Array');
end

add_line(harness, 'vw_const/1', 'RefGen/1');
add_line(harness, 'tw_const/1', 'RefGen/2');
add_line(harness, 'tr_const/1', 'RefGen/3');
add_line(harness, 'ts_const/1', 'RefGen/4');
for i = 1:numel(outs)
    add_line(harness, ['RefGen/' num2str(i)], [outs{i} '_out/1']);
end

set_param(harness, 'StopTime', num2str(tw + tr, 16));
set_param(harness, 'FixedStep', num2str(ts, 16));
set_param(harness, 'SolverType', 'Fixed-step');

try
    simOut = sim(harness);
catch simError
    throw(addCause(MException('reference:simulationFailed', ...
        'Simulation of reference harness %s failed.', harness), simError));
end

% Fail loudly when the harness produced no data: an empty reference silently
% turns into empty/NaN maxima that propagate through the whole design chain.
required = [{'tout'}, outs];
for i = 1:numel(required)
    name = required{i};
    try
        loggedSignal = simOut.(name);
    catch missingSignalError
        throw(addCause(MException('reference:missingSignal', ...
            'Reference harness did not log signal ''%s''.', name), missingSignalError));
    end
    if isempty(loggedSignal)
        error('reference:emptySignal', ...
            'Reference harness logged signal ''%s'' but it is empty.', name);
    end
end

ref = struct();
ref.t            = simOut.tout;
ref.r            = simOut.r;
ref.dr           = simOut.dr;
ref.ddr          = simOut.ddr;
ref.dddr         = simOut.dddr;
ref.a2p          = simOut.a2p(1);
ref.theta_max    = max(abs(simOut.r));
ref.dtheta_max   = max(abs(simOut.dr));
ref.ddtheta_max  = max(abs(simOut.ddr));
ref.dddtheta_max = max(abs(simOut.dddr));

end

function closeModels(names)
% Best-effort close: report, but never mask, a failure that is already
% propagating out of the caller.
for i = 1:numel(names)
    if bdIsLoaded(names{i})
        try
            close_system(names{i}, 0);
        catch closeError
            warning('reference:closeFailed', ...
                'Could not close model %s: %s', names{i}, closeError.message);
        end
    end
end
end
