close all; clc;

addpath('utils');

%% Extract data
t = simout.time;
u = simout.signals.values(:,1);   % input voltage [V]
y = simout.signals.values(:,2);   % output angle [rad]

% Remove first sample
t = t(2:end);
u = u(2:end);
y = y(2:end);

%% Select signal type
% inputType = 0 -> multisine
% inputType = 1 -> chirp
if ~exist('inputType','var')
    warning('inputType not found. Assuming multisine.');
    inputType = 0;
end

%% Basic frequency settings
fs = 1/ts;             % sampling frequency [Hz]
fNyq = fs/2;           % Nyquist frequency [Hz]

%% Processing settings
averageBlockSize = 100;  % Use 5-10. Larger = smoother, lower frequency resolution.
fMinUseful = 0.5;      % [Hz]
fMaxUseful = 120;      % [Hz]
inputThresholdRaw = 0.01;     % for raw FRF point selection
inputThresholdSmooth = 0.05;  % for smoothed FRF point selection

%% Process data
if inputType == 0
    %% Multisine processing with period averaging

    N = round(T/ts);               % samples per period
    Nr = floor(length(u)/N);       % number of complete periods

    if Nr < 1
        error('Not enough data for one full period. Increase simulation time Ttot.');
    end

    % Keep only complete periods at the end of the measurement
    u = u(end+1-Nr*N:end);
    y = y(end+1-Nr*N:end);
    t = t(end+1-Nr*N:end);

    ur_all = reshape(u,N,[]);
    yr_all = reshape(y,N,[]);

    % Plot all periods to inspect transient behavior
    figure;
    subplot(2,1,1)
    plot((0:N-1)*ts, ur_all');
    grid on
    xlabel('Time in period [s]');
    ylabel('Voltage [V]');
    title('Input voltage per period');

    subplot(2,1,2)
    plot((0:N-1)*ts, yr_all');
    grid on
    xlabel('Time in period [s]');
    ylabel('Angle [rad]');
    title('Measured angle per period');

    % Choose periods after transients
    chosenPeriods = ceil(Nr/2):Nr;

    ur = mean(ur_all(:,chosenPeriods),2);
    yr = mean(yr_all(:,chosenPeriods),2);

    N = length(ur);
    fResolution = 1/T;
    fgrid = (0:N-1)' * fResolution;

else
    %% Chirp processing

    ur = u;
    yr = y;

    N = length(ur);
    fResolution = fs/N;
    fgrid = (0:N-1)' * fResolution;
end

%% FFT
uf = fft(ur);
yf = fft(yr);

%% Use only positive frequencies up to Nyquist
validPos = fgrid > 0 & fgrid <= fNyq;

fgrid_pos = fgrid(validPos);
uf_pos = uf(validPos);
yf_pos = yf(validPos);

%% Select excited frequencies
threshold = inputThresholdRaw * max(abs(uf_pos));
ind = abs(uf_pos) > threshold;

f_meas = fgrid_pos(ind);
Hm = -yf_pos(ind) ./ uf_pos(ind);   % measured FRF: angle / voltage

%% Raw FRD object
Hm_frd = frd(Hm, 2*pi*f_meas);

%% Chirp noise reduction by block averaging
if inputType == 1

    % Restrict to useful frequency band before averaging
    useful = f_meas >= fMinUseful & f_meas <= fMaxUseful;

    f_for_avg = f_meas(useful);
    Hm_for_avg = Hm(useful);

    % Remove any leftover points that do not fit into complete blocks
    nBlocks = floor(length(f_for_avg)/averageBlockSize);
    nUse = nBlocks * averageBlockSize;

    f_for_avg = f_for_avg(1:nUse);
    Hm_for_avg = Hm_for_avg(1:nUse);

    % Reshape into blocks of 5-10 frequency points
    f_blocks = reshape(f_for_avg, averageBlockSize, []);
    Hm_blocks = reshape(Hm_for_avg, averageBlockSize, []);

    % Average frequency and complex FRF inside each block
    f_smooth = mean(f_blocks, 1).';
    Hm_smooth = mean(Hm_blocks, 1).';

else

    % For multisine, do not average neighbouring frequencies
    f_smooth = f_meas;
    Hm_smooth = Hm;

    useful = f_smooth >= fMinUseful & f_smooth <= fMaxUseful;
    f_smooth = f_smooth(useful);
    Hm_smooth = Hm_smooth(useful);

end

%% Optional extra median smoothing after block averaging
Hm_real_s = smoothdata(real(Hm_smooth(:)), 'movmedian', 3);
Hm_imag_s = smoothdata(imag(Hm_smooth(:)), 'movmedian', 3);
Hm_smooth = Hm_real_s + 1i*Hm_imag_s;

%% Smoothed FRD object
Hm_frd_smooth = frd(Hm_smooth, 2*pi*f_smooth, ts);

%% Save results
save('Hm_frd_deliverable_P.mat', ...
     'Hm_frd','f_meas','Hm','ts','T','inputType');

save('Hm_frd_deliverable_P_smooth.mat', ...
     'Hm_frd_smooth','f_smooth','Hm_smooth','ts','T','inputType', ...
     'averageBlockSize');

%% Plot raw and smoothed Bode
figure;
opts = bode_options('FreqUnits', 'Hz', 'PhaseWrapping', 'on');

bodeplot(Hm_frd, Hm_frd_smooth, opts);
grid on;
legend('Raw measured FRF','Block-averaged FRF');
title('Measured frequency response: raw vs block-averaged');

%% Plot FFT magnitudes
figure;
subplot(1,2,1)
loglog(fgrid_pos, abs(uf_pos));
grid on
xlabel('Frequency [Hz]');
ylabel('|U(f)|');
title('FFT of input voltage');

subplot(1,2,2)
loglog(fgrid_pos, abs(yf_pos));
grid on
xlabel('Frequency [Hz]');
ylabel('|Y(f)|');
title('FFT of measured angle');

%% Bode plot of measured frequency response
figure;
opts = bode_options('FreqUnits', 'Hz', 'XLim', [min(f_meas) max(f_meas)]);

bodeplot(Hm_frd_smooth,'r.',opts);
grid on
title('Measured frequency response: angle / voltage');

%% Optional: overlay model if available
He = spacar_sim_out.plant_voltage_to_sensor_em;

if exist('He','var') && ~isempty(He)
    hold on
    bodeplot(He,opts);
    legend('Measured FRF','Model');
else
    legend('Measured FRF');
end

%% Print summary
fprintf('\nDeliverable P identification complete.\n');
fprintf('Sample time ts          = %.6g s\n', ts);
fprintf('Sampling frequency      = %.3f Hz\n', fs);
fprintf('Nyquist frequency       = %.3f Hz\n', fNyq);
fprintf('Frequency resolution    = %.3f Hz\n', fResolution);
fprintf('Raw FRF points          = %d\n', length(f_meas));
fprintf('Smoothed FRF points     = %d\n', length(f_smooth));
fprintf('Averaging block size    = %d points\n', averageBlockSize);
fprintf('Saved file: Hm_frd_deliverable_P.mat\n');
fprintf('Saved file: Hm_frd_deliverable_P_smooth.mat\n');