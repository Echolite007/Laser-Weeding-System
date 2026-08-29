%% Logarithmic chirp
%% Averaging over 5 points and reduce to one

%% Deliverable P - Initialization Script
clc;
close all;

Umax = 6.4512;          % [V]
GainU = 0.5;           % [-] recommended first test: 0.05 to 0.10

K_sign = 1;             % [-]

%% Type
% inputType = 0 -> multisine
% inputType = 1 -> chirp
inputType = 1;

%% Simulation properties
ts = 5e-4;             % [s]

% Period time for multisine.
% Frequency resolution of FFT is 1/T.
% T = 10 s gives 0.1 Hz resolution.
T = 50;                 % [s]

% Number of periods for multisine.
% The identification script averages the later periods.
Nr = 10;                % [-]

% Total experiment duration.
Ttot = Nr * T;            % [s]

multisineFrequencies_Hz = [0.5 1 2 3 5 7 9 12 15 17 20 25 30 40 50 70 90 120 160 200 250];

multisineFrequencies = 2 * pi * multisineFrequencies_Hz;   % [rad/s]

%% Chirp properties

% Chirp frequency range in Hz.
f_start = 0.5;          % [Hz]
f_end   = 150;          % [Hz]

%% Nominal parameters
J = 0.3502;              % [kg m^2]
b = 8.08;              % [Nms/rad]
k = 136.35;                % [Nm/rad]

% Simple nominal voltage-to-angle model placeholder.
% Only valid if the input gain from voltage to torque is included separately.
% In most cases, replace this by your SPACAR + actuator model.
s = tf('s');
He = [];

%% Derived check

fs = 1 / ts;              % [Hz]
fNyq = fs / 2;            % [Hz]
fResolution = 1 / T;      % [Hz]

fprintf('\nDeliverable P initialization complete.\n');
fprintf('Sample time ts          = %.6g s\n', ts);
fprintf('Sampling frequency fs   = %.3f Hz\n', fs);
fprintf('Nyquist frequency       = %.3f Hz\n', fNyq);
fprintf('Period time T           = %.3f s\n', T);
fprintf('Frequency resolution    = %.3f Hz\n', fResolution);
fprintf('Number of periods Nr    = %d\n', Nr);
fprintf('Total experiment Ttot   = %.3f s\n', Ttot);
fprintf('GainU                   = %.4f\n', GainU);
fprintf('Max excitation estimate = %.4f V before protection\n', GainU);
fprintf('VCM voltage limit Umax  = %.4f V\n', Umax);

if inputType == 0
    fprintf('Excitation type         = multisine\n');
    fprintf('Maximum multisine freq  = %.3f Hz\n', max(multisineFrequencies_Hz));

    if max(multisineFrequencies_Hz) >= fNyq
        warning('Maximum multisine frequency is at or above Nyquist. Reduce frequencies or decrease ts.');
    end
else
    fprintf('Excitation type         = chirp\n');
    fprintf('Chirp start frequency   = %.3f Hz\n', f_start);
    fprintf('Chirp end frequency     = %.3f Hz\n', f_end);

    if f_end >= fNyq
        warning('Chirp end frequency is at or above Nyquist. Reduce f_end or decrease ts.');
    end
end

if GainU > 0.2
    warning('GainU is relatively high for a first hardware test. Start around 0.05 to 0.10.');
end
