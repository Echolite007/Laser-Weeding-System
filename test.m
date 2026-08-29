clc;

addpath('utils');

load('Hm_frd_deliverable_P_smooth.mat');

Ts = discretized_system.Cz_retuned.Ts;
Hm_frd_smooth.Ts = Ts;

P = Hm_frd_smooth;

% Optimized controller parameters
K      = 28.481;
alpha  = 0.3741;
fc_lead = 326.04;          % Hz
tau_z  = 0.000798091;      % s
tau_p  = 0.000298566;      % s

% Lead controller, discretized using Tustin
C_lead_z = discrete_lead_filter(tau_z, tau_p, Ts);

% Full controller
C_new = K * C_lead_z;

% Attach controller to plant
L_new = C_new * P;

% Plot margins
figure;
margin(L_new);
grid on;
title('Margin of Retuned Controller Attached to Smooth Plant');

% Compute margins
margins_new = loop_margins(L_new);

fprintf('\nMargins for new controller attached to plant:\n');
fprintf('PM = %.2f deg at %.2f Hz\n', margins_new.PM_deg, margins_new.fc_hz);
fprintf('GM = %.2f dB\n', margins_new.GM_dB);
fprintf('Gain crossover wc = %.2f rad/s\n', margins_new.wcp_rad_s);
fprintf('Phase crossover wg = %.2f rad/s\n', margins_new.wcg_rad_s);
