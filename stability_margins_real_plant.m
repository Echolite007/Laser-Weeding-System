% clc; clearvars -except discretized_system;
% 
% load('Hm_frd_deliverable_P_smooth.mat');
% 
% Ts = discretized_system.Cz_retuned.Ts;
% Hm_frd_smooth.Ts = Ts;
% 
% P = Hm_frd_smooth;
% C_old = discretized_system.Cz_retuned;
% 
% % Targets
% fc_target = 100;              % Hz
% PM_min = 30;                  % deg
% PM_max = 45;                  % deg
% 
% % Search ranges
% K_range       = logspace(-4, 4, 45);
% alpha_range   = logspace(log10(0.02), log10(0.8), 35);
% fc_lead_range = logspace(log10(20), log10(400), 45);
% 
% best.J = inf;
% best.K = NaN;
% best.alpha = NaN;
% best.fc_lead = NaN;
% best.tau_z = NaN;
% best.tau_p = NaN;
% best.PM = NaN;
% best.fc = NaN;
% best.GM_dB = NaN;
% 
% fprintf('Starting grid search...\n\n');
% 
% for K = K_range
%     for alpha = alpha_range
%         for fc_lead = fc_lead_range
% 
%             wc_lead = 2*pi*fc_lead;
% 
%             tau_z = 1/(wc_lead*sqrt(alpha));
%             tau_p = alpha*tau_z;
% 
%             C_lead_c = tf([tau_z 1],[tau_p 1]);
%             C_lead_z = c2d(C_lead_c, Ts, 'tustin');
% 
%             C_try = K * C_lead_z * C_old;
%             L_try = C_try * P;
% 
%             [Gm, PM, ~, Wcp] = margin(L_try);
% 
%             if isempty(PM) || isempty(Wcp) || isnan(PM) || isnan(Wcp) || Wcp <= 0
%                 continue
%             end
% 
%             fc = Wcp/(2*pi);
%             GM_dB = 20*log10(Gm);
% 
%             % Feasible = stable margin result and PM within requested range
%             feasible = (PM >= PM_min) && (PM <= PM_max);
% 
%             if feasible
%                 fprintf(['WORKING: PM = %6.2f deg, fc = %7.2f Hz, ', ...
%                          'K = %.6g, alpha = %.6g, fc_lead = %.2f Hz, ', ...
%                          'tau_z = %.6g s, tau_p = %.6g s, GM = %.2f dB\n'], ...
%                          PM, fc, K, alpha, fc_lead, tau_z, tau_p, GM_dB);
%             end
% 
%             % Objective: hit 100 Hz and keep PM inside [30, 45] deg
%             fc_penalty = (log(fc/fc_target))^2;
%             PM_low_penalty = max(0, PM_min - PM)^2;
%             PM_high_penalty = max(0, PM - PM_max)^2;
% 
%             J = 100*PM_low_penalty + 100*PM_high_penalty + 50*fc_penalty;
% 
%             if feasible
%                 J = J - 10;
%             end
% 
%             if J < best.J
%                 best.J = J;
%                 best.K = K;
%                 best.alpha = alpha;
%                 best.fc_lead = fc_lead;
%                 best.tau_z = tau_z;
%                 best.tau_p = tau_p;
%                 best.PM = PM;
%                 best.fc = fc;
%                 best.GM_dB = GM_dB;
%             end
%         end
%     end
% end
% 
% fprintf('\nBest grid result:\n');
% fprintf('K        = %.6g\n', best.K);
% fprintf('alpha    = %.6g\n', best.alpha);
% fprintf('fc_lead  = %.2f Hz\n', best.fc_lead);
% fprintf('tau_z    = %.6g s\n', best.tau_z);
% fprintf('tau_p    = %.6g s\n', best.tau_p);
% fprintf('PM       = %.2f deg\n', best.PM);
% fprintf('fc       = %.2f Hz\n', best.fc);
% fprintf('GM       = %.2f dB\n', best.GM_dB);
% 
% % Local refinement using fminsearch
% x0 = [log10(best.K), log10(best.alpha), log10(best.fc_lead)];
% 
% objective = @(x) local_objective(x, C_old, P, Ts, fc_target, PM_min, PM_max);
% 
% opts = optimset('Display','iter','MaxIter',200,'MaxFunEvals',500);
% xopt = fminsearch(objective, x0, opts);
% 
% K_opt = 10^xopt(1);
% alpha_opt = 10^xopt(2);
% fc_lead_opt = 10^xopt(3);
% 
% % Clamp to allowed ranges
% K_opt = min(max(K_opt, 1e-4), 1e4);
% alpha_opt = min(max(alpha_opt, 0.02), 0.8);
% fc_lead_opt = min(max(fc_lead_opt, 20), 400);
% 
% wc_lead = 2*pi*fc_lead_opt;
% 
% tau_z = 1/(wc_lead*sqrt(alpha_opt));
% tau_p = alpha_opt*tau_z;
% 
% C_lead_c = tf([tau_z 1],[tau_p 1]);
% C_lead_z = c2d(C_lead_c, Ts, 'tustin');
% 
% C_retuned_100Hz = K_opt * C_lead_z * C_old;
% L_smooth = C_retuned_100Hz * P;
% 
% figure;
% margin(L_smooth);
% grid on;
% title('Final Optimized Open-Loop Margin');
% 
% [Gm, Pm, Wcg, Wcp] = margin(L_smooth);
% 
% fprintf('\nFinal optimized controller:\n');
% fprintf('K        = %.6g\n', K_opt);
% fprintf('alpha    = %.6g\n', alpha_opt);
% fprintf('fc_lead  = %.2f Hz\n', fc_lead_opt);
% fprintf('tau_z    = %.6g s\n', tau_z);
% fprintf('tau_p    = %.6g s\n', tau_p);
% fprintf('PM       = %.2f deg at %.2f Hz\n', Pm, Wcp/(2*pi));
% fprintf('GM       = %.2f dB\n', 20*log10(Gm));
% fprintf('Wcg      = %.2f rad/s\n', Wcg);
% fprintf('Wcp      = %.2f rad/s\n', Wcp);
% 
% discretized_system.Cz_retuned_100Hz = C_retuned_100Hz;
% 
% function J = local_objective(x, C_old, P, Ts, fc_target, PM_min, PM_max)
% 
%     K = 10^x(1);
%     alpha = 10^x(2);
%     fc_lead = 10^x(3);
% 
%     if K < 1e-4 || K > 1e4 || alpha < 0.02 || alpha > 0.8 || fc_lead < 20 || fc_lead > 400
%         J = 1e6;
%         return
%     end
% 
%     wc_lead = 2*pi*fc_lead;
% 
%     tau_z = 1/(wc_lead*sqrt(alpha));
%     tau_p = alpha*tau_z;
% 
%     C_lead_c = tf([tau_z 1],[tau_p 1]);
%     C_lead_z = c2d(C_lead_c, Ts, 'tustin');
% 
%     C_try = K * C_lead_z * C_old;
%     L_try = C_try * P;
% 
%     [~, PM, ~, Wcp] = margin(L_try);
% 
%     if isempty(PM) || isempty(Wcp) || isnan(PM) || isnan(Wcp) || Wcp <= 0
%         J = 1e6;
%         return
%     end
% 
%     fc = Wcp/(2*pi);
% 
%     fc_penalty = (log(fc/fc_target))^2;
%     PM_low_penalty = max(0, PM_min - PM)^2;
%     PM_high_penalty = max(0, PM - PM_max)^2;
% 
%     J = 100*PM_low_penalty + 100*PM_high_penalty + 50*fc_penalty;
% 
%     if PM >= PM_min && PM <= PM_max
%         J = J - 10;
%     end
% end

clc; clearvars -except discretized_system;

addpath('utils');

load('Hm_frd_deliverable_P_smooth.mat');

Ts = discretized_system.Cz_retuned.Ts;
Hm_frd_smooth.Ts = Ts;

P = Hm_frd_smooth;
C_old = discretized_system.Cz_retuned;

L_old = P * C_old;

margins_old = loop_margins(L_old);

% Targets
fc_target = 100;      % Hz
PM_min = 30;          % deg
PM_max = 45;          % deg

% Faster search ranges
K_range       = logspace(-2, 3, 25);
alpha_range   = logspace(log10(0.05), log10(0.8), 18);
fc_lead_range = logspace(log10(50), log10(400), 22);

best.J = inf;
best.K = NaN;
best.alpha = NaN;
best.fc_lead = NaN;
best.tau_z = NaN;
best.tau_p = NaN;
best.PM = NaN;
best.fc = NaN;
best.GM_dB = NaN;

fprintf('Starting fast grid search...\n\n');

for K = K_range
    for alpha = alpha_range
        for fc_lead = fc_lead_range

            [tau_z, tau_p] = lead_time_constants(alpha, fc_lead);

            C_lead_z = discrete_lead_filter(tau_z, tau_p, Ts);

            C_try = K * C_lead_z * C_old;
            L_try = C_try * P;

            margins_try = loop_margins(L_try);

            if ~has_valid_crossover(margins_try)
                continue
            end

            PM = margins_try.PM_deg;
            fc = margins_try.fc_hz;
            GM_dB = margins_try.GM_dB;

            [J, feasible] = controller_search_cost(PM, fc, fc_target, PM_min, PM_max);

            if feasible
                fprintf(['WORKING: PM = %6.2f deg, fc = %7.2f Hz, ', ...
                         'K = %.6g, alpha = %.6g, fc_lead = %.2f Hz, ', ...
                         'tau_z = %.6g s, tau_p = %.6g s, GM = %.2f dB\n'], ...
                         PM, fc, K, alpha, fc_lead, tau_z, tau_p, GM_dB);
            end

            if J < best.J
                best.J = J;
                best.K = K;
                best.alpha = alpha;
                best.fc_lead = fc_lead;
                best.tau_z = tau_z;
                best.tau_p = tau_p;
                best.PM = PM;
                best.fc = fc;
                best.GM_dB = GM_dB;
            end

            % Early stop if close enough
            if feasible && abs(fc - fc_target) < 2
                fprintf('\nGood enough solution found. Stopping early.\n');
                break
            end
        end

        if isfinite(best.fc) && best.PM >= PM_min && best.PM <= PM_max && abs(best.fc - fc_target) < 2
            break
        end
    end

    if isfinite(best.fc) && best.PM >= PM_min && best.PM <= PM_max && abs(best.fc - fc_target) < 2
        break
    end
end

% Build final controller
K_opt = best.K;
alpha_opt = best.alpha;
fc_lead_opt = best.fc_lead;

[tau_z, tau_p] = lead_time_constants(alpha_opt, fc_lead_opt);

C_lead_z = discrete_lead_filter(tau_z, tau_p, Ts);

C_retuned_100Hz = K_opt * C_lead_z * C_old;
L_smooth = C_retuned_100Hz * P;

figure;
margin(L_smooth);
grid on;
title('Final Fast-Optimized Open-Loop Margin');

margins_smooth = loop_margins(L_smooth);

fprintf('\nFinal optimized controller:\n');
fprintf('K        = %.6g\n', K_opt);
fprintf('alpha    = %.6g\n', alpha_opt);
fprintf('fc_lead  = %.2f Hz\n', fc_lead_opt);
fprintf('tau_z    = %.6g s\n', tau_z);
fprintf('tau_p    = %.6g s\n', tau_p);
fprintf('PM       = %.2f deg at %.2f Hz\n', margins_smooth.PM_deg, margins_smooth.fc_hz);
fprintf('GM       = %.2f dB\n', margins_smooth.GM_dB);
fprintf('Wcg      = %.2f rad/s\n', margins_smooth.wcg_rad_s);
fprintf('Wcp      = %.2f rad/s\n', margins_smooth.wcp_rad_s);

discretized_system.Cz_retuned_100Hz = C_retuned_100Hz;