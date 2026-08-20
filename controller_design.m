function controller = controller_design(p, delB, ref_nom)
% Determine required crossover frequency 
% Lead compensation 
% Tracking error constants 


%% --- Given design choices ----------------------------------------------
beta = 2;
g1 = 0.9; g2 = 0.9; g3 = 0.9;
PM_deg = delB.zeta*100;   
               % guidance. TODO: revisit/justify explicitly in report.

emax = p.spec.angular_accuracy_rad;   

%% --- Plant parameters from deliverable b (J, k_eq, d, zeta ALL together) 
J    = delB.J_kgm2;
k_eq = delB.k_Nm_per_rad;
d    = delB.d_Nms_per_rad;   
                             
zeta = delB.zeta;            
w1   = delB.wn_rad_s;        

%% --- alpha from phase margin (EQ 12.9) 
alpha = (1 - sind(45 - PM_deg)) / (1 + sind(45 - PM_deg));

% alpha outside (0,1) turns the intended lead filter into a lag filter, which
% would otherwise propagate as a plausible-looking but wrong controller.
if ~isfinite(alpha) || alpha <= 0 || alpha >= 1
    error('controllerDesign:invalidAlpha', ...
        ['alpha = %g is outside (0,1) for PM = %.2f deg (from delB.zeta = %g); ', ...
         'no lead filter exists for this phase margin.'], alpha, PM_deg, delB.zeta);
end

if ~isfinite(emax) || emax <= 0
    error('controllerDesign:invalidSpec', ...
        'p.spec.angular_accuracy_rad must be a finite positive value, got %g.', emax);
end

if ~all(isfinite([J, k_eq, d, zeta, w1]))
    error('controllerDesign:invalidPlant', ...
        ['Plant parameters from the sizing step contain non-finite values ', ...
         '(J=%g, k_eq=%g, d=%g, zeta=%g, w1=%g).'], J, k_eq, d, zeta, w1);
end

% Reference quantities 
if ~isfinite(ref_nom.a2p) || ref_nom.a2p == 0
    error('controllerDesign:invalidReference', ...
        'ref.a2p must be finite and non-zero to convert driving speed to angular speed, got %g.', ...
        ref_nom.a2p);
end

v_weeding = p.spec.driving_speed_nom_mps / ref_nom.a2p;

dtheta_max   = ref_nom.dtheta_max;    
ddtheta_max  = ref_nom.ddtheta_max;   
dddtheta_max = ref_nom.dddtheta_max;  

%  APPROACH A: weeding-phase velocity term only
wcA3 = w1^2 * beta * (1 - g3) * v_weeding / (alpha * emax);
wcA  = wcA3^(1/3);

kjA = beta / (alpha * wcA^3);
kaA = 2 * zeta * w1 * kjA;  
kvA = w1^2 * kjA;

%  APPROACH B: worst-case sum over full reference (return + weeding)
rhs_B = (1 - g1) * dddtheta_max + (1 - g2) * 2 * zeta * w1 * ddtheta_max + (1 - g3) * w1^2 * dtheta_max;
wcB3  = beta * rhs_B / (alpha * emax);
wcB   = wcB3^(1/3);

if ~isfinite(wcA) || wcA <= 0 || ~isfinite(wcB) || wcB <= 0
    error('controllerDesign:invalidCrossover', ...
        ['Required crossover frequencies are not usable (wcA = %g rad/s, ', ...
         'wcB = %g rad/s); check the reference derivatives and error spec.'], wcA, wcB);
end

kjB = beta / (alpha * wcB^3);
kaB = 2 * zeta * w1 * kjB;
kvB = w1^2 * kjB;

%  Expected tracking-error time plots, both approaches, full reference
t    = ref_nom.t;
dr   = ref_nom.dr;
ddr  = ref_nom.ddr;
dddr = ref_nom.dddr;

e_A = kjA*(1-g1)*dddr + kaA*(1-g2)*ddr + kvA*(1-g3)*dr;
e_B = kjB*(1-g1)*dddr + kaB*(1-g2)*ddr + kvB*(1-g3)*dr;

figure();
plot(t*1e3, e_A*1e3, 'LineWidth', 1.5); hold on;
plot(t*1e3, e_B*1e3, 'LineWidth', 1.5);
yline( emax*1e3, '--r', 'LineWidth', 1);
yline(-emax*1e3, '--r', 'LineWidth', 1);
xline(p.spec.return_time_s*1e3, ':', 'Color', [0.5 0.5 0.5]);
xlabel('Time [ms]');
ylabel('Tracking error e_{LF} [mrad]');
legend('Approach A (weeding-only \omega_c)', 'Approach B (worst-case \omega_c)', ...
       '\pm spec (0.931 mrad)', 'Location', 'best');
title('Deliverable (d): expected tracking error vs. spec');
grid on;

maxAbsErrA = max(abs(e_A));
maxAbsErrB = max(abs(e_B));
specOkA = maxAbsErrA <= emax;
specOkB = maxAbsErrB <= emax;

%% Report spec violations instead of only recording them in the output struct
if ~specOkA
    warning('controllerDesign:specViolatedApproachA', ...
        ['Approach A exceeds the tracking-error spec over the full reference: ', ...
         'max|e| = %.4f mrad vs %.4f mrad. Approach A only enforces the spec during ', ...
         'the weeding phase, so the fast return transient is not covered.'], ...
        maxAbsErrA*1e3, emax*1e3);
end

if ~specOkB
    warning('controllerDesign:specViolatedApproachB', ...
        ['Approach B exceeds the tracking-error spec over the full reference: ', ...
         'max|e| = %.4f mrad vs %.4f mrad.'], maxAbsErrB*1e3, emax*1e3);
end

% fprintf('--- Deliverable d: controller design (PM=%.0f deg) ---\n', PM_deg);
% fprintf('alpha = %.4f, beta = %.1f, zeta = %.6g (from delB, d=%.6g N*m*s/rad, NOT zero), w1 = %.4f rad/s\n', ...
%     alpha, beta, zeta, d, w1);
% fprintf('emax (spec)  = %.4f mrad\n\n', emax*1e3);
% 
% fprintf('Approach A (weeding-phase velocity term only, v_weeding=%.6f rad/s):\n', v_weeding);
% fprintf('  wc = %.4f rad/s (%.3f Hz)\n', wcA, wcA/(2*pi));
% fprintf('  kj=%.6g, ka=%.6g, kv=%.6g\n', kjA, kaA, kvA);
% fprintf('  max|e(t)| over FULL reference = %.4f mrad  -> %s\n\n', maxAbsErrA*1e3, ternary(specOkA,'OK','SPEC VIOLATED'));
% 
% fprintf('Approach B (worst-case sum, full reference incl. return):\n');
% fprintf('  wc = %.4f rad/s (%.3f Hz)\n', wcB, wcB/(2*pi));
% fprintf('  kj=%.6g, ka=%.6g, kv=%.6g\n', kjB, kaB, kvB);
% fprintf('  max|e(t)| over FULL reference = %.4f mrad  -> %s\n', maxAbsErrB*1e3, ternary(specOkB,'OK','SPEC VIOLATED'));
% 
% if ~specOkA
%     fprintf(['\nNOTE: Approach A only enforces the spec during the weeding phase by\n', ...
%              'construction; the fast return transient is NOT covered and can exceed\n', ...
%              'the +/-0.931 mrad bound, as shown in the plot and confirmed above.\n']);
% end

controller = struct();
controller.PM_deg = PM_deg;
controller.alpha = alpha;
controller.beta = beta;
controller.zeta = zeta;
controller.w1_rad_s = w1;
controller.emax_rad = emax;
controller.v_weeding_rad_s = v_weeding;

controller.A = struct('wc_rad_s', wcA, 'kj', kjA, 'ka', kaA, 'kv', kvA, ...
                'max_abs_err_rad', maxAbsErrA, 'spec_ok', specOkA, 'e_t', e_A);
controller.B = struct('wc_rad_s', wcB, 'kj', kjB, 'ka', kaB, 'kv', kvB, ...
                'max_abs_err_rad', maxAbsErrB, 'spec_ok', specOkB, 'e_t', e_B);
controller.t = t;
controller.recommended = 'B';   

end

function s = ternary(cond, a, b)
if cond; s = a; else; s = b; end
end
