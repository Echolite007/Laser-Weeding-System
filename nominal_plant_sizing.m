function nominal_size = nominal_plant_sizing(params, ref)

%% Determine following parameters based on motor limits: 
% Determine actuator arm length r_arm 
% Determine inertia J 
% Determine stiffness k 

R   = params.actuator.R25_ohm;
km  = params.actuator.Kf_N_per_A;     
Umax = params.actuator.Umax_V;        

margin = 0.9;   % Safety margin 

theta_max   = ref.theta_max;
dtheta_max  = ref.dtheta_max;
ddtheta_max = ref.ddtheta_max;

% Sizing divides by these quantities: reject NaN/zero inputs (e.g. the NaN
% placeholders in parameters.m) instead of returning Inf/NaN sizes silently.
checkPositiveFinite('params.actuator.R25_ohm', R);
checkPositiveFinite('params.actuator.Kf_N_per_A', km);
checkPositiveFinite('params.actuator.Umax_V', Umax);
checkPositiveFinite('ref.theta_max', theta_max);
checkPositiveFinite('ref.dtheta_max', dtheta_max);
checkPositiveFinite('ref.ddtheta_max', ddtheta_max);

%% 1) r_arm from the velocity-driven (back-EMF) voltage budget
r_arm_nominal = margin * Umax / (km * dtheta_max);
r_arm = 70e-3;
%% 2) J from acceleration-driven voltage budget 
J = margin * Umax * r_arm * km / (R * ddtheta_max);

%% 3) k_eq from position-driven voltage budget 
k_eq = margin * Umax * r_arm * km / (R * theta_max);

%% Real back-EMF damping and resulting damping ratio 
d    = (km^2 / R) * r_arm^2;
wn   = sqrt(k_eq / J);           
zeta = d / (2 * sqrt(J * k_eq));  

% Geometric stroke check (mirror angular range vs. VCM half-stroke):
stroke_check_m = r_arm * params.spec.mirror_angle_max_rad;
stroke_ok      = stroke_check_m <= params.actuator.stroke_half_m;
offset_ok = (r_arm >= params.spec.mirror_offset_min_m) && (r_arm <= params.spec.mirror_offset_max_m);

%% Voltage-budget verification 
u_v = km * r_arm * dtheta_max;
u_J = (R * J * ddtheta_max) / (r_arm * km);
u_k = (R * k_eq * theta_max) / (r_arm * km);

nominal_size = struct();
nominal_size.r_arm_m       = r_arm;
nominal_size.J_kgm2        = J;
nominal_size.k_Nm_per_rad  = k_eq;
nominal_size.d_Nms_per_rad = d;
nominal_size.zeta          = zeta;
nominal_size.wn_rad_s       = wn;
nominal_size.margin         = margin;
nominal_size.Umax_V         = Umax;
nominal_size.stroke_check_m = stroke_check_m;
nominal_size.stroke_half_m  = params.actuator.stroke_half_m;
nominal_size.stroke_ok      = stroke_ok;
nominal_size.offset_ok      = offset_ok;
nominal_size.mirror_offset_min_m = params.spec.mirror_offset_min_m;
nominal_size.mirror_offset_max_m = params.spec.mirror_offset_max_m;
nominal_size.u_req_check    = struct('u_v_V', u_v, 'u_J_V', u_J, 'u_k_V', u_k);

%% Surface constraint violations instead of only recording them in the output
if ~stroke_ok
    warning('nominalPlantSizing:strokeExceeded', ...
        ['Required actuator travel r_arm*theta_max = %.4f mm exceeds the VCM half-stroke ', ...
         'of %.4f mm.'], stroke_check_m*1e3, params.actuator.stroke_half_m*1e3);
end

if ~offset_ok
    warning('nominalPlantSizing:offsetOutOfRange', ...
        ['r_arm = %.2f mm is outside the allowed mirror-offset range [%.0f, %.0f] mm. ', ...
         'r_arm is sized from the back-EMF voltage budget only; this conflict between ', ...
         'the two constraints is unresolved.'], ...
        r_arm*1e3, params.spec.mirror_offset_min_m*1e3, params.spec.mirror_offset_max_m*1e3);
end

% r_arm is overridden with a fixed value, so the back-EMF budget it was
% derived from is no longer guaranteed: report it rather than ignore it.
voltageBudget_V = margin * Umax;
if r_arm > r_arm_nominal
    warning('nominalPlantSizing:backEmfBudgetExceeded', ...
        ['Fixed r_arm = %.2f mm exceeds the back-EMF-limited value %.2f mm; the ', ...
         'velocity voltage term is %.4f V against a budget of %.4f V.'], ...
        r_arm*1e3, r_arm_nominal*1e3, u_v, voltageBudget_V);
end

if any([u_v, u_J, u_k] > Umax)
    warning('nominalPlantSizing:voltageLimitExceeded', ...
        ['Required voltage terms [%.4f %.4f %.4f] V exceed the continuous actuator ', ...
         'limit Umax = %.4f V.'], u_v, u_J, u_k, Umax);
end

% fprintf('--- Deliverable b: nominal plant sizing ---\n');
% fprintf('Umax (continuous)   = %.4f V  (= Ic*R25 = %.3f A * %.2f Ohm)\n', Umax, params.actuator.Ic_A, R);
% fprintf('margin               = %.2f\n', margin);
% fprintf('r_arm                = %.6g m  (%.3f mm)\n', r_arm, r_arm*1e3);
% fprintf('J                    = %.6g kg*m^2\n', J);
% fprintf('k_eq                 = %.6g N*m/rad\n', k_eq);
% fprintf('d (back-EMF damping) = %.6g N*m*s/rad  (= km^2*r_arm^2/R, NOT zero/neglected)\n', d);
% fprintf('zeta                 = %.6g\n', zeta);
% fprintf('wn                   = %.6g rad/s  (%.3f Hz)\n', wn, wn/(2*pi));
% fprintf('Stroke check: r_arm*theta_max = %.4f mm  vs  stroke_half = %.4f mm  -> %s\n', ...
%     stroke_check_m*1e3, params.actuator.stroke_half_m*1e3, ternary(stroke_ok,'OK','VIOLATED'));
% fprintf('Mirror-offset check: r_arm = %.2f mm  vs  allowed [%.0f, %.0f] mm  -> %s\n', ...
%     r_arm*1e3, params.spec.mirror_offset_min_m*1e3, params.spec.mirror_offset_max_m*1e3, ternary(offset_ok,'OK','VIOLATED (KNOWN ISSUE)'));
% if ~offset_ok
%     fprintf(['  NOTE: r_arm is sized purely from the back-EMF voltage budget and does\n', ...
%              '  not respect the 50-150 mm mechanism offset range. This is a known open\n', ...
%              '  conflict between the two constraints -- flag and discuss in the report\n', ...
%              '  rather than silently resolving it here.\n']);
% end
% fprintf('Voltage budget check (each should equal margin*Umax = %.4f V):\n', margin*Umax);
% fprintf('  velocity/back-EMF term : %.4f V\n', u_v);
% fprintf('  J (acceleration) term  : %.4f V\n', u_J);
% fprintf('  k_eq (position) term   : %.4f V\n', u_k);
% 
% end

function s = ternary(cond, a, b)
if cond; s = a; else; s = b; end

function checkPositiveFinite(name, value)
if ~isscalar(value) || ~isfinite(value) || value <= 0
    error('nominalPlantSizing:invalidInput', ...
        '%s must be a finite positive scalar, got %s.', name, mat2str(value));
end

