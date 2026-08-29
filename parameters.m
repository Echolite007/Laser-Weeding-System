function params = parameters()
    % Loader for system parameters

    params = struct();

    %% Specs

    spec = struct();
    spec.spot_accuracy_m       = 2e-3;
    spec.optical_path_length_m = 1.07;
    spec.angular_accuracy_rad  = spec.spot_accuracy_m / (2 * spec.optical_path_length_m);
    spec.mirror_angle_max_rad  = deg2rad(1);
    spec.spot_amplitude_m      = 37.6e-3;
    spec.driving_speed_nom_mps = 0.1;
    spec.driving_speed_max_mps = 0.42;
    spec.weeding_time_s        = 0.75;
    spec.return_time_s         = 0.03;
    spec.mirror_offset_min_m   = 50e-3;
    spec.mirror_offset_max_m   = 150e-3;
    spec.max_laser_cut_parts   = 6;
    params.spec = spec;

    %% Actuator — Akribis AVM30-15
    actuator = struct();
    actuator.name             = 'Akribis AVM30-15';
    actuator.stroke_m         = 15.0e-3;
    actuator.stroke_half_m    = 7.5e-3;
    actuator.Fc_N             = 4.63;
    actuator.Fpk_N            = 29.4;
    actuator.Kf_N_per_A       = 7.35;
    actuator.Ke_V_per_mps     = 7.35;
    actuator.Km_N_per_sqrtW   = 2.30;
    actuator.R25_ohm          = 10.24;
    actuator.Lcoil_H          = 2.82e-3;
    actuator.tau_e_s          = 0.28e-3;
    actuator.Ic_A             = 0.63;
    actuator.Ipk_A            = 4.0;
    actuator.Umax_datasheet_V = 60;
    actuator.Umax_V           = actuator.Ic_A * actuator.R25_ohm;
    actuator.m_coil_kg        = 36e-3;
    actuator.m_core_kg        = 95.6e-3;
    actuator.running_clearance_m = 0.6e-3;
    actuator.tau_e_from_LR_s  = actuator.Lcoil_H / actuator.R25_ohm;
    params.actuator = actuator;

    %% Amplifier — not yet specified
    amplifier = struct();
    amplifier.G_amp_A_per_V = NaN;
    amplifier.bandwidth_Hz  = NaN;
    amplifier.Umax_V        = NaN;
    amplifier.Imax_A        = NaN;
    params.amplifier = amplifier;

    %% Sensor — RLS LM10
    sensor = struct();
    sensor.name                 = 'RLS LM10 incremental magnetic encoder';
    sensor.resolution_min_um    = 0.244;
    sensor.resolution_max_um    = 250;
    sensor.pole_length_mm       = 2;
    sensor.hysteresis_um        = 4;
    sensor.ride_height_min_mm   = 0.1;
    sensor.ride_height_max_mm   = 1.5;
    sensor.resolution_um        = NaN;
    sensor.output_type          = '';
    sensor.mounting_radius_m    = NaN;
    params.sensor = sensor;

    %% Mechanism —
    mech = struct();
    mech.J_kgm2        = NaN;
    mech.k_Nm_per_rad  = NaN;
    mech.d_Nms_per_rad = 0;
    mech.r_arm_m       = NaN;
    params.mech = mech;

    %% Controller / Sampling
    ctrl = struct();
    ctrl.ts_s    = 1 / 2000;   % [s]   hardware sample time (2 kHz DSP board)
    ctrl.fs_hz   = 2000;     % [Hz]  sampling frequency
    ctrl.wc_rads = 600;      % [rad/s] target crossover: 100 Hz = Nyquist/10
    params.ctrl = ctrl;

    %% Flexure / Material — filled during deliverable f
    flex = struct();
    flex.E_Pa                = NaN;
    flex.sigma_y_Pa          = NaN;
    flex.thickness_options_m = [];
    params.flex = flex;

end
