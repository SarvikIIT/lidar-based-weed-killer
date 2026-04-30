function params = lidar_params()
% LIDAR_PARAMS  Complete parameter configuration for end-to-end LiDAR simulation.
%
%   params = lidar_params() returns a struct containing all subsystem
%   parameters for the 905 nm weed-detection LiDAR system.
%
%   Subsystems:
%       params.tx       - Laser transmitter
%       params.scan     - Scanning optics (polygon & galvo)
%       params.rx       - Receiver (telescope, APD, TIA)
%       params.timing   - TDC & FPGA timing
%       params.gimbal   - Gimbal kinematics
%       params.ml       - Random Forest classifier
%       params.errors   - Error injection / tolerances
%       params.power    - Power budget
%       params.sim      - Simulation control knobs

%% -----------------------------------------------------------------------
%  1. LASER TRANSMITTER
%  -----------------------------------------------------------------------
tx.wavelength       = 905e-9;           % [m]   centre wavelength
tx.peak_power       = 75;               % [W]   peak optical power
tx.pulse_duration   = 5e-9;             % [s]   default pulse width (5 ns)
tx.pulse_dur_range  = [5e-9, 100e-9];   % [s]   allowable range
tx.rep_rate         = 50e3;             % [Hz]  pulse repetition rate
tx.spectral_width   = 3e-9;            % [m]   spectral linewidth
tx.beam_divergence  = 4e-3;            % [rad] full-angle divergence
tx.temp_range       = [-40, 85];        % [°C]  operating temperature
tx.lens_focal       = 8e-3;            % [m]   collimating lens focal length
tx.lens_NA          = 0.5;             % [-]   numerical aperture
params.tx = tx;

%% -----------------------------------------------------------------------
%  2. OPTICAL SCANNING & TRANSMISSION
%  -----------------------------------------------------------------------
%  --- Polygon Mirror (PDF §III-C) ---
scan.polygon.facets          = 8;
scan.polygon.rpm             = 10000;          % [RPM]
scan.polygon.mech_angle      = 60;             % [deg]  mechanical scan angle
scan.polygon.opt_angle       = 120;            % [deg]  optical scan angle (2x mech)
scan.polygon.angular_vel     = scan.polygon.rpm * 360 / 60; % [deg/s]
scan.polygon.surface_flatness = 632.8e-9 / 10; % [m]  λ/10 at 632.8 nm (PDF §III-C)

%  --- Galvanometer Scanner (PDF §III-C) ---
scan.galvo.freq              = 100;            % [Hz]
scan.galvo.ang_res           = 0.001;          % [deg]  angular resolution
scan.galvo.settling          = 1e-3;           % [s]   1 ms settling time
scan.galvo.repeatability     = 0.005;          % [deg]  ± repeatability
scan.galvo.scan_range        = 120;            % [deg]  total optical scan range

%  --- Transmission Optics (PDF §IV) ---
scan.beam_splitter_ratio     = 0.5;            % 50:50 at 905 nm (Eq. 2: R=((n1-n2)/(n1+n2))^2)
scan.ftheta_focal            = 100e-3;         % [m]   F-theta lens focal length
scan.ftheta_field            = [200e-3, 200e-3]; % [m]  scan field (X × Y)
scan.ftheta_distortion_max   = 0.001;          % <0.1% distortion (PDF §IV-B)
scan.ftheta_wavefront_error  = 0.1;            % ≤0.1λ wavefront error (PDF §IV-B)

%  --- Default scanning method ---
scan.method = 'polygon';   % 'polygon' | 'galvo'
params.scan = scan;

%% -----------------------------------------------------------------------
%  3. SIGNAL RECEPTION (Cassegrain + APD + TIA)
%  -----------------------------------------------------------------------
%  --- Collection Optics (Cassegrain Telescope) ---
rx.telescope.aperture       = 50e-3;           % [m]   aperture diameter
rx.telescope.focal          = 150e-3;          % [m]   effective focal length
rx.telescope.fov            = 2e-3;            % [rad] field of view
rx.telescope.filter_cw      = 905e-9;          % [m]   bandpass centre
rx.telescope.filter_fwhm    = 10e-9;           % [m]   filter FWHM
rx.telescope.filter_rejection = 6;             % [OD]  >OD6 filter rejection (PDF §V-B)

%  --- Avalanche Photodiode ---
rx.apd.active_diameter      = 200e-6;          % [m]
rx.apd.responsivity         = 40;              % [A/W] at 905 nm (with gain)
rx.apd.gain                 = 150;             % [-]   internal multiplication
rx.apd.bandwidth            = 100e6;           % [Hz]
rx.apd.NEP                  = 0.5e-12;         % [W/sqrt(Hz)]
rx.apd.dark_current         = 1e-9;            % [A]
rx.apd.voltage_range        = [150, 200];      % [V]

%  --- Transimpedance Amplifier ---
rx.tia.gain_range           = [10e3, 1e6];     % [Ω]  selectable
rx.tia.gain                 = 100e3;           % [Ω]  default (100 kΩ)
rx.tia.bandwidth            = 200e6;           % [Hz]
rx.tia.noise_density        = 2e-12;           % [A/sqrt(Hz)]
rx.tia.rise_time            = 2e-9;            % [s]
params.rx = rx;

%% -----------------------------------------------------------------------
%  4. TIMING & SIGNAL PROCESSING
%  -----------------------------------------------------------------------
timing.tdc_resolution       = 50e-12;          % [s]   50 ps
timing.tdc_sample_rate      = 500e6;           % [Sa/s]
timing.meas_range           = [10e-9, 100e-6]; % [s]   10 ns -> 100 µs
timing.tdc_dead_time        = 10e-9;           % [s]
timing.fpga_clock           = 450e6;           % [Hz]
timing.signal_averaging     = 16;              % [-]   number of averaged pulses
timing.jitter               = 100e-12;         % [s]   100 ps timing jitter
params.timing = timing;

%% -----------------------------------------------------------------------
%  5. GIMBAL KINEMATICS & SYSTEM BOUNDARIES (PDF §VIII-B, §XI-B)
%  -----------------------------------------------------------------------
gimbal.pan_range            = [-180, 180];      % [deg]  ±180° (PDF §VIII-B)
gimbal.tilt_range           = [-30, 90];        % [deg]  -30° to +90° (PDF §VIII-B)
gimbal.resolution           = 0.01;             % [deg]  0.01° mechanical resolution (PDF §VIII-B)
gimbal.max_speed            = 60;               % [deg/s] (PDF §VIII-B)
gimbal.max_range            = 50;               % [m]   max target range (PDF §XI-B)
gimbal.range_resolution     = 0.02;             % [m]   2 cm range resolution (PDF §XI-B)
gimbal.angular_resolution   = 0.1;              % [deg]  0.1° angular resolution (PDF §XI-B)
params.gimbal = gimbal;

%% -----------------------------------------------------------------------
%  6. RANDOM FOREST CLASSIFIER (Point-Cloud ML)
%  -----------------------------------------------------------------------
ml.num_trees                = 50;
ml.max_depth                = 6;
ml.features_per_split       = 'sqrt';          % sqrt(n_features)
ml.training_samples         = 50000;
ml.target_accuracy          = 0.925;           % 92.5 %
ml.feature_names            = {'height_above_ground', ...     % PDF §VII-B
                               'point_density', ...           % PDF §VII-B
                               'spatial_distribution_var', ...% PDF §VII-B
                               'reflectivity_intensity', ...  % PDF §VII-B, Eq. 4
                               'geometric_moments'};          % PDF §VII-B
params.ml = ml;

%% -----------------------------------------------------------------------
%  7. ERROR INJECTION & TOLERANCES
%  -----------------------------------------------------------------------
errors.bs_polarization_dep  = 0.05;            % <5 % polarisation dependence (PDF §IV-A)
errors.ftheta_distortion    = 0.001;           % <0.1 % distortion (PDF §IV-B)
errors.filter_angle_tol     = 5;               % [deg] ±5° (PDF §V-B)
errors.filter_temp_stab     = 0.01e-9;         % [m/°C]  0.01 nm/°C (PDF §V-B)
errors.filter_pol_dep       = 0.05;            % <5 % filter polarization dep. (PDF §V-B)
errors.tdc_linearity        = 0.5;             % [LSB]  ±0.5 LSB (PDF Table III)
params.errors = errors;

%% -----------------------------------------------------------------------
%  8. POWER BUDGET
%  -----------------------------------------------------------------------
power.regulator_efficiency  = 0.96;            % 96 %
power.laser  = struct('V', 5,   'I_max', 10);  % [V, A]
power.apd    = struct('V', 200, 'I_max', 50e-6);
power.fpga   = struct('V', [1.0, 2.5], 'I_max', 3);
power.gimbal = struct('V', 24,  'I_max', 2);
power.cooling= struct('V', 12,  'I_max', 2);
params.power = power;

%% -----------------------------------------------------------------------
%  9. SIMULATION CONTROL
%  -----------------------------------------------------------------------
sim_ctrl.c               = 299792458;          % [m/s] speed of light
sim_ctrl.h               = 6.62607015e-34;     % [J·s] Planck constant
sim_ctrl.k_B             = 1.380649e-23;       % [J/K] Boltzmann constant
sim_ctrl.q               = 1.602176634e-19;    % [C]   electron charge
sim_ctrl.n_targets        = 200;               % number of scene targets
sim_ctrl.target_range     = [1, 50];           % [m]   range bracket
sim_ctrl.target_reflectivity = 0.3;            % mean Lambertian reflectivity
sim_ctrl.atm_extinction   = 0.1;              % [1/km] atmospheric extinction coeff
sim_ctrl.atm_transmission = @(R) exp(-sim_ctrl.atm_extinction * R / 1e3);  % one-way; R in [m]
sim_ctrl.dt               = timing.tdc_resolution; % simulation time step
sim_ctrl.N_mc             = 500;               % Monte-Carlo iterations
sim_ctrl.rng_seed         = 42;
params.sim = sim_ctrl;

%% -----------------------------------------------------------------------
%  10. ENVIRONMENTAL ENCLOSURE  (PDF §VIII-A)
%  -----------------------------------------------------------------------
enclosure.ip_rating      = 'IP67';
enclosure.temp_control   = 'Active Peltier (TEC) cooling';
enclosure.vibration      = 'Damped mounting system';
enclosure.dust           = 'Positive pressure with HEPA filter';
params.enclosure = enclosure;

%% -----------------------------------------------------------------------
%  11. TABLE V PERFORMANCE TARGETS  (PDF §XI-A)
%  -----------------------------------------------------------------------
%  condition:  string label
%  precision, recall, f_score:  values from PDF Table V
perf_targets.conditions  = {'Clear Day', 'Overcast', 'Light Rain', 'Dawn/Dusk'};
perf_targets.precision   = [0.926, 0.915, 0.885, 0.902];
perf_targets.recall      = [0.922, 0.911, 0.879, 0.895];
perf_targets.f_score     = [0.924, 0.913, 0.882, 0.898];
params.perf_targets = perf_targets;

end
