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
%  --- Polygon Mirror ---
scan.polygon.facets          = 8;
scan.polygon.rpm             = 10000;          % [RPM]
scan.polygon.mech_angle      = 60;             % [deg]  mechanical scan angle
scan.polygon.opt_angle        = 120;            % [deg]  optical scan angle
scan.polygon.angular_vel     = scan.polygon.rpm * 360 / 60; % [deg/s]

%  --- Galvanometer Scanner ---
scan.galvo.freq              = 100;            % [Hz]
scan.galvo.ang_res           = 0.001;          % [deg]
scan.galvo.settling          = 1e-3;           % [s]
scan.galvo.repeatability     = 0.005;          % [deg]  ±

%  --- Transmission Optics ---
scan.beam_splitter_ratio     = 0.5;            % 50:50 at 905 nm
scan.ftheta_focal            = 100e-3;         % [m]   F-theta lens focal length
scan.ftheta_field            = [200e-3, 200e-3]; % [m]  scan field (X × Y)

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
%  5. GIMBAL KINEMATICS & SYSTEM BOUNDARIES
%  -----------------------------------------------------------------------
gimbal.pan_range            = [-180, 180];      % [deg]
gimbal.tilt_range           = [-30, 90];        % [deg]
gimbal.max_speed            = 60;               % [deg/s]
gimbal.max_range            = 50;               % [m]   max target range
gimbal.range_resolution     = 0.02;             % [m]   2 cm
params.gimbal = gimbal;

%% -----------------------------------------------------------------------
%  6. RANDOM FOREST CLASSIFIER (Point-Cloud ML)
%  -----------------------------------------------------------------------
ml.num_trees                = 100;
ml.max_depth                = 20;
ml.features_per_split       = 'sqrt';          % sqrt(n_features)
ml.training_samples         = 50000;
ml.target_accuracy          = 0.947;           % 94.7 %
ml.feature_names            = {'height_above_ground', ...
                               'point_density', ...
                               'spatial_variance', ...
                               'reflectivity', ...
                               'geometric_moments'};
params.ml = ml;

%% -----------------------------------------------------------------------
%  7. ERROR INJECTION & TOLERANCES
%  -----------------------------------------------------------------------
errors.bs_polarization_dep  = 0.15;            % 15 % dependence
errors.ftheta_distortion    = 0.101;           % 10.1 %
errors.filter_angle_tol     = 5;               % [deg] ±
errors.filter_temp_stab     = 0.01e-9;         % [m/°C]  0.01 nm/°C
errors.tdc_linearity        = 0.5;             % [LSB]
params.errors = errors;

%% -----------------------------------------------------------------------
%  8. POWER BUDGET
%  -----------------------------------------------------------------------
power.regulator_efficiency  = 0.92;            % 92 %
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
sim_ctrl.atm_extinction   = 0.1;              % [1/km] atmospheric extinction
sim_ctrl.atm_transmission = @(R) exp(-sim_ctrl.atm_extinction * R / 1e3);
sim_ctrl.dt               = timing.tdc_resolution; % simulation time step
sim_ctrl.N_mc             = 500;               % Monte-Carlo iterations
sim_ctrl.rng_seed         = 42;
params.sim = sim_ctrl;

end
