function results = run_lidar_simulation()
% RUN_LIDAR_SIMULATION  End-to-end LiDAR system simulation.
%
%   results = run_lidar_simulation()
%
%   This is the top-level orchestrator that:
%       1. Loads all parameters
%       2. Generates a synthetic scene (targets at various ranges)
%       3. Runs the full electro-optical signal chain
%       4. Performs Monte-Carlo error analysis
%       5. Processes the point cloud (denoise → segment → classify)
%       6. Produces comprehensive diagnostic plots
%       7. Generates power budget and error reports
%
%   Outputs:
%       results - master struct containing every subsystem's output

fprintf('==========================================================\n');
fprintf('  END-TO-END LiDAR SYSTEM SIMULATION\n');
fprintf('  905 nm Weed-Detection LiDAR — Full Signal Chain\n');
fprintf('==========================================================\n\n');

%% =====================================================================
%  0.  LOAD PARAMETERS
%  =====================================================================
params = lidar_params();
rng(params.sim.rng_seed);

N_targets = params.sim.n_targets;

fprintf('[0]  Parameters loaded.  N_targets = %d\n', N_targets);

%% =====================================================================
%  1.  GENERATE SYNTHETIC SCENE
%  =====================================================================
fprintf('[1]  Generating synthetic scene ...\n');

% Random target ranges (uniform between 1 m and 50 m)
R_min = params.sim.target_range(1);
R_max = params.sim.target_range(2);
true_range = R_min + (R_max - R_min) * rand(1, N_targets);

% Target reflectivity (Beta distribution centred on 0.3, spread 0.1..0.8)
rho_mean = params.sim.target_reflectivity;
true_rho = max(0.05, min(0.95, rho_mean + 0.2 * randn(1, N_targets)));

% Assign ground-truth labels:  1 = crop   2 = weed
%   Weeds are shorter and have slightly different reflectivity
weed_fraction = 0.35;
is_weed = rand(1, N_targets) < weed_fraction;
gt_labels = ones(1, N_targets);
gt_labels(is_weed) = 2;

% Adjust reflectivity by class  (weeds slightly lower)
true_rho(is_weed) = true_rho(is_weed) * 0.75;

fprintf('      Crops: %d   Weeds: %d\n', sum(~is_weed), sum(is_weed));

%% =====================================================================
%  2.  TRANSMITTER — GENERATE LASER PULSE
%  =====================================================================
fprintf('[2]  Transmitter: generating laser pulse ...\n');

[pulse, t_pulse, beam] = lidar_transmitter(params, false);

fprintf('      Pulse energy  = %.2f nJ\n', pulse.energy * 1e9);
fprintf('      Beam waist    = %.2f mm\n', beam.w0 * 1e3);
fprintf('      Coll. diverg. = %.4f mrad\n', beam.divergence_coll * 1e3);

%% =====================================================================
%  3.  SCANNER — COMPUTE SCAN ANGLES
%  =====================================================================
fprintf('[3]  Scanner (%s): computing scan angles ...\n', params.scan.method);

t_sim = linspace(0, N_targets / params.tx.rep_rate, N_targets);
[scan_angles, scan_info] = lidar_scanner(params, t_sim);

fprintf('      Line rate = %.0f lines/s\n', scan_info.line_rate);

%% =====================================================================
%  4.  FREE-SPACE CHANNEL — LINK BUDGET
%  =====================================================================
fprintf('[4]  Channel: computing received power ...\n');

[P_received, channel] = lidar_channel(params, beam, true_range, true_rho, false);

fprintf('      P_rx range: [%.2e , %.2e] W\n', min(P_received), max(P_received));

%% =====================================================================
%  5.  RECEIVER — APD + TIA
%  =====================================================================
fprintf('[5]  Receiver: APD + TIA signal conversion ...\n');

[V_out, snr_db, noise_info] = lidar_receiver(params, P_received, false);

fprintf('      SNR range: [%.1f , %.1f] dB\n', min(snr_db), max(snr_db));

%% =====================================================================
%  6.  TIMING — TIME-OF-FLIGHT MEASUREMENT
%  =====================================================================
fprintf('[6]  Timing: TDC measurement ...\n');

[tof, range_est, timing_info] = lidar_timing(params, true_range, V_out, false);

valid = ~isnan(range_est);
range_err = range_est(valid) - true_range(valid);
fprintf('      Valid returns  = %d / %d\n', sum(valid), N_targets);
fprintf('      Range error (RMS) = %.4f m\n', rms(range_err));

%% =====================================================================
%  7.  GIMBAL — 3-D POINT CLOUD
%  =====================================================================
fprintf('[7]  Gimbal: generating 3-D point cloud ...\n');

% Use valid returns only
R_valid = range_est(valid);
[trajectory, points_3d] = lidar_gimbal(params, scan_angles, R_valid);

fprintf('      Point cloud size = %d × 3\n', size(points_3d, 1));

%% =====================================================================
%  8.  POINT-CLOUD PROCESSING & CLASSIFICATION
%  =====================================================================
fprintf('[8]  Point cloud: processing pipeline ...\n');

refl_valid = channel.I_normalised(valid)';
[pc_clean, features, pred_labels, pipe_info] = ...
    lidar_point_cloud(params, points_3d, refl_valid, false);

fprintf('      Filtered: %d   Ground: %d   Objects: %d\n', ...
    pipe_info.n_filtered, pipe_info.n_ground, pipe_info.n_objects);

%% =====================================================================
%  9.  MONTE-CARLO ERROR ANALYSIS
%  =====================================================================
fprintf('[9]  Monte-Carlo error analysis (N = %d) ...\n', params.sim.N_mc);

N_mc = params.sim.N_mc;
mc_range_errors = zeros(N_mc, numel(true_range));

for i = 1:N_mc
    % Re-run channel + receiver + timing with errors injected
    [P_rx_mc, ~]     = lidar_channel(params, beam, true_range, true_rho, true);
    [V_mc, ~, ~]     = lidar_receiver(params, P_rx_mc, true);
    [~, R_mc, ~]     = lidar_timing(params, true_range, V_mc, true);
    mc_range_errors(i, :) = R_mc - true_range;
end

mc_rms   = sqrt(mean(mc_range_errors.^2, 1, 'omitnan'));
mc_mean  = mean(mc_range_errors, 1, 'omitnan');

fprintf('      MC range RMS  : %.4f m (mean across targets)\n', mean(mc_rms, 'omitnan'));
fprintf('      MC range bias : %.4f m\n', mean(mc_mean, 'omitnan'));

%% =====================================================================
%  10. ERROR INJECTION REPORT
%  =====================================================================
fprintf('[10] Error injection report:\n');
err_report = lidar_error_injection(params);

%% =====================================================================
%  11. POWER BUDGET
%  =====================================================================
fprintf('[11] Power budget:\n');
budget = lidar_power_budget(params);

%% =====================================================================
%  12. VISUALIZATION
%  =====================================================================
fprintf('[12] Generating diagnostic plots ...\n\n');

plot_lidar_results(params, pulse, t_pulse, beam, ...
    true_range, true_rho, P_received, V_out, snr_db, ...
    range_est, timing_info, points_3d, pipe_info, ...
    mc_range_errors, mc_rms, budget, gt_labels, valid, channel);

%% =====================================================================
%  PACK RESULTS
%  =====================================================================
results.params       = params;
results.pulse        = pulse;
results.beam         = beam;
results.scan_angles  = scan_angles;
results.scan_info    = scan_info;
results.channel      = channel;
results.P_received   = P_received;
results.V_out        = V_out;
results.snr_db       = snr_db;
results.noise_info   = noise_info;
results.tof          = tof;
results.range_est    = range_est;
results.timing_info  = timing_info;
results.trajectory   = trajectory;
results.points_3d    = points_3d;
results.pc_clean     = pc_clean;
results.features     = features;
results.pred_labels  = pred_labels;
results.pipe_info    = pipe_info;
results.mc_range_errors = mc_range_errors;
results.mc_rms       = mc_rms;
results.err_report   = err_report;
results.budget       = budget;
results.gt_labels    = gt_labels;

fprintf('==========================================================\n');
fprintf('  SIMULATION COMPLETE\n');
fprintf('==========================================================\n');

end
