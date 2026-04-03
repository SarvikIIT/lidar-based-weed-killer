function plot_lidar_results(params, pulse, t_pulse, beam, ...
    true_range, true_rho, P_received, V_out, snr_db, ...
    range_est, timing_info, points_3d, pipe_info, ...
    mc_range_errors, mc_rms, budget, gt_labels, valid, channel)
% PLOT_LIDAR_RESULTS  Generate comprehensive diagnostic plots.
%
%   12 subplots across 3 figures covering every subsystem output.

set(0, 'DefaultAxesFontSize', 10);
set(0, 'DefaultLineLineWidth', 1.5);

%% =======================================================================
%  FIGURE 1: Electro-Optical Signal Chain
%  =======================================================================
figure('Name', 'LiDAR — Electro-Optical Signal Chain', ...
       'Position', [50, 100, 1400, 900], 'Color', 'w');

% --- 1a. Laser Pulse Waveform ---
subplot(3, 3, 1);
plot(t_pulse * 1e9, pulse.power_profile, 'Color', [0.85 0.2 0.1], 'LineWidth', 2);
xlabel('Time [ns]'); ylabel('Power [W]');
title('Laser Pulse Waveform');
grid on;
text(0.05, 0.9, sprintf('E = %.2f nJ', pulse.energy*1e9), ...
    'Units', 'normalized', 'FontSize', 9);

% --- 1b. Beam Spot Size vs Range ---
subplot(3, 3, 2);
R_plot = linspace(0.1, 50, 500);
w_spot = beam.spot_size(R_plot) * 1e3;   % [mm]
plot(R_plot, w_spot, 'Color', [0.1 0.5 0.8], 'LineWidth', 2);
xlabel('Range [m]'); ylabel('Spot radius [mm]');
title('Beam Spot Size Propagation');
grid on;

% --- 1c. Received Power vs Range ---
subplot(3, 3, 3);
[R_sorted, idx] = sort(true_range);
semilogy(R_sorted, P_received(idx), '.', 'Color', [0.2 0.6 0.3], 'MarkerSize', 8);
xlabel('Range [m]'); ylabel('P_{received} [W]');
title('Received Power vs Range');
grid on;

% --- 1d. SNR vs Range ---
subplot(3, 3, 4);
plot(true_range, snr_db, '.', 'Color', [0.8 0.4 0.1], 'MarkerSize', 8);
hold on;
yline(10, '--r', 'SNR = 10 dB', 'LineWidth', 1);
xlabel('Range [m]'); ylabel('SNR [dB]');
title('Signal-to-Noise Ratio');
grid on;

% --- 1e. TIA Output Voltage ---
subplot(3, 3, 5);
plot(true_range, V_out * 1e3, '.', 'Color', [0.5 0.2 0.7], 'MarkerSize', 8);
xlabel('Range [m]'); ylabel('V_{out} [mV]');
title('TIA Output Voltage');
grid on;

% --- 1f. Range Error ---
subplot(3, 3, 6);
range_err = range_est - true_range;
plot(true_range(valid), range_err(valid) * 100, '.', ...
    'Color', [0.1 0.3 0.7], 'MarkerSize', 8);
hold on;
yline(0, '--k');
yline(params.gimbal.range_resolution * 100, '--r', 'Spec: 2 cm');
yline(-params.gimbal.range_resolution * 100, '--r');
xlabel('True Range [m]'); ylabel('Range Error [cm]');
title('Range Estimation Error');
grid on;

% --- 1g. Atmospheric Transmission ---
subplot(3, 3, 7);
eta_atm_plot = params.sim.atm_transmission(R_plot);
plot(R_plot, eta_atm_plot * 100, 'Color', [0.4 0.6 0.2], 'LineWidth', 2);
xlabel('Range [m]'); ylabel('\eta_{atm} [%]');
title('Atmospheric Transmission');
grid on; ylim([99 100.1]);

% --- 1h. Noise Breakdown (bar chart) ---
subplot(3, 3, 8);

% Use a representative target (median range)
[~, mid_idx] = min(abs(true_range - median(true_range)));
[~, ~, noise_single] = lidar_receiver(params, P_received(mid_idx));
noise_components = [noise_single.i2_shot_signal, ...
                    noise_single.i2_shot_dark, ...
                    noise_single.i2_tia, ...
                    noise_single.i2_thermal];
bar_colours = [0.85 0.2 0.1; 0.2 0.6 0.3; 0.1 0.4 0.8; 0.8 0.6 0.1];
b = bar(noise_components, 'FaceColor', 'flat');
b.CData = bar_colours;
set(gca, 'XTickLabel', {'Shot', 'Dark', 'TIA', 'Thermal'});
ylabel('Noise Power [A^2]');
title(sprintf('Noise Breakdown (R=%.0f m)', true_range(mid_idx)));
grid on;

% --- 1i. Normalised Reflectivity ---
subplot(3, 3, 9);
scatter(true_range, channel.I_normalised, 20, true_rho, 'filled');
colorbar; colormap(gca, parula);
xlabel('Range [m]'); ylabel('I_{normalised}');
title('Normalised Reflectivity');
grid on;

sgtitle('LiDAR System — Electro-Optical Signal Chain', 'FontSize', 14, 'FontWeight', 'bold');

%% =======================================================================
%  FIGURE 2: 3-D Point Cloud & Classification
%  =======================================================================
figure('Name', 'LiDAR — Point Cloud & Classification', ...
       'Position', [100, 50, 1400, 900], 'Color', 'w');

% --- 2a. 3-D Point Cloud (coloured by range) ---
subplot(2, 2, 1);
N_pts = size(points_3d, 1);
R_pts = sqrt(sum(points_3d.^2, 2));
scatter3(points_3d(:,1), points_3d(:,2), points_3d(:,3), ...
    10, R_pts, 'filled');
colorbar; colormap(gca, jet);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3-D Point Cloud (colour = range)');
grid on; axis equal; view(45, 30);

% --- 2b. Ground vs Object segmentation ---
subplot(2, 2, 2);
if ~isempty(pipe_info.pc_ground)
    scatter3(pipe_info.pc_ground(:,1), pipe_info.pc_ground(:,2), ...
        pipe_info.pc_ground(:,3), 10, [0.4 0.3 0.2], 'filled');
end
hold on;
if ~isempty(pipe_info.pc_objects)
    scatter3(pipe_info.pc_objects(:,1), pipe_info.pc_objects(:,2), ...
        pipe_info.pc_objects(:,3), 10, [0.1 0.7 0.3], 'filled');
end
legend('Ground', 'Objects', 'Location', 'best');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Ground Segmentation');
grid on; axis equal; view(45, 30);

% --- 2c. Classification Result ---
subplot(2, 2, 3);
if ~isempty(pipe_info.pc_objects) && ~isempty(pipe_info.labels)
    crop_mask = pipe_info.labels == 1;
    weed_mask = pipe_info.labels == 2;
    obj = pipe_info.pc_objects;
    
    if any(crop_mask)
        scatter3(obj(crop_mask,1), obj(crop_mask,2), obj(crop_mask,3), ...
            15, [0.1 0.7 0.3], 'filled');
    end
    hold on;
    if any(weed_mask)
        scatter3(obj(weed_mask,1), obj(weed_mask,2), obj(weed_mask,3), ...
            15, [0.9 0.2 0.1], 'filled');
    end
    legend('Crop', 'Weed', 'Location', 'best');
end
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Crop vs Weed Classification');
grid on; axis equal; view(45, 30);

% --- 2d. Feature distributions ---
subplot(2, 2, 4);
if ~isempty(pipe_info.features) && size(pipe_info.features, 1) > 1
    feature_names = {'Height', 'Density', 'Spat.Var', 'Refl.', 'Geo.Mom'};
    try
        boxplot(pipe_info.features, 'Labels', feature_names);
    catch
        % Fallback if Statistics Toolbox is not available
        bar(mean(pipe_info.features, 1, 'omitnan'), 'FaceColor', 'flat', ...
            'CData', lines(5));
        set(gca, 'XTickLabel', feature_names);
    end
    ylabel('Feature Value');
    title('Feature Distributions');
    grid on;
else
    text(0.3, 0.5, 'Insufficient object points for features', 'FontSize', 12);
    title('Feature Distributions');
end

sgtitle('LiDAR System — Point Cloud Processing & Classification', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% =======================================================================
%  FIGURE 3: Monte-Carlo & System Reports
%  =======================================================================
figure('Name', 'LiDAR — Monte-Carlo & System Diagnostics', ...
       'Position', [150, 50, 1400, 900], 'Color', 'w');

% --- 3a. Monte-Carlo range error histogram ---
subplot(2, 3, 1);
all_mc_err = mc_range_errors(:);
all_mc_err = all_mc_err(~isnan(all_mc_err));
if ~isempty(all_mc_err)
    histogram(all_mc_err * 100, 60, 'FaceColor', [0.3 0.5 0.8], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.8);
end
xlabel('Range Error [cm]'); ylabel('Count');
title(sprintf('MC Range Error (N=%d)', params.sim.N_mc));
grid on;
xline(0, '--r', 'LineWidth', 1.5);

% --- 3b. MC RMS error vs Range ---
subplot(2, 3, 2);
plot(true_range, mc_rms * 100, '.', 'Color', [0.7 0.2 0.5], 'MarkerSize', 10);
hold on;
yline(params.gimbal.range_resolution * 100, '--r', 'Spec: 2 cm');
xlabel('True Range [m]'); ylabel('RMS Error [cm]');
title('MC RMS Range Error vs Range');
grid on;

% --- 3c. MC error correlogram (range vs reflectivity) ---
subplot(2, 3, 3);
scatter(true_range, true_rho, 30, mc_rms * 100, 'filled');
colorbar;
xlabel('Range [m]'); ylabel('Reflectivity');
title('MC RMS Error (colour = cm)');
grid on;

% --- 3d. Power budget pie chart ---
subplot(2, 3, 4);
power_vals = [budget.P_laser, budget.P_apd, budget.P_fpga, ...
              budget.P_gimbal, budget.P_cooling, budget.P_loss];
power_labels = {'Laser', 'APD', 'FPGA', 'Gimbal', 'Cooling', 'Reg. Loss'};
pie(power_vals, power_labels);
title(sprintf('Power Budget (%.1f W total)', budget.P_supply_total));

% --- 3e. Timing breakdown ---
subplot(2, 3, 5);
% Timing at median range
t_components = [timing_info.t_laser, ...
                median(timing_info.t_flight), ...
                timing_info.t_detection, ...
                timing_info.t_processing] * 1e9;  % [ns]
bar_colours2 = [0.85 0.2 0.1; 0.2 0.6 0.8; 0.4 0.7 0.3; 0.8 0.5 0.2];
b2 = bar(t_components, 'FaceColor', 'flat');
b2.CData = bar_colours2;
set(gca, 'XTickLabel', {'Laser', 'Flight', 'Detect', 'Process'});
ylabel('Time [ns]');
title(sprintf('Timing Breakdown (t_{total}=%.1f ns)', sum(t_components)));
grid on;

% --- 3f. System specs summary (text) ---
subplot(2, 3, 6);
axis off;
specs = { ...
    sprintf('Wavelength:        %d nm', params.tx.wavelength * 1e9), ...
    sprintf('Peak Power:        %.0f W', params.tx.peak_power), ...
    sprintf('Rep Rate:          %.0f kHz', params.tx.rep_rate / 1e3), ...
    sprintf('Scanner:           %s', params.scan.method), ...
    sprintf('APD Gain:          %d', params.rx.apd.gain), ...
    sprintf('TIA Gain:          %.0f kΩ', params.rx.tia.gain / 1e3), ...
    sprintf('TDC Resolution:    %.0f ps', params.timing.tdc_resolution * 1e12), ...
    sprintf('Max Range:         %.0f m', params.gimbal.max_range), ...
    sprintf('Range Res:         %.0f cm', params.gimbal.range_resolution * 100), ...
    sprintf('MC RMS Error:      %.2f cm', mean(mc_rms, 'omitnan') * 100), ...
    sprintf('Total Power:       %.1f W', budget.P_supply_total) ...
};
text(0.05, 0.95, 'SYSTEM SPECIFICATIONS', 'FontSize', 12, ...
    'FontWeight', 'bold', 'VerticalAlignment', 'top');
for k = 1:numel(specs)
    text(0.05, 0.88 - 0.075*k, specs{k}, 'FontSize', 10, ...
        'FontName', 'Consolas', 'VerticalAlignment', 'top');
end

sgtitle('LiDAR System — Monte-Carlo Analysis & Diagnostics', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% =======================================================================
%  FIGURE 4: Pulse Waveform at Different Ranges
%  =======================================================================
figure('Name', 'LiDAR — Pulse Propagation', ...
       'Position', [200, 50, 800, 500], 'Color', 'w');

test_ranges = [5, 15, 25, 40, 50];
colours = lines(numel(test_ranges));

for k = 1:numel(test_ranges)
    R = test_ranges(k);
    tof_k = 2 * R / params.sim.c;
    
    % Scale pulse by 1/R^2 attenuation (simplified)
    atten = (test_ranges(1) / R)^2;
    pulse_atten = pulse.power_profile * atten;
    
    plot((t_pulse + tof_k) * 1e9, pulse_atten, ...
        'Color', colours(k,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('R = %d m', R));
    hold on;
end

xlabel('Time [ns]');
ylabel('Received Power [W] (relative)');
title('Return Pulse at Different Ranges');
legend('Location', 'best');
grid on;

fprintf('  ✓  All plots generated.\n');

end
