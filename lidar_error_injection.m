function err_report = lidar_error_injection(params)
% LIDAR_ERROR_INJECTION  Summarise and validate all error/tolerance parameters.
%
%   err_report = lidar_error_injection(params)
%
%   This function consolidates every error source defined in the architecture
%   and returns a structured report suitable for Monte-Carlo analysis.
%
%   Error Sources:
%       1. Beam Splitter Polarisation Dependence  :  15 %
%       2. F-theta Lens Distortion                :  10.1 %
%       3. Optical Filter Angle Tolerance         :  ±5°
%       4. Optical Filter Temperature Stability   :  0.01 nm/°C
%       5. TDC Linearity Error                    :  +0.5 LSB
%       6. Timing Jitter                          :  100 ps
%       7. Galvo Repeatability                    :  ±0.005°

err = params.errors;
timing = params.timing;
scan = params.scan;

%% -----------------------------------------------------------------------
%  Compile Error Table
%  -----------------------------------------------------------------------
names = { ...
    'BS Polarisation Dep.', ...
    'F-theta Distortion', ...
    'Filter Angle Tol.', ...
    'Filter Temp Stability', ...
    'TDC Linearity', ...
    'Timing Jitter', ...
    'Galvo Repeatability'};

values = [ ...
    err.bs_polarization_dep, ...
    err.ftheta_distortion, ...
    err.filter_angle_tol, ...
    err.filter_temp_stab / 1e-9, ...   % convert to nm/°C for display
    err.tdc_linearity, ...
    timing.jitter / 1e-12, ...         % convert to ps
    scan.galvo.repeatability];

units = { ...
    '%', '%', '° (±)', 'nm/°C', 'LSB', 'ps', '° (±)'};

scale_factors = [100, 100, 1, 1, 1, 1, 1];   % for display scaling
display_vals  = values .* scale_factors;

%% -----------------------------------------------------------------------
%  Print Summary
%  -----------------------------------------------------------------------
fprintf('\n========================================\n');
fprintf('  ERROR INJECTION SUMMARY\n');
fprintf('========================================\n');
for k = 1:numel(names)
    fprintf('  %-28s : %8.3f  %s\n', names{k}, display_vals(k), units{k});
end
fprintf('========================================\n\n');

%% -----------------------------------------------------------------------
%  Range Error Budget (RSS)
%  -----------------------------------------------------------------------
%  Convert each error source to an equivalent range error [m] at R = 25 m.
R_ref = 25;  % [m] reference range
c = params.sim.c;

% TDC linearity → range
delta_t_tdc = err.tdc_linearity * timing.tdc_resolution;  % [s]
delta_R_tdc = c * delta_t_tdc / 2;                          % [m]

% Timing jitter → range
delta_R_jitter = c * timing.jitter / 2;                    % [m]

% F-theta distortion → angular → range error (small angle)
delta_R_ftheta = R_ref * err.ftheta_distortion * ...
                 params.tx.beam_divergence;                 % [m]

% RSS total
delta_R_rss = sqrt(delta_R_tdc^2 + delta_R_jitter^2 + delta_R_ftheta^2);

fprintf('  Range Error Budget (RSS at R = %.0f m)\n', R_ref);
fprintf('    TDC Linearity   : %.4f m\n', delta_R_tdc);
fprintf('    Timing Jitter   : %.4f m\n', delta_R_jitter);
fprintf('    F-theta Dist.   : %.4f m\n', delta_R_ftheta);
fprintf('    --------------------------\n');
fprintf('    RSS Total       : %.4f m  (spec: %.2f m)\n', ...
    delta_R_rss, params.gimbal.range_resolution);
fprintf('\n');

%% -----------------------------------------------------------------------
%  Output
%  -----------------------------------------------------------------------
err_report.names         = names;
err_report.values        = values;
err_report.units         = units;
err_report.delta_R_tdc   = delta_R_tdc;
err_report.delta_R_jitter = delta_R_jitter;
err_report.delta_R_ftheta = delta_R_ftheta;
err_report.delta_R_rss    = delta_R_rss;
err_report.spec_resolution = params.gimbal.range_resolution;

end
