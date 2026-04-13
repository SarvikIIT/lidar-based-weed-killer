function alignment = lidar_alignment(params)
% LIDAR_ALIGNMENT  Optical alignment and calibration procedure (PDF §X-A).
%
%   alignment = lidar_alignment(params)
%
%   Implements the 4-step alignment protocol from PDF §X-A:
%       1) Coarse mechanical alignment using alignment lasers
%       2) Fine adjustment with micrometers
%       3) Wavefront error measurement using interferometer
%       4) System response calibration using reference targets
%
%   Also validates optical calibration parameters against specifications.

%% -----------------------------------------------------------------------
%  STEP 1: Coarse Mechanical Alignment  (PDF §X-A, Step 1)
%  -----------------------------------------------------------------------
%  Use visible (HeNe 632.8 nm) alignment laser collinear with 905 nm beam.
%  Verify beam hits centre of each optical element within ±1 mm.

step1.name          = 'Coarse mechanical alignment using alignment lasers';
step1.laser_type    = 'HeNe 632.8 nm (visible)';
step1.tolerance     = 1e-3;    % [m] ±1 mm positioning accuracy
step1.status        = 'PASS';  % simulation assumes ideal alignment

fprintf('    STEP 1: %s\n', step1.name);
fprintf('            Tolerance: ±%.1f mm → %s\n', step1.tolerance*1e3, step1.status);

%% -----------------------------------------------------------------------
%  STEP 2: Fine Adjustment with Micrometers  (PDF §X-A, Step 2)
%  -----------------------------------------------------------------------
%  Adjust collimating lens position using kinematic mount (PDF §III-B).
%  Minimise beam divergence at far field.

step2.name          = 'Fine adjustment with micrometers';
step2.resolution    = 1e-6;    % [m] 1 µm micrometer resolution
step2.target_div    = params.tx.wavelength / (pi * params.tx.lens_focal * params.tx.lens_NA);
step2.achieved_div  = step2.target_div * (1 + 0.02 * randn);  % ±2% error
step2.div_error_pct = abs(step2.achieved_div - step2.target_div) / step2.target_div * 100;
step2.status        = 'PASS';
if step2.div_error_pct > 5
    step2.status = 'MARGINAL';
end

fprintf('    STEP 2: %s\n', step2.name);
fprintf('            Target divergence: %.3f mrad\n', step2.target_div * 1e3);
fprintf('            Achieved: %.3f mrad (%.1f%% error) → %s\n', ...
    step2.achieved_div * 1e3, step2.div_error_pct, step2.status);

%% -----------------------------------------------------------------------
%  STEP 3: Wavefront Error Measurement  (PDF §X-A, Step 3)
%  -----------------------------------------------------------------------
%  Use Shack-Hartmann wavefront sensor or Twyman-Green interferometer.
%  Specification: wavefront error ≤ 0.1λ (PDF §IV-B).

step3.name          = 'Wavefront error measurement using interferometer';
step3.spec_limit    = params.scan.ftheta_wavefront_error;  % 0.1 λ
step3.measured_wfe  = step3.spec_limit * (0.5 + 0.4 * rand);  % randomly 50–90% of limit
step3.margin_pct    = (1 - step3.measured_wfe / step3.spec_limit) * 100;
step3.status        = 'PASS';
if step3.measured_wfe > step3.spec_limit
    step3.status = 'FAIL';
end

fprintf('    STEP 3: %s\n', step3.name);
fprintf('            Spec: ≤%.2fλ  Measured: %.3fλ  Margin: %.0f%% → %s\n', ...
    step3.spec_limit, step3.measured_wfe, step3.margin_pct, step3.status);

%% -----------------------------------------------------------------------
%  STEP 4: System Response Calibration  (PDF §X-A, Step 4)
%  -----------------------------------------------------------------------
%  Calibrate using reference targets at known ranges with known reflectivity.
%  Verify range accuracy and signal amplitude against expected values.

step4.name          = 'System response calibration using reference targets';
step4.cal_ranges    = [5, 10, 25, 50];    % [m] calibration distances
step4.cal_rho       = 0.5;                 % calibration target reflectivity
step4.range_spec    = params.gimbal.range_resolution;  % 0.02 m

% Simulate calibration measurement errors
n_cal = numel(step4.cal_ranges);
step4.range_errors  = params.timing.tdc_resolution * params.sim.c / 2 * randn(1, n_cal);
step4.max_err       = max(abs(step4.range_errors));
step4.status        = 'PASS';
if step4.max_err > step4.range_spec
    step4.status = 'MARGINAL';
end

fprintf('    STEP 4: %s\n', step4.name);
fprintf('            Cal. ranges: %s m\n', mat2str(step4.cal_ranges));
fprintf('            Max range error: %.4f m (spec: %.2f m) → %s\n', ...
    step4.max_err, step4.range_spec, step4.status);

%% -----------------------------------------------------------------------
%  Summary
%  -----------------------------------------------------------------------
all_pass = strcmp(step1.status, 'PASS') && strcmp(step2.status, 'PASS') && ...
           strcmp(step3.status, 'PASS') && strcmp(step4.status, 'PASS');

fprintf('\n    ─── ALIGNMENT RESULT: ');
if all_pass
    fprintf('ALL STEPS PASSED ───\n\n');
else
    fprintf('REVIEW REQUIRED ───\n\n');
end

alignment.step1 = step1;
alignment.step2 = step2;
alignment.step3 = step3;
alignment.step4 = step4;
alignment.all_pass = all_pass;

end
