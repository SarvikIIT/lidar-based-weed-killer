function [tof, range_est, timing_info] = lidar_timing(params, true_range, ...
                                                      V_out, inject_errors)
% LIDAR_TIMING  Time-of-flight measurement via TDC simulation.
%
%   [tof, range_est, timing_info] = lidar_timing(params, true_range, V_out)
%   [tof, range_est, timing_info] = lidar_timing(params, true_range, V_out, true)
%
%   Models:
%       Total system timing:
%           t_total = t_laser + t_flight + t_detection + t_processing
%
%       Range from ToF:
%           R = c * tof / 2
%
%       TDC quantisation:
%           tof_measured = round(tof / tdc_res) * tdc_res
%
%   Inputs:
%       true_range     - ground-truth target ranges [m]
%       V_out          - TIA output voltage [V] (used for threshold crossing)
%       inject_errors  - logical (optional, default false)
%
%   Outputs:
%       tof        - measured time of flight [s]  (vector)
%       range_est  - estimated range [m]
%       timing_info - struct with timing breakdown

if nargin < 4, inject_errors = false; end

c      = params.sim.c;
timing = params.timing;

%% -----------------------------------------------------------------------
%  True Time-of-Flight
%  -----------------------------------------------------------------------
t_flight = 2 * true_range / c;        % round-trip [s]

%% -----------------------------------------------------------------------
%  System Timing Breakdown
%  -----------------------------------------------------------------------
%  t_total = t_laser + t_flight + t_detection + t_processing
t_laser      = params.tx.pulse_duration / 2;        % trigger-to-peak delay
t_detection  = params.rx.tia.rise_time;              % TIA rise time
t_processing = 1 / timing.fpga_clock;               % one FPGA clock cycle
t_total      = t_laser + t_flight + t_detection + t_processing;

%% -----------------------------------------------------------------------
%  Add Jitter & Errors
%  -----------------------------------------------------------------------
jitter = timing.jitter * randn(size(true_range));   % Gaussian jitter

if inject_errors
    % TDC linearity error
    tdc_lsb_error = params.errors.tdc_linearity * timing.tdc_resolution;
    linearity_err = tdc_lsb_error * (2*rand(size(true_range)) - 1);
else
    linearity_err = zeros(size(true_range));
end

%% -----------------------------------------------------------------------
%  TDC Quantisation
%  -----------------------------------------------------------------------
tof_analog  = t_flight + jitter + linearity_err;
tdc_res     = timing.tdc_resolution;

% Quantise to TDC bins
tof = round(tof_analog / tdc_res) * tdc_res;

% Enforce dead time: pulses arriving within tdc_dead_time are masked
dead_mask = [true, diff(tof) > timing.tdc_dead_time];
tof(~dead_mask) = NaN;

%% -----------------------------------------------------------------------
%  Signal Averaging
%  -----------------------------------------------------------------------
%  In a real system, N pulses are averaged. Here we model the jitter
%  reduction by dividing the jitter std by sqrt(N_avg).
N_avg = timing.signal_averaging;
tof_avg_jitter = timing.jitter / sqrt(N_avg);

%% -----------------------------------------------------------------------
%  Range Estimation
%  -----------------------------------------------------------------------
range_est = c * tof / 2;              % [m]

%% -----------------------------------------------------------------------
%  Clamp to Measurement Range
%  -----------------------------------------------------------------------
t_min = timing.meas_range(1);
t_max = timing.meas_range(2);
out_of_range = tof < t_min | tof > t_max;
range_est(out_of_range) = NaN;

%% -----------------------------------------------------------------------
%  Pack Outputs
%  -----------------------------------------------------------------------
timing_info.t_laser      = t_laser;
timing_info.t_flight     = t_flight;
timing_info.t_detection  = t_detection;
timing_info.t_processing = t_processing;
timing_info.t_total      = t_total;
timing_info.jitter_applied = jitter;
timing_info.linearity_err  = linearity_err;
timing_info.tof_analog   = tof_analog;
timing_info.dead_mask    = dead_mask;
timing_info.avg_jitter   = tof_avg_jitter;
timing_info.range_error  = range_est - true_range;

end
