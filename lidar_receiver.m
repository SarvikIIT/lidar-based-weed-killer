function [V_out, snr_db, noise] = lidar_receiver(params, P_received, inject_errors)
% LIDAR_RECEIVER  Model the opto-electronic receiver chain (APD + TIA).
%
%   [V_out, snr_db, noise] = lidar_receiver(params, P_received)
%   [V_out, snr_db, noise] = lidar_receiver(params, P_received, true)
%
%   The receiver converts received optical power to a voltage signal and
%   computes the signal-to-noise ratio.
%
%   Noise contributions:
%       1. Shot noise (signal + background)
%       2. APD excess noise
%       3. Dark-current noise
%       4. TIA input-referred noise
%       5. Thermal (Johnson) noise
%
%   Inputs:
%       P_received   - received optical power [W] (scalar or vector)
%       inject_errors - logical (optional)
%
%   Outputs:
%       V_out    - output voltage from TIA [V]
%       snr_db   - signal-to-noise ratio [dB]
%       noise    - struct with individual noise components

if nargin < 3, inject_errors = false; end

rx  = params.rx;
sim = params.sim;
q   = sim.q;
kB  = sim.k_B;

M   = rx.apd.gain;
R0  = rx.apd.responsivity / M;        % primary responsivity (without gain)
BW  = rx.apd.bandwidth;
Id  = rx.apd.dark_current;
Z_T = rx.tia.gain;                     % transimpedance [Ω]
i_n = rx.tia.noise_density;            % TIA noise density [A/√Hz]

%% -----------------------------------------------------------------------
%  APD Excess-Noise Factor
%  -----------------------------------------------------------------------
%  McIntyre model: F(M) = k_eff * M + (1 - k_eff) * (2 - 1/M)
%  For InGaAs / Si APD, k_eff ≈ 0.02 – 0.05
k_eff = 0.03;                          % ionisation ratio
F_M   = k_eff * M + (1 - k_eff) * (2 - 1/M);

%% -----------------------------------------------------------------------
%  Signal Photocurrent
%  -----------------------------------------------------------------------
I_signal = R0 * M .* P_received;       % [A]

%% -----------------------------------------------------------------------
%  Noise Components (RMS current, single-sided BW)
%  -----------------------------------------------------------------------
%  1. Signal shot noise (amplified)
i2_shot_signal = 2 * q * R0 * P_received * M^2 * F_M * BW;

%  2. Dark-current shot noise (amplified)
i2_shot_dark   = 2 * q * Id * M^2 * F_M * BW;

%  3. TIA input-referred noise
i2_tia = i_n^2 * BW;

%  4. Thermal noise at TIA feedback resistor (T = 300 K)
T = 300;                               % [K]
i2_thermal = 4 * kB * T * BW / Z_T;

%  Total noise variance
i2_total = i2_shot_signal + i2_shot_dark + i2_tia + i2_thermal;
i_noise  = sqrt(i2_total);

%% -----------------------------------------------------------------------
%  SNR Calculation
%  -----------------------------------------------------------------------
snr_linear = I_signal.^2 ./ i2_total;
snr_db     = 10 * log10(snr_linear);

%% -----------------------------------------------------------------------
%  TIA Output Voltage
%  -----------------------------------------------------------------------
V_signal = I_signal * Z_T;             % [V]

%  Add realistic noise realisation
V_noise_realization = i_noise .* randn(size(P_received)) * Z_T;
V_out = V_signal + V_noise_realization;

%% -----------------------------------------------------------------------
%  Pack Noise Info
%  -----------------------------------------------------------------------
noise.i_signal         = I_signal;
noise.i2_shot_signal   = i2_shot_signal;
noise.i2_shot_dark     = i2_shot_dark;
noise.i2_tia           = i2_tia;
noise.i2_thermal       = i2_thermal;
noise.i2_total         = i2_total;
noise.i_noise_rms      = i_noise;
noise.F_excess         = F_M;
noise.V_signal         = V_signal;
noise.V_noise          = V_noise_realization;
noise.NEP_system       = i_noise ./ (R0 * M);  % [W/√Hz] system NEP

end
