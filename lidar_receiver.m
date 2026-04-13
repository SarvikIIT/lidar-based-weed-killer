function [V_out, snr_db, noise] = lidar_receiver(params, P_received, inject_errors)
% LIDAR_RECEIVER  Model the opto-electronic receiver chain (APD + TIA).
%
%   [V_out, snr_db, noise] = lidar_receiver(params, P_received)
%   [V_out, snr_db, noise] = lidar_receiver(params, P_received, true)
%
%   The receiver converts received optical power to a voltage signal and
%   computes the signal-to-noise ratio.
%
%   Noise contributions (PDF §V-A, Fig. 2 circuit schematic):
%       1. Shot noise (signal + background) — APD intrinsic
%       2. APD excess noise — McIntyre F(M) model
%       3. Dark-current noise — APD leakage (I_d = 1 nA, PDF Table II)
%       4. TIA input-referred noise (i_n = 2 pA/√Hz, PDF §VI-A)
%       5. Thermal (Johnson) noise at TIA feedback resistor
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
BW  = rx.apd.bandwidth;               % APD bandwidth for shot noise
BW_tia = rx.tia.bandwidth;            % TIA bandwidth for TIA noise (200 MHz)
Id  = rx.apd.dark_current;
Z_T = rx.tia.gain;                     % transimpedance [Ω]
i_n = rx.tia.noise_density;            % TIA noise density [A/√Hz]

%% -----------------------------------------------------------------------
%  APD Excess-Noise Factor (McIntyre model)
%  -----------------------------------------------------------------------
%  F(M) = k_eff * M + (1 - k_eff) * (2 - 1/M)
%  For Si APD at 905 nm, k_eff ≈ 0.02–0.05 (ionisation ratio).
%  PDF Fig. 2 schematic: F(M) = k_eff*M + (1-k_eff)*(2-1/M)
k_eff = 0.03;                          % ionisation ratio (Si APD)
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

%  3. TIA input-referred noise (integrates over TIA BW = 200 MHz)
i2_tia = i_n^2 * BW_tia;

%  4. Thermal noise at TIA feedback resistor (T = 300 K)
%     Integrates over TIA bandwidth, not APD bandwidth.
T = 300;                               % [K]
i2_thermal = 4 * kB * T * BW_tia / Z_T;

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
noise.NEP_system       = i_noise ./ (R0 * M * sqrt(BW));  % [W/√Hz] system NEP

% Sanity check: system NEP should be close to component NEP (0.5 pW/√Hz)
assert(all(noise.NEP_system < 1e-9), ...
    'lidar_receiver: System NEP exceeds 1 nW/rtHz — check noise model.');

end
