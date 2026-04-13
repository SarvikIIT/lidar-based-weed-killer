function [pulse, t_pulse, beam] = lidar_transmitter(params, inject_errors)
% LIDAR_TRANSMITTER  Generate a single laser pulse and compute beam parameters.
%
%   [pulse, t_pulse, beam] = lidar_transmitter(params)
%   [pulse, t_pulse, beam] = lidar_transmitter(params, true)
%
%   Outputs:
%       pulse    - struct with fields: power_profile [W], energy [J]
%       t_pulse  - time vector [s] for the pulse waveform
%       beam     - struct with collimated beam properties
%
%   Mathematical models (PDF §III-B):
%       Collimated divergence (Eq. 1):  theta_coll = lambda / (pi * w0)  [half-angle]
%       Full-angle divergence (PDF):    theta_full = 2 * lambda / (pi * w0)
%       Beam waist:                     w0 = f * NA  (thin-lens approximation)

if nargin < 2, inject_errors = false; end

tx  = params.tx;
sim = params.sim;

%% -----------------------------------------------------------------------
%  Pulse Waveform Generation
%  -----------------------------------------------------------------------
%  Model the pulse as a Gaussian envelope with the specified FWHM.
fwhm   = tx.pulse_duration;                    % FWHM [s]
sigma  = fwhm / (2 * sqrt(2 * log(2)));        % Gaussian sigma
n_pts  = 512;                                  % points in waveform

t_pulse = linspace(-3*fwhm, 3*fwhm, n_pts);   % time axis centred on 0

% Gaussian pulse shape
power_profile = tx.peak_power * exp(-t_pulse.^2 / (2 * sigma^2));

% Add timing jitter if errors are injected
if inject_errors
    jitter = params.timing.jitter * randn;     % random Gaussian jitter
    t_pulse = t_pulse + jitter;
end

pulse.power_profile = power_profile;
pulse.energy        = trapz(t_pulse, power_profile);   % [J] pulse energy
pulse.peak_power    = tx.peak_power;
pulse.fwhm          = fwhm;
pulse.sigma         = sigma;

%% -----------------------------------------------------------------------
%  Beam Collimation Model (PDF §III-B)
%  -----------------------------------------------------------------------
%  w0 ≈ f × NA  (simple thin-lens approximation for the laser diode)
w0 = tx.lens_focal * tx.lens_NA;               % beam waist [m]

%  Collimated divergence (Eq. 1, PDF §III-B):
%    theta_coll = lambda / (pi * w0)  — this is the HALF-ANGLE divergence
%    The PDF notation theta_collimated is the full-angle = 2 * theta_coll
%    Internally we use half-angle for Gaussian beam propagation.
theta_coll = tx.wavelength / (pi * w0);        % [rad] half-angle divergence

%  Rayleigh range
z_R = pi * w0^2 / tx.wavelength;               % [m]

beam.w0              = w0;
beam.divergence_half = theta_coll;              % half-angle divergence [rad]
beam.divergence_coll = theta_coll;              % alias (used in channel model)
beam.divergence_full = 2 * theta_coll;          % full-angle = PDF Eq. 1 notation
beam.divergence_raw  = tx.beam_divergence;      % raw diode divergence (4 mrad)
beam.rayleigh_range  = z_R;
beam.wavelength      = tx.wavelength;
beam.spectral_width  = tx.spectral_width;

%% -----------------------------------------------------------------------
%  Beam Diameter at Arbitrary Range
%  -----------------------------------------------------------------------
%  w(z) = w0 * sqrt(1 + (z / z_R)^2)
beam.spot_size = @(z) w0 * sqrt(1 + (z ./ z_R).^2);  % function handle

end
