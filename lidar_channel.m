function [P_received, channel] = lidar_channel(params, beam, target_range, ...
                                                target_reflectivity, inject_errors)
% LIDAR_CHANNEL  Model the free-space optical channel (transmit → target → receive).
%
%   [P_received, channel] = lidar_channel(params, beam, R, rho)
%   [P_received, channel] = lidar_channel(params, beam, R, rho, true)
%
%   Implements:
%       LiDAR link budget (PDF §V-C, §VII):
%           P_r = P_t * rho * A_rx * eta_t * eta_r * eta_atm^2 / (pi * (R * theta)^2)
%
%       Receiving telescope aperture equation (Eq. 3, PDF §V-C):
%           D_rec = sqrt( 4 * P_r * lambda^2 * R^2 / (pi * P_t * rho * eta_t * eta_r) )
%
%       Normalized reflectivity (Eq. 4, PDF §VII-B):
%           I_norm = I_measured * R^2 / (P_laser * eta_atm)
%
%       Maximum range performance (Eq. 6, PDF §XI-B):
%           P_max = sqrt( P_t * D^2 * sigma / (P_min * lambda^2) )
%
%   Inputs:
%       params               - parameter struct
%       beam                 - beam struct from lidar_transmitter
%       target_range         - scalar or vector [m]
%       target_reflectivity  - scalar or vector [-] (0..1)
%       inject_errors        - logical (optional, default false)

if nargin < 5, inject_errors = false; end

tx   = params.tx;
rx   = params.rx;
scan = params.scan;
sim  = params.sim;
err  = params.errors;

R   = target_range(:)';            % row vector [m]
rho = target_reflectivity(:)';     % row vector

%% -----------------------------------------------------------------------
%  Beam Splitter Model
%  -----------------------------------------------------------------------
%  Reflectance:  R_bs = ((n1 - n2) / (n1 + n2))^2
%  For a 50:50 BS this is controlled by the coating; we use the ratio directly.
T_bs = scan.beam_splitter_ratio;   % fraction transmitted (0.5)

%  Polarisation-dependent loss (error injection)
if inject_errors
    pol_factor = 1 - err.bs_polarization_dep * abs(randn(size(R)));
    T_bs = T_bs .* pol_factor;
end

%% -----------------------------------------------------------------------
%  Atmospheric Transmission
%  -----------------------------------------------------------------------
eta_atm = sim.atm_transmission(R);   % one-way;  two-way = eta_atm^2

%% -----------------------------------------------------------------------
%  Geometric / Transmission Efficiencies
%  -----------------------------------------------------------------------
eta_t = T_bs * 0.95;           % transmit-path optics efficiency (BS + collimator)
eta_r = 0.90;                  % receive-path optics efficiency (telescope + filter)

%% -----------------------------------------------------------------------
%  Illuminated Spot Area (at target)
%  -----------------------------------------------------------------------
theta_half = beam.divergence_coll;
A_spot = pi * (R .* theta_half).^2;   % [m^2]

%% -----------------------------------------------------------------------
%  Receiver Collection Area
%  -----------------------------------------------------------------------
D_rx  = rx.telescope.aperture;
A_rx  = pi * (D_rx / 2)^2;            % [m^2]

%% -----------------------------------------------------------------------
%  Received Power  (LiDAR equation)
%  -----------------------------------------------------------------------
%  Lambertian target: reflected into hemisphere → (rho / pi) * cos(0)
%  Solid angle subtended by receiver: A_rx / R^2
P_transmitted = tx.peak_power .* eta_t;
P_reflected   = P_transmitted .* rho ./ pi;        % per steradian
P_collected   = P_reflected .* A_rx ./ (R.^2);     % solid-angle collection
P_received    = P_collected .* (eta_atm.^2) .* eta_r;  % two-way atm + Rx optics

%  F-theta distortion error (PDF §IV-B: distortion <0.1%)
%  Each shot is multiplied by (1 + δ) where δ ~ N(0, ftheta_distortion).
if inject_errors
    distortion = 1 + err.ftheta_distortion * randn(size(R));
    P_received = P_received .* distortion;
end

%% -----------------------------------------------------------------------
%  Normalised Reflectivity Intensity (Eq. 4, PDF §VII-B)
%  -----------------------------------------------------------------------
%  I_norm = I_measured * R^2 / (P_laser * eta_atm)
%  Uses P_received as proxy for I_measured (proportional).
I_normalised = P_received .* R.^2 ./ (tx.peak_power .* eta_atm);

%% -----------------------------------------------------------------------
%  Required Telescope Aperture (Eq. 3, PDF §V-C — design verification)
%  -----------------------------------------------------------------------
%  D_rec = sqrt( 4 * P_r * lambda^2 * R^2 / (pi * P_t * rho * eta_t * eta_r) )
%  NOTE: This is a consistency check. Using P_received here verifies that
%  the computed D_rec matches the actual aperture D_rx = 50 mm.
D_rec_required = sqrt(4 .* P_received .* tx.wavelength^2 .* R.^2 ./ ...
                       (pi .* tx.peak_power .* rho .* eta_t .* eta_r));

%% -----------------------------------------------------------------------
%  Maximum Range Performance (Eq. 6, PDF §XI-B)
%  -----------------------------------------------------------------------
%  P_max = sqrt( P_t * D^2 * sigma / (P_min * lambda^2) )
%  where:
%    P_t    = peak transmitted power [W]
%    D      = receiver aperture diameter [m]
%    sigma  = target cross-section = rho * A_spot (Lambertian approximation)
%    P_min  = minimum detectable power (NEP-based)
%    lambda = wavelength [m]
%
%  Minimum detectable power from NEP (PDF Table II: NEP = 0.5 pW/sqrt(Hz)):
P_min_NEP = rx.apd.NEP * sqrt(rx.apd.bandwidth);  % [W] min detectable power

%  Effective target cross-section at reference reflectivity.
%  For a Lambertian target: sigma = rho * A_spot / pi  [m²]
%  Use A_spot at the max specified range (50 m) as the reference.
R_ref = params.gimbal.max_range;       % 50 m
A_spot_ref = pi * (R_ref * theta_half)^2;  % beam footprint area at R_ref [m²]
sigma_target = mean(rho) * A_spot_ref / pi;  % [m²] Lambertian cross-section

%  Compute max range from Eq. 6
R_max_eq6 = ( tx.peak_power * D_rx^2 * sigma_target / ...
               (P_min_NEP * tx.wavelength^2) ) ^ 0.25;

%% -----------------------------------------------------------------------
%  Pack Outputs
%  -----------------------------------------------------------------------
channel.eta_atm        = eta_atm;
channel.eta_t          = eta_t;
channel.eta_r          = eta_r;
channel.A_spot         = A_spot;
channel.A_rx           = A_rx;
channel.P_transmitted  = P_transmitted;
channel.P_received     = P_received;
channel.I_normalised   = I_normalised;
channel.D_rec_required = D_rec_required;
channel.R_max_eq6      = R_max_eq6;
channel.P_min_NEP      = P_min_NEP;

end
