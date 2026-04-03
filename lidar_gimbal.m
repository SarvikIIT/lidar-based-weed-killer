function [trajectory, points_3d] = lidar_gimbal(params, scan_angles, ranges)
% LIDAR_GIMBAL  Gimbal kinematics and 3-D point-cloud generation.
%
%   [trajectory, points_3d] = lidar_gimbal(params, scan_angles, ranges)
%
%   Converts pan/tilt gimbal angles combined with fast-axis scanner
%   deflections and measured ranges into a 3-D point cloud in a
%   world-fixed coordinate frame.
%
%   Inputs:
%       scan_angles  - struct with .azimuth, .elevation [deg] from scanner
%       ranges       - measured ranges [m] (from lidar_timing)
%
%   Outputs:
%       trajectory   - struct with gimbal pan/tilt vs. time
%       points_3d    - [N x 3] matrix of (X, Y, Z) coordinates [m]

gimbal = params.gimbal;
N = numel(ranges);

%% -----------------------------------------------------------------------
%  Gimbal Trajectory (slow axes)
%  -----------------------------------------------------------------------
%  Simple raster: pan sweeps linearly, tilt steps per scan line.
%  For demonstration, we create a smooth S-curve trajectory.

pan_range  = gimbal.pan_range;     % [deg]
tilt_range = gimbal.tilt_range;    % [deg]
max_speed  = gimbal.max_speed;     % [deg/s]

% Generate a smooth pan trajectory (triangular wave)
total_pan_sweep  = diff(pan_range);             % total pan excursion [deg]
t_pan_half       = total_pan_sweep / max_speed;  % time for one sweep [s]

%  Normalised time for each point (0..1 over the scan)
t_norm = linspace(0, 1, N);

% Pan: triangular (bidirectional)
pan = pan_range(1) + total_pan_sweep * abs(2*t_norm - 1);

% Tilt: slow linear ramp bottom-to-top
tilt = linspace(tilt_range(1), tilt_range(2), N);

trajectory.pan  = pan;       % [deg]
trajectory.tilt = tilt;      % [deg]

%% -----------------------------------------------------------------------
%  Combine Scanner + Gimbal Angles → 3-D Coordinates
%  -----------------------------------------------------------------------
%  Total pointing:
%     Az_total  = gimbal_pan  + scanner_azimuth
%     El_total  = gimbal_tilt + scanner_elevation
az_total = pan  + scan_angles.azimuth(1:N);    % [deg]
el_total = tilt + scan_angles.elevation(1:N);  % [deg]

az_rad = deg2rad(az_total);
el_rad = deg2rad(el_total);

R = ranges(1:N);

%  Spherical → Cartesian
X = R .* cos(el_rad) .* cos(az_rad);
Y = R .* cos(el_rad) .* sin(az_rad);
Z = R .* sin(el_rad);

points_3d = [X(:), Y(:), Z(:)];

%% -----------------------------------------------------------------------
%  Enforce Gimbal Limits
%  -----------------------------------------------------------------------
%  Flag points where gimbal was at a kinematic limit
at_pan_limit  = (pan <= pan_range(1))  | (pan >= pan_range(2));
at_tilt_limit = (tilt <= tilt_range(1)) | (tilt >= tilt_range(2));

trajectory.at_pan_limit  = at_pan_limit;
trajectory.at_tilt_limit = at_tilt_limit;
trajectory.az_total      = az_total;
trajectory.el_total      = el_total;

end
