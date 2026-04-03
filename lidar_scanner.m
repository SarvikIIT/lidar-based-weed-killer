function [angles, scan_info] = lidar_scanner(params, t_sim)
% LIDAR_SCANNER  Compute time-resolved scan angles for polygon or galvo.
%
%   [angles, scan_info] = lidar_scanner(params, t_sim)
%
%   Inputs:
%       params   - parameter struct (from lidar_params)
%       t_sim    - simulation time vector [s]
%
%   Outputs:
%       angles   - struct with .azimuth and .elevation [deg]
%       scan_info - metadata about scan pattern

scan = params.scan;
method = scan.method;

switch lower(method)
    
    case 'polygon'
        %% Rotating Polygon Mirror
        %  Each facet subtends 360 / N_facets degrees mechanically.
        %  The optical scan angle per facet = 2 × mechanical angle per facet.
        n_facets     = scan.polygon.facets;
        omega_mech   = scan.polygon.rpm * 2 * pi / 60;   % [rad/s]
        
        % Mechanical angle as a function of time (wrapping)
        mech_angle_rad = mod(omega_mech * t_sim, 2*pi);
        mech_angle_deg = rad2deg(mech_angle_rad);
        
        % Optical deflection = 2 × mechanical angle within each facet
        facet_angle    = 360 / n_facets;   % [deg] per facet
        in_facet_angle = mod(mech_angle_deg, facet_angle);
        
        % Map to centred optical scan: [-opt_angle/2, +opt_angle/2]
        opt_half = scan.polygon.opt_angle / 2;
        azimuth  = (in_facet_angle / facet_angle) * scan.polygon.opt_angle - opt_half;
        
        % Elevation is fixed per scan line; slow axis from gimbal
        elevation = zeros(size(t_sim));
        
        scan_info.type     = 'polygon';
        scan_info.facets   = n_facets;
        scan_info.line_rate = scan.polygon.rpm / 60 * n_facets; % [lines/s]
        
    case 'galvo'
        %% Galvanometer Scanner
        %  Sinusoidal scan at the specified frequency.
        f_scan = scan.galvo.freq;   % [Hz]
        half_range = scan.polygon.opt_angle / 2;  % reuse optical range
        
        azimuth   = half_range * sin(2 * pi * f_scan * t_sim);
        elevation = zeros(size(t_sim));
        
        % Add repeatability error
        azimuth   = azimuth + scan.galvo.repeatability * randn(size(t_sim));
        
        scan_info.type     = 'galvo';
        scan_info.freq     = f_scan;
        scan_info.line_rate = 2 * f_scan;       % bi-directional
        
    otherwise
        error('lidar_scanner:badMethod', ...
              'Unknown scan method "%s". Use "polygon" or "galvo".', method);
end

angles.azimuth   = azimuth;    % [deg]
angles.elevation = elevation;  % [deg]

end
