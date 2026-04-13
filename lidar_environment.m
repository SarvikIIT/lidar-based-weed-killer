function env = lidar_environment(params)
% LIDAR_ENVIRONMENT  Environmental enclosure specifications (PDF §VIII-A).
%
%   env = lidar_environment(params)
%
%   Documents and validates the environmental protection subsystem:
%       - IP67 rating (dustproof + 1m water immersion)
%       - Active Peltier (TEC) cooling for temperature control
%       - Damped mounting system for vibration isolation
%       - Positive pressure with HEPA filter for dust protection
%
%   These specifications are critical for field deployment in agricultural
%   environments where dust, moisture, and temperature extremes are common.

%% -----------------------------------------------------------------------
%  Environmental Enclosure Specifications (PDF §VIII-A)
%  -----------------------------------------------------------------------
env.ip_rating            = 'IP67';
env.temperature_control  = 'Active Peltier (TEC) cooling';
env.vibration_isolation  = 'Damped mounting system';
env.dust_protection      = 'Positive pressure with HEPA filter';

% Operating limits (from laser diode spec, PDF Table I)
env.temp_range           = params.tx.temp_range;   % [-40, 85] °C
env.target_temp          = 25;                     % [°C] nominal operating point

% Peltier cooler sizing estimate
%   Worst case: summer outdoor → 50°C ambient, laser generating 50 W heat
%   Required ΔT = 50 - 25 = 25°C
%   Typical TEC: 60W cooling capacity at ΔT=25°C
env.tec_cooling_capacity = 60;  % [W]
env.tec_delta_T_max      = 40;  % [°C] max temperature differential

% Vibration specs for agricultural environment
%   Typical agricultural vehicle: 0.5–5 g RMS broadband
%   Damped mounting targets < 0.1 g at sensor
env.vibration_input      = 5;     % [g] max vehicle vibration
env.vibration_damped     = 0.1;   % [g] after isolation
env.damping_ratio        = env.vibration_damped / env.vibration_input;

%% -----------------------------------------------------------------------
%  Report
%  -----------------------------------------------------------------------
fprintf('\n  ════════════════════════════════════════════\n');
fprintf('  ENVIRONMENTAL ENCLOSURE (PDF §VIII-A)\n');
fprintf('  ════════════════════════════════════════════\n');
fprintf('    IP Rating           : %s\n', env.ip_rating);
fprintf('    Temperature Control : %s\n', env.temperature_control);
fprintf('    Operating Range     : %d°C to %d°C\n', ...
    env.temp_range(1), env.temp_range(2));
fprintf('    TEC Cooling         : %.0f W capacity\n', env.tec_cooling_capacity);
fprintf('    Vibration Isolation : %s\n', env.vibration_isolation);
fprintf('    Input Vibration     : %.1f g → %.2f g (damped)\n', ...
    env.vibration_input, env.vibration_damped);
fprintf('    Dust Protection     : %s\n', env.dust_protection);
fprintf('  ════════════════════════════════════════════\n\n');

end
