function budget = lidar_power_budget(params)
% LIDAR_POWER_BUDGET  Compute total system power consumption.
%
%   budget = lidar_power_budget(params)
%
%   Uses switching regulator efficiency of 92 % and the per-subsystem
%   voltage/current specifications from the architecture document.
%
%   Outputs:
%       budget - struct with per-subsystem and total power draw [W]

pwr = params.power;
eta = pwr.regulator_efficiency;

%% -----------------------------------------------------------------------
%  Subsystem Power (load side)
%  -----------------------------------------------------------------------
P_laser   = pwr.laser.V   * pwr.laser.I_max;       % [W]
P_apd     = pwr.apd.V     * pwr.apd.I_max;         % [W]

% FPGA has dual rails — sum both
P_fpga    = sum(pwr.fpga.V) * pwr.fpga.I_max;      % [W]  (simplified)

P_gimbal  = pwr.gimbal.V  * pwr.gimbal.I_max;      % [W]
P_cooling = pwr.cooling.V * pwr.cooling.I_max;      % [W]

P_load_total = P_laser + P_apd + P_fpga + P_gimbal + P_cooling;

%% -----------------------------------------------------------------------
%  Supply-Side Power  (accounting for regulator losses)
%  -----------------------------------------------------------------------
P_supply_total = P_load_total / eta;
P_regulator_loss = P_supply_total - P_load_total;

%% -----------------------------------------------------------------------
%  Report
%  -----------------------------------------------------------------------
fprintf('\n========================================\n');
fprintf('  POWER BUDGET SUMMARY\n');
fprintf('========================================\n');
fprintf('  Laser Diode     : %8.2f W  (%4.1f V × %5.2f A)\n', ...
    P_laser, pwr.laser.V, pwr.laser.I_max);
fprintf('  APD Bias         : %8.4f W  (%4.0f V × %5.1f µA)\n', ...
    P_apd, pwr.apd.V, pwr.apd.I_max * 1e6);
fprintf('  FPGA             : %8.2f W  (%s V × %5.2f A)\n', ...
    P_fpga, mat2str(pwr.fpga.V), pwr.fpga.I_max);
fprintf('  Gimbal Motors    : %8.2f W  (%4.0f V × %5.2f A)\n', ...
    P_gimbal, pwr.gimbal.V, pwr.gimbal.I_max);
fprintf('  Cooling System   : %8.2f W  (%4.0f V × %5.2f A)\n', ...
    P_cooling, pwr.cooling.V, pwr.cooling.I_max);
fprintf('  ----------------------------------------\n');
fprintf('  Total Load       : %8.2f W\n', P_load_total);
fprintf('  Regulator Loss   : %8.2f W  (η = %.0f%%)\n', ...
    P_regulator_loss, eta * 100);
fprintf('  Total Supply     : %8.2f W\n', P_supply_total);
fprintf('========================================\n\n');

%% -----------------------------------------------------------------------
%  Pack Output
%  -----------------------------------------------------------------------
budget.P_laser       = P_laser;
budget.P_apd         = P_apd;
budget.P_fpga        = P_fpga;
budget.P_gimbal      = P_gimbal;
budget.P_cooling     = P_cooling;
budget.P_load_total  = P_load_total;
budget.P_supply_total = P_supply_total;
budget.P_loss        = P_regulator_loss;
budget.efficiency    = eta;

end
