function build_simulink_model()
% BUILD_SIMULINK_MODEL  Create Simulink model using ONLY the real hardware
%   components specified in the 905 nm LiDAR system architecture.
%
%   Hardware signal chain (from architecture document):
%     Laser Diode → Collimating Lens → Beam Splitter → Scanner
%       → F-theta Lens → [Free Space] → Target → [Free Space]
%       → Cassegrain Telescope → Bandpass Filter → APD → TIA → TDC → FPGA
%
%   Every subsystem contains ONLY the physics of that real component.
%   Dark current, shot noise, excess noise are INTERNAL properties of
%   the APD — not separate floating blocks.
%
%   Usage:  build_simulink_model()

modelName = 'lidar_system_model';
params = lidar_params();

%% Derived constants (from architecture spec)
c      = params.sim.c;
R0     = params.rx.apd.responsivity / params.rx.apd.gain;  % unmultiplied responsivity
M      = params.rx.apd.gain;          % 150
k_eff  = 0.03;                        % Si APD ionisation ratio
F_M    = k_eff*M + (1-k_eff)*(2-1/M); % McIntyre excess noise factor
BW     = params.rx.apd.bandwidth;     % 100 MHz
q      = params.sim.q;
kB     = params.sim.k_B;
T      = 300;                         % K

% Default target for demo
R_target = 25;      % m
rho      = 0.3;     % reflectivity

% Pre-computed channel parameters
eta_optics_tx = 0.95;                                    % collimating lens + optics
BS_transmit   = params.scan.beam_splitter_ratio;         % 0.5 (50:50)
A_rx          = pi*(params.rx.telescope.aperture/2)^2;   % telescope aperture area
eta_atm       = exp(-params.sim.atm_extinction*R_target/1e3);
eta_optics_rx = 0.90;                                    % telescope + filter efficiency
tof           = 2*R_target/c;                            % round-trip time

% Full channel attenuation (Laser → Target → Telescope output)
% P_rx = P_laser × BS_tx × eta_tx × (rho/pi) × (A_rx/R²) × eta_atm² × eta_rx
G_channel = BS_transmit * eta_optics_tx * (rho/pi) * (A_rx/R_target^2) ...
            * eta_atm^2 * eta_optics_rx;

% TIA gain (from params, selectable 10kΩ–1MΩ per PDF §VI-A)
Zt = params.rx.tia.gain;   % [Ohm] default 100 kΩ

% APD noise parameters (all internal to APD)
Id         = params.rx.apd.dark_current;       % 1 nA
i2_dark    = 2*q*Id*M^2*F_M*BW;               % dark current noise variance
i_tia_n    = params.rx.tia.noise_density;      % 2 pA/√Hz
tau_tia    = 1/(2*pi*params.rx.tia.bandwidth); % TIA time constant

% Pulse shape
fwhm    = params.tx.pulse_duration;
sigma_t = fwhm/(2*sqrt(2*log(2)));
t_center = 3*fwhm;
T_rep   = 1/params.tx.rep_rate;

%% Close/delete old model
if bdIsLoaded(modelName), close_system(modelName, 0); end
if exist([modelName,'.slx'], 'file'), delete([modelName,'.slx']); end

fprintf('Building Simulink model from architecture spec...\n');

new_system(modelName);
open_system(modelName);
% Use ode1 (Euler) fixed-step solver — required for continuous Transfer Fcn
% block in Component 5 (TIA bandwidth filter). FixedStepDiscrete cannot
% simulate continuous dynamics.
set_param(modelName, ...
    'SolverType','Fixed-step', 'Solver','ode1', ...
    'FixedStep','5e-10', 'StopTime','4e-5');

s = 'simulink';

%% =====================================================================
%  COMPONENT 1: LASER DIODE  (905nm, 75W peak, 5ns, 50kHz)
%  =====================================================================
%  Generates Gaussian optical pulse.  Output = optical power [W].

add_block([s,'/Sources/Clock'], [modelName,'/Time'], ...
    'Position',[50,97,80,113]);

pulse_expr = sprintf('%.0f*exp(-0.5*((mod(u(1),%.6e)-%.6e)/%.6e)^2)', ...
    params.tx.peak_power, T_rep, t_center, sigma_t);
add_block([s,'/User-Defined Functions/Fcn'], [modelName,'/Laser_Diode'], ...
    'Position',[130,85,310,125], 'Expr',pulse_expr);
add_line(modelName,'Time/1','Laser_Diode/1','autorouting','smart');

% Scope: laser output
add_block([s,'/Sinks/Scope'], [modelName,'/Scope_LaserPulse'], ...
    'Position',[380,35,420,75]);
add_line(modelName,'Laser_Diode/1','Scope_LaserPulse/1','autorouting','smart');

anno(modelName,[50,30],'[1] LASER DIODE: 905nm, Ppk=75W, tau=5ns FWHM, PRF=50kHz');

%% =====================================================================
%  COMPONENT 2: COLLIMATING LENS + BEAM SPLITTER  (f=8mm, NA=0.5, 50:50)
%  =====================================================================
%  Combined as a single transmission efficiency.
%  Output = optical power entering free space [W].

G_tx_optics = BS_transmit * eta_optics_tx;   % 0.5 × 0.95 = 0.475
add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/Collimator_and_BS'], ...
    'Position',[130,190,310,220], ...
    'Gain', sprintf('%.4f', G_tx_optics));
add_line(modelName,'Laser_Diode/1','Collimator_and_BS/1','autorouting','smart');

anno(modelName,[50,170], ...
    sprintf('[2] COLLIMATING LENS (f=8mm,NA=0.5) + BEAM SPLITTER (50:50) → G=%.3f', G_tx_optics));

%% =====================================================================
%  COMPONENT 3: FREE SPACE → TARGET → FREE SPACE  (round-trip)
%  =====================================================================
%  Transport Delay = 2R/c models the round-trip propagation time.
%  Gain = Lambertian reflection × geometric collection × atmosphere.
%
%  Physics:  P_rx = P_tx_out × (rho/pi) × (A_rx/R²) × eta_atm² × eta_rx

add_block([s,'/Continuous/Transport Delay'], ...
    [modelName,'/Free_Space_Propagation'], ...
    'Position',[130,295,310,325], ...
    'DelayTime', sprintf('%.10e', tof));
add_line(modelName,'Collimator_and_BS/1','Free_Space_Propagation/1','autorouting','smart');

G_target_and_collection = (rho/pi) * (A_rx/R_target^2) * eta_atm^2 * eta_optics_rx;
add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/Target_Reflection_and_Collection'], ...
    'Position',[380,295,560,325], ...
    'Gain', sprintf('%.6e', G_target_and_collection));
add_line(modelName,'Free_Space_Propagation/1','Target_Reflection_and_Collection/1','autorouting','smart');

anno(modelName,[50,270], ...
    sprintf('[3] FREE SPACE (R=%.0fm, ToF=%.1fns) → TARGET (rho=%.1f, Lambertian) → CASSEGRAIN (D=50mm) + BANDPASS FILTER (905nm±5nm)', ...
    R_target, tof*1e9, rho));

% Scope: received optical power at telescope output
add_block([s,'/Sinks/Scope'], [modelName,'/Scope_Prx'], ...
    'Position',[640,245,680,285]);
add_line(modelName,'Target_Reflection_and_Collection/1','Scope_Prx/1','autorouting','smart');

%% =====================================================================
%  COMPONENT 4: APD  (M=150, R=40A/W, BW=100MHz, Id=1nA, NEP=0.5pW/√Hz)
%  =====================================================================
%  The APD converts optical power to electrical current.
%  Dark current and shot noise are INTRINSIC to the APD — they happen
%  inside the device, not as external components.
%
%  I_out = R0 × M × P_rx  +  Id  +  noise
%  where noise = sqrt(2q(R0×P_rx + Id)×M²×F(M)×BW) × N(0,1)

% --- Photocurrent: I_ph = R0 × M × P_rx ---
add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/APD_Photocurrent'], ...
    'Position',[130,410,280,440], ...
    'Gain', sprintf('%.4f', R0*M));    % = 40 A/W total responsivity
add_line(modelName,'Target_Reflection_and_Collection/1','APD_Photocurrent/1','autorouting','smart');

% --- APD internal noise (shot + dark, multiplied by avalanche gain) ---
%  Total APD noise current variance:
%    i²_noise = 2q × (R0×P_rx + Id) × M² × F(M) × BW
%  Since P_rx varies with time, we split into:
%    (a) Signal-dependent shot noise:  K_shot × sqrt(P_rx) × N(0,1)
%    (b) Dark current noise:           K_dark × N(0,1)
%    (c) Dark current DC offset:       Id

K_shot = sqrt(2*q*R0*M^2*F_M*BW);     % signal-dependent scale
K_dark = sqrt(2*q*Id*M^2*F_M*BW);     % constant dark noise

% (a) Signal-dependent shot noise inside APD
add_block([s,'/Math Operations/Math Function'], ...
    [modelName,'/APD_Abs'], 'Position',[200,470,240,500], 'Operator','magnitude');
add_block([s,'/Math Operations/Math Function'], ...
    [modelName,'/APD_Sqrt'], 'Position',[270,470,310,500], 'Operator','sqrt');
add_block([s,'/Sources/Random Number'], ...
    [modelName,'/APD_ShotNoise_RNG'], 'Position',[270,520,310,550], ...
    'SampleTime','5e-10','Variance','1');
add_block([s,'/Math Operations/Product'], ...
    [modelName,'/APD_ShotMult'], 'Position',[350,475,390,510]);
add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/APD_ShotScale'], 'Position',[430,475,530,510], ...
    'Gain', sprintf('%.6e', K_shot));

add_line(modelName,'Target_Reflection_and_Collection/1','APD_Abs/1','autorouting','smart');
add_line(modelName,'APD_Abs/1','APD_Sqrt/1','autorouting','smart');
add_line(modelName,'APD_Sqrt/1','APD_ShotMult/1','autorouting','smart');
add_line(modelName,'APD_ShotNoise_RNG/1','APD_ShotMult/2','autorouting','smart');
add_line(modelName,'APD_ShotMult/1','APD_ShotScale/1','autorouting','smart');

% (b) Dark current noise (intrinsic APD leakage noise)
add_block([s,'/Sources/Random Number'], ...
    [modelName,'/APD_DarkNoise_RNG'], 'Position',[350,555,390,585], ...
    'SampleTime','5e-10','Variance','1');
add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/APD_DarkNoiseScale'], 'Position',[430,555,530,585], ...
    'Gain', sprintf('%.6e', K_dark));
add_line(modelName,'APD_DarkNoise_RNG/1','APD_DarkNoiseScale/1','autorouting','smart');

% (c) Dark current DC (1 nA constant leakage)
add_block([s,'/Sources/Constant'], ...
    [modelName,'/APD_DarkCurrent_DC'], 'Position',[430,610,490,640], ...
    'Value', sprintf('%.2e', Id));

% Sum: I_total = I_photocurrent + I_shot_noise + I_dark_noise + I_dark_DC
add_block([s,'/Math Operations/Add'], ...
    [modelName,'/APD_Output_Sum'], ...
    'Position',[600,410,640,490], 'Inputs','++++');
add_line(modelName,'APD_Photocurrent/1',  'APD_Output_Sum/1','autorouting','smart');
add_line(modelName,'APD_ShotScale/1',     'APD_Output_Sum/2','autorouting','smart');
add_line(modelName,'APD_DarkNoiseScale/1','APD_Output_Sum/3','autorouting','smart');
add_line(modelName,'APD_DarkCurrent_DC/1','APD_Output_Sum/4','autorouting','smart');

anno(modelName,[50,390], ...
    sprintf('[4] APD: R0×M=%.0fA/W, M=%d, F(M)=%.2f, Id=%gnA, BW=%dMHz, NEP=0.5pW/rtHz', ...
    R0*M, M, F_M, Id*1e9, BW/1e6));
anno(modelName,[180,460],'APD internal: shot noise (signal-dependent) + dark current (1nA DC + noise)');

%% =====================================================================
%  COMPONENT 5: TIA  (Zt=100kΩ default, 10kΩ–1MΩ range, BW=200MHz, in=2pA/√Hz, tr=2ns)
%  =====================================================================
%  Converts APD output current to voltage.
%  Bandwidth-limited by single-pole at 200 MHz.
%  TIA input-referred noise and feedback resistor thermal noise
%  are INTRINSIC to the TIA — they happen inside the amplifier.
%
%  V_out = (I_apd × Zt) filtered by H(s)=1/(1+s×tau)  +  V_noise_tia

% Transimpedance gain
add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/TIA_Zt'], ...
    'Position',[130,720,260,750], ...
    'Gain', sprintf('%.0f', Zt));
add_line(modelName,'APD_Output_Sum/1','TIA_Zt/1','autorouting','smart');

% Bandwidth filter: H(s) = 1/(1 + s×tau), tau = 1/(2π×200MHz)
add_block([s,'/Continuous/Transfer Fcn'], ...
    [modelName,'/TIA_Bandwidth'], ...
    'Position',[330,718,460,752], ...
    'Numerator','[1]', ...
    'Denominator',sprintf('[%.6e 1]', tau_tia));
add_line(modelName,'TIA_Zt/1','TIA_Bandwidth/1','autorouting','smart');

% TIA internal noise (must use TIA bandwidth = 200 MHz, not APD bandwidth)
BW_tia = params.rx.tia.bandwidth;                             % 200 MHz
V_tia_noise_rms   = i_tia_n * sqrt(BW_tia) * Zt;             % TIA input noise as voltage
V_thermal_rms     = sqrt(4*kB*T*BW_tia/Zt) * Zt;             % thermal noise as voltage
V_total_noise_rms = sqrt(V_tia_noise_rms^2 + V_thermal_rms^2); % combined TIA noise

add_block([s,'/Sources/Random Number'], ...
    [modelName,'/TIA_Noise_RNG'], 'Position',[330,780,370,810], ...
    'SampleTime','5e-10','Variance','1');
add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/TIA_NoiseScale'], 'Position',[420,780,530,810], ...
    'Gain', sprintf('%.6e', V_total_noise_rms));
add_line(modelName,'TIA_Noise_RNG/1','TIA_NoiseScale/1','autorouting','smart');

% Sum: V_out = V_signal + V_noise
add_block([s,'/Math Operations/Add'], ...
    [modelName,'/TIA_Output'], ...
    'Position',[580,720,620,760], 'Inputs','++');
add_line(modelName,'TIA_Bandwidth/1',  'TIA_Output/1','autorouting','smart');
add_line(modelName,'TIA_NoiseScale/1', 'TIA_Output/2','autorouting','smart');

% Scope: TIA output voltage
add_block([s,'/Sinks/Scope'], [modelName,'/Scope_TIA_Vout'], ...
    'Position',[700,680,740,720]);
add_line(modelName,'TIA_Output/1','Scope_TIA_Vout/1','autorouting','smart');

anno(modelName,[50,700], ...
    sprintf('[5] TIA: Zt=%gkOhm, BW=200MHz, in=2pA/rtHz, tr=2ns (noise combined: %.2fmV rms)', ...
    Zt/1e3, V_total_noise_rms*1e3));

%% =====================================================================
%  COMPONENT 6: TDC  (50ps res, 500MSPS, dead=10ns, jitter=100ps)
%  =====================================================================
%  Architecture:  The TDC measures the TIME at which the return pulse
%  crosses the detection threshold (not the binary 0/1 output).
%
%  Signal chain:
%    V_tia → Comparator → triggers time-latch → TDC quantises time
%    → adds jitter → FPGA computes R = c*t/2
%
%  Implementation:
%    - The comparator detects threshold crossing (output = 0 or 1)
%    - A "time latch" block (Product of comparator × Clock) captures
%      the simulation time when the pulse arrives.
%    - This time is then quantised to 50 ps bins and jitter is added.

V_th = 0.05;  % 50 mV detection threshold

% Threshold comparator: outputs 1 when V_tia > V_th
add_block([s,'/Discontinuities/Relay'], ...
    [modelName,'/TDC_Comparator'], ...
    'Position',[130,870,250,900], ...
    'OnSwitchValue', sprintf('%.4f',V_th), ...
    'OffSwitchValue', sprintf('%.4f',V_th*0.8), ...
    'OnOutputValue','1','OffOutputValue','0');
add_line(modelName,'TIA_Output/1','TDC_Comparator/1','autorouting','smart');

% Scope: digital comparator output
add_block([s,'/Sinks/Scope'], [modelName,'/Scope_TDC'], ...
    'Position',[330,820,370,855]);
add_line(modelName,'TDC_Comparator/1','Scope_TDC/1','autorouting','smart');

% Time-latch: capture simulation time at threshold crossing.
% Output = comparator(t) × Clock(t).  When comparator = 0, output = 0.
% When comparator = 1, output = current simulation time.
% This gives us the time-of-arrival of the return pulse.
add_block([s,'/Sources/Clock'], [modelName,'/TDC_Clock'], ...
    'Position',[130,930,170,960]);
add_block([s,'/Math Operations/Product'], ...
    [modelName,'/TDC_TimeLatch'], 'Position',[300,880,340,920]);
add_line(modelName,'TDC_Comparator/1','TDC_TimeLatch/1','autorouting','smart');
add_line(modelName,'TDC_Clock/1','TDC_TimeLatch/2','autorouting','smart');

% TDC quantiser applied to TIME (not to binary output).
% Quantisation interval = 50 ps — this is the TDC's resolution.
add_block([s,'/Discontinuities/Quantizer'], ...
    [modelName,'/TDC_Quantizer'], ...
    'Position',[400,880,500,920], ...
    'QuantizationInterval', sprintf('%.2e', params.timing.tdc_resolution));
add_line(modelName,'TDC_TimeLatch/1','TDC_Quantizer/1','autorouting','smart');

% Timing jitter (100 ps RMS, Gaussian — from PLL laser/TDC sync, PDF §X-B)
% Added as a random time error to the quantised ToF measurement.
add_block([s,'/Sources/Random Number'], ...
    [modelName,'/Jitter_100ps'], 'Position',[400,940,440,970], ...
    'SampleTime','5e-10','Variance',sprintf('%.4e', params.timing.jitter^2));
add_block([s,'/Math Operations/Add'], ...
    [modelName,'/TDC_Output'], 'Position',[560,880,600,930], 'Inputs','++');
add_line(modelName,'TDC_Quantizer/1','TDC_Output/1','autorouting','smart');
add_line(modelName,'Jitter_100ps/1', 'TDC_Output/2','autorouting','smart');

anno(modelName,[50,850], ...
    sprintf('[6] TDC: Comparator(Vth=%.0fmV) → Time-Latch → Quantiser(50ps) + Jitter(100ps)', V_th*1e3));

%% =====================================================================
%  COMPONENT 7: FPGA  (450MHz clock, 16-sample averaging)
%  =====================================================================
%  Range calculation: R = c × ToF / 2
%  The TDC_Output is a quantised + jittered TIME value (in seconds).
%  The FPGA multiplies by c/2 to convert to range in metres.

add_block([s,'/Math Operations/Gain'], ...
    [modelName,'/FPGA_Range_Calc'], ...
    'Position',[680,880,800,920], ...
    'Gain', sprintf('%.6f', c/2));
add_line(modelName,'TDC_Output/1','FPGA_Range_Calc/1','autorouting','smart');

add_block([s,'/Sinks/Display'], [modelName,'/Range_Display_m'], ...
    'Position',[870,880,980,920]);
add_line(modelName,'FPGA_Range_Calc/1','Range_Display_m/1','autorouting','smart');

anno(modelName,[50,980], ...
    sprintf('[7] FPGA: 450MHz clk, 16-sample avg → R = c×ToF/2'));

%% =====================================================================
%  OVERLAY SCOPE: Transmitted vs Received pulse
%  =====================================================================
add_block([s,'/Signal Routing/Mux'], [modelName,'/Mux_Compare'], ...
    'Position',[600,130,610,170], 'Inputs','2');
add_block([s,'/Sinks/Scope'], [modelName,'/Scope_TX_vs_RX'], ...
    'Position',[670,125,730,175]);
add_line(modelName,'Laser_Diode/1','Mux_Compare/1','autorouting','smart');
add_line(modelName,'Target_Reflection_and_Collection/1','Mux_Compare/2','autorouting','smart');
add_line(modelName,'Mux_Compare/1','Scope_TX_vs_RX/1','autorouting','smart');

anno(modelName,[600,110],'TX pulse vs RX pulse (time delayed + attenuated)');

%% =====================================================================
%  SYSTEM PARAMETER PANEL
%  =====================================================================
anno(modelName, [900,300], sprintf([ ...
    '════════════════════════════════\n', ...
    '  SYSTEM PARAMETERS (from arch. doc)\n', ...
    '════════════════════════════════\n', ...
    '  Laser:  905nm, 75W, 5ns, 50kHz\n', ...
    '  Lens:   f=8mm, NA=0.5\n', ...
    '  BS:     50:50 @ 905nm\n', ...
    '  Scanner: Polygon 8-facet 10kRPM\n', ...
    '  F-theta: f=100mm\n', ...
    '  Telescope: D=50mm, f=150mm\n', ...
    '  Filter: 905nm, FWHM=10nm\n', ...
    '  APD:    M=%d, R=%.0fA/W, Id=%gnA\n', ...
    '  TIA:    Zt=%gkΩ, BW=200MHz\n', ...
    '  TDC:    50ps, 500MSPS\n', ...
    '  FPGA:   450MHz\n', ...
    '  Range:  %.0fm (configurable)\n', ...
    '  Channel Gain: %.2e\n', ...
    '════════════════════════════════\n', ...
    '  To change target range:\n', ...
    '    1. Free_Space_Propagation delay\n', ...
    '    2. Target_Reflection gain\n', ...
    '════════════════════════════════'], ...
    M, R0*M, Id*1e9, Zt/1e3, R_target, G_channel));

%% Save
save_system(modelName);

fprintf('\n══════════════════════════════════════════\n');
fprintf('  Simulink model: %s.slx\n', modelName);
fprintf('══════════════════════════════════════════\n');
fprintf('  Real components from architecture:\n');
fprintf('  [1] Laser Diode (Gaussian pulse gen)\n');
fprintf('  [2] Collimating Lens + Beam Splitter\n');
fprintf('  [3] Free Space + Target + Telescope + Filter\n');
fprintf('  [4] APD (photocurrent + internal noise)\n');
fprintf('  [5] TIA (Zt gain + BW filter + internal noise)\n');
fprintf('  [6] TDC (comparator + 50ps quantizer + jitter)\n');
fprintf('  [7] FPGA (range = c*ToF/2)\n');
fprintf('  5 Scopes + 1 Range Display\n');
fprintf('══════════════════════════════════════════\n');
end

function anno(mdl,pos,txt)
    persistent nc; if isempty(nc), nc=0; end; nc=nc+1;
    try
        a = Simulink.Annotation([mdl,'/A',num2str(nc)]);
        a.Text=txt; a.Position=pos; a.FontSize=9;
    catch
        add_block('built-in/Note',[mdl,'/A',num2str(nc)], ...
            'Position',[pos(1),pos(2),pos(1)+500,pos(2)+15], ...
            'Text',txt,'FontSize',9);
    end
end
