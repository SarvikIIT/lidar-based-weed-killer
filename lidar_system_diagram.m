function lidar_system_diagram()
% LIDAR_SYSTEM_DIAGRAM  Draw a comprehensive block diagram / circuit
%   schematic of the complete 905 nm weed-detection LiDAR system.
%
%   Usage:  lidar_system_diagram()
%
%   Generates 3 figures:
%     Figure 1 — Full system signal-chain block diagram
%     Figure 2 — Receiver analog front-end circuit schematic
%     Figure 3 — Power distribution tree

%% =====================================================================
%  FIGURE 1:  FULL SYSTEM BLOCK DIAGRAM
%  =====================================================================
fig1 = figure('Name', 'LiDAR System — Block Diagram', ...
    'Position', [40, 60, 1600, 900], 'Color', 'w', ...
    'MenuBar', 'none', 'NumberTitle', 'off');
ax1 = axes('Parent', fig1, 'Position', [0 0 1 1]);
axis(ax1, [0 160 0 100]);
axis off; hold on;

% ---- Colour palette ----
col_tx      = [0.90 0.25 0.20];   % red-orange
col_optics  = [0.20 0.55 0.85];   % blue
col_rx      = [0.18 0.65 0.35];   % green
col_digital = [0.55 0.30 0.75];   % purple
col_mech    = [0.85 0.55 0.15];   % amber
col_bg      = [0.96 0.96 0.98];   % light background
col_arrow   = [0.25 0.25 0.25];   % dark grey

% ---- Title ----
text(80, 97, '905 nm Weed-Detection LiDAR — System Architecture', ...
    'FontSize', 18, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'Color', [0.15 0.15 0.15]);
text(80, 94, 'Full Electro-Optical Signal Chain', ...
    'FontSize', 11, 'HorizontalAlignment', 'center', ...
    'Color', [0.4 0.4 0.4], 'FontAngle', 'italic');

% =========== TRANSMITTER CHAIN (top row, left) ===========
y_tx = 75;

% Laser Diode
draw_block(ax1, 5, y_tx, 18, 12, col_tx, ...
    {'LASER DIODE', '', '905 nm', 'P_{pk} = 75 W', ...
     '\tau = 5 ns', 'PRF = 50 kHz', '\Delta\lambda = 3 nm'}, 'w');

% Arrow
draw_arrow(ax1, 23, y_tx, 27, y_tx, col_arrow);

% Collimating Lens
draw_block(ax1, 27, y_tx, 16, 12, col_optics, ...
    {'COLLIMATING', 'LENS', '', 'f = 8 mm', 'NA = 0.5', ...
     '\theta_{div} = 4 mrad'}, 'w');

% Arrow
draw_arrow(ax1, 43, y_tx, 47, y_tx, col_arrow);

% Beam Splitter
draw_block(ax1, 47, y_tx, 14, 12, col_optics, ...
    {'BEAM', 'SPLITTER', '', '50:50 @ 905nm', ...
     'R=((n1-n2)/', '(n1+n2))^2'}, 'w');

% Arrow right → Scanner
draw_arrow(ax1, 61, y_tx, 65, y_tx, col_arrow);

% Arrow down from BS → Reference detector (dashed)
draw_dashed_arrow(ax1, 54, y_tx-6, 54, y_tx-14, [0.5 0.5 0.5]);
text(56, y_tx-10, 'Ref', 'FontSize', 8, 'Color', [0.5 0.5 0.5]);

% =========== SCANNING OPTICS (top row, right) ===========

% Scanner (Polygon / Galvo)
draw_block(ax1, 65, y_tx, 18, 12, col_mech, ...
    {'SCANNER', '(Polygon Mirror)', '', '8 facets', ...
     '10,000 RPM', '\theta_{opt} = 120°'}, 'w');

% Arrow
draw_arrow(ax1, 83, y_tx, 87, y_tx, col_arrow);

% F-theta Lens
draw_block(ax1, 87, y_tx, 16, 12, col_optics, ...
    {'F-THETA', 'LENS', '', 'f = 100 mm', ...
     'Field: 200×200 mm', 'Dist: 10.1%'}, 'w');

% Arrow to target
draw_arrow(ax1, 103, y_tx, 110, y_tx, col_arrow);
text(106.5, y_tx+2, 'TX Beam', 'FontSize', 8, 'Color', col_arrow, ...
    'HorizontalAlignment', 'center');

% =========== TARGET ===========
draw_block(ax1, 115, y_tx-14, 16, 40, [0.85 0.80 0.70], ...
    {'TARGET', 'SCENE', '', 'R: 1–50 m', ...
     '\rho = 0.3', '(Lambertian)', '', ...
     'Crops &', 'Weeds'}, [0.3 0.2 0.1]);
% Wavy lines for vegetation
for yy = [y_tx-8, y_tx-4, y_tx+0, y_tx+4]
    xx = linspace(117, 129, 30);
    plot(ax1, xx, yy + 0.8*sin(xx*1.5), 'Color', [0.2 0.6 0.2], 'LineWidth', 0.8);
end

% Return arrow (target → receiver telescope)
draw_arrow(ax1, 115, y_tx-14, 103, y_tx-28, [0.1 0.5 0.1]);
text(112, y_tx-22, {'Backscatter', '\eta_{atm}'}, 'FontSize', 8, ...
    'Color', [0.1 0.5 0.1], 'HorizontalAlignment', 'center');

% =========== RECEIVER CHAIN (bottom row, right to left) ===========
y_rx = 38;

% Cassegrain Telescope
draw_block(ax1, 87, y_rx, 16, 12, col_rx, ...
    {'CASSEGRAIN', 'TELESCOPE', '', 'D = 50 mm', ...
     'f = 150 mm', 'FOV = 2 mrad'}, 'w');

% Arrow
draw_arrow(ax1, 87, y_rx, 83, y_rx, col_arrow);

% Bandpass Filter
draw_block(ax1, 67, y_rx, 16, 12, col_rx, ...
    {'BANDPASS', 'FILTER', '', '\lambda_c = 905 nm', ...
     'FWHM = 10 nm', '\pm5° angle tol.'}, 'w');

% Arrow
draw_arrow(ax1, 67, y_rx, 63, y_rx, col_arrow);

% APD
draw_block(ax1, 43, y_rx, 20, 12, col_rx, ...
    {'APD', '(Avalanche PD)', '', '\phi = 200 \mum', ...
     'R = 40 A/W', 'M = 150', 'BW = 100 MHz', ...
     'I_d = 1 nA'}, 'w');

% Arrow
draw_arrow(ax1, 43, y_rx, 39, y_rx, col_arrow);

% TIA
draw_block(ax1, 21, y_rx, 18, 12, col_rx, ...
    {'TIA', '(Transimpedance)', '', 'Z_T = 10k–1M\Omega', ...
     'BW = 200 MHz', 'i_n = 2 pA/\surdHz', ...
     't_r = 2 ns'}, 'w');

% Arrow
draw_arrow(ax1, 21, y_rx, 17, y_rx, col_arrow);

% =========== DIGITAL PROCESSING (bottom row, left) ===========
y_dg = 14;

% TDC
draw_block(ax1, 5, y_dg, 18, 12, col_digital, ...
    {'TDC', '(Time-to-Digital)', '', 'Res: 50 ps', ...
     '500 MSPS', 'Dead: 10 ns', 'INL: ±0.5 LSB'}, 'w');

% Arrow from TIA down to TDC
draw_arrow(ax1, 14, y_rx-6, 14, y_dg+6, col_arrow);

% Arrow
draw_arrow(ax1, 23, y_dg, 27, y_dg, col_arrow);

% FPGA
draw_block(ax1, 27, y_dg, 20, 12, col_digital, ...
    {'FPGA', 'PROCESSOR', '', 'f_{clk} = 450 MHz', ...
     'Avg: 16 samples', 'Jitter: 100 ps', '', ...
     'Point Cloud Gen'}, 'w');

% Arrow
draw_arrow(ax1, 47, y_dg, 51, y_dg, col_arrow);

% ML Classifier
draw_block(ax1, 51, y_dg, 20, 12, [0.75 0.20 0.55], ...
    {'RANDOM FOREST', 'CLASSIFIER', '', '100 trees', ...
     'Depth: 20', '5 features', 'Acc: 94.7%', ...
     '50k training pts'}, 'w');

% Arrow
draw_arrow(ax1, 71, y_dg, 75, y_dg, col_arrow);

% Output
draw_block(ax1, 75, y_dg, 18, 12, [0.2 0.2 0.2], ...
    {'OUTPUT', '', 'Weed Map', '3D Point Cloud', ...
     'Classification', 'Labels'}, [0.1 0.9 0.4]);

% =========== GIMBAL (right side, bottom) ===========
draw_block(ax1, 115, y_dg, 16, 12, col_mech, ...
    {'GIMBAL', 'SYSTEM', '', 'Pan: \pm180°', ...
     'Tilt: -30°..+90°', 'Speed: 60°/s'}, 'w');

% Arrow from gimbal to scanner
draw_dashed_arrow(ax1, 123, y_dg+6, 123, y_tx-20, col_mech);
draw_dashed_arrow(ax1, 123, y_tx-20, 74, y_tx-6, col_mech);
text(100, y_tx-18, 'Pointing Control', 'FontSize', 8, ...
    'Color', col_mech, 'FontAngle', 'italic');

% Arrow from FPGA to Gimbal
draw_dashed_arrow(ax1, 47, y_dg-2, 115, y_dg-2, col_digital);
text(80, y_dg-4, 'Gimbal Commands', 'FontSize', 8, ...
    'Color', col_digital, 'FontAngle', 'italic');

% =========== SIGNAL FLOW LABELS ===========
% TX path label
text(55, y_tx+8, '\bf{TRANSMIT PATH}', 'FontSize', 11, ...
    'Color', col_tx, 'HorizontalAlignment', 'center', 'Interpreter', 'tex');
% RX path label
text(55, y_rx+8, '\bf{RECEIVE PATH}', 'FontSize', 11, ...
    'Color', col_rx, 'HorizontalAlignment', 'center', 'Interpreter', 'tex');
% Digital label
text(40, y_dg+8, '\bf{DIGITAL PROCESSING}', 'FontSize', 11, ...
    'Color', col_digital, 'HorizontalAlignment', 'center', 'Interpreter', 'tex');

% =========== LEGEND ===========
legend_x = 140; legend_y = 38;
draw_block(ax1, legend_x, legend_y, 18, 22, [0.97 0.97 0.97], {}, 'k');
text(legend_x+9, legend_y+9, '\bf{Legend}', 'FontSize', 10, ...
    'HorizontalAlignment', 'center');
draw_small_rect(ax1, legend_x+1, legend_y+6, col_tx);
text(legend_x+4, legend_y+6, 'Laser Source', 'FontSize', 8);
draw_small_rect(ax1, legend_x+1, legend_y+4, col_optics);
text(legend_x+4, legend_y+4, 'Optics', 'FontSize', 8);
draw_small_rect(ax1, legend_x+1, legend_y+2, col_rx);
text(legend_x+4, legend_y+2, 'Receiver', 'FontSize', 8);
draw_small_rect(ax1, legend_x+1, legend_y+0, col_digital);
text(legend_x+4, legend_y+0, 'Digital / DSP', 'FontSize', 8);
draw_small_rect(ax1, legend_x+1, legend_y-2, col_mech);
text(legend_x+4, legend_y-2, 'Mechanical', 'FontSize', 8);
draw_small_rect(ax1, legend_x+1, legend_y-4, [0.75 0.20 0.55]);
text(legend_x+4, legend_y-4, 'ML / Classifier', 'FontSize', 8);


%% =====================================================================
%  FIGURE 2:  RECEIVER ANALOG FRONT-END CIRCUIT
%  =====================================================================
fig2 = figure('Name', 'LiDAR — Receiver Circuit Schematic', ...
    'Position', [80, 40, 1400, 700], 'Color', 'w', ...
    'MenuBar', 'none', 'NumberTitle', 'off');
ax2 = axes('Parent', fig2, 'Position', [0 0 1 1]);
axis(ax2, [0 140 0 70]);
axis off; hold on;

text(70, 67, 'Receiver Analog Front-End — Circuit Schematic', ...
    'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% ---- Photodiode Symbol (APD) ----
apd_x = 15; apd_y = 40;

% Incoming light arrows
for dy = [-2, 0, 2]
    draw_arrow(ax2, apd_x-8, apd_y+dy+3, apd_x-3, apd_y+dy+1, [0.9 0.6 0.1]);
end
text(apd_x-10, apd_y+6, {'h\nu', '(905 nm)'}, 'FontSize', 10, ...
    'Color', [0.9 0.6 0.1], 'FontWeight', 'bold');

% Diode triangle
triangle_x = [apd_x-2, apd_x+2, apd_x-2, apd_x-2];
triangle_y = [apd_y+3, apd_y, apd_y-3, apd_y+3];
fill(ax2, triangle_x, triangle_y, col_rx, 'EdgeColor', 'k', 'LineWidth', 1.5);
% Cathode bar
plot(ax2, [apd_x+2, apd_x+2], [apd_y-3, apd_y+3], 'k', 'LineWidth', 2);
% APD gain symbol (zigzag / avalanche)
plot(ax2, [apd_x+0.5, apd_x+1.5], [apd_y+4, apd_y+5], 'r', 'LineWidth', 1.5);
plot(ax2, [apd_x+1.5, apd_x+0.5], [apd_y+5, apd_y+6], 'r', 'LineWidth', 1.5);

% APD label
text(apd_x, apd_y-6, {'APD', 'M = 150', 'R = 40 A/W', ...
    'BW = 100 MHz', 'I_d = 1 nA'}, ...
    'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', col_rx);

% HV Bias supply
plot(ax2, [apd_x, apd_x], [apd_y+3, apd_y+12], 'k', 'LineWidth', 1.2);
plot(ax2, [apd_x, apd_x+4], [apd_y+12, apd_y+12], 'k', 'LineWidth', 1.2);
draw_block(ax2, apd_x+4, apd_y+12, 10, 4, [0.9 0.85 0.95], ...
    {'HV BIAS', '150–200 V'}, col_digital);

% Wire from cathode to TIA
plot(ax2, [apd_x+2, 35], [apd_y, apd_y], 'k', 'LineWidth', 1.5);
% Current label
text(25, apd_y+2, 'I_{ph} = R \cdot M \cdot P_{rx}', ...
    'FontSize', 10, 'HorizontalAlignment', 'center', 'Interpreter', 'tex');

% ---- DC Blocking Capacitor ----
cap_x = 32;
plot(ax2, [cap_x, cap_x], [apd_y-2, apd_y+2], 'k', 'LineWidth', 2.5);
plot(ax2, [cap_x+1.5, cap_x+1.5], [apd_y-2, apd_y+2], 'k', 'LineWidth', 2.5);
text(cap_x+0.75, apd_y-3.5, 'C_{dc}', 'FontSize', 9, 'HorizontalAlignment', 'center');

% ---- TIA (Op-Amp Circuit) ----
tia_x = 50; tia_y = apd_y;

% Op-amp triangle
op_x = [tia_x, tia_x+12, tia_x, tia_x];
op_y = [tia_y+8, tia_y, tia_y-8, tia_y+8];
fill(ax2, op_x, op_y, [0.95 0.98 0.95], 'EdgeColor', 'k', 'LineWidth', 1.5);

% + and - inputs
text(tia_x+1.5, tia_y+4, '–', 'FontSize', 14, 'FontWeight', 'bold');
text(tia_x+1.5, tia_y-4, '+', 'FontSize', 14, 'FontWeight', 'bold');

% Input wire to – (inverting)
plot(ax2, [35, tia_x], [apd_y, tia_y+4], 'k', 'LineWidth', 1.5);

% Non-inverting input to ground
plot(ax2, [tia_x, tia_x-3], [tia_y-4, tia_y-4], 'k', 'LineWidth', 1.2);
% Ground symbol
gnd_x = tia_x - 3;
gnd_y = tia_y - 4;
plot(ax2, [gnd_x-1, gnd_x+1], [gnd_y-1, gnd_y-1], 'k', 'LineWidth', 1.5);
plot(ax2, [gnd_x-0.6, gnd_x+0.6], [gnd_y-1.7, gnd_y-1.7], 'k', 'LineWidth', 1.2);
plot(ax2, [gnd_x-0.2, gnd_x+0.2], [gnd_y-2.3, gnd_y-2.3], 'k', 'LineWidth', 0.8);

% Feedback resistor (Z_T)
fb_y_top = tia_y + 10;
plot(ax2, [tia_x+1, tia_x+1], [tia_y+8, fb_y_top], 'k', 'LineWidth', 1.2);
plot(ax2, [tia_x+1, tia_x+11], [fb_y_top, fb_y_top], 'k', 'LineWidth', 1.2);

% Zigzag resistor symbol
draw_resistor(ax2, tia_x+3, fb_y_top, tia_x+9, fb_y_top);
text(tia_x+6, fb_y_top+2.5, {'Z_T = 10 k\Omega – 1 M\Omega', '(Selectable)'}, ...
    'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', [0.1 0.1 0.6]);

% Feedback to output
plot(ax2, [tia_x+11, tia_x+12], [fb_y_top, tia_y], 'k', 'LineWidth', 1.2);

% Output wire
plot(ax2, [tia_x+12, tia_x+20], [tia_y, tia_y], 'k', 'LineWidth', 1.5);

% TIA label
text(tia_x+6, tia_y-11, {'TIA', 'BW = 200 MHz', 'i_n = 2 pA/\surdHz', ...
    't_r = 2 ns'}, ...
    'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', col_rx);

% Output voltage label
text(tia_x+16, tia_y+3, 'V_{out} = I_{ph} \times Z_T', ...
    'FontSize', 10, 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'Interpreter', 'tex');

% ---- Comparator / Threshold ----
comp_x = 78; comp_y = tia_y;
op2_x = [comp_x, comp_x+10, comp_x, comp_x];
op2_y = [comp_y+6, comp_y, comp_y-6, comp_y+6];
fill(ax2, op2_x, op2_y, [0.95 0.92 0.98], 'EdgeColor', 'k', 'LineWidth', 1.5);
text(comp_x+1.5, comp_y+3, '+', 'FontSize', 12, 'FontWeight', 'bold');
text(comp_x+1.5, comp_y-3, '–', 'FontSize', 12, 'FontWeight', 'bold');

% Input from TIA
plot(ax2, [tia_x+20, comp_x], [tia_y, comp_y+3], 'k', 'LineWidth', 1.2);

% Threshold voltage
plot(ax2, [comp_x-5, comp_x], [comp_y-3, comp_y-3], 'k', 'LineWidth', 1.2);
text(comp_x-7, comp_y-3, 'V_{th}', 'FontSize', 10, 'HorizontalAlignment', 'center');

% Output to TDC
plot(ax2, [comp_x+10, comp_x+18], [comp_y, comp_y], 'k', 'LineWidth', 1.5);
text(comp_x+5, comp_y-8, {'COMPARATOR', '(Threshold', 'Crossing)'}, ...
    'FontSize', 9, 'HorizontalAlignment', 'center', 'Color', col_digital);

% ---- TDC Block ----
draw_block(ax2, comp_x+18, comp_y, 16, 8, col_digital, ...
    {'TDC', '50 ps / 500 MSPS', 'Dead: 10 ns'}, 'w');

% Arrow to FPGA
draw_arrow(ax2, comp_x+34, comp_y, comp_x+38, comp_y, col_arrow);

% FPGA Block
draw_block(ax2, comp_x+38, comp_y, 16, 8, col_digital, ...
    {'FPGA', '450 MHz', '16× Averaging'}, 'w');

% ---- Noise Sources Annotation ----
noise_x = 25; noise_y = 10;
draw_block(ax2, noise_x, noise_y, 40, 10, [1 0.97 0.92], {}, 'k');
text(noise_x + 20, noise_y + 3.5, '\bf{Noise Sources}', 'FontSize', 11, ...
    'HorizontalAlignment', 'center', 'Interpreter', 'tex');
noise_texts = {
    'i^2_{shot} = 2qR_0 P_{rx} M^2 F(M) B', ...
    'i^2_{dark} = 2qI_d M^2 F(M) B', ...
    'i^2_{TIA} = i_n^2 \cdot B', ...
    'i^2_{thermal} = 4k_BT B / Z_T'
};
for k = 1:4
    text(noise_x + 20, noise_y + 1.5 - 2*(k-1), noise_texts{k}, ...
        'FontSize', 9, 'HorizontalAlignment', 'center', ...
        'Interpreter', 'tex', 'Color', [0.6 0.1 0.1]);
end

% SNR equation
text(85, noise_y+2, {'SNR = I_{signal}^2 / \Sigma i^2_{noise}', '', ...
    'F(M) = k_{eff}M + (1-k_{eff})(2 - 1/M)', ...
    'k_{eff} \approx 0.03  (Si APD)'}, ...
    'FontSize', 10, 'HorizontalAlignment', 'center', ...
    'Interpreter', 'tex', 'Color', [0.1 0.1 0.6], ...
    'BackgroundColor', [0.95 0.95 1], 'EdgeColor', [0.5 0.5 0.8]);


%% =====================================================================
%  FIGURE 3:  POWER DISTRIBUTION TREE
%  =====================================================================
fig3 = figure('Name', 'LiDAR — Power Distribution', ...
    'Position', [120, 80, 1200, 600], 'Color', 'w', ...
    'MenuBar', 'none', 'NumberTitle', 'off');
ax3 = axes('Parent', fig3, 'Position', [0 0 1 1]);
axis(ax3, [0 120 0 60]);
axis off; hold on;

text(60, 57, 'Power Distribution — Switching Regulators (\eta = 92%)', ...
    'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Main supply
draw_block(ax3, 45, 48, 30, 6, [0.3 0.3 0.3], ...
    {'MAIN POWER SUPPLY', 'Total: ~125 W (supply side)'}, 'w');

% Branches
subsystems = {
    'LASER DIODE',    '5 V × 10 A',    '50.0 W',  [0.90 0.25 0.20], 10;
    'APD BIAS',       '200 V × 50 µA', '0.01 W',  [0.18 0.65 0.35], 30;
    'FPGA',           '1.0/2.5 V × 3A','10.5 W',  [0.55 0.30 0.75], 50;
    'GIMBAL',         '24 V × 2 A',    '48.0 W',  [0.85 0.55 0.15], 70;
    'COOLING',        '12 V × 2 A',    '24.0 W',  [0.20 0.55 0.85], 90;
};

y_sub = 30;
for k = 1:size(subsystems, 1)
    name = subsystems{k, 1};
    spec = subsystems{k, 2};
    pwr  = subsystems{k, 3};
    clr  = subsystems{k, 4};
    x_pos = subsystems{k, 5};
    
    % Vertical line from main bus
    plot(ax3, [60, x_pos+10], [48, 42], 'k-', 'LineWidth', 1.2);
    
    % Regulator block
    draw_block(ax3, x_pos, 38, 20, 6, [0.9 0.9 0.85], ...
        {'DC-DC', ['\eta = 92%']}, [0.3 0.3 0.3]);
    
    % Line to load
    plot(ax3, [x_pos+10, x_pos+10], [35, y_sub+5], 'k-', 'LineWidth', 1.5);
    
    % Load block
    draw_block(ax3, x_pos, y_sub, 20, 8, clr, ...
        {name, spec, pwr}, 'w');
    
    % Power bar (proportional)
    pwr_val = str2double(strtok(pwr));
    bar_width = max(0.5, pwr_val / 50 * 18);
    fill(ax3, [x_pos+1, x_pos+1+bar_width, x_pos+1+bar_width, x_pos+1], ...
         [y_sub-5.5, y_sub-5.5, y_sub-4.5, y_sub-4.5], clr, ...
         'EdgeColor', 'none', 'FaceAlpha', 0.6);
end

% Total power annotation
text(60, 15, sprintf('Total Load: ~132.5 W   |   Supply (after reg. loss): ~144.0 W   |   Regulator Loss: ~11.5 W'), ...
    'FontSize', 11, 'HorizontalAlignment', 'center', ...
    'BackgroundColor', [0.95 0.95 0.95], 'EdgeColor', [0.7 0.7 0.7]);

fprintf('System diagrams generated (3 figures).\n');
fprintf('  Figure 1: Full System Block Diagram\n');
fprintf('  Figure 2: Receiver Analog Front-End Circuit\n');
fprintf('  Figure 3: Power Distribution Tree\n');

end

%% =======================================================================
%  HELPER FUNCTIONS
%  =======================================================================

function draw_block(ax, cx, cy, w, h, face_color, labels, text_color)
    % Draw a rectangle with centred text labels
    hw = w/2; hh = h/2;
    % Use rectangle for proper rendering (supports Curvature)
    rectangle('Parent', ax, 'Position', [cx-hw, cy-hh, w, h], ...
        'FaceColor', face_color, 'EdgeColor', min(face_color*0.6, [1 1 1]), ...
        'LineWidth', 1.5, 'Curvature', 0.15);
    
    n = numel(labels);
    for k = 1:n
        y_pos = cy + hh*0.7 - (k-1) * (h / (n + 0.5));
        fs = 9;
        fw = 'normal';
        if k == 1
            fs = 10; fw = 'bold';
        end
        text(cx, y_pos, labels{k}, 'FontSize', fs, 'FontWeight', fw, ...
            'HorizontalAlignment', 'center', 'Color', text_color, ...
            'Parent', ax);
    end
end

function draw_arrow(ax, x1, y1, x2, y2, color)
    % Simple thick arrow using quiver
    quiver(ax, x1, y1, x2-x1, y2-y1, 0, ...
        'Color', color, 'LineWidth', 2, 'MaxHeadSize', 0.8);
end

function draw_dashed_arrow(ax, x1, y1, x2, y2, color)
    plot(ax, [x1 x2], [y1 y2], '--', 'Color', color, 'LineWidth', 1.2);
    % Small arrowhead
    quiver(ax, x1, y1, x2-x1, y2-y1, 0, ...
        'Color', color, 'LineWidth', 1, 'MaxHeadSize', 0.5, ...
        'LineStyle', '--');
end

function draw_small_rect(ax, x, y, color)
    fill(ax, [x, x+2, x+2, x, x], [y-0.6, y-0.6, y+0.6, y+0.6, y-0.6], ...
        color, 'EdgeColor', 'k', 'Parent', ax);
end

function draw_resistor(ax, x1, y_pos, x2, ~)
    % Zigzag resistor symbol along horizontal line at y_pos
    n_zag = 6;
    dx = (x2 - x1) / n_zag;
    amp = 1.2;
    xx = x1;
    yy = y_pos;
    for k = 1:n_zag
        x_next = xx + dx;
        if mod(k, 2) == 1
            y_next = y_pos + amp;
        else
            y_next = y_pos - amp;
        end
        plot(ax, [xx, x_next], [yy, y_next], 'k', 'LineWidth', 1.5);
        xx = x_next;
        yy = y_next;
    end
    % Connect ends back to horizontal
    plot(ax, [x2, x2], [yy, y_pos], 'k', 'LineWidth', 1.5);
end
