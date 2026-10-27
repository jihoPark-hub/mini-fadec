function make_figures(ids)
%MAKE_FIGURES  Regenerate every figure and results table for the Mini FADEC project.
%
%   make_figures                       % everything (a minute or two)
%   make_figures("05")                 % one figure
%   make_figures(["03" "robustness"])  % a selection
%
%   ID           What                                   Output
%   01           block diagram                          figures/01_block_diagram.png
%   02           linear vs nonlinear spool (open loop)  figures/02_linear_vs_nonlinear.png
%   03           gain scheduling: 2% steps at 60/85/100 figures/03_gain_scheduling_overlay.png
%   04           anti-windup comparison                 figures/04_antiwindup_comparison.png
%   05           acceleration with protection           figures/05_accel_protection.png
%   06           deceleration with protection           figures/06_decel_protection.png
%   07           Bode plot at take-off + margins        figures/07_bode_takeoff.png
%   tauK         K and tau vs operating point           figures/extra/tau_K_vs_N.png
%   c            choice of c (K_i factor)               figures/extra/c_tradeoff.png
%   margin       acceleration-margin study              figures/extra/margin_study.png
%   robustness   k +/-30%, valve lag, sensor noise      figures/extra/robustness_k.png
%   noise        sensor noise close-up                  figures/extra/sensor_noise.png
%
%   Exact numbers for the README tables go to results/*.csv (also printed).
%
%   Assumptions (they match the final model):
%     - mini_fadec.slx and parameters.m are in this folder
%     - the N_demand step happens at t = 2 s (change S.tStep below if not)
%     - logged signals: N_demand, N, N_meas, Wf_PI, Wf_max, Wf_min, Wf_cmd, Wf
%     - blocks use the parameters.m names: N0, dNd, Wf0, c, Kp, Kt, m_acc,
%       m_dec, k_true, tau_valve, tau_sensor, noise_power, Ts, ...
%
%   The saved model is never changed: test values are applied temporarily.

% ---------------------------------------------------------------- settings
S.root     = fileparts(mfilename('fullpath'));
S.model    = 'mini_fadec';
S.tStep    = 2;     % time of the N_demand step in the model [s]
S.tStop    = 20;    % simulation stop time [s]
S.band     = 1;     % "take-off -> idle within 1%" means N <= 60 + band  [% speed]
S.figDir   = fullfile(S.root, 'figures');
S.extraDir = fullfile(S.figDir, 'extra');
S.resDir   = fullfile(S.root, 'results');

jobs = {
    '01',         @fig01_blockDiagram
    '02',         @fig02_linearVsNonlinear
    '03',         @fig03_gainScheduling
    '04',         @fig04_antiWindup
    '05',         @fig05_accel
    '06',         @fig06_decel
    '07',         @fig07_bode
    'tauK',       @extra_tauK
    'c',          @extra_cStudy
    'margin',     @extra_marginStudy
    'robustness', @extra_robustness
    'noise',      @extra_sensorNoise
    };

if nargin < 1 || isempty(ids), ids = jobs(:,1); else, ids = cellstr(ids); end
bad = setdiff(ids, jobs(:,1));
if ~isempty(bad)
    error('Unknown ID(s): %s.  Valid IDs: %s', strjoin(bad, ', '), strjoin(jobs(:,1).', ', '));
end
S.keepOpen = numel(ids) <= 3;          % on full runs, close each figure after saving

for d = {S.figDir, S.extraDir, S.resDir}
    if ~exist(d{1}, 'dir'), mkdir(d{1}); end
end

S.P = loadParameters(S.root);          % nominal design values as a struct
S.C = colours();

% Work in the project folder (Simulink writes its cache here), and switch off
% the InitFcn callback while this runs: otherwise it re-runs parameters.m before
% every simulation and overwrites the test values set below. Both are restored
% at the end, even after an error or Ctrl+C.
oldDir = cd(S.root);
load_system(S.model);
oldInit  = get_param(S.model, 'InitFcn');
oldDirty = get_param(S.model, 'Dirty');
set_param(S.model, 'InitFcn', '');
cleanup = onCleanup(@() restoreState(S, oldDir, oldInit, oldDirty)); %#ok<NASGU>

failed = {};
tStart = tic;
for i = 1:size(jobs, 1)
    if ~ismember(jobs{i,1}, ids), continue; end
    fprintf('\n[%s] %s\n', jobs{i,1}, func2str(jobs{i,2}));
    try
        jobs{i,2}(S);
    catch err
        failed{end+1} = jobs{i,1}; %#ok<AGROW>
        fprintf(2, '[%s] FAILED: %s\n', jobs{i,1}, err.message);
    end
end
fprintf('\nDone in %.0f s. Figures: %s   Tables: %s\n', toc(tStart), S.figDir, S.resDir);
if ~isempty(failed)
    fprintf(2, 'Failed: %s (messages above)\n', strjoin(failed, ', '));
end
end


%% ======================================================================
%  README figures
%  ======================================================================

function fig01_blockDiagram(S)
open_system(S.model);
set_param(S.model, 'ShowPortDataTypes', 'off');      % no 'double' labels
file = fullfile(S.figDir, '01_block_diagram.png');
print(['-s' S.model], '-dpng', '-r300', file);
fprintf('  saved %s\n', file);
end


function fig02_linearVsNonlinear(S)
% Open-loop spool at idle: nonlinear ODE vs linear model, for three fuel steps.
P = S.P;  C = S.C;
N0  = 60;
Wf0 = (N0/100)^P.n;
a   = (P.n/100)*(N0/100)^(P.n - 1);
K   = 1/a;
tau = 1/(P.k*a);
dW    = [0.02, 0.40 - Wf0, 1.00 - Wf0];     % small / medium / up to take-off fuel
names = {'Small step', 'Medium step', 'Step to take-off fuel'};

f  = newFig(26, 9);
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:3
    Wf = @(t) Wf0 + dW(i)*(t >= 0);
    [t, N] = ode45(@(t, N) P.k*(Wf(t) - (max(N, 0)/100)^P.n), [-1 10], N0, ...
                   odeset('MaxStep', 0.01));
    Nlin = N0 + K*dW(i)*(1 - exp(-t/tau)).*(t >= 0);

    ax = newAx(tl);
    plot(ax, t, N,    '-',  'Color', C.s{1}, 'LineWidth', 1.8, 'DisplayName', 'Nonlinear spool');
    plot(ax, t, Nlin, '--', 'Color', C.s{2}, 'LineWidth', 1.5, 'DisplayName', 'Linearised at idle');
    title(ax, sprintf('%s: \\DeltaW_f = +%.2f', names{i}, dW(i)), 'FontWeight', 'normal');
    xlabel(ax, 'Time after fuel step [s]');
    if i == 1
        ylabel(ax, 'Shaft speed N [%]');
        legend(ax, 'Location', 'southeast');
    end
end
title(tl, 'Open-loop spool at idle: the linear model is only valid near its operating point');
saveFig(S, f, fullfile(S.figDir, '02_linear_vs_nonlinear.png'));
end


function fig03_gainScheduling(S)
% 2% demand steps at idle, cruise and take-off, normalised so they overlay.
P = S.P;  C = S.C;
cases = { 60,  2, 'Idle 60 \rightarrow 62%'
          85,  2, 'Cruise 85 \rightarrow 87%'
         100, -2, 'Take-off 100 \rightarrow 98%' };

f  = newFig(16, 10);
tl = tiledlayout(f, 1, 1, 'Padding', 'compact');
ax = newAx(tl);
rows = {};
for i = 1:size(cases, 1)
    N0  = cases{i,1};
    dNd = cases{i,2};
    [t, N] = getSig(runCase(S, N0, dNd), 'N');
    plot(ax, t - S.tStep, (N - N0)/dNd, 'Color', C.s{i}, 'LineWidth', 1.6, ...
         'DisplayName', cases{i,3});
    [os, t63] = smallStepMetrics(t, N, S, N0, dNd);
    rows(end+1, :) = {sprintf('%d to %d', N0, N0 + dNd), r2(os), r2(os*abs(dNd)/100), r2(t63)}; %#ok<AGROW>
end
yline(ax, 1 - exp(-1), ':', '63%', 'Color', C.muted, 'HandleVisibility', 'off', ...
      'LabelHorizontalAlignment', 'left');
xline(ax, P.tau_cl, '--', sprintf('\\tau_{cl} = %.1f s', P.tau_cl), 'Color', C.muted, ...
      'HandleVisibility', 'off', 'LabelVerticalAlignment', 'bottom');
xlim(ax, [-0.5 6]);  ylim(ax, [-0.05 1.15]);
xlabel(ax, 'Time after demand step [s]');
ylabel(ax, '\DeltaN / step size [-]');
legend(ax, 'Location', 'southeast');
title(tl, '2% demand steps at three operating points with gain-scheduled K_i');
saveFig(S, f, fullfile(S.figDir, '03_gain_scheduling_overlay.png'));

writeResults(S, rows, {'Step', 'Overshoot_pct_of_step', 'Overshoot_pct_speed', 't63_s'}, ...
             'small_steps.csv');
end


function fig04_antiWindup(S)
% Idle -> take-off with a tight acceleration margin (0.10), so the PI spends a
% long time behind the acceleration limit. Tracking is "removed" by making
% K_t negligible rather than switching the track port off (that would
% disconnect the Wf_cmd line).
P = S.P;  C = S.C;
pid     = findPID(S.model);
noTrack = {'m_acc', 0.10, 'Kt', 1e-9};
cases = {
    'No anti-windup',               noTrack,          {pid, 'LimitOutput', 'off'}
    'Output limit + clamping only', noTrack,          {pid, 'LimitOutput', 'on'; pid, 'AntiWindupMode', 'clamping'}
    sprintf('Tracking, K_t = %g (final design)', P.Kt), {'m_acc', 0.10}, {}
    };

f  = newFig(16, 14);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = newAx(tl);
ax2 = newAx(tl);
rows = {};
WpPeak = NaN;
for i = 1:size(cases, 1)
    out      = runCase(S, 60, 40, cases{i,2}, cases{i,3});
    [t,  N ] = getSig(out, 'N');
    [tp, Wp] = getSig(out, 'Wf_PI');
    plot(ax1, t  - S.tStep, N,  'Color', C.s{i}, 'LineWidth', 1.6, 'DisplayName', cases{i,1});
    plot(ax2, tp - S.tStep, Wp, 'Color', C.s{i}, 'LineWidth', 1.6, 'DisplayName', cases{i,1});
    if i == 1, WpPeak = max(Wp); end
    if i == size(cases, 1)
        [tm, Wm] = getSig(out, 'Wf_max');
        plot(ax2, tm - S.tStep, Wm, '--', 'Color', C.muted, 'LineWidth', 1.2, ...
             'DisplayName', 'W_{f,max} (acceleration limit)');
    end
    [tA, os] = accelMetrics(t, N, S);
    rows(end+1, :) = {cases{i,1}, r2(tA), r2(os), r2(max(Wp))}; %#ok<AGROW>
end
yline(ax1, 98.3, ':', '95% thrust', 'Color', C.muted, 'HandleVisibility', 'off', ...
      'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
ylabel(ax1, 'Shaft speed N [%]');
legend(ax1, 'Location', 'southeast');

ylim(ax2, [0 2]);
if WpPeak > 2
    text(ax2, 0.2, 1.9, sprintf('No anti-windup: W_{f,PI} winds up to %.1f (off scale)', WpPeak), ...
         'VerticalAlignment', 'top', 'FontSize', 8, 'Color', C.ink);
end
ylabel(ax2, 'PI output W_{f,PI} [-]');
xlabel(ax2, 'Time after demand step [s]');
legend(ax2, 'Location', 'east');
linkaxes([ax1 ax2], 'x');
xlim(ax1, [-1 S.tStop - S.tStep]);
title(tl, 'Anti-windup: idle \rightarrow take-off with acceleration margin 0.10');
saveFig(S, f, fullfile(S.figDir, '04_antiwindup_comparison.png'));

writeResults(S, rows, {'Method', 'Accel_t95_s', 'Overshoot_pct', 'Peak_Wf_PI'}, ...
             'antiwindup.csv');
end


function fig05_accel(S)
out = runCase(S, 60, 40);
f = plotProtection(S, out, true, ...
    'Idle \rightarrow take-off: acceleration limit, then the PI takes over');
saveFig(S, f, fullfile(S.figDir, '05_accel_protection.png'));
[t, N] = getSig(out, 'N');
[tA, os] = accelMetrics(t, N, S);
fprintf('  idle -> 95%% thrust: %.2f s, overshoot %.2f%%\n', tA, os);
end


function fig06_decel(S)
out = runCase(S, 100, -40);
f = plotProtection(S, out, false, ...
    'Take-off \rightarrow idle: deceleration limit, then the PI takes over');
saveFig(S, f, fullfile(S.figDir, '06_decel_protection.png'));
[t, N] = getSig(out, 'N');
[tD, us] = decelMetrics(t, N, S);
fprintf('  take-off -> idle (N <= %g%%): %.2f s, undershoot %.2f%%\n', 60 + S.band, tD, us);
end


function fig07_bode(S)
% Margins at three operating points (table) and the Bode plot at take-off.
P = S.P;  C = S.C;
ops   = [60 85 100];
names = {'Idle', 'Cruise', 'Take-off'};
rows  = {};
for i = 1:numel(ops)
    [Gm, Pm, Wcg, Wcp] = margin(openLoop(P, ops(i)));
    rows(end+1, :) = {names{i}, ops(i), r2(20*log10(Gm)), r2(Wcg), r2(Pm), r2(Wcp)}; %#ok<AGROW>
end
writeResults(S, rows, {'Operating_point', 'N0_pct', 'GM_dB', 'Phase_crossover_rad_s', ...
                       'PM_deg', 'Gain_crossover_rad_s'}, 'margins.csv');

% Bode plot drawn from bode() data so the styling and limits are fully controlled
L = openLoop(P, 100);
[Gm, Pm, Wcg, Wcp] = margin(L);
GmdB = 20*log10(Gm);
w = logspace(-1, 2, 2000);
[mag, ph] = bode(L, w);
mag = 20*log10(squeeze(mag));
ph  = squeeze(ph);
if ph(1) > 0, ph = ph - 360; end

f  = newFig(16, 13);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = newAx(tl);  ax1.XScale = 'log';
plot(ax1, w, mag, 'Color', C.s{1}, 'LineWidth', 1.6);
refLine(ax1, 'y', 0, C.muted);
if isfinite(Gm)
    plot(ax1, [Wcg Wcg], [-GmdB 0], 'Color', C.s{2}, 'LineWidth', 2);
    text(ax1, Wcg*1.15, -GmdB/2, sprintf('GM = %.1f dB', GmdB), 'FontSize', 9);
end
ylabel(ax1, 'Magnitude [dB]');

ax2 = newAx(tl);  ax2.XScale = 'log';
plot(ax2, w, ph, 'Color', C.s{1}, 'LineWidth', 1.6);
refLine(ax2, 'y', -180, C.muted);
plot(ax2, [Wcp Wcp], [-180, -180 + Pm], 'Color', C.s{2}, 'LineWidth', 2);
text(ax2, Wcp*1.15, -180 + Pm/2, sprintf('PM = %.1f^{\\circ}', Pm), 'FontSize', 9);
ylim(ax2, [-360 0]);  yticks(ax2, -360:90:0);
ylabel(ax2, 'Phase [deg]');
xlabel(ax2, 'Frequency [rad/s]');

linkaxes([ax1 ax2], 'x');
xlim(ax1, [0.1 100]);
title(tl, sprintf('Open loop at take-off: GM %.1f dB, PM %.1f^{\\circ}', GmdB, Pm));
subtitle(tl, 'L(s) = PI \times valve \times engine \times sensor \times sampling delay');
saveFig(S, f, fullfile(S.figDir, '07_bode_takeoff.png'));
end


%% ======================================================================
%  Supporting figures
%  ======================================================================

function extra_tauK(S)
% Linearised gain and time constant across the operating range.
P = S.P;  C = S.C;
N     = linspace(50, 100, 300);
a     = (P.n/100)*(N/100).^(P.n - 1);
ops   = [60 85 100];
aOp   = (P.n/100)*(ops/100).^(P.n - 1);
tauOp = 1./(P.k*aOp);
KOp   = 1./aOp;
lbl   = @(fmt, v) arrayfun(@(x) sprintf(fmt, x), v, 'UniformOutput', false);

f  = newFig(20, 8);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = newAx(tl);
plot(ax1, N, 1./(P.k*a), 'Color', C.s{1}, 'LineWidth', 1.6);
plot(ax1, ops, tauOp, 'o', 'Color', C.s{1}, 'MarkerFaceColor', C.s{1}, 'MarkerSize', 6);
text(ax1, ops + 1, tauOp, lbl('%.2f s', tauOp), 'FontSize', 8, 'VerticalAlignment', 'bottom');
xlabel(ax1, 'Operating point N_0 [%]');  ylabel(ax1, '\tau [s]');
title(ax1, 'Time constant \tau = 1/(k a)', 'FontWeight', 'normal');

ax2 = newAx(tl);
plot(ax2, N, 1./a, 'Color', C.s{1}, 'LineWidth', 1.6);
plot(ax2, ops, KOp, 'o', 'Color', C.s{1}, 'MarkerFaceColor', C.s{1}, 'MarkerSize', 6);
text(ax2, ops + 1, KOp, lbl('%.1f', KOp), 'FontSize', 8, 'VerticalAlignment', 'bottom');
xlabel(ax2, 'Operating point N_0 [%]');  ylabel(ax2, 'K [% speed per unit fuel]');
title(ax2, 'Steady-state gain K = 1/a', 'FontWeight', 'normal');
title(tl, sprintf('Linearised spool G(s) = K/(\\tau s + 1): %.1f\\times faster at take-off than at idle', ...
                  tauOp(1)/tauOp(3)));
saveFig(S, f, fullfile(S.extraDir, 'tau_K_vs_N.png'));
end


function extra_cStudy(S)
% Ki = c*Kp/tau: c = 1 is exact pole cancellation.
C  = S.C;
cs = [1 1.5 2];
f  = newFig(16, 14);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = newAx(tl);
ax2 = newAx(tl);
rows = {};
for i = 1:numel(cs)
    v      = {'c', cs(i)};
    chosen = abs(cs(i) - S.P.c) < 1e-9;
    lw     = 1.2 + 0.8*chosen;                    % chosen value drawn thicker
    nm     = sprintf('c = %g', cs(i));
    if chosen, nm = [nm ' (chosen)']; end

    [tS, NS] = getSig(runCase(S, 60,   2, v), 'N');
    [tA, NA] = getSig(runCase(S, 60,  40, v), 'N');
    [tD, ND] = getSig(runCase(S, 100, -40, v), 'N');

    plot(ax1, tS - S.tStep, (NS - 60)/2, 'Color', C.s{i}, 'LineWidth', lw, 'DisplayName', nm);
    plot(ax2, tD - S.tStep, ND,          'Color', C.s{i}, 'LineWidth', lw, 'DisplayName', nm);

    osS       = smallStepMetrics(tS, NS, S, 60, 2);
    [ta, osA] = accelMetrics(tA, NA, S);
    [td, usD] = decelMetrics(tD, ND, S);
    rows(end+1, :) = {cs(i), r2(osS), r2(ta), r2(osA), r2(td), r2(usD)}; %#ok<AGROW>
end
xlim(ax1, [-0.5 6]);  ylim(ax1, [-0.05 1.2]);
ylabel(ax1, '\DeltaN / step size [-]');
title(ax1, 'Small step at idle (60 \rightarrow 62%): the PI zero causes overshoot', 'FontWeight', 'normal');
legend(ax1, 'Location', 'southeast');

yline(ax2, 60 + S.band, ':', sprintf('within %g%%', S.band), 'Color', C.muted, ...
      'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
xlim(ax2, [0 S.tStop - S.tStep]);  ylim(ax2, [59.5 66]);
ylabel(ax2, 'Shaft speed N [%]');
xlabel(ax2, 'Time after demand step [s]');
title(ax2, 'Take-off \rightarrow idle, close-up: exact cancellation (c = 1) leaves a slow tail', ...
      'FontWeight', 'normal');
legend(ax2, 'Location', 'northeast');
title(tl, 'Choosing c in K_i = c K_p / \tau');
saveFig(S, f, fullfile(S.extraDir, 'c_tradeoff.png'));

writeResults(S, rows, {'c', 'SmallStep_overshoot_pct_of_step', 'Accel_t95_s', ...
                       'Accel_overshoot_pct', 'Decel_t_s', 'Decel_undershoot_pct'}, 'c_study.csv');
end


function extra_marginStudy(S)
C = S.C;
margins = [0.10 0.15 0.20];
f  = newFig(16, 10);
tl = tiledlayout(f, 1, 1, 'Padding', 'compact');
ax = newAx(tl);
rows = {};
for i = 1:numel(margins)
    [t, N] = getSig(runCase(S, 60, 40, {'m_acc', margins(i)}), 'N');
    plot(ax, t - S.tStep, N, 'Color', C.s{i}, 'LineWidth', 1.6, ...
         'DisplayName', sprintf('margin %.2f', margins(i)));
    [tA, os] = accelMetrics(t, N, S);
    rows(end+1, :) = {margins(i), r2(tA), r2(os)}; %#ok<AGROW>
end
yline(ax, 98.3, ':', '95% thrust', 'Color', C.muted, 'HandleVisibility', 'off', ...
      'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
xline(ax, 5, '--', '5 s target', 'Color', C.muted, 'HandleVisibility', 'off', ...
      'LabelVerticalAlignment', 'bottom');
xlim(ax, [-1 S.tStop - S.tStep]);  ylim(ax, [55 103]);
xlabel(ax, 'Time after demand step [s]');  ylabel(ax, 'Shaft speed N [%]');
legend(ax, 'Location', 'southeast');
title(tl, 'Idle \rightarrow take-off: effect of the acceleration margin (W_{f,max} = W_{f,ss}(N) + margin)');
saveFig(S, f, fullfile(S.extraDir, 'margin_study.png'));

writeResults(S, rows, {'Accel_margin', 'Accel_t95_s', 'Overshoot_pct'}, 'margin_study.csv');
end


function extra_robustness(S)
% Every case: idle -> take-off, take-off -> idle and a 2% step at idle.
P = S.P;  C = S.C;
cases = {
    'Nominal',                     {}
    'Spool 30% heavier (k -30%)',  {'k_true', 0.7*P.k}
    'Spool 30% lighter (k +30%)',  {'k_true', 1.3*P.k}
    'Fuel valve lag 0.2 s',        {'tau_valve', 0.2}
    'Sensor noise (power 1e-4)',   {'noise_power', 1e-4}
    };
nPlot = 3;                                  % only the k cases are plotted

f  = newFig(16, 14);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = newAx(tl);
ax2 = newAx(tl);
rows = {};
for i = 1:size(cases, 1)
    v = cases{i,2};
    [tA, NA] = getSig(runCase(S, 60,  40, v), 'N');
    [tD, ND] = getSig(runCase(S, 100, -40, v), 'N');
    [tS, NS] = getSig(runCase(S, 60,   2, v), 'N');
    if i <= nPlot
        plot(ax1, tA - S.tStep, NA, 'Color', C.s{i}, 'LineWidth', 1.6, 'DisplayName', cases{i,1});
        plot(ax2, tD - S.tStep, ND, 'Color', C.s{i}, 'LineWidth', 1.6, 'DisplayName', cases{i,1});
    end
    [ta, os] = accelMetrics(tA, NA, S);
    [td, us] = decelMetrics(tD, ND, S);
    osS      = smallStepMetrics(tS, NS, S, 60, 2);
    rows(end+1, :) = {cases{i,1}, r2(ta), r2(os), r2(td), r2(us), r2(osS)}; %#ok<AGROW>
end
yline(ax1, 98.3, ':', '95% thrust', 'Color', C.muted, 'HandleVisibility', 'off', ...
      'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
xline(ax1, 5, '--', '5 s target', 'Color', C.muted, 'HandleVisibility', 'off', ...
      'LabelVerticalAlignment', 'bottom');
ylabel(ax1, 'Shaft speed N [%]');
title(ax1, 'Idle \rightarrow take-off', 'FontWeight', 'normal');
legend(ax1, 'Location', 'southeast');

ylabel(ax2, 'Shaft speed N [%]');
xlabel(ax2, 'Time after demand step [s]');
title(ax2, 'Take-off \rightarrow idle', 'FontWeight', 'normal');
legend(ax2, 'Location', 'northeast');
linkaxes([ax1 ax2], 'x');
xlim(ax1, [-1 S.tStop - S.tStep]);
title(tl, 'Robustness to engine variation (controller not re-tuned)');
saveFig(S, f, fullfile(S.extraDir, 'robustness_k.png'));

writeResults(S, rows, {'Case', 'Accel_t95_s', 'Accel_overshoot_pct', 'Decel_t_s', ...
                       'Decel_undershoot_pct', 'SmallStep_overshoot_pct_of_step'}, 'robustness.csv');
end


function extra_sensorNoise(S)
C = S.C;
out = runCase(S, 60, 40, {'noise_power', 1e-4});
[t,  N ] = getSig(out, 'N');
[tm, Nm] = getSig(out, 'N_meas');
[tc, Wc] = getSig(out, 'Wf_cmd');
[tw, W ] = getSig(out, 'Wf');
win = [8 18];                       % steady take-off, after the transient [s after step]

f  = newFig(16, 12);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = newAx(tl);
plot(ax1, tm - S.tStep, Nm, 'Color', C.muted, 'LineWidth', 0.7, 'DisplayName', 'Measured N (to controller)');
plot(ax1, t  - S.tStep, N,  'Color', C.s{1},  'LineWidth', 1.8, 'DisplayName', 'True N');
ylim(ax1, [99.5 100.5]);
ylabel(ax1, 'Shaft speed N [%]');
legend(ax1, 'Location', 'northeast');

ax2 = newAx(tl);
plot(ax2, tc - S.tStep, Wc, 'Color', C.muted, 'LineWidth', 0.7, 'DisplayName', 'W_{f,cmd} (command)');
plot(ax2, tw - S.tStep, W,  'Color', C.s{2},  'LineWidth', 1.8, 'DisplayName', 'W_f (delivered by valve)');
ylim(ax2, [0.98 1.02]);
ylabel(ax2, 'Fuel flow W_f [-]');
xlabel(ax2, 'Time after demand step [s]');
legend(ax2, 'Location', 'northeast');
linkaxes([ax1 ax2], 'x');
xlim(ax1, win);
title(tl, 'Sensor noise at take-off: the noise reaches the fuel command, not the shaft speed');
saveFig(S, f, fullfile(S.extraDir, 'sensor_noise.png'));

inWin = @(tt) tt - S.tStep >= win(1) & tt - S.tStep <= win(2);
fprintf('  std over %g-%g s: measured N %.3f%%, true N %.4f%%, Wf_cmd %.4f, Wf %.4f\n', ...
        win, std(Nm(inWin(tm))), std(N(inWin(t))), std(Wc(inWin(tc))), std(W(inWin(tw))));
end


%% ======================================================================
%  Simulation helpers
%  ======================================================================

function out = runCase(S, N0, dNd, vars, blocks)
% One simulation of the demand step N0 -> N0 + dNd.
%   vars   = {'name', value, ...}               temporary variable overrides
%   blocks = {blockPath, 'Param', 'value'; ...} temporary block-parameter overrides
if nargin < 4, vars   = {}; end
if nargin < 5, blocks = {}; end

% 1) nominal values for this operating point (parameters.m keeps N0/dNd and
%    recomputes Wf0 etc.), 2) then the overrides on top
evalin('base', sprintf('N0 = %.15g; dNd = %.15g;', N0, dNd));
evalin('base', sprintf('run(''%s'');', fullfile(S.root, 'parameters.m')));
for i = 1:2:numel(vars)
    assignin('base', vars{i}, vars{i+1});
end

in = Simulink.SimulationInput(S.model);
in = in.setModelParameter('StopTime', num2str(S.tStop));
for i = 1:size(blocks, 1)
    in = in.setBlockParameter(blocks{i,1}, blocks{i,2}, blocks{i,3});
end
out = sim(in);
if ~isempty(out.ErrorMessage), error(out.ErrorMessage); end
end


function [t, y] = getSig(out, name)
% Time and data of a logged signal (first match if it is logged in two places).
el = out.logsout.get(name);
if isempty(el)
    logged = out.logsout.getElementNames();
    error('Signal "%s" is not logged. Logged signals: %s', name, strjoin(logged(:).', ', '));
end
if isa(el, 'Simulink.SimulationData.Dataset'), el = el.getElement(1); end
t = el.Values.Time;
y = squeeze(el.Values.Data);
end


function [tA, os] = accelMetrics(t, N, S)
% Time from the demand step to 95% thrust (N = 98.3%, since F ~ N^3),
% and overshoot above 100% [% speed].
i  = find(t >= S.tStep & N >= 98.3, 1);
tA = NaN;  if ~isempty(i), tA = t(i) - S.tStep; end
os = max(max(N) - 100, 0);
end


function [tD, us] = decelMetrics(t, N, S)
% Time from the demand step until N <= 60 + band, and undershoot below 60% [% speed].
after = t >= S.tStep;
i  = find(after & N <= 60 + S.band, 1);
tD = NaN;  if ~isempty(i), tD = t(i) - S.tStep; end
us = max(60 - min(N(after)), 0);
end


function [osPct, t63] = smallStepMetrics(t, N, S, N0, dNd)
% Overshoot as % of the step size, and time to 63% of the step.
after = t >= S.tStep;
y     = (N - N0)/dNd;
osPct = max(max(y(after)) - 1, 0)*100;
i     = find(after & y >= 1 - exp(-1), 1);
t63   = NaN;  if ~isempty(i), t63 = t(i) - S.tStep; end
end


function L = openLoop(P, N0)
% Linear loop gain at N0: scheduled PI x valve lag x engine x sensor lag,
% plus half a sample of delay for the discrete controller.
a   = (P.n/100)*(N0/100)^(P.n - 1);
tau = 1/(P.k*a);
K   = 1/a;
Ki  = P.c*P.Kp/tau;
s   = tf('s');
L   = (P.Kp + Ki/s) * 1/(P.tau_valve*s + 1) * K/(tau*s + 1) * 1/(P.tau_sensor*s + 1);
L.InputDelay = P.Ts/2;
end


function p = findPID(model)
p = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                'Regexp', 'on', 'MaskType', '^PID');
if isempty(p), error('No PID Controller block found in %s.', model); end
p = p{1};
end


function P = loadParameters(root)
% Runs parameters.m in this function's own workspace and returns its variables.
run(fullfile(root, 'parameters.m'));
names = setdiff(who, {'root'});
P = struct();
for i = 1:numel(names)
    P.(names{i}) = eval(names{i});
end
end


function restoreState(S, oldDir, oldInit, oldDirty)
if bdIsLoaded(S.model)
    set_param(S.model, 'InitFcn', oldInit);
    if strcmp(oldDirty, 'off'), set_param(S.model, 'Dirty', 'off'); end
end
evalin('base', 'clear N0 dNd');                               % back to the default test case
evalin('base', sprintf('run(''%s'');', fullfile(S.root, 'parameters.m')));
cd(oldDir);
end


%% ======================================================================
%  Plot helpers
%  ======================================================================

function f = plotProtection(S, out, isAccel, ttl)
% Fuel signals (top) and shaft speed (bottom) for a big demand step.
C = S.C;
[t1, Wp]   = getSig(out, 'Wf_PI');
[t2, Wmax] = getSig(out, 'Wf_max');
[t3, Wmin] = getSig(out, 'Wf_min');
[t4, Wc]   = getSig(out, 'Wf_cmd');
[t5, W]    = getSig(out, 'Wf');
[t6, Nd]   = getSig(out, 'N_demand');
[t7, N]    = getSig(out, 'N');
t0 = S.tStep;
if isAccel, loc = 'southeast'; else, loc = 'northeast'; end

f  = newFig(16, 14);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = newAx(tl);
plot(ax1, t2 - t0, Wmax, '--', 'Color', C.muted, 'LineWidth', 1.2, 'DisplayName', 'W_{f,max} (surge protection)');
plot(ax1, t3 - t0, Wmin, '-.', 'Color', C.muted, 'LineWidth', 1.2, 'DisplayName', 'W_{f,min} (flameout protection)');
plot(ax1, t1 - t0, Wp,   '-',  'Color', C.s{2},  'LineWidth', 1.3, 'DisplayName', 'W_{f,PI} (PI request)');
plot(ax1, t4 - t0, Wc,   '-',  'Color', C.s{1},  'LineWidth', 2.0, 'DisplayName', 'W_{f,cmd} (selected command)');
plot(ax1, t5 - t0, W,    ':',  'Color', C.ink,   'LineWidth', 1.4, 'DisplayName', 'W_f (delivered by valve)');
ylabel(ax1, 'Fuel flow W_f [-]');
legend(ax1, 'Location', loc);

ax2 = newAx(tl);
plot(ax2, t6 - t0, Nd, '--', 'Color', C.muted, 'LineWidth', 1.2, 'DisplayName', 'N_{demand}');
plot(ax2, t7 - t0, N,  '-',  'Color', C.s{1},  'LineWidth', 2.0, 'DisplayName', 'N');
if isAccel
    yline(ax2, 98.3, ':', '95% thrust', 'Color', C.muted, 'HandleVisibility', 'off', ...
          'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
else
    yline(ax2, 60 + S.band, ':', sprintf('within %g%%', S.band), 'Color', C.muted, ...
          'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
end
ylabel(ax2, 'Shaft speed N [%]');
xlabel(ax2, 'Time after demand step [s]');
legend(ax2, 'Location', loc);

linkaxes([ax1 ax2], 'x');
xlim(ax1, [-1 S.tStop - t0]);
title(tl, ttl);
end


function refLine(ax, xy, v, col)
% Plain reference line without a label or legend entry.
if xy == 'x', h = xline(ax, v); else, h = yline(ax, v); end
h.Color = col;
h.HandleVisibility = 'off';
end


function f = newFig(w, h)
% White figure, w x h centimetres.
f = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 w h]);
try, theme(f, 'light'); catch, end           % R2025a+: force the light theme
end


function ax = newAx(tl)
ax = nexttile(tl);
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'off');
ax.GridAlpha = 0.12;
ax.FontSize  = 9;
end


function C = colours()
hex     = @(h) sscanf(h(2:end), '%2x').'/255;
C.s     = {hex('#2a78d6'), hex('#eb6834'), hex('#1baf7a')};   % series 1-3: blue, orange, aqua
C.ink   = hex('#0b0b0b');                                     % near-black
C.muted = hex('#6f6e69');                                     % limits and reference lines
end


function saveFig(S, f, file)
exportgraphics(f, file, 'Resolution', 300);
fprintf('  saved %s\n', file);
if ~S.keepOpen, close(f); end
end


function writeResults(S, rows, names, file)
T = cell2table(rows, 'VariableNames', names);
disp(T);
writetable(T, fullfile(S.resDir, file));
end


function y = r2(x)
y = round(x, 2);
end
