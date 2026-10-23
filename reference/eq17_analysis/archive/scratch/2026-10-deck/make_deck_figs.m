% STATUS: SCRATCH -- 2026-10-20 (user): figures for the "Estimators and Regulation" slide deck (Interview Slides colours:
%   true = ink #15222c, estimate = signal #14607f, guide lines = ink-muted). One idea per figure, large text for a 1920x1080 slide.
%   Data (all existing runs, seed 1): constant-velocity trajectory 22.2 -> 1.10 R in 1 s, held 0.2 s, plane and cell walls
%   (apff_off_plane_s1 = first-order estimator; apcmp_<arm>_<wall>_desc: apP002F400H110 = a' as a state variable, conly, s8fc = w_s);
%   regulation: gainreg_plane_tau100_T0.5_alWS / T0.00875_alAP (a law, tau 0.1 s, no cap), alaw_tau2_long_{ws,ap}_T40 (tau 2 s).
%   Output test_results/deck_figs/deck_<name>.png.
function make_deck_figs()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  out = fullfile(root, 'test_results', 'deck_figs');  if ~exist(out, 'dir'); mkdir(out); end
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    INK = [21 34 44] / 255;  SIG = [20 96 127] / 255;  MUT = [78 93 104] / 255;  BG = [244 246 247] / 255;  AB = char(257);  HAT = char(770);
    FS = 26;  LW_T = 5;  LW_E = 2.6;

    % 1a position-dependent motion gain: a-bar = 1/C_perp(h/R) against h/R, nominal value dashed
    hh = logspace(log10(1.05), log10(22.2), 400);  [f, ax] = local_fig(1, 1, BG);  hold(ax, 'on');
    plot(ax, hh, local_a(hh, 'plane'), '-', 'Color', INK, 'LineWidth', LW_T, 'DisplayName', [AB '_z true']);
    plot(ax, hh, ones(size(hh)), '--', 'Color', SIG, 'LineWidth', LW_E + 1, 'DisplayName', 'nominal value');
    set(ax, 'XScale', 'log', 'XTick', [1.1 2 5 10 20]);  local_ax(ax, FS, [1.05 22.2], [0 1.15], 'h / R', [AB '_z']);  local_save(f, out, 'l1_gain_h');

    % 1b control law with the nominal motion gain vs real-time estimation: z_d and z near the wall, delta z_m
    F = load(fullfile(od, 'apff_off_plane_s1.mat'));  Nm = load(fullfile(od, 'apff_nominal_plane_s1.mat'));
    [f, axs] = local_fig(2, 2, BG);  f.Position(3:4) = [1150 860];  RR = {Nm, F};  NM2 = {'nominal motion gain', 'real-time estimation'};
    for i = 1:2
        S = RR{i};  ax = axs(i);  hold(ax, 'on');
        plot(ax, S.t, S.h_d * pc.R, '-', 'Color', INK, 'LineWidth', LW_T, 'DisplayName', 'z_d');
        plot(ax, S.t, S.h * pc.R, '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', 'z');
        yl = '';  if i == 1; yl = ['z  (' char(181) 'm)']; end
        local_ax(ax, FS, [0.85 1.2], [2 8], 'Time (s)', yl);  title(ax, NM2{i}, 'FontSize', FS, 'FontWeight', 'normal', 'Color', MUT);
        ax = axs(i + 2);  hold(ax, 'on');
        plot(ax, S.t, S.dxm, '-', 'Color', SIG, 'LineWidth', 1.2, 'DisplayName', [char(948) 'z_m']);
        yl = '';  if i == 1; yl = [char(948) 'z_m  (' char(181) 'm)']; end
        local_ax(ax, FS, [0 1.2], [-0.35 0.2], 'Time (s)', yl);  legend(ax, 'off');
    end
    local_save(f, out, 'l1_track');

    % 2 autoregressive model of the motion gain: a-hat lags the motion gain along the trajectory; estimation error below
    [f, axs] = local_fig(2, 1, BG);  f.Position(3:4) = [1500 940];  hold(axs(1), 'on');
    plot(axs(1), F.t, F.a_true, '-', 'Color', INK, 'LineWidth', LW_T, 'DisplayName', [AB '_z true']);
    plot(axs(1), F.t, F.a_hat, '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', [AB HAT '_z']);
    local_ax(axs(1), FS, [0 1.2], [0 1.15], '', [AB '_z']);
    hold(axs(2), 'on');  yline(axs(2), 0, '-', 'Color', MUT, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(axs(2), F.t, F.a_true - F.a_hat, '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', ['e_{' AB '} = ' AB '_z - ' AB HAT '_z']);
    local_ax(axs(2), FS, [0 1.2], [-0.18 0.08], 'Time (s)', ['e_{' AB '}']);  local_save(f, out, 'l2_ahat');

    % 3a a' as a state variable vs the autoregressive model: estimation error, same trajectory, seed 1
    A = load(fullfile(od, 'apcmp_apP002F400H110_plane_desc.mat'));
    [f, ax] = local_fig(1, 1, BG);  f.Position(3:4) = [1500 640];  hold(ax, 'on');
    yline(ax, 0, '-', 'Color', MUT, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(ax, F.t, F.a_true - F.a_hat, '-', 'Color', MUT, 'LineWidth', LW_E, 'DisplayName', 'autoregressive model');
    plot(ax, A.t, A.a_true(:, 1) - A.a_hat(:, 1), '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', [AB ''' as a state variable']);
    local_ax(ax, FS, [0 1.2], [-0.18 0.08], 'Time (s)', ['e_{' AB '}']);  local_save(f, out, 'l3_err');

    % 3b estimating a': true a' and its estimate (log)
    [f, ax] = local_fig(1, 1, BG);  f.Position(3:4) = [1500 640];  hold(ax, 'on');
    apt = local_ap(A.h_true(:, 1), 'plane');  y = A.ap_hat(:, 1) / a_nom;  y(y <= 0) = NaN;
    plot(ax, A.t, apt, '-', 'Color', INK, 'LineWidth', LW_T, 'DisplayName', [AB '''_z true']);
    plot(ax, A.t, y, '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', [AB HAT '''_z']);
    set(ax, 'YScale', 'log');  local_ax(ax, FS, [0 1.2], [1e-3 2], 'Time (s)', [AB '''_z']);  local_save(f, out, 'l3_ap');

    % 4a c-only (q on c off) vs a' as a state variable, plane, estimation error, seed 1
    C = load(fullfile(od, 'apcmp_conlyQ0_plane_desc.mat'));  Cc = load(fullfile(od, 'apcmp_conlyQ0_cell_desc.mat'));
    [f, ax] = local_fig(1, 1, BG);  f.Position(3:4) = [1500 640];  hold(ax, 'on');
    yline(ax, 0, '-', 'Color', MUT, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(ax, A.t, A.a_true(:, 1) - A.a_hat(:, 1), '-', 'Color', MUT, 'LineWidth', LW_E, 'DisplayName', [AB ''' as a state variable']);
    plot(ax, C.t, C.a_true(:, 1) - C.a_hat(:, 1), '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', 'c-only');
    local_ax(ax, FS, [0 1.2], [-0.18 0.08], 'Time (s)', ['e_{' AB '}']);  local_save(f, out, 'l4_conly');

    % 4b c-only (q on c off): plane vs cell wall, estimator starts from the plane, seed 1
    [f, ax] = local_fig(1, 1, BG);  f.Position(3:4) = [1500 640];  hold(ax, 'on');
    yline(ax, 0, '-', 'Color', MUT, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(ax, C.t, C.a_true(:, 1) - C.a_hat(:, 1), '-', 'Color', MUT, 'LineWidth', LW_E, 'DisplayName', 'plane wall');
    plot(ax, Cc.t, Cc.a_true(:, 1) - Cc.a_hat(:, 1), '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', 'cell wall');
    local_ax(ax, FS, [0 1.2], [-0.08 0.18], 'Time (s)', ['e_{' AB '}']);  local_save(f, out, 'l4_conly_cell');

    % 5 w_s as a state variable vs c-only (q on c off): plane | cell, estimation error, seed 1
    [f, axs] = local_fig(1, 2, BG);  f.Position(3:4) = [1700 640];  W = {'plane', 'cell'};  CC = {C, Cc};
    for iw = 1:2
        S = load(fullfile(od, sprintf('apcmp_s8fc_%s_desc.mat', W{iw})));  ax = axs(iw);  hold(ax, 'on');
        yline(ax, 0, '-', 'Color', MUT, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        plot(ax, CC{iw}.t, CC{iw}.a_true(:, 1) - CC{iw}.a_hat(:, 1), '-', 'Color', MUT, 'LineWidth', LW_E, 'DisplayName', 'c-only');
        plot(ax, S.t, S.a_true(:, 1) - S.a_hat(:, 1), '-', 'Color', SIG, 'LineWidth', LW_E, 'DisplayName', ['w' char(772) '_s as a state variable']);
        yl = '';  if iw == 1; yl = ['e_{' AB '}']; end
        local_ax(ax, FS, [0 1.2], [-0.08 0.14], 'Time (s)', yl);  title(ax, [W{iw} ' wall'], 'FontSize', FS, 'FontWeight', 'normal', 'Color', MUT);
    end
    local_save(f, out, 'l5_ws');

    % 6 contact with the wall, tau 0.1 s: z_d and z, w_s | a' state, first 12 ms
    Wr = load(fullfile(od, 'gainreg_plane_tau100_T0.5_alWS.mat'));  Ar = load(fullfile(od, 'gainreg_plane_tau100_T0.00875_alAP.mat'));
    [f, axs] = local_fig(1, 2, BG);  RR = {Wr, Ar};  NM = {['w' char(772) '_s as a state variable'], [AB ''' as a state variable']};
    for i = 1:2
        S = RR{i};  k = 2:min(numel(S.t), 20);  hold(axs(i), 'on');
        plot(axs(i), 1000 * S.t(k), S.wd(k) * pc.R, 'o-', 'Color', INK, 'LineWidth', 3.5, 'MarkerSize', 7, 'MarkerFaceColor', INK, 'DisplayName', 'z_d');
        plot(axs(i), 1000 * S.t(k), S.h_true(k) * pc.R, 'o--', 'Color', SIG, 'LineWidth', 2.4, 'MarkerSize', 7, 'MarkerFaceColor', SIG, 'DisplayName', 'z');
        yline(axs(i), pc.R, '-', 'Color', MUT, 'LineWidth', 3, 'HandleVisibility', 'off');
        if i == 2
            tc = 1000 * (S.t(end) + pc.Ts);  plot(axs(i), [1000 * S.t(end) tc], [S.h_true(end) * pc.R, pc.R], ':', 'Color', SIG, 'LineWidth', 2.4, 'HandleVisibility', 'off');
            plot(axs(i), tc, pc.R, 'x', 'Color', SIG, 'MarkerSize', 26, 'LineWidth', 5, 'DisplayName', 'contact');
        end
        yl = '';  if i == 1; yl = ['z  (' char(181) 'm)']; end
        local_ax(axs(i), FS, [0 12], [0 55], 'Time (ms)', yl);  title(axs(i), NM{i}, 'FontSize', FS, 'FontWeight', 'normal', 'Color', MUT);
    end
    local_save(f, out, 'reg_contact');

    % 7 tau 2 s: |a - a_d| against e^(-t/tau), w_s | a' state
    T2 = {load(fullfile(od, 'alaw_tau2_long_ws_T40.mat')), load(fullfile(od, 'alaw_tau2_long_ap_T40.mat'))};
    [f, axs] = local_fig(1, 2, BG);
    for i = 1:2
        S = T2{i};  t = S.t;  n01 = round(0.1 / (t(2) - t(1)));  e = abs(movmean(S.a_true(:, 1), n01) - 0.07);  hold(axs(i), 'on');
        plot(axs(i), t, e(1) * exp(-t / 2), '--', 'Color', MUT, 'LineWidth', 3, 'DisplayName', 'e^{-t/\tau_a}');
        plot(axs(i), t, e, '-', 'Color', SIG, 'LineWidth', LW_E + 0.6, 'DisplayName', ['|' AB '_z - ' AB '_d|']);
        set(axs(i), 'YScale', 'log');  yl = '';  if i == 1; yl = ['|' AB '_z - ' AB '_d|']; end
        local_ax(axs(i), FS, [0 20], [1e-3 1], 'Time (s)', yl);  title(axs(i), NM{i}, 'FontSize', FS, 'FontWeight', 'normal', 'Color', MUT);
    end
    local_save(f, out, 'reg_tau2');
    % 8 long hold, tau 2 s, plane: estimation error (5-seed mean, 1 s moving mean), w_s drifts, a' state does not
    [f, ax] = local_fig(1, 1, BG);  f.Position(3:4) = [1500 640];  hold(ax, 'on');
    yline(ax, 0, '-', 'Color', MUT, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    NM8 = {['w' char(772) '_s as a state variable'], [AB ''' as a state variable']};  CL8 = {SIG, MUT};
    for i = 1:2
        S = T2{i};  n1 = round(1 / (S.t(2) - S.t(1)));  e = movmean(mean(S.a_true - S.a_hat, 2), n1);
        plot(ax, S.t, e, '-', 'Color', CL8{i}, 'LineWidth', LW_E + 0.6, 'DisplayName', NM8{i});
    end
    local_ax(ax, FS, [5 40], [-0.008 0.004], 'Time (s)', ['e_{' AB '}']);  local_save(f, out, 'reg_drift');
    fprintf('figures in %s\n', out);
end

function [f, ax] = local_fig(nr, nc, BG)
    f = figure('Position', [40 40 1000 * (nc == 1) + 1500 * (nc == 2), 640 + 380 * (nr == 2)], 'Color', BG, 'Visible', 'off');
    tl = tiledlayout(f, nr, nc, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(nr * nc, 1);
    for i = 1:nr * nc; ax(i) = nexttile(tl, i);  set(ax(i), 'Color', BG); end
end

function local_ax(ax, FS, xl, yl, xlab, ylab)
    xlim(ax, xl);  ylim(ax, yl);  box(ax, 'on');  grid(ax, 'off');
    set(ax, 'FontSize', FS, 'LineWidth', 2, 'XColor', [21 34 44] / 255, 'YColor', [21 34 44] / 255, 'FontName', 'Helvetica');
    if ~isempty(xlab); xlabel(ax, xlab, 'FontSize', FS); end
    if ~isempty(ylab); ylabel(ax, ylab, 'FontSize', FS); end
    legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 2, 'Box', 'off');
end

function local_save(f, out, name)
    exportgraphics(f, fullfile(out, ['deck_' name '.png']), 'Resolution', 192, 'BackgroundColor', 'current');  close(f);
end

function ap = local_ap(h, wall)
    d = 1e-4;  h = max(h, 1.001);  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
