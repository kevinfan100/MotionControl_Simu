% PURPOSE (2026-09-23, user: "the sigma_6..sigma_1 curves, overlaid for the different speeds; and one whole-run figure with the
%   holds, to show where the constants' observability comes from"): the singular values of the whitened O ACCUMULATED step by
%   step (G(k) = sum_{j<=k} Phi' H' R^-1 H Phi from the window start), law A (delta, p), plane init, one seed.
%     figure 1  obs_ramp_sigma_curves_<wall>.png : 2 x 3 panels = sigma_1 .. sigma_6, x = commanded height (22.2 -> 1.1 R, log,
%               reversed), one curve per T_d (colour), accumulated from the descent start. Same height range for every T_d, so
%               the curves overlay: where along the descent each direction gains its strength, and how speed scales it.
%     figure 2  obs_ramp_sigma_wholerun_<wall>.png : one panel, T_d = 2 s, sigma_1..6 vs time accumulated from t = 0 across
%               hold0 - descent - hold, curves numbered 1..6; the constants' curves (5, 6) are flat in both holds.
%               The simpler two-wall contrast in the house style is plot_obs_ramp_hold_contrast.m (reads this cache).
%   Default TD = the agreed three speeds 0.25 / 2 / 16 s (user 09-23); the first pass used seven and T_d = 1 s for figure 2.
%     figure 3  obs_ramp_sigma_ratio_<wall>.png : one panel, sigma_6 / sigma_1 (min / max of the whitened O) accumulated
%               from the descent start vs commanded height, one curve per T_d (user 09-23: "最大最小的比值的疊圖").
%   The accumulated sigma_i per T_d are cached in test_results/temp_plaw_walls/obs_ramp_sigma_curves_<wall>.mat;
%   plot_obs_ramp_sigma_curves(wall, 'plot') replots from the cache without rerunning the simulations.
%   Output reference/eq17_analysis/derivation/figures/  EXPIRES: with the T_d design line
function plot_obs_ramp_sigma_curves(wall, TD, seed)
    if nargin < 1 || isempty(wall); wall = 'plane'; end
    replot = nargin >= 2 && ischar(TD) && strcmp(TD, 'plot');
    if nargin < 2 || isempty(TD) || replot; TD = [0.25 2 16]; end
    if nargin < 3 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    cache_dir = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(cache_dir, 'dir'); mkdir(cache_dir); end
    cache = fullfile(cache_dir, sprintf('obs_ramp_sigma_curves_%s.mat', wall));
    pc = physical_constants();  Ts = pc.Ts;  T_HOLD = 0.5;  FREE = [1 2 3 4 5 7];  STRIDE = 4;
    plant = [];  if strcmp(wall, 'cell'); plant = @cell_cperp_delta; elseif strcmp(wall, 'sphere_pt'); plant = @sphere_cperp_fig11_ptail; end
    WLAB = struct('plane', 'PLANE', 'cell', 'CELL', 'sphere_pt', 'SPHERE (plane tail)');
    FS = 14; LFS = 10;  ctd = turbo(numel(TD) + 2);  ctd = ctd(2:end-1, :);
    % ---------- accumulate (or load) sigma_i along the descent for every T_d, plus the whole run at T_d = 1 s
    if replot
        S = load(cache);  TD = S.TD;  SIGc = S.SIGc;  HKc = S.HKc;  SIGw = S.SIGw;  TKw = S.TKw;  Td_w = S.Td_w;
        ctd = turbo(numel(TD) + 2);  ctd = ctd(2:end-1, :);
    else
        SIGc = cell(numel(TD), 1);  HKc = cell(numel(TD), 1);
        for it = 1:numel(TD)
            Td = TD(it);
            [Lz, wd] = local_run(root, wall, plant, Td, T_HOLD, seed);
            t = [Lz.k]' * Ts;  idx = find(t >= T_HOLD & t < T_HOLD + Td);
            [SIGc{it}, HKc{it}] = local_accum_sigma(Lz, idx, FREE, wd, STRIDE);
        end
        Td_w = 2.0;  [Lz, wd] = local_run(root, wall, plant, Td_w, T_HOLD, seed);
        t = [Lz.k]' * Ts;  idx = (1:numel(Lz))';
        [SIGw, HKw, TKw] = local_accum_sigma(Lz, idx, FREE, wd, STRIDE, t);
        save(cache, 'TD', 'SIGc', 'HKc', 'SIGw', 'HKw', 'TKw', 'Td_w', 'wall', 'seed', 'FREE', 'STRIDE', 'T_HOLD');
        fprintf('cache: %s\n', cache);
    end
    if replot && ~isfield(S, 'HKw')                     % older cache without the whole-run commanded height: redo that one run
        [Lz, wd] = local_run(root, wall, plant, Td_w, T_HOLD, seed);
        t = [Lz.k]' * Ts;  idx = (1:numel(Lz))';
        [SIGw, HKw, TKw] = local_accum_sigma(Lz, idx, FREE, wd, STRIDE, t);
        S.SIGw = SIGw;  S.HKw = HKw;  S.TKw = TKw;  S.Td_w = Td_w;  save(cache, '-struct', 'S');  fprintf('cache updated: %s\n', cache);
    end
    % ---------- figure 1: accumulated sigma_i along the descent, one curve per T_d
    fig = figure('Position', [60 60 1600 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(1, 6);  for i = 1:6; ax(i) = nexttile(i); hold on; box on; end
    hcurves = gobjects(numel(TD), 1);
    for it = 1:numel(TD)
        SIGk = SIGc{it};  hk = HKc{it};
        for i = 1:6
            h = plot(ax(i), hk, SIGk(i, :), '-', 'Color', ctd(it, :), 'LineWidth', 1.6);
            if i == 1; hcurves(it) = h; set(h, 'DisplayName', sprintf('T_d = %g s', TD(it))); end
        end
    end
    for i = 1:6
        set(ax(i), 'XScale', 'log', 'YScale', 'log', 'XDir', 'reverse');  xlim(ax(i), [1.0 25]);
        text(ax(i), 0.05, 0.92, sprintf('\\sigma_%d', i), 'Units', 'normalized', 'FontSize', 18, 'FontWeight', 'bold');
        if i > 3; xlabel(ax(i), 'commanded height  w_d  (R)', 'FontSize', FS, 'FontWeight', 'bold'); end
        if i == 1 || i == 4; ylabel(ax(i), '\sigma_i  accumulated from the descent start', 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax(i), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Box', 'on'); grid(ax(i), 'off');
    end
    lg = legend(ax(1), hcurves, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 7);
    lg.Layout.Tile = 'north';
    h0 = plot(ax(2), NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, A (\\delta, p), plane init; states 1-3 dw, 4 a_w, 5 p, 6 delta ordered by sigma at each step', WLAB.(wall)));
    fn = fullfile(od, sprintf('obs_ramp_sigma_curves_%s.png', wall));  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
    % ---------- figure 3: sigma_6 / sigma_1 (min / max) along the descent, one curve per T_d
    fig = figure('Position', [60 60 1100 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  hold on; box on;
    hr = gobjects(numel(TD), 1);
    for it = 1:numel(TD)
        SIGk = SIGc{it};  hk = HKc{it};
        hr(it) = plot(hk, SIGk(6, :) ./ SIGk(1, :), '-', 'Color', ctd(it, :), 'LineWidth', 1.8, 'DisplayName', sprintf('T_d = %g s', TD(it)));
    end
    set(gca, 'XScale', 'log', 'YScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([1e-8 1]);
    legend(hr, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 7);
    xlabel('commanded height  w_d  (R)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('\sigma_6 / \sigma_1   (min / max, accumulated from the descent start)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Box', 'on'); grid off;
    fn = fullfile(od, sprintf('obs_ramp_sigma_ratio_%s.png', wall));  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
    % ---------- figure 2: whole run at T_d = 1 s, accumulated from t = 0, holds marked
    Td = Td_w;  SIGk = SIGw;  tk = TKw;
    fig = figure('Position', [60 60 1100 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  hold on; box on;
    cols = lines(6);
    for i = 1:6
        plot(tk, max(SIGk(i, :), 1e-3), '-', 'Color', cols(i, :), 'LineWidth', 1.8);
        text(tk(end) * 1.01, max(SIGk(i, end), 1e-3), sprintf('%d', i), 'Color', cols(i, :), 'FontSize', 14, 'FontWeight', 'bold');
    end
    set(gca, 'YScale', 'log');  ylim([1 2e4]);  xlim([0 tk(end) * 1.06]);
    xline(T_HOLD, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);  xline(T_HOLD + Td, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
    text(T_HOLD / 2, 1.5, 'hold', 'HorizontalAlignment', 'center', 'FontSize', FS, 'FontWeight', 'bold', 'Color', [0.4 0.4 0.4]);
    text(T_HOLD + Td / 2, 1.5, 'constant-velocity descent', 'HorizontalAlignment', 'center', 'FontSize', FS, 'FontWeight', 'bold', 'Color', [0.4 0.4 0.4]);
    text(T_HOLD + Td + 0.25, 1.5, 'hold', 'HorizontalAlignment', 'center', 'FontSize', FS, 'FontWeight', 'bold', 'Color', [0.4 0.4 0.4]);
    h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s wall, A (\\delta, p), plane init, T_d = %g s: \\sigma_1..\\sigma_6 accumulated from t = 0 (whitened O of the free slots)', WLAB.(wall), Td));
    legend(h0, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel('\sigma_i', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Box', 'on'); grid off;
    fn = fullfile(od, sprintf('obs_ramp_sigma_wholerun_%s.png', wall));  exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
end

function [Lz, wd] = local_run(root, wall, plant, Td, T_HOLD, seed)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-0.9, 3], ...
                'plaw_dp_slope', true, 'p_init', 1.950, 'Pf_p_std', 0.10, 'plaw_p_lim', [1.2, 4], 'delta_init', 0.059, 'Pf_delta_std', 0.64, 'obs_dump', true);
    OV = struct('trajectory_type', 'ramp_descent', 't_hold', T_HOLD, 't_descend_override', Td, 'T_sim', T_HOLD + Td + 0.5, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1);
    o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', seed, 'log_P_full', false);
    if ~isempty(plant); o.plant_cperp = plant; end
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');  L = obs_dump('get');  Lz = L([L.ax] == 3);  wd = R.runs{1}.h_bar_d_out(:);
    if nargin < 6; end
end

function [SIGk, hk, tk] = local_accum_sigma(Lz, idx, FREE, wd, STRIDE, t)
% sigma_i of the whitened O accumulated from idx(1) to each step (every STRIDE steps); hk = commanded height at that step
    n = 7;  G = zeros(n);  Phi = eye(n);  m = numel(idx);  ks = 1:STRIDE:m;
    SIGk = zeros(6, numel(ks));  hk = zeros(1, numel(ks));  tk = zeros(1, numel(ks));  c = 0;
    for j = 1:m
        i = idx(j);
        H1 = Lz(i).H{1};  H2 = Lz(i).H{2};  R1 = Lz(i).R(1);  R2 = Lz(i).R(2);
        r1 = H1 * Phi;  G = G + (r1' * r1) / R1;
        if ~isempty(H2) && R2 > 0; r2 = H2 * Phi;  G = G + (r2' * r2) / R2; end
        Phi = Lz(i).F * Phi;
        if any(j == ks)
            c = c + 1;  Gf = G(FREE, FREE);  d = sort(eig((Gf + Gf') / 2), 'descend');  SIGk(:, c) = sqrt(max(d, 0));
            k = Lz(i).k + 1;  hk(c) = wd(min(max(k, 1), numel(wd)));  if nargin >= 6; tk(c) = t(i); end
        end
    end
end
