function plot_var_ahat_6state(freq, opts)
%PLOT_VAR_AHAT_6STATE  Visualize Var(a_hat) from gain_compare data (read-only).
%
%   plot_var_ahat_6state(freq)
%   plot_var_ahat_6state(freq, opts)
%
%   Reads test_results/gain_compare/f<freq>Hz/runs.mat WITHOUT touching the
%   production analyze_gain_6state pipeline, and writes two figures into the
%   same folder (test_results/ is gitignored):
%
%     fig_var_ahat_abs.png    a_hat ensemble mean +/- across-seed sigma band
%                             (band half-width = sqrt(Var(a_hat)) pointwise)
%                             overlaid on a_true, x (top) / z (bottom).
%     fig_var_ahat_relerr.png relative error (a_hat - a_true)/a_true [%]
%                             decomposed into bias (across-seed mean) and a
%                             +/- spread band (across-seed sigma), x / z.
%
%   This realizes the gain_compare_findings.md section 6 TODO: the per-seed
%   SCATTER (= Var(a_hat)) that the ensemble-mean fig_gain_compare averages
%   away. Var(a_hat) is the across-seed variance at FIXED time: a_true is
%   common-mode across seeds (same commanded trajectory), so the spread
%   isolates the estimation error, not the commanded a_true sweep. std() uses
%   the unbiased (N-1) normalization, so no ensemble-mean deflation applies.
%
%   opts (all optional):
%     arm        'B' (a=a_hat, deployed estimator; DEFAULT) | 'A' (a=a_true clean)
%     data_root  source dir for f<freq>Hz/runs.mat (default test_results/gain_compare)
%     runs, cfg  preloaded structs -> skip the disk load (fast iteration)
%     save_fig   true (default)
%
%   See also: analyze_gain_6state, compare_gain_6state

    if nargin < 1 || isempty(freq); freq = 1; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'arm');      opts.arm = 'B';      end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    if ~isfield(opts, 'data_root')
        opts.data_root = fullfile(project_root, 'test_results', 'gain_compare');
    end
    out_dir = fullfile(opts.data_root, sprintf('f%gHz', freq));

    % --- load (or accept preloaded) ---
    if isfield(opts, 'runs') && isfield(opts, 'cfg')
        runs = opts.runs; cfg = opts.cfg;
    else
        S = load(fullfile(out_dir, 'runs.mat'));   % runs, cfg, opts, ...
        runs = S.runs; cfg = S.cfg; clear S;
    end

    % --- physical constants + aligned time grid (e[k] = pd[k+1]-p[k] convention) ---
    P     = runs.A.det.simOut.meta.params_value;
    R     = P.common.R; w_hat = P.wall.w_hat; pz = P.wall.pz;
    t_e   = runs.A.det.simOut.tout(2:end);
    pd_al = runs.A.det.simOut.p_d_out(2:end, :);
    h_bar_d = (pd_al * w_hat - pz) / R;            % desired-trajectory h_bar

    % --- stack a_hat / a_true across non-diverged seeds of the chosen arm ---
    nz = runs.(opts.arm).noisy;
    ok = find(~[nz.diverged]);
    assert(~isempty(ok), 'plot_var_ahat_6state: all arm-%s noisy seeds diverged', opts.arm);
    ah_stk = []; at_stk = [];
    for s = ok
        ah_stk = cat(3, ah_stk, nz(s).simOut.diag.a_hat(2:end, :));   % [N-1 x 3]
        at_stk = cat(3, at_stk, nz(s).simOut.a_true_out(2:end, :));
    end
    Ns = numel(ok);

    ah_mean = mean(ah_stk, 3);            % [N-1 x 3]
    ah_std  = std(ah_stk, 0, 3);          % [N-1 x 3]  = sqrt(Var(a_hat)) pointwise
    at_mean = mean(at_stk, 3);
    erel    = (ah_stk - at_stk) ./ at_stk * 100;     % per-seed rel error [%]
    bias    = mean(erel, 3);                          % across-seed mean = bias(t)
    spread  = std(erel, 0, 3);                        % across-seed sigma  = spread(t)

    % --- console summary (near = Guard-3 region h_bar<1.5, far otherwise) ---
    gate = h_bar_d < 1.5;
    axn = 'xyz';
    for ax = [1 3]
        fprintf(['[var_ahat:%gHz arm=%s axis=%c] far(h>=1.5) bias=%+.1f%% ', ...
                 'spread=%.1f%% | near(h<1.5) bias=%+.1f%% spread=%.1f%%\n'], ...
                freq, opts.arm, axn(ax), mean(bias(~gate, ax)), mean(spread(~gate, ax)), ...
                mean(bias(gate, ax)), mean(spread(gate, ax)));
    end

    if ~opts.save_fig; return; end

    % --- shared EXP style (mirrors analyze_gain_6state make_figs) ---
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95];
    FS = 18; LFS = 13; AXLW = 2.0; cols = [1 3]; axl = 'xz'; T_END = ceil(t_e(end));
    tcol = t_e(:);

    % ===== FIG 1: absolute, a_hat mean +/- across-seed sigma band vs a_true =====
    f1 = figure('Position', [80 80 1100 720], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        c = cols(r); nexttile; hold on;
        yb = [ah_mean(:, c) - ah_std(:, c); flipud(ah_mean(:, c) + ah_std(:, c))];
        hb = fill([tcol; flipud(tcol)], yb, BANDC, 'FaceAlpha', 0.30, ...
                  'EdgeColor', 'none', 'DisplayName', '\^a \pm \sigma_{seed}');
        ht = plot(tcol, at_mean(:, c), '-', 'Color', COL_TRUE, 'LineWidth', 2.0, ...
                  'DisplayName', 'a_{true}');
        hh = plot(tcol, ah_mean(:, c), '-', 'Color', COL_HAT, 'LineWidth', 2.0, ...
                  'DisplayName', '\^a mean');
        xlim([0 T_END]); ylim([0, 1.25 * max(at_mean(:, c))]);
        ylabel(sprintf('a_%c  (\\mum/pN)', axl(r)), 'FontSize', FS, 'FontWeight', 'bold');
        if r == 1
            legend([hh ht hb], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    exportgraphics(f1, fullfile(out_dir, 'fig_var_ahat_abs.png'), 'Resolution', 150);
    close(f1);

    % ===== FIG 2: relative-error decomposition, bias +/- spread [%] =====
    f2 = figure('Position', [80 80 1100 720], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        c = cols(r); nexttile; hold on;
        yb = [bias(:, c) - spread(:, c); flipud(bias(:, c) + spread(:, c))];
        hb = fill([tcol; flipud(tcol)], yb, BANDC, 'FaceAlpha', 0.30, ...
                  'EdgeColor', 'none', 'DisplayName', '\pm spread (\sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        hbi = plot(tcol, bias(:, c), '-', 'Color', COL_HAT, 'LineWidth', 2.0, ...
                   'DisplayName', 'bias (mean)');
        xlim([0 T_END]);
        ylabel(sprintf('(\\^a_%c - a_{true})/a_{true}  (%%)', axl(r)), 'FontSize', FS, 'FontWeight', 'bold');
        if r == 1
            legend([hbi hb], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    exportgraphics(f2, fullfile(out_dir, 'fig_var_ahat_relerr.png'), 'Resolution', 150);
    close(f2);

    fprintf('[var_ahat:%gHz arm=%s] Ns=%d -> %s\n', freq, opts.arm, Ns, out_dir);
end
