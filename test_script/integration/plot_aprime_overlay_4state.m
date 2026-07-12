function plot_aprime_overlay_4state(opts)
%PLOT_APRIME_OVERLAY_4STATE Overlay figures: taylor 'known' vs 'diff' a' source.
%
%   plot_aprime_overlay_4state()
%   plot_aprime_overlay_4state(opts)
%
%   Runs the taylor-gain 4-state controller (4state_del_hd.tex taylor section) on the
%   standard 1 Hz osc scenario with two a' sources and per-seed overlay
%   figures in the selfrw-study style (z axis):
%     row 1: a_hat vs a_true vs a_det(h_bar_d)
%     row 2: a'_used vs a'_oracle(h_bar_d) = -a_det(h_bar_d)*K_h(h_bar_d)/R
%
%   Arms:
%     known : a' = -a_det(h_bar_d)*K_h(h_bar_d)/R   (fully known c(h_bar))
%     diff  : a' = per-step self-diff of a_hat, gate 1e-3 um + EWMA 0.05
%             (selfrw convention, chat 2026-07-07)
%
%   Output: test_results/aprime_4state/overlay/fig_overlay_<arm>.png
%
%   opts fields (all optional):
%     seeds (1:3) | T_sim (4.0) | clamp_pos (false, 'diff' a'>0 post-EWMA clamp)
%
%   See also: compare_aprime_4state, analyze_aprime_4state

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');     opts.seeds = 1:3;       end
    if ~isfield(opts, 'T_sim');     opts.T_sim = 4.0;       end
    if ~isfield(opts, 'clamp_pos'); opts.clamp_pos = false; end
    if ~isfield(opts, 'pos_only');  opts.pos_only = false;  end   % input-side gate (skip negative raw)
    if ~isfield(opts, 'gap_fig');   opts.gap_fig = false;   end   % known-arm a_hat-a_det vs a_true-a_det figure
    if ~isfield(opts, 'dx3_fig');   opts.dx3_fig = false;   end   % known-arm dx3hat vs true delta_x figure

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);
    out_dir = fullfile(project_root, 'test_results', 'aprime_4state', 'overlay');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % --- scenario (same as compare_aprime_4state) ---
    cfg = user_config();
    cfg.eq17_variant    = '4state';
    cfg.trajectory_type = 'osc';
    cfg.h_init = 50; cfg.h_bottom = 2.7; cfg.amplitude = 2.5;
    cfg.frequency = 1; cfg.n_cycles = 2;
    cfg.t_hold = 0.5; cfg.t_descend_override = 1.0; cfg.T_sim = opts.T_sim;
    pc = physical_constants(); cfg.h_min = 1.05 * pc.R;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true; cfg.h_bar_safe = 1;
    cfg.use_taylor_gain = true;

    arms = {'known', 'diff'};
    Ns = numel(opts.seeds);

    % --- run + collect ---
    data = struct();
    for ai = 1:2
        cfg_arm = cfg;
        cfg_arm.aprime_source = arms{ai};
        if strcmp(arms{ai}, 'diff')
            cfg_arm.aprime_clamp_pos = opts.clamp_pos;
            cfg_arm.aprime_pos_only  = opts.pos_only;
        end
        for si = 1:Ns
            ro = struct('seed', opts.seeds(si), 'verbose', false, 'collect_diag', true);
            so = run_pure_simulation(cfg_arm, ro);
            data.(arms{ai})(si).simOut = so;
        end
    end

    % --- desired-trajectory oracle series (from shared p_d) ---
    so0 = data.known(1).simOut;
    t  = so0.tout;
    P  = so0.meta.params_value;
    w_hat = P.wall.w_hat; pz = P.wall.pz; R = P.common.R;
    a_nom = P.ctrl.Ts / P.ctrl.gamma;
    hbd = max((so0.p_d_out * w_hat - pz) / R, 1.001);
    N_t = numel(t);
    [a_det_d, ap_oracle] = deal(zeros(N_t, 1));
    for k = 1:N_t
        [~, c_pe, drv] = calc_correction_functions(hbd(k), true);
        a_det_d(k)   = a_nom / c_pe;                       % z axis
        ap_oracle(k) = -a_det_d(k) * drv.K_h_perp / R;
    end

    % --- figure: stacked 2x1 overlay, both a' methods on shared axes ---
    %     (EXP style, mirrors plot_q55_6state: no grid/title, bold FS18,
    %      AXLW 2.0, legend northoutside on the top tile only, nm/pN units)
    COL_TH = [0 0.6 0]; COL_K = [0.8 0 0]; COL_D = [0 0.35 0.8];
    FS = 18; LFS = 14; AXLW = 2.0;
    if ~isfield(opts, 'plot_seed'); opts.plot_seed = opts.seeds(1); end
    si = find(opts.seeds == opts.plot_seed, 1);
    so_k = data.known(si).simOut;
    so_d = data.diff(si).simOut;
    T_END = ceil(t(end));

    f = figure('Position', [80 80 1100 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile; hold on;                                    % --- top: a_hat overlay
    hT = plot(t, so_k.a_true_out(:, 3) * 1e3, '-', 'Color', COL_TH, 'LineWidth', 3.0, ...
              'DisplayName', 'a_{true}');
    hK = plot(t, so_k.diag.a_hat(:, 3) * 1e3, '-', 'Color', COL_K, 'LineWidth', 1.4, ...
              'DisplayName', 'a\_hat, a'' known c(h)');
    hD = plot(t, so_d.diag.a_hat(:, 3) * 1e3, '-', 'Color', COL_D, 'LineWidth', 1.4, ...
              'DisplayName', 'a\_hat, a'' self-diff');
    xlim([0 T_END]);
    ylabel('a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hK hD], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    nexttile; hold on;                                    % --- bottom: a' overlay
    plot(t, ap_oracle * 1e3, '-', 'Color', COL_TH, 'LineWidth', 3.0);
    plot(t, so_k.diag.a_prime_used(:, 3) * 1e3, '--', 'Color', COL_K, 'LineWidth', 1.4);
    plot(t, so_d.diag.a_prime_used(:, 3) * 1e3, '-', 'Color', COL_D, 'LineWidth', 1.4);
    xlim([0 T_END]);
    ylabel('a''_z  (nm/pN/\mum)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    clamp_tag = '';
    if opts.clamp_pos; clamp_tag = [clamp_tag '_clamp']; end
    if opts.pos_only;  clamp_tag = [clamp_tag '_posonly']; end
    fout = fullfile(out_dir, sprintf('fig_overlay_stacked_seed%d%s.png', opts.plot_seed, clamp_tag));
    exportgraphics(f, fout, 'Resolution', 150);
    close(f);
    fprintf('[overlay] %s\n', fout);

    % --- optional: known-arm gap overlay (single panel) ---
    %     a_hat - a_det(h_bar_d)  vs  a_true - a_det(h_bar_d) (= a_ram)
    if opts.gap_fig
        fg = figure('Position', [80 80 1100 480], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        hold on;
        hT = plot(t, (so_k.a_true_out(:, 3) - a_det_d) * 1e3, '-', 'Color', COL_TH, ...
                  'LineWidth', 2.2, 'DisplayName', 'a_{true} - a(p_d)  (= a_{ram})');
        hK = plot(t, (so_k.diag.a_hat(:, 3) - a_det_d) * 1e3, '-', 'Color', COL_K, ...
                  'LineWidth', 1.4, 'DisplayName', 'a\_hat - a(p_d)');
        xlim([0 T_END]);
        ylabel('a_z - a_z(p_d)  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([hT hK], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        fgout = fullfile(out_dir, sprintf('fig_overlay_gap_known_seed%d.png', opts.plot_seed));
        exportgraphics(fg, fgout, 'Resolution', 150);
        close(fg);
        fprintf('[overlay] %s\n', fgout);
    end

    % --- optional: known-arm dx3hat overlay (single panel, z axis) ---
    %     true delta_x[k] = p_d[k] - x[k] with x[k] = pre-integration position
    %     (= p_true_out[k-1], POST-integration logging convention) vs the EKF
    %     current tracking-error estimate delta_x_hat_3[k].
    if opts.dx3_fig
        dx_true = [0; so_k.p_d_out(2:end, 3) - so_k.p_true_out(1:end-1, 3)];   % [um]
        fd3 = figure('Position', [80 80 1100 480], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        hold on;
        hT = plot(t, dx_true * 1e3, '-', 'Color', COL_TH, 'LineWidth', 1.2, ...
                  'DisplayName', '\deltax_z true');
        hK = plot(t, so_k.diag.delta_x_hat_3(:, 3) * 1e3, '-', 'Color', COL_K, ...
                  'LineWidth', 1.4, 'DisplayName', '\deltax\_hat_3');
        xlim([0 T_END]);
        ylabel('\deltax_z  (nm)', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([hT hK], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        fd3out = fullfile(out_dir, sprintf('fig_overlay_dx3_known_seed%d.png', opts.plot_seed));
        exportgraphics(fd3, fd3out, 'Resolution', 150);
        close(fd3);
        fprintf('[overlay] %s\n', fd3out);
    end

    % --- console: per-seed osc-window stats (z) ---
    t_osc0  = cfg.t_hold + cfg.t_descend_override;
    osc_dur = cfg.n_cycles / cfg.frequency;
    t_disc  = min(1 / cfg.frequency, 0.5 * osc_dur);
    t_osc   = (t >= t_osc0 + t_disc) & (t < t_osc0 + osc_dur);
    for ai = 1:2
        nm = arms{ai};
        for si = 1:Ns
            so = data.(nm)(si).simOut;
            rel = so.diag.a_hat(t_osc, 3) ./ so.a_true_out(t_osc, 3) - 1;
            apc = corrcoef(so.diag.a_prime_used(t_osc, 3), ap_oracle(t_osc));
            fprintf('%-6s seed %d: a_hat bias=%+6.2f%%  std=%6.2f%%  a''-corr=%+.3f\n', ...
                    nm, opts.seeds(si), 100*mean(rel), 100*std(rel), apc(1, 2));
        end
    end
end
