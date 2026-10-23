% STATUS: SCRATCH -- 2026-10-19 (user): a law with tau = 2 s (the smallest tau without contact for the a' state in probe_alaw_tau_min),
%   no speed cap, slope gate off, plane a_d 0.07 (h* 1.078 R), parked at 22.2 R, seed 1, 6 s. Columns w_s state | a' state.
%   Rows: h_d (red) / h (blue dashed) / h* [R] (log); a-bar true / estimate / a_d; a-bar' true / estimate (log); delta z_m [um].
%   Runs if the files are missing. Output test_results/temp_1009_ws/fig_mc_alaw_tau2.png.
function plot_mc_alaw_tau2(wall, ad, tau)
    if nargin < 1 || isempty(wall); wall = 'plane'; end   % 10-19: cell version (same layout)
    if nargin < 2 || isempty(ad); ad = 0.07; end
    if nargin < 3 || isempty(tau); tau = 2; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    AP = struct('ap_slot', true, 'fe_cmd_eval', true, 'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_delta_lim', [-1 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1);
    WS = struct('fe_cmd_eval', true);  GX = struct('beta', 2 / 51, 'thr', 0, 'ap_min', 1 / 22.2^2);
    tags = {'WS', 'AP'};  EE = {WS, AP};  NM = {['w' char(772) '_s state'], [char(257) ''' state']};
    for i = 1:2
        f = fullfile(od, sprintf('gainreg_%s_tau%03d_T6_t2%s.mat', wall, round(1000 * tau), tags{i}));
        if ~exist(f, 'file'); clear run_formC_b motion_control_law_formC_b;  E = EE{i};  tg = ['_t2' tags{i}];
            evalc('run_mc_s9_gainreg({wall}, 1, ad, tau, 6, 0, E, tg, [], 0, GX);'); end
    end
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  HAT = char(770);
    fig = figure('Position', [40 40 1700 1900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:2
        S = load(fullfile(od, sprintf('gainreg_%s_tau%03d_T6_t2%s.mat', wall, round(1000 * tau), tags{i})));  t = S.t;  hd = S.wd;  hd(1) = hd(2);
        ax = nexttile(tl, i);  hold on; box on;
        h1 = plot(ax, t, hd * pc.R, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'z_d');  h2 = plot(ax, t, S.h_true * pc.R, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
        hs = fzero(@(x) local_a(x, wall) - ad, [1.001 30]);  h3 = yline(ax, hs * pc.R, '--', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', 'z^*');
        set(ax, 'YScale', 'log');  ylim(ax, [2 60]);  yticks(ax, [2 3 5 10 20 50]);
        lg = legend(ax, [h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        lg.Title.String = sprintf('%s wall, %s, %s_d = %.2f', wall, NM{i}, AB, ad);
        ax2 = nexttile(tl, 2 + i);  hold on; box on;
        g1 = plot(ax2, t, S.a_true, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);  g2 = plot(ax2, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        g3 = yline(ax2, ad, '--', 'Color', 'k', 'LineWidth', 1.5, 'DisplayName', [AB '_d']);  ylim(ax2, [-0.2 1.2]);
        legend(ax2, [g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax3 = nexttile(tl, 4 + i);  hold on; box on;
        y = S.ap_hat / a_nom;  y(y <= 0) = NaN;
        k1 = plot(ax3, t, S.ap_true / a_nom, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '''_z']);  k2 = plot(ax3, t, y, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '''_z']);
        set(ax3, 'YScale', 'log');  ylim(ax3, [1e-3 3]);  legend(ax3, [k1 k2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax4 = nexttile(tl, 6 + i);  hold on; box on;
        plot(ax4, t, S.dxm, '-', 'Color', cE, 'LineWidth', 1.0);  ylim(ax4, [-0.2 0.2]);  yticks(ax4, [-0.1 0 0.1]);  xlabel(ax4, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for a_ = [ax ax2 ax3 ax4]; xlim(a_, [0 6]);  set(a_, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a_, 'off'); end
        if i == 1
            ylabel(ax, ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, [AB '_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax3, [AB '''_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax4, [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        tk = [0.5 1 2 3 4 6 - 1e-3];  idx = arrayfun(@(x) find(t >= x, 1), tk);
        fprintf('%-9s | h at %s s: %s | a_true - a_hat at the same times: %s | min h %.3f\n', NM{i}, mat2str(tk, 2), mat2str(S.h_true(idx)', 3), ...
                mat2str((S.a_true(idx) - S.a_hat(idx))', 2), min(S.h_true));
        for hb = [10 5 2]
            k = find(S.h_true < hb, 1);  if isempty(k); continue; end
            fprintf('           at h = %2d R (t %.2f s): a''-hat %.4f vs true %.4f (ratio %.2f)\n', hb, t(k), S.ap_hat(k) / a_nom, S.ap_true(k) / a_nom, S.ap_hat(k) / S.ap_true(k));
        end
    end
    out = fullfile(od, 'fig_mc_alaw_tau2.png');  if ~strcmp(wall, 'plane') || tau ~= 2; out = fullfile(od, sprintf('fig_mc_alaw_%s_tau%g.png', wall, tau)); end;  exportgraphics(fig, out, 'Resolution', 150);  close(fig);  fprintf('figure: %s\n', out);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    [~, c] = calc_correction_functions(h);  a = 1 / c;
end
