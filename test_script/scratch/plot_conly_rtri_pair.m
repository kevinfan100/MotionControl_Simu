% PURPOSE (2026-10-05, user: "inside c-only, compare Q55 = eps R2 with sqrt(P_fix) = 0.15, and sqrt(P_fix) 0.15 with the
%   segmented 0.15 new / 0.05 revisit"): same panel style as plot_5state_noap_rtri.m (one seed, no P), columns = the two arms,
%   one figure per wall. Rows: (1) a_true (red), a_hat (blue), a_m (light blue)  (2) e_a  (3) delta_x_m (y1) [um].
%   group 1 = eps vs rho15, group 2 = rho15 vs rn15_05.  Reads test_results/temp_plaw_walls/f1p2_<wall>_PI_s1_<arm><traj>.mat
%   (runner: run_plaw_walls with seeds 1, Td, f_osc, tag_suffix _s1_<arm>; job tmp run_conly_pair.m).   STATUS: SCRATCH
%   Custom arms (10-05, for the Layer-1 session): group = {file_arm, legend_label; ...} and name = output tag,
%   e.g. plot_conly_rtri_pair({'eps1e5', 'c-only, Q_{55} = \epsilon R_2, \epsilon = 10^{-5}'; 'rn15_05', 'c-only, ...'}, 'cell', '_rtri_f1_td0p5', 'mine').
%   opt (10-05): .crow = true adds row 4, c_hat (blue) with c_eff(h_d) (red, the local slope truth at the commanded height);
%   .ref = {file per column} adds the true-slope diagnostic arm's e_a (grey) to row 2 (files hold t, AT, AH; seed 1 drawn); .ref_label.
function plot_conly_rtri_pair(group, wall, traj, name, opt)
    if nargin < 5; opt = struct(); end
    crow = isfield(opt, 'crow') && opt.crow;  nR = 3 + crow;
    if nargin < 1; group = 1; end
    if nargin < 2; wall = 'cell'; end
    if nargin < 3; traj = '_rtri_f1_td0p5'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AB = char(257);  HAT = char(770);
    ARMS = struct('eps', ['c-only, Q_{55} = \epsilon R_2, \epsilon = 10^{-5}'], ...
                  'rho15', 'c-only, \surdP_{fix} = 0.15', ...
                  'rn15_05', 'c-only, \surdP_{fix} = 0.15 new / 0.05 revisit');
    G = {{'eps1e5', 'eps'; 'rho15', 'rho15'}, {'rho15', 'rho15'; 'rn15_05', 'rn15_05'}};
    if iscell(group)
        arms = group;  lab = group(:, 2)';  if nargin < 4; name = 'custom'; end
    else
        arms = G{group};  lab = cellfun(@(k) ARMS.(k), arms(:, 2)', 'UniformOutput', false);  name = sprintf('pair%d', group);
    end
    nA = size(arms, 1);
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    fig = figure('Position', [40 40 1700 1150 + 380 * crow], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(fig, nR, nA, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(nR, nA);
    for m = 1:nA
        if endsWith(arms{m, 1}, '.mat'); fn = arms{m, 1}; else; fn = sprintf('f1p2_%s_PI_s1_%s%s.mat', wall, arms{m, 1}, traj); end   % full file name (e.g. fixed-height runs)
        r = load(fullfile(id, fn));  t = r.t;
        e = r.a_true(:, 1) - r.a_hat(:, 1);
        for q = 1:nR; ax(q, m) = nexttile(tl, (q - 1) * nA + m); hold(ax(q, m), 'on'); end
        hm = plot(ax(1, m), t, r.a_m(:, 1), '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', [AB '_m']);
        ht = plot(ax(1, m), t, r.a_true(:, 1), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', [AB '_{true}']);
        hh = plot(ax(1, m), t, r.a_hat(:, 1), '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', [AB HAT]);
        ylim(ax(1, m), [-0.2 1.6]);
        lg = legend(ax(1, m), [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', ...
                    'Box', 'on', 'AutoUpdate', 'off');
        if isempty(wall); lg.Title.String = sprintf('seed 1, %s', lab{m}); else; lg.Title.String = sprintf('%s wall, seed 1, %s', wall, lab{m}); end   % wall '' = walls given in the labels
        if isfield(opt, 'ref')                                             % true-slope diagnostic arm, same seed, grey behind
            q0 = load(fullfile(id, opt.ref{m}));  hr = plot(ax(2, m), q0.t, q0.AT(:, 1) - q0.AH(:, 1), '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.4, 'DisplayName', opt.ref_label);
        end
        he = plot(ax(2, m), t, e, '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', ['e_{' AB '}, c learned']);  yline(ax(2, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        if isfield(opt, 'ref'); legend(ax(2, m), [he hr], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off'); end
        if crow                                                            % row 4: c_hat against the local slope truth c_eff(h_d) = 1 - h_d^2 a'_true(h_d)
            hc = plot(ax(4, m), t, r.c_eff_d, '-', 'Color', COL_TRUE, 'LineWidth', 0.8, 'DisplayName', 'c_{eff}(h_d) = 1 - h_d^2 a''_{true}');
            hk = plot(ax(4, m), t, r.c_hat(:, 1), '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', 'c\^');
            hk.DisplayName = ['c' HAT];
            legend(ax(4, m), [hc hk], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        end
        plot(ax(3, m), t, r.dxm(:, 1), '-', 'Color', C_MEAS, 'LineWidth', 0.8);  yline(ax(3, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        xlabel(ax(nR, m), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        Td = r.seg(2);  kd = t >= 0.8 * Td & t < Td;  ko = t >= Td;
        fprintf('%-5s %-8s | min h %.3f R | e_a last 20%% of descent %+.3f | triangle mean|e_a| %.3f, worst %+.3f | dx_m rms descent %.4f, triangle %.4f um\n', ...
            wall, arms{m, 2}, min(r.h_true(:, 1)), mean(e(kd)), mean(abs(e(ko))), e(find(abs(e) == max(abs(e(ko))) & ko, 1)), rms(r.dxm(t < Td, 1)), rms(r.dxm(ko, 1)));
    end
    ylabel(ax(1, 1), AB, 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), ['e_{' AB '}'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), '\delta x_m  [\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    if crow; ylabel(ax(4, 1), 'c', 'FontSize', FS, 'FontWeight', 'bold'); end
    for q = 2:3; linkaxes(ax(q, :), 'y'); end
    if isfield(opt, 'ylim2'); ylim(ax(2, 1), opt.ylim2); end   % fixed limits to match another figure (linked across columns)
    if isfield(opt, 'ylim3'); ylim(ax(3, 1), opt.ylim3); end
    for q = 1:nR
        for m = 1:nA
            xlim(ax(q, m), [0 t(end)]);  set(ax(q, m), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax(q, m), 'off');
        end
    end
    out = fullfile(fd, sprintf('fig_conly_%s_%s%s_s1.png', name, wall, traj));
    exportgraphics(fig, out, 'Resolution', 150);  close(fig);  fprintf('figure: %s\n', out);
end
