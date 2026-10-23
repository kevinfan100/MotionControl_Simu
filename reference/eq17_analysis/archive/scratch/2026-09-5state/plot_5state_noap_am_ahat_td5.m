% PURPOSE (2026-09-29, user: "at T_d = 5 s put a_hat, a_true and a_m on one figure"): first order, lambda_f = 1, plane,
%   one seed, hold 0.5 s -> 22.2 -> 1.10 R at constant velocity in 5 s -> hold 0.5 s (ramp16 template of run_5state_noap_ladder).
%   a_m = the gain readout a_xm the y2 channel is built from (diag.a_xm), a_hat = filter estimate, a_true = plant.
%   Black dashed = running mean of a_m from t = 0 (with the initial estimate counted as 1/P44[0] / (1/R2) readings).
%   Rows: full run | last 1.5 s zoomed.   STATUS: SCRATCH
function plot_5state_noap_am_ahat_td5(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  T_d = 5;
    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.amplitude = 0;
    cfg.t_hold = 0.5;  cfg.t_descend_override = T_d;  cfg.T_sim = 0.5 + T_d + 0.5;  cfg.h_min = 1.10 * R;
    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0');
    s = run_pure_simulation(cfg, struct('seed', seed, 'collect_diag', true, 'ctrl_const_override', cc));
    t = s.tout(:);  at = s.a_true_out(:, 3) / a_nom;  ah = s.diag.a_hat(:, 3) / a_nom;  am = s.diag.a_xm(:, 3) / a_nom;
    seg = [0.5, 0.5 + T_d];
    W0 = 1 / 0.05^2;  w2 = 0.26;                                   % initial confidence and y2 weight per step (a/a_nom units)
    runmean = (W0 * ah(1) + w2 * cumsum(am)) ./ (W0 + w2 * (1:numel(am)).');
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1500 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2
        ax = nexttile(tl, row);  hold(ax, 'on');
        hm = plot(ax, t, am, '-', 'Color', C_MEAS, 'LineWidth', 0.8, 'DisplayName', 'a_m (gain readout fed to y_2)');
        ht = plot(ax, t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        % running mean of a_m kept in the console only (not drawn, 2026-09-29)
        he = plot(ax, t, ah, '-', 'Color', C_EST, 'LineWidth', 2.0, 'DisplayName', 'a_{hat} (first order, \lambda_f = 1)');
        yl = [-0.2 1.6];  if row == 2; xlim(ax, [4.5 6]); else; xlim(ax, [0 t(end)]); end
        ylim(ax, yl);  for sg = seg; plot(ax, [sg sg], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        ylabel(ax, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
        if row == 1
            h0 = plot(ax, nan, nan, 'w-', 'DisplayName', sprintf('plane, hold 0.5 s, 22.2 \\rightarrow 1.10 R in %g s, hold 0.5 s, seed %d', T_d, seed));
            legend(ax, [h0 ht hm he], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        else
            xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    i1 = find(t <= seg(2), 1, 'last');  wall = t > seg(2) - 0.3 & t <= seg(2);  hold2 = t > seg(2);
    fprintf('descent end: a_true %.3f  a_hat %.3f  running mean %.3f\n', at(i1), ah(i1), runmean(i1));
    fprintf('last 0.3 s of descent: mean a_m %.3f (a_true %.3f) | final hold: mean a_m %.3f (a_true %.3f), a_hat end %.3f\n', ...
        mean(am(wall)), mean(at(wall)), mean(am(hold2)), mean(at(hold2)), ah(end));
    fprintf('sd of a_m around a_true, final hold: %.3f\n', std(am(hold2) - at(hold2)));
    out = fullfile(fd, 'fig_noap_am_ahat_td5.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
