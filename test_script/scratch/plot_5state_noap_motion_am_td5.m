% PURPOSE (2026-09-29, user: "where does the red line come from; show the motion / motion error; how is a_m built from it"):
%   first order, lambda_f = 1, plane, one seed, ramp16 template, T_d = 5 s. Rows:
%   (1) motion: commanded height h_d and true height h (in R)
%   (2) motion error measured by the sensor, delta_x_m (y1), and its slow part dx_bar (the deterministic lag, EWMA a_pd)
%   (3) random part dx_r = delta_x_m - dx_bar, with +-sqrt of its running variance sigma2_dxr_hat (EWMA a_cov, the a_m input)
%       and +-sqrt of the variance the readout formula expects for the TRUE gain, C_dpmr*4kBT*a_true + C_n*sigma_n^2
%   (4) gain: a_true = a_nom/c_perp(h_true) (red), a_m = (sigma2_dxr_hat - C_n sigma_n^2)/(C_dpmr 4kBT) (light blue), a_hat (blue)
%   STATUS: SCRATCH
function plot_5state_noap_motion_am_td5(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  kBT = pc.k_B * pc.T;  T_d = 5;
    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.amplitude = 0;
    cfg.t_hold = 0.5;  cfg.t_descend_override = T_d;  cfg.T_sim = 0.5 + T_d + 0.5;  cfg.h_min = 1.10 * R;
    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0');
    s = run_pure_simulation(cfg, struct('seed', seed, 'collect_diag', true, 'ctrl_const_override', cc));
    t = s.tout(:);  hd = s.p_d_out(:, 3) / R;  h = s.p_true_out(:, 3) / R;
    dxm = s.diag.delta_x_m(:, 3) * 1e3;  dxr = s.diag.dx_r(:, 3) * 1e3;  dxbar = dxm - dxr;       % nm
    s2 = s.diag.sigma2_dxr_hat(:, 3);                                                            % um^2
    at_phys = s.a_true_out(:, 3);  at = at_phys / a_nom;  ah = s.diag.a_hat(:, 3) / a_nom;  am = s.diag.a_xm(:, 3) / a_nom;
    C_dpmr = s.ctrl_const.C_dpmr;  C_n = s.ctrl_const.C_n;  s2n = cfg.meas_noise_std(3)^2;
    s2_true = C_dpmr * 4 * kBT * at_phys + C_n * s2n;                                            % what a_m expects at the true gain
    seg = [0.5, 0.5 + T_d];
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 16;  LFS = 11;  AXLW = 2.0;
    f = figure('Position', [40 40 1500 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(1, 4);  for q = 1:4; ax(q) = nexttile(tl, q); hold(ax(q), 'on'); end
    % (1) motion
    h1 = plot(ax(1), t, hd, '-', 'Color', [0.3 0.3 0.3], 'LineWidth', 2.0, 'DisplayName', 'commanded height h_d');
    h2 = plot(ax(1), t, h, '--', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', 'true height h');
    ylabel(ax(1), 'h  [R]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    % (2) motion error
    h3 = plot(ax(2), t, dxm, '-', 'Color', C_MEAS, 'LineWidth', 0.8, 'DisplayName', '\delta x_m (measured motion error, y_1)');
    h4 = plot(ax(2), t, dxbar, '-', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'slow part (low-pass, a_{pd})');
    ylabel(ax(2), '[nm]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    % (3) random part and its spread
    h5 = plot(ax(3), t, dxr, '-', 'Color', C_MEAS, 'LineWidth', 0.8, 'DisplayName', 'random part dx_r');
    h6 = plot(ax(3), t, sqrt(s2) * 1e3, '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', '\pm running std (EWMA a_{cov}) \rightarrow a_m');
    plot(ax(3), t, -sqrt(s2) * 1e3, '-', 'Color', C_EST, 'LineWidth', 1.8);
    h7 = plot(ax(3), t, sqrt(s2_true) * 1e3, '--', 'Color', COL_TRUE, 'LineWidth', 1.8, 'DisplayName', '\pm std expected at a_{true}');
    plot(ax(3), t, -sqrt(s2_true) * 1e3, '--', 'Color', COL_TRUE, 'LineWidth', 1.8);
    ylabel(ax(3), '[nm]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(3), [h5 h6 h7], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    % (4) gain
    h8 = plot(ax(4), t, am, '-', 'Color', C_MEAS, 'LineWidth', 0.8, 'DisplayName', 'a_m (readout)');
    h9 = plot(ax(4), t, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} = a_{nom} / c_\perp(h)');
    h10 = plot(ax(4), t, ah, '-', 'Color', C_EST, 'LineWidth', 2.0, 'DisplayName', 'a_{hat}');
    ylim(ax(4), [-0.2 1.6]);  ylabel(ax(4), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(4), [h9 h8 h10], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 1:4
        xlim(ax(q), [0 t(end)]);  yl = ylim(ax(q));
        for sg = seg; plot(ax(q), [sg sg], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
        set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    fprintf('C_dpmr %.3f  C_n %.3f  sigma_n %.2f nm\n', C_dpmr, C_n, sqrt(s2n) * 1e3);
    for win = {[0.1 0.5], [2 3], [5.2 5.5], [5.6 6.0]}
        m = t >= win{1}(1) & t < win{1}(2);
        fprintf('t %.1f-%.1f s | h %.2f R | mean dx_bar %+.2f nm | running std %.2f nm, expected at a_true %.2f nm | a_m %.3f a_true %.3f a_hat %.3f\n', ...
            win{1}(1), win{1}(2), mean(h(m)), mean(dxbar(m)), mean(sqrt(s2(m))) * 1e3, mean(sqrt(s2_true(m))) * 1e3, mean(am(m)), mean(at(m)), mean(ah(m)));
    end
    out = fullfile(fd, 'fig_noap_motion_am_td5.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
