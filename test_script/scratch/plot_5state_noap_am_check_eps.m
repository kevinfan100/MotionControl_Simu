% PURPOSE (2026-10-02, user: "same case as fig_noap_am_ahat_td5, compare epsilon = 0 and 1e-5 around the stop, the random
%   motion, to check a_m"): plane, hold 0.5 s -> 22.2 -> 1.10 R in 5 s -> hold 2 s, first order. Columns epsilon = 0 | 1e-5.
%   Rows (seed 1, window 4.5-6.5 s): (1) random part dx_r [nm] with +- the sd the readout expects at a_TRUE (red dashed)
%   and the running sd that a_m is built from (blue) | (2) a_m, a_true, a_hat | (3) a_m / a_true (81-step mean).
%   Console: 10-seed statistics in the final hold (t > 6 s raw): std(dx_r) measured vs expected at a_true, mean a_m/a_true,
%   a_hat/a_true, i.e. the loop-gain ratio s = a_hat/a_true the readout bias depends on.   STATUS: SCRATCH
function plot_5state_noap_am_check_eps(seeds)
    if nargin < 1; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  kBT = pc.k_B * pc.T;
    ARMS = {'q0', '\epsilon = 0', 0; 'eps', '\epsilon = Q_{44}/R_2 = 10^{-5}', 1e-5};
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 16;  LFS = 11;  AXLW = 2.0;  WIN = [4.5 6.5];
    f = figure('Position', [40 40 1800 1300], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('\nfinal hold (t > 6.0 s), %d seeds\n%-4s | %-14s %-14s | %-12s | %-12s\n', numel(seeds), 'arm', 'std dx_r [nm]', 'expected [nm]', 'a_m/a_true', 'a_hat/a_true');
    for c = 1:2
        S = struct('sd', [], 'ex', [], 'am', [], 'ah', []);
        for q = 1:numel(seeds)
            fn = fullfile(id, sprintf('amchk_plane_td5_%s_s%d.mat', ARMS{c, 1}, seeds(q)));
            if exist(fn, 'file'); r = load(fn);
            else
                cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';  cfg.amplitude = 0;
                cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.h_min = 1.10 * R;
                cfg.t_hold = 0.5;  cfg.t_descend_override = 5;  cfg.T_sim = 0.5 + 5 + 2.0;
                cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', ARMS{c, 3});
                s = run_pure_simulation(cfg, struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc));
                r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'a_m', s.diag.a_xm(:, 3), ...
                           'dx_r', s.diag.dx_r(:, 3), 's2', s.diag.sigma2_dxr_hat(:, 3), 'C_dpmr', s.ctrl_const.C_dpmr, ...
                           'C_n', s.ctrl_const.C_n, 's2n', cfg.meas_noise_std(3)^2);
                save(fn, '-struct', 'r');
            end
            ex = sqrt(r.C_dpmr * 4 * kBT * r.a_true + r.C_n * r.s2n);          % sd the readout expects at the true gain
            k = r.t > 6.0;
            S.sd(end + 1) = std(r.dx_r(k)) * 1e3;  S.ex(end + 1) = mean(ex(k)) * 1e3;
            S.am(end + 1) = mean(r.a_m(k)) / mean(r.a_true(k));  S.ah(end + 1) = mean(r.a_hat(k)) / mean(r.a_true(k));
            if q == 1; r1 = r;  ex1 = ex; end
        end
        fprintf('%-4s | %6.2f +- %-5.2f %6.2f         | %5.2f +- %-4.2f | %5.2f\n', ARMS{c, 1}, mean(S.sd), std(S.sd), mean(S.ex), mean(S.am), std(S.am), mean(S.ah));
        t = r1.t;  m = t >= WIN(1) & t <= WIN(2);
        % row 1: random motion
        nexttile(c); hold on; box on;
        h1 = plot(t(m), r1.dx_r(m) * 1e3, '-', 'Color', C_MEAS, 'LineWidth', 0.7, 'DisplayName', 'dx_r, random part of the motion error (seed 1)');
        h2 = plot(t(m), sqrt(r1.s2(m)) * 1e3, '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', '\pm running sd (what a_m is built from)');
        plot(t(m), -sqrt(r1.s2(m)) * 1e3, '-', 'Color', C_EST, 'LineWidth', 1.8);
        h3 = plot(t(m), ex1(m) * 1e3, '--', 'Color', COL_TRUE, 'LineWidth', 1.8, 'DisplayName', '\pm sd expected at a_{true}');
        plot(t(m), -ex1(m) * 1e3, '--', 'Color', COL_TRUE, 'LineWidth', 1.8);
        ylim([-40 40]);  if c == 1; ylabel('[nm]', 'FontSize', FS, 'FontWeight', 'bold'); end
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('plane, T_d = 5 s, %s', ARMS{c, 2}));
        legend([h0 h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        % row 2: gains
        nexttile(2 + c); hold on; box on;
        h4 = plot(t(m), r1.a_m(m) / a_nom, '-', 'Color', C_MEAS, 'LineWidth', 0.7, 'DisplayName', 'a_m');
        h5 = plot(t(m), r1.a_true(m) / a_nom, '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a_{true}');
        h6 = plot(t(m), r1.a_hat(m) / a_nom, '-', 'Color', C_EST, 'LineWidth', 2.0, 'DisplayName', 'a_{hat}');
        ylim([0 1.2]);  if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h5 h4 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        % row 3: readout ratio
        nexttile(4 + c); hold on; box on;
        plot(t(m), movmean(r1.a_m(m), 81) ./ r1.a_true(m), '-', 'Color', C_MEAS, 'LineWidth', 1.6);
        yline(1, '-', 'Color', COL_TRUE, 'LineWidth', 1.6);  ylim([0 4]);
        if c == 1; ylabel('a_m / a_{true} (81-step mean)', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 0:2
            nexttile(2 * q + c);  xlim(WIN);  yl = ylim;  plot([5.5 5.5], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid off;
        end
    end
    out = fullfile(fd, 'fig_noap_am_check_eps_td5.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
