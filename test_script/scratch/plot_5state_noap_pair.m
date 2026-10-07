% PURPOSE (2026-10-05, user: "first order, plane: fixed at the triangle top and bottom with eps = 0; and eps = 0 vs 1e-5"):
%   same panel style as plot_5state_noap_rtri.m (one seed, no P, rows a_bar | e_a | delta_x_m [um]) but columns = arms.
%   arms = {label, spec; ...}, spec = struct('kind', 'fixed' | 'rtri', 'h', [R] (fixed), 'T_d', 'f' (rtri), 'eps' (Q44/R2)).
%   Runs cached in test_results/temp_5state_noap/pair_<wall>_<kind..>_eps<eps>_s<seed>.mat.   STATUS: SCRATCH
%   e.g. plot_5state_noap_pair({'fixed at 2.00 R', struct('kind','fixed','h',2,'eps',0); 'fixed at 1.10 R', struct('kind','fixed','h',1.1,'eps',0)}, 'eps0_fixed')
function plot_5state_noap_pair(arms, name, wall, seed, T_fix)
    if nargin < 3; wall = 'plane'; end
    if nargin < 4; seed = 1; end
    if nargin < 5; T_fix = 2.5; end                                              % [s] fixed-height run length (as the triangle runs)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  R = pc.R;
    nA = size(arms, 1);  D = cell(1, nA);
    for m = 1:nA
        sp = arms{m, 2};
        if strcmp(sp.kind, 'fixed'); key = sprintf('fixed_h%s_T%s', num2str(sp.h), num2str(T_fix));
        else; key = sprintf('rtri_td%s_f%s', num2str(sp.T_d), num2str(sp.f)); end
        fn = fullfile(id, strrep(sprintf('pair_%s_%s_eps%g_s%d', wall, key, sp.eps, seed), '.', 'p'));  fn = [fn '.mat']; %#ok<AGROW>
        if exist(fn, 'file'); r = load(fn);
        else
            if strcmp(sp.kind, 'fixed')
                cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);
                cfg.trajectory_type = 'positioning';  cfg.h_init = sp.h * R;  cfg.h_bottom = sp.h * R;  cfg.amplitude = 0;
                cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = T_fix;
            else
                cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', max(sp.f, 1));
                cfg.t_descend_override = sp.T_d;  cfg.T_sim = sp.T_d + 2 * (sp.f > 0);
                if sp.f > 0; cfg.frequency = sp.f;  cfg.n_cycles = 2 * sp.f; end
            end
            cfg.eq17_variant = '5state_noap';
            cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', sp.eps);
            o = struct('seed', seed, 'collect_diag', true, 'ctrl_const_override', cc);
            if strcmp(wall, 'cell'); o.plant_cperp = @cell_cperp_delta; end
            s = run_pure_simulation(cfg, o);
            r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'a_m', s.diag.a_xm(:, 3), ...
                       'dxm', s.diag.delta_x_m(:, 3), 'h_d', s.p_d_out(:, 3) / R, 'h', s.p_true_out(:, 3) / R);
            save(fn, '-struct', 'r');
        end
        D{m} = r;
        e = (r.a_true - r.a_hat) / a_nom;  last = r.t >= r.t(end) - 1;
        fprintf('%-5s %-28s | h %.3f-%.3f R | e_a mean %+.4f, mean|e_a| %.4f, last 1 s mean %+.4f | dx_m rms %.4f um\n', ...
            wall, arms{m, 1}, min(r.h), max(r.h), mean(e), mean(abs(e)), mean(e(last)), rms(r.dxm));
    end
    AB = char(257);  HAT = char(770);
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    fig = figure('Position', [40 40 1700 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(fig, 3, nA, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, nA);
    for m = 1:nA
        r = D{m};  t = r.t;
        for q = 1:3; ax(q, m) = nexttile(tl, (q - 1) * nA + m); hold(ax(q, m), 'on'); end
        hm = plot(ax(1, m), t, r.a_m / a_nom, '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', [AB '_m']);
        ht = plot(ax(1, m), t, r.a_true / a_nom, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', [AB '_{true}']);
        hh = plot(ax(1, m), t, r.a_hat / a_nom, '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', [AB HAT]);
        ylim(ax(1, m), [-0.2 1.6]);
        lg = legend(ax(1, m), [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', ...
                    'Box', 'on', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, seed %d, first order, %s', wall, seed, arms{m, 1});
        plot(ax(2, m), t, (r.a_true - r.a_hat) / a_nom, '-', 'Color', C_EST, 'LineWidth', 1.8);  yline(ax(2, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        plot(ax(3, m), t, r.dxm, '-', 'Color', C_MEAS, 'LineWidth', 0.8);  yline(ax(3, m), 0, '-', 'Color', [0.6 0.6 0.6]);
        xlabel(ax(3, m), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    end
    ylabel(ax(1, 1), AB, 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), ['e_{' AB '}'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), '\delta x_m  [\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 2:3; linkaxes(ax(q, :), 'y'); end
    for q = 1:3
        for m = 1:nA
            xlim(ax(q, m), [0 D{m}.t(end)]);  set(ax(q, m), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax(q, m), 'off');
        end
    end
    out = fullfile(fd, sprintf('fig_noap_%s_%s_s%d.png', name, wall, seed));
    exportgraphics(fig, out, 'Resolution', 150);  close(fig);  fprintf('figure: %s\n', out);
end
