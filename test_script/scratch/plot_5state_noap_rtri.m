% LABELS (10-05): a_bar = a/a_nom (house normalization), hat on the bar; Unicode a-bar (char 257) + combining hat (char 770), house bold font.
% PURPOSE (2026-10-05, user: "fix the figure style first; first order only; one seed, no P; add the motion error"):
%   first order (5state_noap, epsilon = Q44/R2 = 1e-5), ramp_tri family, ONE seed, plane | cell (shared y per row).
%   Two knobs: T_d = constant-velocity descent 22.2 -> 1.10 R [s]; f = triangle frequency [Hz] (1.10 <-> 2.00 R for 2 s);
%   f = 0 -> pure descent (the run stops at the bottom).  Default T_d ([] or omitted): 2 s for the pure descent, 0.5 s with the triangle.  Rows: (1) a_true (red), a_hat (blue), a_m (light blue)
%   (2) e_a = a_true - a_hat  (3) motion error measured by the sensor, delta_x_m = pd[k-2] - p_m[k] (y1) [um].
%   Runs cached in test_results/temp_5state_noap/rtri_<wall>_td<T_d>_f<f>_s<seed>.mat.   STATUS: SCRATCH
function plot_5state_noap_rtri(T_d, f, seed)
    if nargin < 2; f = 1; end
    if nargin < 1 || isempty(T_d); if f > 0; T_d = 0.5; else; T_d = 2; end; end
    if nargin < 3; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    WALLS = {'plane', 'cell'};  EPS = 1e-5;
    tag = sprintf('td%s_f%s_s%d', num2str(T_d), num2str(f), seed);  tag = strrep(tag, '.', 'p');
    D = cell(1, numel(WALLS));
    for iw = 1:numel(WALLS)
        fn = fullfile(id, sprintf('rtri_%s_%s.mat', WALLS{iw}, tag));
        if exist(fn, 'file'); r = load(fn);
        else
            f_cfg = f;  if f == 0; f_cfg = 1; end                       % f = 0: the triangle never starts (T_sim = T_d)
            cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', f_cfg);  cfg.eq17_variant = '5state_noap';
            cfg.t_descend_override = T_d;
            if f > 0; cfg.T_sim = T_d + 2; else; cfg.T_sim = T_d; end
            cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', EPS);
            o = struct('seed', seed, 'collect_diag', true, 'ctrl_const_override', cc);
            if strcmp(WALLS{iw}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
            s = run_pure_simulation(cfg, o);
            r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'a_m', s.diag.a_xm(:, 3), ...
                       'dxm', s.diag.delta_x_m(:, 3), 'h_d', s.p_d_out(:, 3) / pc.R, 'h', s.p_true_out(:, 3) / pc.R);
            save(fn, '-struct', 'r');
        end
        D{iw} = r;
        t = r.t;  e = (r.a_true - r.a_hat) / a_nom;  dx = r.dxm;
        kd = t >= 0.8 * T_d & t < T_d;  ko = t >= T_d;
        fprintf('%-5s %s | min h %.3f R | e_a last 20%% of descent %+.3f | dx_m rms descent %.4f um', WALLS{iw}, tag, min(r.h), mean(e(kd)), rms(dx(t < T_d)));
        if f > 0; fprintf(' | osc mean|e_a| %.3f, dx_m rms %.4f um', mean(abs(e(ko))), rms(dx(ko))); end
        fprintf('\n');
    end
    AB = char(257);  HAT = char(770);                            % a-bar, combining hat
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_MEAS = [0.55 0.75 1.0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    fig = figure('Position', [40 40 1700 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(fig, 3, numel(WALLS), 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, numel(WALLS));
    for iw = 1:numel(WALLS)
        r = D{iw};  t = r.t;
        for q = 1:3; ax(q, iw) = nexttile(tl, (q - 1) * numel(WALLS) + iw); hold(ax(q, iw), 'on'); end
        % (1) gain
        hm = plot(ax(1, iw), t, r.a_m / a_nom, '-', 'Color', C_MEAS, 'LineWidth', 0.6, 'DisplayName', [AB '_m']);
        ht = plot(ax(1, iw), t, r.a_true / a_nom, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', [AB '_{true}']);
        hh = plot(ax(1, iw), t, r.a_hat / a_nom, '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', [AB HAT ', \epsilon = Q_{44}/R_2 = 10^{-5}']);
        ylim(ax(1, iw), [-0.2 1.6]);
        lg = legend(ax(1, iw), [ht hh hm], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', ...
                    'Box', 'on', 'AutoUpdate', 'off');
        lg.Title.String = sprintf('%s wall, seed %d', WALLS{iw}, seed);
        % (2) gain error
        plot(ax(2, iw), t, (r.a_true - r.a_hat) / a_nom, '-', 'Color', C_EST, 'LineWidth', 1.8);
        yline(ax(2, iw), 0, '-', 'Color', [0.6 0.6 0.6]);
        % (3) motion error (y1)
        plot(ax(3, iw), t, r.dxm, '-', 'Color', C_MEAS, 'LineWidth', 0.8);
        yline(ax(3, iw), 0, '-', 'Color', [0.6 0.6 0.6]);
        xlabel(ax(3, iw), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    end
    ylabel(ax(1, 1), AB, 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), ['e_{' AB '}'], 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3, 1), '\delta x_m  [\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    for q = 2:3; linkaxes(ax(q, :), 'y'); end
    for q = 1:3
        for iw = 1:numel(WALLS)
            xlim(ax(q, iw), [0 D{iw}.t(end)]);  set(ax(q, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax(q, iw), 'off');
        end
    end
    out = fullfile(fd, sprintf('fig_noap_rtri_%s.png', tag));  exportgraphics(fig, out, 'Resolution', 150);  close(fig);  fprintf('figure: %s\n', out);
end
