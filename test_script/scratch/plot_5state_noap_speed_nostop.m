% PURPOSE (2026-10-02, user: "the rebound after the stop is odd -- drop the final hold, keep the constant-velocity descent
%   going deeper, but only look up to h = 1.10 R, in time"): plane, epsilon = Q44/R2 = 1e-5, 10 seeds. The ramp keeps the
%   speed of the 22.2 -> 1.10 R descent in T_d but continues to 1.05 R (T_d' = T_d (22.22 - 1.05)/(22.22 - 1.10)); the figure
%   is cut at t = T_d, the moment the command passes 1.10 R, so the filter is still moving there (no stop, no rebound).
%   Columns T_d = 5 | 0.5 s; rows a_true, a_hat (mean +- sd) | e_a. Console: e_a at the same commanded heights.
%   Runs cached in test_results/temp_5state_noap/nostop_plane_td<T_d>_s<seed>.mat.   STATUS: SCRATCH
function plot_5state_noap_speed_nostop(TDs, seeds)
    if nargin < 1; TDs = [5 0.5]; end
    if nargin < 2; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  h0 = 50 / R;  hcut = 1.10;  hend = 1.05;
    COL_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  B_EST = [0.45 0.55 0.95];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:numel(TDs)
        T_d = TDs(c);  T_dx = T_d * (h0 - hend) / (h0 - hcut);  AH = [];  AT = [];
        for q = 1:numel(seeds)
            fn = fullfile(id, strrep(sprintf('nostop_plane_td%g_s%d.mat', T_d, seeds(q)), '0.5', '0p5'));
            if exist(fn, 'file'); r = load(fn);
            else
                cfg = canonical_scenario(0.05, hend, 'deep');  cfg.eq17_variant = '5state_noap';  cfg.amplitude = 0;
                cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = hend * R;  cfg.h_min = hend * R;
                cfg.t_hold = 0.5;  cfg.t_descend_override = T_dx;  cfg.T_sim = 0.5 + T_dx + 0.05;
                cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', 1e-5);
                s = run_pure_simulation(cfg, struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc));
                r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'h_d', s.p_d_out(:, 3) / R, 'h', s.p_true_out(:, 3) / R);
                save(fn, '-struct', 'r');
            end
            AH(:, q) = r.a_hat / a_nom;  AT(:, q) = r.a_true / a_nom; %#ok<AGROW>
        end
        t = r.t - 0.5;  k = t >= 0 & r.h_d >= hcut;                  % up to the moment the command passes 1.10 R
        if c == 1; fprintf('\ncheck: h_d at the cut = %.3f R (T_d %g), speed %.2f R/s\n', min(r.h_d(k)), T_d, (h0 - hcut) / T_d); end
        E = AT - AH;  am = mean(AH, 2);  as = std(AH, 0, 2);  em = mean(E, 2);  es = std(E, 0, 2);  tk = t(k);
        nexttile(c); hold on; box on;
        h0l = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('plane, constant speed %.1f R/s (T_d = %g s), cut at h_d = 1.10 R, %d seeds', (h0 - hcut) / T_d, T_d, numel(seeds)));
        fill([tk; flipud(tk)], [am(k) + as(k); flipud(am(k) - as(k))], B_EST, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        ht = plot(tk, mean(AT(k, :), 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true}');
        he = plot(tk, am(k), '-', 'Color', C_EST, 'LineWidth', 1.8, 'DisplayName', 'a_{hat}, \epsilon = Q_{44}/R_2 = 10^{-5}');
        ylim([0 1.1]);  xlim([tk(1) tk(end)]);  local_seg(0);
        if c == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0l ht he], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        nexttile(2 + c); hold on; box on;
        fill([tk; flipud(tk)], [em(k) + es(k); flipud(em(k) - es(k))], B_EST, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        plot(tk, em(k), '-', 'Color', C_EST, 'LineWidth', 1.8);  yline(0, '-', 'Color', [0.6 0.6 0.6]);
        ylim([-0.25 0.1]);  xlim([tk(1) tk(end)]);  local_seg(0);
        if c == 1; ylabel('e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        for hq = [5 3 2 1.5 1.2 1.10]
            [~, i] = min(abs(r.h_d - hq) + 1e3 * (t < 0));
            fprintf('T_d %-4g | h_d %4.2f R (t = %.3f s): a_true %.3f a_hat %.3f e_a %+.3f +- %.3f\n', T_d, hq, t(i), mean(AT(i, :)), am(i), em(i), es(i));
        end
    end
    out = fullfile(fd, 'fig_noap_speed_nostop_plane.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function local_seg(x)
    yl = ylim;  plot([x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');  ylim(yl);
end
