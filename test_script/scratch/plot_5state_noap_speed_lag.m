% PURPOSE (2026-10-02, user: "use T_d = 5 s vs 0.5 s to see whether going faster lags -- in time it cannot be seen"):
%   same runs as fig_noap_story3 (epsilon = Q44/R2 = 1e-5, plane, 10 seeds), overlaid on common axes:
%   left  = descent, x = commanded height h_d [R] (22.2 -> 1.10, log, wall on the right): a_true and a_hat | e_a
%   right = after the stop, x = time since the stop [s]: a_hat - a_true settling.
%   h_d is the linear ramp h_d = 22.22 - (22.22 - 1.10) (t - 0.5)/T_d (calc_traj_params 'ramp_descent').
%   Reads story_plane_ramp<T_d>_rho_s*.mat.   STATUS: SCRATCH
function plot_5state_noap_speed_lag(TDs, seeds)
    if nargin < 1; TDs = [5 0.5]; end
    if nargin < 2; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  h0 = 50 / pc.R;  hb = 1.10;
    COL = {[0 0.2 0.9], [0.85 0.45 0]};  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    a1 = nexttile(1); hold on; box on;  a2 = nexttile(3); hold on; box on;  a3 = nexttile(2); hold on; box on;  a4 = nexttile(4); hold on; box on;
    hl = gobjects(0);  hr = gobjects(0);
    for c = 1:numel(TDs)
        T_d = TDs(c);  stag = strrep(sprintf('ramp%g', T_d), '.', 'p');  AH = [];  AT = [];
        for q = 1:numel(seeds)
            r = load(fullfile(id, sprintf('story_plane_%s_rho_s%d.mat', stag, seeds(q))));
            AH(:, q) = r.a_hat / a_nom;  AT(:, q) = r.a_true / a_nom; %#ok<AGROW>
        end
        t = r.t - 0.5;  hd = h0 - (h0 - hb) * t / T_d;  dsc = t >= 0 & t <= T_d;  aft = t >= T_d & t <= T_d + 1.5;
        E = AT - AH;  em = mean(E, 2);  es = std(E, 0, 2);  lab = sprintf('T_d = %g s', T_d);
        lsr = '-';  if c == 2; lsr = '--'; end   % each speed against its own a_true (the particle trails h_d at speed)
        hl(end + 1) = plot(a1, hd(dsc), mean(AT(dsc, :), 2), lsr, 'Color', [0.8 0 0], 'LineWidth', 2.4, 'DisplayName', ['a_{true}, ' lab]); %#ok<AGROW>
        hl(end + 1) = plot(a1, hd(dsc), mean(AH(dsc, :), 2), '-', 'Color', COL{c}, 'LineWidth', 2.0, 'DisplayName', ['a_{hat}, ' lab]); %#ok<AGROW>
        fill(a2, [hd(dsc); flipud(hd(dsc))], [em(dsc) + es(dsc); flipud(em(dsc) - es(dsc))], COL{c}, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
        plot(a2, hd(dsc), em(dsc), '-', 'Color', COL{c}, 'LineWidth', 2.0);
        ts = t(aft) - T_d;
        hr(end + 1) = plot(a3, ts, mean(AH(aft, :), 2), '-', 'Color', COL{c}, 'LineWidth', 2.0, 'DisplayName', ['a_{hat}, ' lab]); %#ok<AGROW>
        if c == 1; hr(end + 1) = plot(a3, ts, mean(AT(aft, :), 2), '-', 'Color', [0.8 0 0], 'LineWidth', 2.4, 'DisplayName', 'a_{true}'); end %#ok<AGROW>
        fill(a4, [ts; flipud(ts)], [em(aft) + es(aft); flipud(em(aft) - es(aft))], COL{c}, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
        plot(a4, ts, em(aft), '-', 'Color', COL{c}, 'LineWidth', 2.0);
        for hq = [10 5 3 2 1.5 1.2]
            [~, k] = min(abs(hd - hq) + 1e3 * ~dsc);
            fprintf('T_d %-4g | h_d %4.1f R: a_true %.3f a_hat %.3f e_a %+.3f\n', T_d, hq, mean(AT(k, :)), mean(AH(k, :)), em(k));
        end
        ka = find(aft);  [mn, im] = min(em(aft));  k02 = ka(find(ts >= 0.2, 1));
        fprintf('T_d %-4g | after stop: worst e_a %+.3f at +%.3f s, e_a at +0.2 s %+.3f\n', T_d, mn, ts(im), em(k02));
    end
    set(a1, 'XScale', 'log', 'XDir', 'reverse');  set(a2, 'XScale', 'log', 'XDir', 'reverse');
    for ax = [a1 a2]; xlim(ax, [hb h0]); set(ax, 'XTick', [1.1 1.5 2 3 5 10 20]); end
    ylim(a1, [0 1.05]);  ylim(a2, [-0.25 0.1]);  ylim(a3, [0 0.3]);  ylim(a4, [-0.25 0.1]);
    for ax = [a2 a4]; yline(ax, 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off'); end
    ylabel(a1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, 'e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a2, 'commanded height h_d [R]  (descent, wall on the right)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a4, 'time since the stop (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for ax = [a1 a2 a3 a4]; set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off'); end
    legend(a1, hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    legend(a3, hr, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    out = fullfile(fd, 'fig_noap_speed_lag_plane.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
