% STATUS: SCRATCH -- 2026-10-20 (user: "用真實牆座標 seed 1 和 4 全程畫"; "我也想看 motion 和 motion error"). Layout of Meng diss.
%   Fig. 45: left = motion in the TRUE local coordinate (u, v along the wall, w = distance from the wall plane), desired (red) and actual
%   (blue); right = motion error delta = desired - actual in the same coordinate. Units um. One figure per seed.
%   -> test_results/temp_3d_wsonly/fig_t2_motion_<tag>_s<seed>.png | run: plot_t2_motion('blk_both_c020', [1 4])
function plot_t2_motion(tag, seeds, lbl)
    if nargin < 1; tag = 'blk_both_c020'; end
    if nargin < 2; seeds = 1; end
    if nargin < 3; lbl = repmat({''}, 1, numel(seeds)); end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));
    S = load(fullfile(od, ['temp_t2_' tag '.mat']));  ty = S.R.opts.theta_y;  pc = physical_constants();
    V = [cosd(ty) 0 sind(ty); 0 1 0; -sind(ty) 0 cosd(ty)];       % true local coordinate [u v w] (theta_x = 0)
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  MU = char(181);  DL = char(948);  NM = {'u', 'v', 'w'};
    sd = cellfun(@(q) q.seed, S.R.runs);
    for is = 1:numel(seeds)
        s = seeds(is);
        r = S.R.runs{find(sd == s, 1)};  k = 1:r.kend;  t = r.t(k);
        L = (V' * r.p(k, :)')';  Ld = (V' * r.pd(k, :)')';  E = Ld - L;      % [um]
        f = figure('Position', [40 40 2000 1500], 'Color', 'w', 'Visible', 'off');
        tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for q = 1:3
            ax = nexttile(tl, 2 * q - 1);  hold on;  box on;
            h1 = plot(t, Ld(:, q), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [NM{q} '_d']);
            h2 = plot(t, L(:, q), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', NM{q});
            if q == 3
                plot(t, r.h_star * pc.R + 0 * t, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 2, 'DisplayName', 'h* R');
                set(ax, 'YScale', 'log');
            end
            ylabel([NM{q} '  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            if q == 1
                legend(ax, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            end
            if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;  xlim([0 r.t(end)]);
            ax = nexttile(tl, 2 * q);  hold on;  box on;
            plot(t, E(:, q), '-', 'Color', cE, 'LineWidth', 0.8);
            ylim([-0.3 0.3]);  ylabel([DL NM{q} '  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;  xlim([0 r.t(end)]);
        end
        out = fullfile(od, sprintf('fig_t2_motion_%s_s%d.png', tag, s));
        exportgraphics(f, out, 'Resolution', 120);  close(f);  fprintf('figure: %s\n', out);
    end
end
