% FORK OF test_script/scratch/plot_ladder_tw_abs.m (2026-09-14) | PURPOSE: the plain pair -- original estimator (constant b, arm
%   bseed0) against the same estimator with the b1 state added (arm bseed0_b1np: only difference = slot 6 = b1, P66[0] from the
%   envelope), Meng left, canon right, shared y per row. Rows: (1) e_a = a_hat - a [um/pN] seed mean +- sigma_seed per arm,
%   (2) paired difference e_a(b1) - e_a(original) mean +- SEM, (3) b_hat seed mean per arm with b_true(w) at the true height.
%   Usage: plot_b1_vs_original('plane') | plot_b1_vs_original('ramp')   Output b1_vs_original_<wall>.png
%   | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
function plot_b1_vs_original(wall)
    if nargin < 1 || isempty(wall); wall = 'plane'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'meng', 'canon'};  NM = {'Meng', 'canon'};  tag = '_plane-ramp_bseed0-btseed0-bseed0_b1-bseed0_b1np';
    COL_TRUE = [0.8 0 0];  COL_O = [0 0.2 0.9];  COL_B1 = [0.85 0.33 0.10];
    FS = 15; LFS = 11; AXLW = 1.8;
    f = figure('Units', 'inches', 'Position', [0 0 13 12], 'Color', 'w', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, 2);
    for c = 1:2
        S = load(fullfile(od, sprintf('three_walls_%s%s.mat', TR{c}, tag)));
        o = S.([wall '_bseed0']);  b = S.([wall '_bseed0_b1np']);  tr = S.([wall '_btseed0']);
        t = o.t(:);  T_END = ceil(t(end));  ph = S.phases;  nS = size(o.E, 2);
        eo = o.E * o.a_nom;  eb = b.E * b.a_nom;
        % row 1
        ax(1, c) = nexttile(c); hold on;
        mo = mean(eo, 2); so = std(eo, 0, 2); mb = mean(eb, 2); sb = std(eb, 0, 2);
        fill([t; flipud(t)], [mo + so; flipud(mo - so)], COL_O, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        fill([t; flipud(t)], [mb + sb; flipud(mb - sb)], COL_B1, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(t, mo, '-', 'Color', COL_O, 'LineWidth', 1.8);  h2 = plot(t, mb, '-', 'Color', COL_B1, 'LineWidth', 1.8);
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        legend([h1 h2], {'original  (constant b)', 'with b_1'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, '\^a_z - a_z  [\mum/pN]   mean \pm \sigma_{seed}', false, T_END, ph, FS, AXLW);
        % row 2
        ax(2, c) = nexttile(2 + c); hold on;
        d = eb - eo;  md = mean(d, 2);  se = std(d, 0, 2) / sqrt(nS);
        fill([t; flipud(t)], [md + se; flipud(md - se)], COL_B1, 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(t, md, '-', 'Color', COL_B1, 'LineWidth', 1.8);
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        legend(h1, {sprintf('with b_1  -  original   (same seed, mean \\pm SEM, %d seeds)', nS)}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, '\Delta(\^a_z - a_z)  [\mum/pN]', false, T_END, ph, FS, AXLW);
        % row 3
        ax(3, c) = nexttile(4 + c); hold on;
        h0 = plot(t, mean(tr.B, 2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        h1 = plot(t, mean(o.B, 2), '-', 'Color', COL_O, 'LineWidth', 1.8);  h2 = plot(t, mean(b.B, 2), '-', 'Color', COL_B1, 'LineWidth', 1.8);
        legend([h1 h2 h0], {'\^b  original', '\^b  with b_1', 'b_{true}(w) at the true height'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, 'b   seed mean', true, T_END, ph, FS, AXLW);
        % console: motion windows first, hold last
        md_w = t > ph(1) & t <= ph(2);  mnw = md_w & o.hd(:) < 2;  mos = t > ph(2) & t <= ph(3);  mh = t > ph(3);
        fprintf('[%s %-5s] e_a [um/pN]  original | with b1 | paired diff (SEM)\n', wall, NM{c});
        fprintf('   near-wall descent worst |seed mean|  %.2e | %.2e\n', max(abs(mo(mnw))), max(abs(mb(mnw))));
        fprintf('   near-wall descent mean               %+.2e | %+.2e | %+.2e (%.1e)\n', mean(eo(mnw, :), 'all'), mean(eb(mnw, :), 'all'), mean(mean(d(mnw, :), 1)), std(mean(d(mnw, :), 1)) / sqrt(nS));
        fprintf('   osc / bottom dwell mean              %+.2e | %+.2e | %+.2e (%.1e)\n', mean(eo(mos, :), 'all'), mean(eb(mos, :), 'all'), mean(mean(d(mos, :), 1)), std(mean(d(mos, :), 1)) / sqrt(nS));
        fprintf('   motion sigma_seed                    %.2e | %.2e | ratio %.1f\n', mean(so(t > ph(1) & t <= ph(3))), mean(sb(t > ph(1) & t <= ph(3))), mean(sb(t > ph(1) & t <= ph(3))) / mean(so(t > ph(1) & t <= ph(3))));
        fprintf('   hold mean                            %+.2e | %+.2e | %+.2e (%.1e)\n', mean(eo(mh, :), 'all'), mean(eb(mh, :), 'all'), mean(mean(d(mh, :), 1)), std(mean(d(mh, :), 1)) / sqrt(nS));
    end
    for r = 1:3; linkaxes(ax(r, :), 'y'); end
    png = fullfile(od, sprintf('b1_vs_original_%s.png', wall));
    exportgraphics(f, png, 'Resolution', 150);  close(f);  fprintf('saved %s\n', png);
end

function local_style(is_left, ylab, is_bottom, T_END, ph, FS, AXLW)
    for x = ph(1:3); xline(x, ':', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off'); end
    if is_left; ylabel(ylab, 'FontSize', FS, 'FontWeight', 'bold'); end
    if is_bottom; xlabel('time  [s]', 'FontSize', FS, 'FontWeight', 'bold'); end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off; xlim([0 T_END]);
end
