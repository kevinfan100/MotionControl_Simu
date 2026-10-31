% FORK OF test_script/scratch/plot_meng_ch4_champion_two_traj.m (style) + test/motion-test plot_ladder_tw_abs.m (layout)
%   (2026-09-14) | PURPOSE: gain error of the Meng Ch4 champion vs the mainline formC_b arms on the same plane plant and the
%   two mainline trajectories (Meng left, canon right). Rows: (1) w_d/R; (2) Ch4 champion vs mainline production, seed mean
%   +- sigma_seed; (3) mainline seed mean by what is known: wall position opened to 1 R (widew), production (Pf_w0 0.111 R),
%   start + wall known with the chord seed (bseed0), b_true fed (btseed0); (4) sigma_seed of all five arms (log).
%   Error = (a_hat - a_true)/a_nom. Ch4: 10 seeds, lambda_c 0.4, records cropped to the mainline length; mainline: canon 10 /
%   Meng 5 seeds, lambda_c 0.7. | EXPIRES: when the user closes the Ch4-vs-mainline question | 產線改動不會自動跟上
function plot_ch4_vs_mainline_gain_error()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    CH = fullfile(root, 'test_results', 'meng_ch4_s0');
    MT = fullfile(fileparts(root), 'MotionControl_Simu-motion-test', 'test_results', 'apd_acov_meng');
    TR = {'meng', 'canon'};  FS = 15; LFS = 11; AXLW = 1.8;
    ARM = {'ch4',    [0.85 0.33 0.10], 'Ch4 champion';
           'widew',  [0.55 0.55 0.55], 'wall \pm1 R';
           'prod',   [0.00 0.20 0.90], 'production';
           'bseed0', [0.00 0.60 0.30], 'start + wall known';
           'btseed0',[0.80 0.00 0.00], 'b_{true} fed'};
    SEG = struct('meng', {{'far', 0.5, 7; 'near', 7, 10.5; 'hold', 10.5, 12.5}}, ...
                 'canon', {{'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8}});
    f = figure('Units', 'inches', 'Position', [0 0 13 14], 'Color', 'w', 'Visible', 'off');
    tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, 2);
    for c = 1:2
        tr = TR{c};
        C = load(fullfile(CH, sprintf('champ_two_traj_%s.mat', tr)));
        A = load(fullfile(MT, sprintf('three_walls_%s_plane-ramp_bseed0-btseed0-bseed0_b1-bseed0_b1np.mat', tr)));
        B = load(fullfile(MT, sprintf('three_walls_%s_plane-sphere-cell_prod-widew-wideb-wide15.mat', tr)));
        t = B.plane_prod.t(:);  T_END = ceil(t(end));  mc = C.t(:) <= t(end) + 1e-9;
        D = struct('ch4', struct('t', C.t(mc), 'E', C.E(mc, :)), 'widew', B.plane_widew, 'prod', B.plane_prod, ...
                   'bseed0', A.plane_bseed0, 'btseed0', A.plane_btseed0);
        % (1) commanded height
        ax(1, c) = nexttile(c); hold on;
        plot(C.t(mc), C.hd(mc), '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.6);
        local_style(c == 1, 'w_d / R', false, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
        % (2) Ch4 champion vs production
        ax(2, c) = nexttile(2 + c); hold on;  H = gobjects(0);
        for a = [1 3]
            d = D.(ARM{a, 1});  tt = d.t(:);  m = mean(d.E, 2);  s = std(d.E, 0, 2);
            fill([tt; flipud(tt)], [m + s; flipud(m - s)], ARM{a, 2}, 'FaceAlpha', 0.22, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(end+1) = plot(tt, m, '-', 'Color', ARM{a, 2}, 'LineWidth', 1.6); %#ok<AGROW>
        end
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        legend(H, {'Ch4 champion  mean \pm \sigma_{seed}', 'production  mean \pm \sigma_{seed}'}, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, '(a\^ - a) / a_{nom}', false, T_END, FS, AXLW);
        % (3) mainline seed mean by what is known
        ax(3, c) = nexttile(4 + c); hold on;  H = gobjects(0);
        for a = 2:5
            d = D.(ARM{a, 1});  H(end+1) = plot(d.t(:), mean(d.E, 2), '-', 'Color', ARM{a, 2}, 'LineWidth', 1.5); %#ok<AGROW>
        end
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        legend(H, ARM(2:5, 3), 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, 'seed mean  (a\^ - a) / a_{nom}', false, T_END, FS, AXLW);
        % (4) sigma_seed, all arms
        ax(4, c) = nexttile(6 + c); hold on;  H = gobjects(0);
        for a = 1:5
            d = D.(ARM{a, 1});  H(end+1) = plot(d.t(:), std(d.E, 0, 2), '-', 'Color', ARM{a, 2}, 'LineWidth', 1.3); %#ok<AGROW>
        end
        legend(H, ARM(:, 3), 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_style(c == 1, '\sigma_{seed}', true, T_END, FS, AXLW);  set(gca, 'YScale', 'log');
        % console: motion windows first
        S = SEG.(tr);
        for s = 1:size(S, 1)
            fprintf('[%s %-7s]', tr, S{s, 1});
            for a = 1:5
                d = D.(ARM{a, 1});  tt = d.t(:);  m = tt > S{s, 2} & tt <= S{s, 3};  E = d.E(m, :);  mE = mean(E, 2);  [~, iw] = max(abs(mE));
                fprintf('  %s mean %+.4f sd %.4f worst %+.4f |', ARM{a, 1}, mean(mE), mean(std(E, 0, 2)), mE(iw));
            end
            fprintf('\n');
        end
    end
    for r = 2:4; linkaxes(ax(r, :), 'y'); end
    set(ax(4, :), 'YLim', [1e-4 0.2]);
    png = fullfile(CH, 'ch4_vs_mainline_gain_error.png');
    exportgraphics(f, png, 'Resolution', 150);  close(f);  fprintf('saved %s\n', png);
end

function local_style(is_left, ylab, is_bottom, T_END, FS, AXLW)
    if is_left; ylabel(ylab, 'FontSize', FS, 'FontWeight', 'bold'); end
    if is_bottom; xlabel('time  [s]', 'FontSize', FS, 'FontWeight', 'bold'); end
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off; xlim([0 T_END]);
end
