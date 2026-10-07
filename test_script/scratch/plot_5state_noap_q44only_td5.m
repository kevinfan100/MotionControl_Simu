% PURPOSE (2026-09-28, discriminator for the mainline-Q divergence): first order, lambda_f = 1, plane, seeds 1:5,
%   22.2 -> 1.10 R in T_d = 5 s. Three arms: original Q (Q34 = Q44 = 0) | mainline Q with a'_true (Q34 = -a'Q33, Q44 = a'^2 Q33)
%   | Q44 only (Q44 = a'^2 Q33, cross terms 0; ctrl_const.q_aprime_cross = false). Drawn: descent only, mainline arm printed not drawn.
%   Runs: ladder_ramp16_Td5_lf1{,_qap,_q44only}_o1.mat (test_results/temp_5state_noap)   STATUS: SCRATCH
function plot_5state_noap_q44only_td5()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    arms = {'',         'original Q (Q_{34} = Q_{44} = 0)',       [0 0.2 0.9],   '-',  true; ...
            '_q44only', 'Q_{44} = a''^2Q_{33} only (Q_{34} = 0)',   [0.1 0.6 0.1], '-.', true; ...
            '_qap',     'mainline Q (not drawn)',                  [0.85 0.45 0], '--', false};
    COL_TRUE = [0.8 0 0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1150 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(tl, 1);  hold(ax1, 'on');  ax2 = nexttile(tl, 2);  hold(ax2, 'on');
    fprintf('\n%-10s | %-10s | %-10s %-10s | %-10s\n', 'arm', 'a_hat end', 'e_a end', 'e_a near', 'e_a hold end');
    hh = gobjects(1, 0);
    for k = 1:3
        r = load(fullfile(id, sprintf('ladder_ramp16_Td5_lf1%s_o1.mat', arms{k, 1})));
        t = r.t(:);  seg = r.seg;  hd = r.hd(:);  m = t >= seg(1) & t <= seg(2);  ts = t(m) - seg(1);
        i1 = find(t <= seg(2), 1, 'last');  mn = t > seg(1) & t <= seg(2) & hd < 3;
        at = mean(r.a_true, 2);  ah = mean(r.a_hat, 2);  ea = at - ah;
        if k == 1
            h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', 'first order, \lambda_f = 1, 22.2 \rightarrow 1.10 R in T_d = 5 s (descent)');
            ht = plot(ax1, ts, at(m), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        end
        if arms{k, 5}
            hh(end + 1) = plot(ax1, ts, ah(m), arms{k, 4}, 'Color', arms{k, 3}, 'LineWidth', 2.0, 'DisplayName', ['a_{hat}, ' arms{k, 2}]); %#ok<AGROW>
            plot(ax2, ts, ea(m), arms{k, 4}, 'Color', arms{k, 3}, 'LineWidth', 2.0);
        end
        fprintf('%-10s | %-10.3f | %+-10.3f %+-10.3f | %+-10.3f\n', strtok(arms{k, 2}), ah(i1), ea(i1), mean(ea(mn)), ea(end));
    end
    plot(ax2, [0 ts(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
    for ax = [ax1 ax2]
        xlim(ax, [0 ts(end)]);
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    ylim(ax1, [0 1.1]);  ylabel(ax1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax2, 'time since descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax1, [h0 ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    out = fullfile(fd, 'fig_noap_q44only_td5.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
