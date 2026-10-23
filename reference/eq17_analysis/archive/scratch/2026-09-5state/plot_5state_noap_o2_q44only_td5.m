% PURPOSE (2026-09-28): second order, lambda_f = 1, plane, seeds 1:5, 22.2 -> 1.10 R in T_d = 5 s, descent only.
%   Drawn: original Q (Q34 = Q44 = 0) vs Q44 only (Q44 = a'_true^2 Q33). The full mainline-Q arm is run and printed but not drawn.
%   Rows: a_true / a_hat | e_a | delta a_hat vs the true increment per step (0.25 s mean).
%   Runs: ladder_ramp16_Td5_lf1{,_q44only,_qap}_o2.mat (test_results/temp_5state_noap)   STATUS: SCRATCH
function plot_5state_noap_o2_q44only_td5()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    arms = {'',         'original Q (Q_{34} = Q_{44} = 0)',     [0 0.2 0.9],   '-',  true; ...
            '_q44only', 'Q_{44} = a''^2Q_{33} only (Q_{34} = 0)', [0.1 0.6 0.1], '-.', true; ...
            '_qap',     'mainline Q (not drawn)',                [0.85 0.45 0], '--', false};
    COL_TRUE = [0.8 0 0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1150 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(1, 3);  for q = 1:3; ax(q) = nexttile(tl, q); hold(ax(q), 'on'); end
    fprintf('\n%-10s | %-10s | %-10s %-10s | %-10s\n', 'arm', 'a_hat end', 'e_a end', 'e_a near', 'e_a hold end');
    hh = gobjects(1, 0);
    for k = 1:3
        r = load(fullfile(id, sprintf('ladder_ramp16_Td5_lf1%s_o2.mat', arms{k, 1})));
        t = r.t(:);  seg = r.seg;  hd = r.hd(:);  m = t >= seg(1) & t <= seg(2);  ts = t(m) - seg(1);
        i1 = find(m, 1, 'last');  mn = m & hd < 3;
        at = mean(r.a_true, 2);  ah = mean(r.a_hat, 2);  ea = at - ah;  da = mean(r.da_hat, 2) * 1e3;
        fprintf('%-10s | %-10.3f | %+-10.3f %+-10.3f | %+-10.3f\n', strtok(arms{k, 2}), ah(i1), ea(i1), mean(ea(mn)), ea(end));
        if k == 1
            h0 = plot(ax(1), nan, nan, 'w-', 'DisplayName', 'second order, \lambda_f = 1, 22.2 \rightarrow 1.10 R in T_d = 5 s');
            ht = plot(ax(1), ts, at(m), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
            dtr = movmean(mean(r.da_true, 2), round(0.25 / (t(2) - t(1)))) * 1e3;
            hr = plot(ax(3), ts, dtr(m), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'true increment (0.25 s mean)');
        end
        if ~arms{k, 5}; continue; end
        hh(end + 1) = plot(ax(1), ts, ah(m), arms{k, 4}, 'Color', arms{k, 3}, 'LineWidth', 2.0, 'DisplayName', ['a_{hat}, ' arms{k, 2}]); %#ok<AGROW>
        plot(ax(2), ts, ea(m), arms{k, 4}, 'Color', arms{k, 3}, 'LineWidth', 2.0);
        plot(ax(3), ts, da(m), arms{k, 4}, 'Color', arms{k, 3}, 'LineWidth', 2.0);
    end
    plot(ax(2), [0 ts(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);  plot(ax(3), [0 ts(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
    for q = 1:3
        xlim(ax(q), [0 ts(end)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    end
    ylim(ax(1), [0 1.1]);
    ylabel(ax(1), 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2), 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(3), '\delta a / a_{nom}  [10^{-3} / step]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax(3), 'time since descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax(1), [h0 ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    legend(ax(3), hr, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    out = fullfile(fd, 'fig_noap_o2_q44only_td5.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
