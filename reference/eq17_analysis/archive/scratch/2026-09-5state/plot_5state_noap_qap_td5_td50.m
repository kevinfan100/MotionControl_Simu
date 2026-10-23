% PURPOSE (2026-09-28, user: "first order, only Q replaced by the mainline formC_b Q with a' = a'_true; compare with the original figure"):
%   first-order gain model, lambda_f = 1, plane, seeds 1:5, 22.2 -> 1.10 R at constant velocity, T_d = 5 s and 50 s.
%   Arms: original Q (Q34 = Q44 = 0)  vs  mainline Q with the true slope (ctrl_const.q_aprime_true: Q34 = -a'_true Q33,
%   Q44 = a'_true^2 Q33; state prediction and F_e unchanged). Drawn: the descent and the final 0.5 s hold (grey dashed line = stop).
%   Runs: ladder_ramp16_Td5_lf1_o1.mat, ladder_ramp16_Td5_lf1_qap_o1.mat, ladder_ramp16_Td50_lf1_s{1..5}_o1.mat,
%         ladder_ramp16_Td50_lf1_qap_s{1..5}_o1.mat (test_results/temp_5state_noap)   STATUS: SCRATCH
function plot_5state_noap_qap_td5_td50()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TD = [5 50];  COL_TRUE = [0.8 0 0];  C_ORIG = [0 0.2 0.9];  C_QAP = [0.85 0.45 0];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('\n%-5s %-10s | %-8s %-8s | %-10s | %-10s %-10s | %-10s\n', 'T_d', 'arm', 'a start', 'a end', 'a_hat end', 'e_a end', 'e_a near', 'e_a hold end');
    for c = 1:2
        ro = local_load(id, TD(c), '');  rq = local_load(id, TD(c), '_qap');
        t = ro.t(:);  seg = ro.seg;  hd = ro.hd(:);
        m = t >= seg(1);  ts = t(m) - seg(1);  t_stop = seg(2) - seg(1);
        i0 = find(m, 1, 'first');  i1 = find(t <= seg(2), 1, 'last');  mn = t > seg(1) & t <= seg(2) & hd < 3;
        ax1 = nexttile(tl, c);  hold(ax1, 'on');  ax2 = nexttile(tl, 2 + c);  hold(ax2, 'on');
        at = mean(ro.a_true, 2);
        h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', sprintf('first order, \\lambda_f = 1, 22.2 \\rightarrow 1.10 R in T_d = %g s', TD(c)));
        ht = plot(ax1, ts, at(m), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        arms = {ro, 'original Q (Q_{34} = Q_{44} = 0)', C_ORIG, '-'; rq, 'mainline Q with a''_{true}', C_QAP, '--'};
        hh = gobjects(1, 2);
        for k = 1:2
            r = arms{k, 1};  ah = mean(r.a_hat, 2);  ea = mean(r.a_true - r.a_hat, 2);  at_k = mean(r.a_true, 2);
            hh(k) = plot(ax1, ts, ah(m), arms{k, 4}, 'Color', arms{k, 3}, 'LineWidth', 2.0, 'DisplayName', ['a_{hat}, ' arms{k, 2}]);
            plot(ax2, ts, ea(m), arms{k, 4}, 'Color', arms{k, 3}, 'LineWidth', 2.0);
            fprintf('%-5g %-10s | %-8.3f %-8.3f | %-10.3f | %+-10.3f %+-10.3f | %+-10.3f\n', TD(c), strtok(arms{k, 2}), at_k(i0), at_k(i1), ah(i1), ...
                ea(i1), mean(ea(mn)), ea(end));
        end
        plot(ax2, [0 ts(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
        for ax = [ax1 ax2]
            xlim(ax, [0 ts(end)]);  yl = ylim(ax);  plot(ax, [t_stop t_stop], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        end
        ylim(ax1, [0 max(1.1, max(ylim(ax1)))]);
        xlabel(ax2, 'time since descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if c == 1
            ylabel(ax1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
        end
        legend(ax1, [h0 ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
    end
    out = fullfile(fd, 'fig_noap_qap_td5_td50.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
function r = local_load(id, td, arm)
    f1 = fullfile(id, sprintf('ladder_ramp16_Td%d_lf1%s_o1.mat', td, arm));
    if exist(f1, 'file'); r = load(f1); return; end
    fl = dir(fullfile(id, sprintf('ladder_ramp16_Td%d_lf1%s_s*_o1.mat', td, arm)));
    assert(~isempty(fl), 'no runs for T_d = %g, arm %s', td, arm);
    [~, ix] = sort({fl.name});  fl = fl(ix);
    for q = 1:numel(fl)
        s = load(fullfile(id, fl(q).name));
        if q == 1; r = s; else
            for fld = {'a_true', 'a_hat', 'P_a', 'hb'}; r.(fld{1}) = [r.(fld{1}), s.(fld{1})]; end
        end
    end
end
