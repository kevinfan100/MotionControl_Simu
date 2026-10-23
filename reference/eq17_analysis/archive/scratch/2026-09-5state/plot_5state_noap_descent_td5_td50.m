% PURPOSE (2026-09-28, user: "only the moving segment, T_d = 5 s vs 50 s; print start a, true end a, final a_hat"):
%   first-order gain model, lambda_f = 1 (no forgetting), plane, seeds 1:5, 22.2 -> 1.10 R at constant velocity.
%   The runs keep the 0.5 s holds before and after the descent (same template as every earlier run); only the descent is drawn,
%   the time axis starts at the descent start. The weighted average of 0927_obs_first_order_v1.tex 4.7 is computed from t = 0
%   (the initial hold is part of the history) with w_j = Delta(1/P44) read from the filter.
%   Rows: a_true / a_hat | e_a = a_true - a_hat.  Columns: T_d = 5 s | T_d = 50 s.
%   Runs: test_results/temp_5state_noap/ladder_ramp16_Td5_lf1_o1.mat, ladder_ramp16_Td50_lf1_s{1..5}_o1.mat   STATUS: SCRATCH
function plot_5state_noap_descent_td5_td50()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TD = [5 50];  COL_TRUE = [0.8 0 0];  C2 = [0 0.2 0.9];  CG = [0.1 0.6 0.1];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('\n%-6s | %-9s %-9s | %-12s %-12s | %-10s\n', 'T_d', 'a start', 'a end', 'a_hat end', 'formula end', 'e_a end');
    for c = 1:2
        r = local_load(id, TD(c));
        t = r.t(:);  seg = r.seg;  nS = size(r.a_hat, 2);  wav = zeros(size(r.a_hat));
        for q = 1:nS
            iP = 1 ./ r.P_a(:, q);  w = max([0; diff(iP)], 0);  W0 = iP(1);
            wav(:, q) = (W0 * r.a_hat(1, q) + cumsum(w .* r.a_true(:, q))) ./ (W0 + cumsum(w));
        end
        at = mean(r.a_true, 2);  ah = mean(r.a_hat, 2);  wv = mean(wav, 2);
        m = t >= seg(1) & t <= seg(2);  ts = t(m) - seg(1);
        i0 = find(m, 1, 'first');  i1 = find(m, 1, 'last');
        fprintf('%-6g | %-9.3f %-9.3f | %-12.3f %-12.3f | %+.3f\n', TD(c), at(i0), at(i1), ah(i1), wv(i1), at(i1) - ah(i1));
        ax1 = nexttile(tl, c);  hold(ax1, 'on');
        h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', sprintf('first order, \\lambda_f = 1, descent 22.2 \\rightarrow 1.10 R in T_d = %g s', TD(c)));
        ht = plot(ax1, ts, at(m), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        hw = plot(ax1, ts, wv(m), '--', 'Color', CG, 'LineWidth', 2.2, 'DisplayName', 'weighted average (4.7)');
        hf = plot(ax1, ts, ah(m), '-', 'Color', C2, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} (filter)');
        plot(ax1, ts(end), ah(i1), 'o', 'Color', C2, 'MarkerFaceColor', C2, 'MarkerSize', 9);
        plot(ax1, ts(end), at(i1), 'o', 'Color', COL_TRUE, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 9);
        ylim(ax1, [0 1.1]);
        legend(ax1, [h0 ht hw hf], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        ax2 = nexttile(tl, 2 + c);  hold(ax2, 'on');
        plot(ax2, ts, at(m) - wv(m), '--', 'Color', CG, 'LineWidth', 2.2);
        plot(ax2, ts, at(m) - ah(m), '-', 'Color', C2, 'LineWidth', 1.6);
        plot(ax2, [0 ts(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);  ylim(ax2, [-0.8 0.05]);
        xlabel(ax2, 'time since descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for ax = [ax1 ax2]
            xlim(ax, [0 ts(end)]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        end
        if c == 1
            ylabel(ax1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, 'fig_noap_descent_td5_td50.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
function r = local_load(id, td)
    f1 = fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_o1.mat', td));
    if exist(f1, 'file'); r = load(f1); return; end
    fl = dir(fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_s*_o1.mat', td)));
    assert(~isempty(fl), 'no runs for T_d = %g', td);
    for q = 1:numel(fl)
        s = load(fullfile(id, fl(q).name));
        if q == 1; r = s; else
            for fld = {'a_true', 'a_hat', 'P_a', 'hb'}; r.(fld{1}) = [r.(fld{1}), s.(fld{1})]; end
        end
    end
end
