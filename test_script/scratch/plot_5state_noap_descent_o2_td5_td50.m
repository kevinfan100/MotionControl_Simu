% PURPOSE (2026-09-28, user: "same concise write-up for the second-order (delta a) model, same scenario; point: larger T_d does not help"):
%   second-order gain model (a + delta a), lambda_f = 1, plane, seeds 1:5, 22.2 -> 1.10 R at constant velocity, T_d = 5 s vs 50 s.
%   Only the descent is drawn (0.5 s holds are still in the runs). Green dashed = weighted straight-line fit of the true gain over the
%   whole history (0928_obs_second_order.tex section 4): a 2-unknown least squares [gain now, change per step] solved step by step
%   in information form, prior = the filter's P44[0], P55[0], weight per step w_j = Delta(1/P44) of the FIRST-order run at the same
%   T_d (the per-step information on the gain through the e_a column of O, y2 + y1).  STATUS: SCRATCH
%   Runs: ladder_ramp16_Td5_lf1_o{1,2}.mat, ladder_ramp16_Td50_lf1_s{1..5}_o{1,2}.mat (test_results/temp_5state_noap)
function plot_5state_noap_descent_o2_td5_td50()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TD = [5 50];  COL_TRUE = [0.8 0 0];  C2 = [0 0.2 0.9];  CG = [0.1 0.6 0.1];  FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1700 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('\n%-6s | %-9s %-9s | %-12s %-12s | %-9s %-9s\n', 'T_d', 'a start', 'a end', 'a_hat end', 'line fit end', 'e_a end', 'e_a near');
    for c = 1:2
        r2 = local_load(id, TD(c), 2);  r1 = local_load(id, TD(c), 1);
        t = r2.t(:);  seg = r2.seg;  hd = r2.hd(:);  nS = size(r2.a_hat, 2);  fit = zeros(size(r2.a_hat));
        for q = 1:nS
            w = max([0; diff(1 ./ r1.P_a(:, q))], 0);
            fit(:, q) = local_line_fit(r2.a_true(:, q), w, r2.a_hat(1, q), r2.P_a(1, q), r2.P_da(1, q));
        end
        at = mean(r2.a_true, 2);  ah = mean(r2.a_hat, 2);  fv = mean(fit, 2);
        m = t >= seg(1) & t <= seg(2);  ts = t(m) - seg(1);  i0 = find(m, 1, 'first');  i1 = find(m, 1, 'last');
        mn = m & hd < 3;
        fprintf('%-6g | %-9.3f %-9.3f | %-12.3f %-12.3f | %+-9.3f %+-9.3f\n', TD(c), at(i0), at(i1), ah(i1), fv(i1), at(i1) - ah(i1), mean(at(mn) - ah(mn)));
        ax1 = nexttile(tl, c);  hold(ax1, 'on');
        h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', sprintf('second order, \\lambda_f = 1, descent 22.2 \\rightarrow 1.10 R in T_d = %g s', TD(c)));
        ht = plot(ax1, ts, at(m), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true} / a_{nom}');
        hw = plot(ax1, ts, fv(m), '--', 'Color', CG, 'LineWidth', 2.2, 'DisplayName', 'weighted line fit (section 4)');
        hf = plot(ax1, ts, ah(m), '-', 'Color', C2, 'LineWidth', 1.6, 'DisplayName', 'a_{hat} (filter)');
        plot(ax1, ts(end), ah(i1), 'o', 'Color', C2, 'MarkerFaceColor', C2, 'MarkerSize', 9);
        plot(ax1, ts(end), at(i1), 'o', 'Color', COL_TRUE, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 9);
        ylim(ax1, [0 1.1]);
        legend(ax1, [h0 ht hw hf], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        ax2 = nexttile(tl, 2 + c);  hold(ax2, 'on');
        plot(ax2, ts, at(m) - fv(m), '--', 'Color', CG, 'LineWidth', 2.2);
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
    out = fullfile(fd, 'fig_noap_descent_o2_td5_td50.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
function fit = local_line_fit(a, w, a0, P44, P55)
    % information-form least squares for [gain now; change per step], model gain[k+1] = gain[k] + change, change constant,
    % each step reads the gain with weight w(k); returns the fitted gain at every step
    n = numel(a);  fit = zeros(n, 1);  A = [1 1; 0 1];
    I = diag([1 / P44, 1 / P55]);  z = I * [a0; 0];               % information matrix and vector at step 1
    for k = 1:n
        if k > 1
            Ai = inv(A);  I = Ai.' * I * Ai;  z = Ai.' * z;          % move to step k (no process noise)
        end
        I = I + w(k) * [1 0; 0 0];  z = z + w(k) * [a(k); 0];       % add this step's reading of the gain
        x = I \ z;  fit(k) = x(1);
    end
end
function r = local_load(id, td, ord)
    f1 = fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_o%d.mat', td, ord));
    if exist(f1, 'file'); r = load(f1); return; end
    fl = dir(fullfile(id, sprintf('ladder_ramp16_Td%d_lf1_s*_o%d.mat', td, ord)));
    assert(~isempty(fl), 'no runs for T_d = %g, order %d', td, ord);
    [~, ix] = sort({fl.name});  fl = fl(ix);
    for q = 1:numel(fl)
        s = load(fullfile(id, fl(q).name));
        if q == 1; r = s; else
            for fld = {'a_true', 'a_hat', 'P_a', 'P_da', 'hb'}
                if isfield(s, fld{1}); r.(fld{1}) = [r.(fld{1}), s.(fld{1})]; end
            end
        end
    end
end
