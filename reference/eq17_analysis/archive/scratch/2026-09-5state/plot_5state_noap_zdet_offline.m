% PURPOSE (2026-09-24): offline preview of the residual-mean forgetting rule before it goes into the controller.
%   z1 = mean_W(nu_1) / sqrt(S_1 / n)  (per seed, window W),  lambda_f = max(exp(-alpha max(z1^2 - Z0^2, 0)), 0.98), Z0 = 2.
%   (First version had no deadband: a healthy N(0,1) z1 then gives E[z1^2] = 1 and the far field forgets at exp(-alpha) every step.)
%   Applied to the LOGGED innovations of two existing arms on the same ramp (22.2 -> 1.10 R over 5 s, second order):
%   the schedule arm (small lag, what the rule would see in normal operation) and the lambda_f = 1 arm (large lag).
%   Open loop only: shows WHEN the rule would fire and whether the far field stays quiet. Closed-loop numbers need a run.
%   Rows: e_a | lambda_f(t) for W = 0.05 s | lambda_f(t) for W = 0.25 s (three alpha each).   STATUS: SCRATCH
function plot_5state_noap_zdet_offline()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    arms = {'_Td5', '_Td5_lf1'};  arm_names = {'\lambda_f schedule log', '\lambda_f = 1 log'};
    ALPHA = [0.002 0.005 0.02];  WIN = [0.05 0.25];  LF_MIN = 0.98;  Z0 = 2;   % deadband: forget only for |z1| > Z0 (95 % of healthy windows stay at 1)
    CA = {[0 0.2 0.9], [0.85 0.45 0], [0.1 0.6 0.1]};  FS = 18; LFS = 12; AXLW = 2.0;
    f = figure('Position', [40 40 1800 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        r = load(fullfile(id, sprintf('ladder_ramp16%s_o2.mat', arms{c})));
        t = r.t(:);  Ts = t(2) - t(1);  seg = r.seg;  hm = mean(r.hb, 2);
        m_far = t > seg(1) & hm > 10;  m_near = hm < 3 & t > seg(1) & t <= seg(2);  m_end = t > seg(2);
        ea = mean(r.a_true - r.a_hat, 2);
        ax = nexttile(tl, c); hold(ax, 'on');
        plot(ax, t, ea, '-', 'Color', [0 0.2 0.9], 'LineWidth', 1.8);  plot(ax, [0 t(end)], [0 0], '-', 'Color', [0.6 0.6 0.6]);
        local_seg(ax, seg);  xlim(ax, [0 t(end)]);  local_ax(ax, FS, AXLW);
        if c == 1; ylabel(ax, 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        title(ax, arm_names{c}, 'FontSize', LFS, 'FontWeight', 'bold');
        fprintf('\n=== %s ===\n', arm_names{c});
        fprintf('   W [s]  alpha | mean lambda_f  far / near / hold-end | steps with lambda_f < 0.999  far / near / hold-end\n');
        for w = 1:numel(WIN)
            nw = max(1, round(WIN(w) / Ts));
            z1 = movmean(r.innov1, nw, 1) ./ sqrt(movmean(r.S_y1, nw, 1) / nw);      % per seed
            ax = nexttile(tl, 2 * w + c); hold(ax, 'on');  hh = gobjects(1, numel(ALPHA));
            for a = 1:numel(ALPHA)
                lf = max(exp(-ALPHA(a) * max(z1.^2 - Z0^2, 0)), LF_MIN);                  % per seed, deadband
                lfm = mean(lf, 2);  on = lf < 0.999;
                hh(a) = plot(ax, t, lfm, '-', 'Color', CA{a}, 'LineWidth', 1.6, 'DisplayName', sprintf('\\alpha = %g', ALPHA(a)));
                fprintf('   %.2f   %.3f | %.4f / %.4f / %.4f | %3.0f%% / %3.0f%% / %3.0f%%\n', WIN(w), ALPHA(a), ...
                    mean(lfm(m_far)), mean(lfm(m_near)), mean(lfm(m_end)), ...
                    100 * mean(on(m_far, :), 'all'), 100 * mean(on(m_near, :), 'all'), 100 * mean(on(m_end, :), 'all'));
            end
            plot(ax, [0 t(end)], [1 1], '-', 'Color', [0.6 0.6 0.6]);  plot(ax, [0 t(end)], [LF_MIN LF_MIN], 'r--', 'LineWidth', 1.2);
            local_seg(ax, seg);  xlim(ax, [0 t(end)]);  ylim(ax, [LF_MIN - 0.002, 1.001]);  local_ax(ax, FS, AXLW);
            if c == 1; ylabel(ax, sprintf('\\lambda_f  (W = %.2f s)', WIN(w)), 'FontSize', FS, 'FontWeight', 'bold'); end
            if w == numel(WIN); xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if w == 1; legend(ax, hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on'); end
        end
    end
    out = fullfile(fd, 'fig_noap_zdet_offline.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
function local_seg(ax, seg)
    yl = ylim(ax);  for s = seg(:).'; plot(ax, [s s], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); end
end
function local_ax(ax, FS, AXLW)
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
end
