% PURPOSE (2026-09-30, user: "start from Q44, try Q44 as a fraction of R22, compare a few rho"):
%   OFFLINE replay of the y2-only scalar Kalman filter (0929_kf_two_tracks.tex 4.1-4.2) on the logged runs
%   y2avg_Td{5,50}_s{1..3} (plot_5state_noap_y2_average.m). Per step:
%     u[k]   = (a_m[k] - (1 - a_cov) a_m[k-1]) / a_cov        the whitened y2 reading, in gain units
%     R2[k]  = 1 / w_y2[k]                                     (w_y2 = H2(4)^2/R22 logged by the controller)
%     Pf = P + Q44[k];  l42 = Pf/(Pf + R2);  a_hat += l42 (u - a_hat);  P = (1 - l42) Pf
%   Arms: Q44 = rho * R2[k] (memory 1/sqrt(rho) steps, fixed) and one fixed Q44 (memory follows R2).
%   No y1, open loop (the logged run's control and R2 are reused).  Rows: a_hat vs a_true | e_a.  Cols: T_d = 5 | 50 s.
%   STATUS: SCRATCH
function plot_5state_noap_q44_offline()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  Ts = pc.Ts;  a_cov = 0.05;  T_HOLD0 = 0.5;
    RHO = [0 1e-6 1e-5 1e-4];
    Q_FIX = 1e-5 * (0.30 * a_nom)^2 / 1;     % fixed-Q arm, set below from the far-field R2 so it matches rho = 1e-5 there
    COL = [0 0 0; 0 0.45 0.85; 0.1 0.6 0.1; 0.85 0.45 0];  C_FIX = [0.6 0 0.6];  COL_TRUE = [0.8 0 0];
    FS = 18;  LFS = 12;  AXLW = 2.0;  TD = [5 50];  seeds = 1:3;
    f = figure('Position', [40 40 1800 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('\n%-5s %-22s | %-10s | %-9s %-9s | %-9s\n', 'T_d', 'arm', 'memory [s]', 'e_a stop', 'e_a +0.5s', 'far sd');
    for c = 1:2
        T_d = TD(c);  runs = cell(1, numel(seeds));
        for q = 1:numel(seeds); runs{q} = load(fullfile(id, sprintf('y2avg_Td%d_s%d.mat', T_d, seeds(q)))); end
        t = runs{1}.t - T_HOLD0;  n = numel(t);  i1 = find(t <= T_d, 1, 'last');  i2 = find(t <= T_d + 0.5, 1, 'last');
        far = t > -0.4 & t <= 0;
        if c == 1   % fixed Q44 = rho 1e-5 times the far-field R2 of the first run
            r = runs{1};  R2far = median(1 ./ r.w_y2(far & r.w_y2 > 0));  Q_FIX = 1e-5 * R2far;
        end
        at = zeros(n, 1);  for q = 1:numel(seeds); at = at + runs{q}.a_true / numel(seeds); end
        ax1 = nexttile(tl, c);  hold(ax1, 'on');  ax2 = nexttile(tl, 2 + c);  hold(ax2, 'on');
        h0 = plot(ax1, nan, nan, 'w-', 'DisplayName', sprintf('offline y_2-only KF, plane, 22.2 \\rightarrow 1.10 R in T_d = %g s', T_d));
        ht = plot(ax1, t, at / a_nom, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true}');
        hh = gobjects(1, numel(RHO) + 1);
        for m = 1:numel(RHO) + 1
            ah = zeros(n, numel(seeds));
            for q = 1:numel(seeds)
                r = runs{q};  R2 = 1 ./ r.w_y2;  u = (r.a_m - (1 - a_cov) * [r.a_m(1); r.a_m(1:end-1)]) / a_cov;
                R2q = R2;  R2q(~isfinite(R2q)) = NaN;  R2q = fillmissing(fillmissing(R2q, 'previous'), 'next');
                if m <= numel(RHO); Q = RHO(m) * R2q; else; Q = Q_FIX * ones(n, 1); end
                ah(:, q) = local_kf(u, R2, Q, r.a_hat(1), r.P_a(1));
            end
            if m <= numel(RHO)
                col = COL(m, :);  ls = '-';
                if RHO(m) == 0; lab = '\rho = 0 (Q_{44} = 0)'; mem = Inf; else; lab = sprintf('\\rho = %g', RHO(m)); mem = Ts / sqrt(RHO(m)); end
            else
                col = C_FIX;  ls = '--';  lab = 'Q_{44} fixed (= 10^{-5} R_2 far field)';  mem = NaN;
            end
            e = at - mean(ah, 2);  sdf = mean(std(ah(far, :) / a_nom, 0, 1));
            fprintf('%-5g %-22s | %-10.3f | %+-9.3f %+-9.3f | %-9.4f\n', T_d, strrep(strrep(lab, '\', ''), '_{44}', '44'), mem, e(i1) / a_nom, e(i2) / a_nom, sdf);
            hh(m) = plot(ax1, t, mean(ah, 2) / a_nom, ls, 'Color', col, 'LineWidth', 1.8, 'DisplayName', lab);
            plot(ax2, t - T_d, e / a_nom, ls, 'Color', col, 'LineWidth', 1.8);
        end
        xlim(ax1, [-T_HOLD0 t(end)]);  ylim(ax1, [0 1.2]);  xlim(ax2, [-min(T_d, 5) 2]);  ylim(ax2, [-0.9 0.3]);
        xlabel(ax1, 'time since the descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax2, 'time since the stop (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for ax = [ax1 ax2]
            plot(ax, xlim(ax), [0 0], '-', 'Color', [0.6 0.6 0.6]);
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        end
        yl = ylim(ax2);  plot(ax2, [0 0], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
        legend(ax1, [h0 ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'AutoUpdate', 'off');
        if c == 1
            ylabel(ax1, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax2, 'e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, 'fig_noap_q44_offline_rho.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function ah = local_kf(u, R2, Q, a0, P0)
    n = numel(u);  ah = zeros(n, 1);  a = a0;  P = P0;
    for k = 1:n
        Pf = P + Q(k);
        if isfinite(R2(k))
            l = Pf / (Pf + R2(k));  a = a + l * (u(k) - a);  P = (1 - l) * Pf;
        else
            P = Pf;
        end
        ah(k) = a;
    end
end
