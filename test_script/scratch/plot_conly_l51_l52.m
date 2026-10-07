% PURPOSE (2026-10-02, user: "draw c-only the same way as first order: Q = 0 vs the P0 design, l51 and l52"): the c-only
%   slope law (law 'f1p2': a' = (1 - c)/h^2, c in slot 5, p pinned at 2), estimator init on the plane (c = 0.0533, prior 0.10),
%   ramp 22.2 -> 1.10 R in T_d = 5 s, plane | cell, 10 seeds. Arms: Q55 = 0 (grey) vs ap model's Q/R-ratio design
%   q55_adapt = 'ratio', sqrt(P0) = 0.2 (blue). Controller settings copied from the ap model's saved runs
%   (temp_plaw_walls/f1p2_<wall>_PI_sig010_ramp5.mat, field cc; ratio arm = the same + q55_adapt, as its rho20 runs).
%   Rows: |L52| = |K(5,2)| | |L51| = |K(5,1)|, the raw KF gain entries of the c row (x = time from the descent start).
%   Runs cached in test_results/temp_5state_noap/conly_<wall>_<arm>.mat.   STATUS: SCRATCH
function plot_conly_l51_l52(seeds)
    if nargin < 1; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pw = fullfile(root, 'test_results', 'temp_plaw_walls');
    A_COV = 0.05;  T_d = 5;  WALLS = {'plane', 'cell'};
    ARMS = {'q0', 'Q_{55} = 0', [0.4 0.4 0.4], struct();
            'ratio', 'Q/R ratio, \surdP_0 = 0.2', [0 0.2 0.9], struct('q55_adapt', 'ratio', 'q55_adapt_P0', 0.2);
            'r2r', 'Q_{55} = 0.03 R_2 (constant ratio)', [0.1 0.6 0.1], struct('q55_adapt', 'r2ratio', 'q55_adapt_rho', 0.03)};   % 10-02: best of ap's r2ratio scan at T_d = 2 s
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1800 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        wall = WALLS{c};  pcp = [];  if strcmp(wall, 'cell'); pcp = @cell_cperp_delta; end
        base = load(fullfile(pw, sprintf('f1p2_%s_PI_sig010_ramp5.mat', wall)), 'cc');
        ax1 = nexttile(c); hold(ax1, 'on'); box(ax1, 'on');  ax2 = nexttile(2 + c); hold(ax2, 'on'); box(ax2, 'on');
        hl = plot(ax1, NaN, NaN, 'w.', 'DisplayName', sprintf('c-only (a'' = (1-c)/h^2), %s wall, plane init, T_d = %g s, %d seeds', wall, T_d, numel(seeds)));
        for m = 1:size(ARMS, 1)
            fn = fullfile(id, sprintf('conly_%s_%s.mat', wall, ARMS{m, 1}));
            if exist(fn, 'file'); D = load(fn);
            else
                cc = base.cc;  for fq = fieldnames(ARMS{m, 4}).'; cc.(fq{1}) = ARMS{m, 4}.(fq{1}); end
                OV = struct('trajectory_type', 'ramp_descent', 't_hold', 0.5, 't_descend_override', T_d, 'T_sim', 0.5 + T_d + 0.5, ...
                            'amplitude', 0, 'frequency', 1, 'n_cycles', 1);
                o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                           'seeds', seeds, 'log_P_full', false);
                if ~isempty(pcp); o.plant_cperp = pcp; end
                clear run_formC_b motion_control_law_formC_b;
                evalc('Rr = run_formC_b(o);');
                ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);  Ts = 0.000625;
                D = struct('t', (0:n - 1)' * Ts - 0.5, 'K1', zeros(n, ns), 'K2', zeros(n, ns), 'c_hat', zeros(n, ns), 'R_um', S1.R);
                for q = 1:ns
                    S = Rr.runs{q};  D.K1(:, q) = S.K_b_y1_out(:, 3);  D.K2(:, q) = S.K_b_y2_out(:, 3);  D.c_hat(:, q) = S.b_hat_out(:, 3);
                end
                save(fn, '-struct', 'D');
            end
            t = D.t;  L2 = abs(D.K2);  L1 = abs(D.K1);                     % raw KF gain entries of the c row: L52 = K(5,2), L51 = K(5,1)
            hl(end + 1) = plot(ax1, t, mean(L2, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 2.0, 'DisplayName', ARMS{m, 2}); %#ok<AGROW>
            plot(ax2, t, mean(L1, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.2);
            for w = {[-0.4 0], [1 2], [4.5 5], [5 5.5]}
                k = t >= w{1}(1) & t < w{1}(2);
                fprintf('%-5s %-5s t %4.1f-%4.1f | |L52| %.2e | |L51| %.2e | c_hat %.3f\n', wall, ARMS{m, 1}, w{1}(1), w{1}(2), ...
                    mean(L2(k, :), 'all'), mean(L1(k, :), 'all'), mean(D.c_hat(k, :), 'all'));
            end
        end
        set(ax1, 'YScale', 'log');  set(ax2, 'YScale', 'log');
        if c == 1
            ylabel(ax1, '|L_{52}|  (KF gain of c on y_2)', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax2, '|L_{51}|  (KF gain of c on y_1)', 'FontSize', FS, 'FontWeight', 'bold');
        end
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for ax = [ax1 ax2]
            xlim(ax, [t(1) t(end)]);  yl = ylim(ax);
            for x = [0 T_d]; plot(ax, [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
        end
        legend(ax1, hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    end
    out = fullfile(fd, 'fig_conly_l51_l52_td5.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
