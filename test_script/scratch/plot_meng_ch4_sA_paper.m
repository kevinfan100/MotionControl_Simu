% STATUS: ACTIVE (scratch) | PURPOSE: Scenario-A figures in the PAPER's panel
%   layout (journal Fig 5: left = with real-time estimation, right = with
%   gamma_N, x/y/z stacked, shared y per row; journal Fig 6-left: the three
%   motion gains, estimate vs true) with the lambda_f comparison overlaid on
%   the gain rows. House style: no grid/title, True red / Estimate blue /
%   Measured light blue, legend northoutside, exportgraphics 150.
%   | EXPIRES: with run_meng_ch4_sA.
function plot_meng_ch4_sA_paper(outs, out_dir)
%   outs: struct with fields e995, e999, e100 (run_meng_ch4_sA outputs)

    if nargin < 2 || isempty(out_dir)
        here = fileparts(mfilename('fullpath'));
        root = fileparts(fileparts(here));
        out_dir = fullfile(root, 'test_results', 'meng_ch4_s0');
    end
    C_TRUE = [0.85 0.10 0.10]; C_EST = [0.10 0.25 0.75]; C_MEAS = [0.55 0.75 0.95];

    o  = outs.e995;  t = o.t;
    rE = o.E98{1};   rN = o.N{1};
    lab = {'x', 'y', 'z'};

    % ---- Fig 5 counterpart: paper layout (left est / right gamma_N) ----
    f1 = figure('Position', [40 40 1000 720], 'Color', 'w');
    tl = tiledlayout(f1, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ax = 1:3
        dE = o.pd(:, ax) - rE.p_true(:, ax);
        dN = o.pd(:, ax) - rN.p_true(:, ax);
        yl = max(max(abs(dE)), max(abs(dN))) * 1.1;
        a1 = nexttile(tl, 2*ax-1); box(a1, 'on');
        plot(a1, t, dE, '-', 'Color', C_EST, 'LineWidth', 0.6);
        ylim(a1, [-yl yl]); ylabel(a1, sprintf('\\delta%s [\\mum]', lab{ax}));
        if ax == 1
            title(a1, 'with real-time estimation', 'FontWeight', 'normal');
        end
        if ax == 3; xlabel(a1, 't [s]'); end
        a2 = nexttile(tl, 2*ax); box(a2, 'on');
        plot(a2, t, dN, '-', 'Color', C_MEAS, 'LineWidth', 0.6);
        ylim(a2, [-yl yl]);
        if ax == 1
            title(a2, 'with \gamma_N (no estimation)', 'FontWeight', 'normal');
        end
        if ax == 3; xlabel(a2, 't [s]'); end
    end
    fp1 = fullfile(out_dir, 'meng_ch4_sA_paperfig5.png');
    exportgraphics(f1, fp1, 'Resolution', 150);

    % ---- Fig 6 counterpart: three gains stacked, lambda_f comparison ----
    f2 = figure('Position', [80 80 900 760], 'Color', 'w');
    t2 = tiledlayout(f2, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    arms = {outs.e995, outs.e999, outs.e100};
    st   = {':', '-', '--'};
    lw   = [0.6 0.8 0.9];
    for ax = 1:3
        b = nexttile(t2); hold(b, 'on'); box(b, 'on');
        plot(b, t, outs.e999.E98{1}.a_true(:, ax), '-', 'Color', C_TRUE, ...
             'LineWidth', 1.5);
        for j = 1:3
            plot(b, t, arms{j}.E98{1}.a_hat(:, ax), st{j}, 'Color', C_EST, ...
                 'LineWidth', lw(j));
        end
        ylabel(b, sprintf('a_%s [\\mum/pN]', lab{ax}));
        if ax == 1
            legend(b, {'true', '\lambda_f=0.995', '\lambda_f=0.999', ...
                       '\lambda_f=1'}, 'Location', 'northoutside', ...
                   'Orientation', 'horizontal');
        end
        if ax == 3; xlabel(b, 't [s]'); end
    end
    fp2 = fullfile(out_dir, 'meng_ch4_sA_paperfig6.png');
    exportgraphics(f2, fp2, 'Resolution', 150);
    fprintf('figures:\n  %s\n  %s\n', fp1, fp2);
end
