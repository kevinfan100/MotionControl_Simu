% FORK OF test_script/integration/plot_formC_dist_compare.m @ 10e51db |
%   PURPOSE: the (a_pd, a_cov) paired page for pair_apd_acov_meng -- adds the
%   two rows the sibling has no reason to carry, namely how much of each
%   update actually came from y2, and R2 against the measured per-sample
%   var(y2) | EXPIRES: with pair_apd_acov_meng | production does NOT follow.
%
%   Left column  = arm A (production a_pd/a_cov)
%   Right column = arm B (the candidate)
%   Row y-limits shared across columns so the arms compare by eye. Arm names
%   ride in the row-1 legend because the style forbids titles.
%
%   Rows:
%     1  a_z/a_o : true (red) / estimate (blue) / a_m readout (light blue)
%                  -- the width of the light-blue band IS var(a_m)
%     2  a_z/a_o error, estimate - true
%     3  b : b_true (red) / b_hat (blue) with the +-sqrt(P55) band
%     4  y2 share of the a_bar update, |L42 e2| / (|L41 e1| + |L42 e2|)
%     5  y2 share of the b update,     |L52 e2| / (|L51 e1| + |L52 e2|)
%     6  R2 (solid) vs sliding-window measured var(y2) (dashed), log axis
%   Shares are built from K*innovation, not from K: the two legs have
%   different units, so |K1| + |K2| would be meaningless.
function plot_pair_apd_acov_meng(out, opts)

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds_to_plot'); opts.seeds_to_plot = out.seeds; end
    if ~isfield(opts, 'win');           opts.win = 400; end   % 0.25 s at 1600 Hz

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od   = fullfile(root, 'test_results', 'apd_acov_meng');
    if ~exist(od, 'dir'); mkdir(od); end
    AX = out.AX;

    b_of_w = local_b_true_curve();
    names = cell(1, 2);
    for c = 1:2
        names{c} = sprintf('a_{pd} %.4g , a_{cov} %.4g', out.ARMS{c}.a_pd, out.ARMS{c}.a_cov);
    end
    names{1} = [names{1} '  (production)'];

    for q = 1:numel(out.seeds)
        sd = out.seeds(q);
        if ~any(opts.seeds_to_plot == sd); continue; end
        d = cell(1, 2);
        for c = 1:2
            d{c} = local_pull(out.O{c}.runs{q}, AX, out.ARMS{c}.a_cov, b_of_w, opts.win);
        end
        f = fullfile(od, sprintf('pair_apd_acov_meng_s%03d.png', sd));
        local_page(d, names, f);
        fprintf('seed %3d -> %s\n', sd, f);
    end
end

% =====================================================================
function s = local_pull(r, ax, a_cov, b_of_w, win)
    ad = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);     % [um/pN] per unit a_bar
    s.t  = r.tout(2:end);
    s.aT = r.a_true_out(2:end, ax) / ad;
    s.aH = r.a_bar_hat_out(2:end, ax);
    s.aM = r.a_xm_out(2:end, ax) / ad;                    % a_bar_wm, pre-whitening
    s.e  = s.aH - s.aT;
    s.bH = r.b_hat_out(2:end, ax);
    s.bS = r.P_b_out(2:end, ax);                          % already a std
    s.bT = b_of_w(r.h_bar_true_out(2:end, 1));
    i1 = r.innov_y1_out(2:end, ax);   i2 = r.innov_y2_out(2:end, ax);
    ca1 = abs(r.K_a_y1_out(2:end, ax) .* i1);  ca2 = abs(r.K_a_y2_out(2:end, ax) .* i2);
    cb1 = abs(r.K_b_y1_out(2:end, ax) .* i1);  cb2 = abs(r.K_b_y2_out(2:end, ax) .* i2);
    s.wa = movmean(ca2 ./ max(ca1 + ca2, realmin), win);
    s.wb = movmean(cb2 ./ max(cb1 + cb2, realmin), win);
    y2   = s.aM - (1 - a_cov) * [s.aM(1); s.aM(1:end-1)];
    s.vy2 = movvar(y2, win);
    s.R2  = r.R2_out(2:end, ax);
    s.ratio = median(s.vy2(win:end) ./ s.R2(win:end), 'omitnan');
end

% ---------------------------------------------------------------------
function local_page(d, names, outfile)
    COL_T = [0.85 0.10 0.10];  COL_E = [0.00 0.20 0.90];
    COL_M = [0.45 0.72 0.95];  COL_G = [0.55 0.55 0.55];
    FS = 15; LFS = 11; AXLW = 1.8; LW = 1.6;
    f = figure('Position', [30 30 1500 1750], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 6, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    A = gobjects(6, 2);
    for c = 1:2
        s = d{c};
        % ---- row 1 : gain tracking + the raw readout band ----
        a = nexttile(tl, c); hold(a, 'on'); A(1, c) = a;
        hm = plot(a, s.t, s.aM, '-', 'Color', COL_M, 'LineWidth', 0.5);
        ht = plot(a, s.t, s.aT, '-', 'Color', COL_T, 'LineWidth', 2.4);
        he = plot(a, s.t, s.aH, '-', 'Color', COL_E, 'LineWidth', 1.6);
        legend(a, [ht he hm], {'a_{true}', 'estimate', ...
               ['a_m  (y_2 readout)   ' names{c}]}, 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
        if c == 1; ylabel(a, 'a_z / a_o', 'FontSize', FS, 'FontWeight', 'bold'); end
        % ---- row 2 : error ----
        a = nexttile(tl, 2 + c); hold(a, 'on'); A(2, c) = a;
        yline(a, 0, '-', 'Color', COL_G, 'LineWidth', 1.0);
        plot(a, s.t, s.e, '-', 'Color', COL_E, 'LineWidth', LW);
        if c == 1
            ylabel(a, {'a_z / a_o  error', '(est - true)'}, 'FontSize', FS, 'FontWeight', 'bold');
        end
        % ---- row 3 : b ----
        a = nexttile(tl, 4 + c); hold(a, 'on'); A(3, c) = a;
        fill(a, [s.t; flipud(s.t)], [s.bH - s.bS; flipud(s.bH + s.bS)], COL_E, ...
             'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(a, s.t, s.bT, '-', 'Color', COL_T, 'LineWidth', 2.4);
        h2 = plot(a, s.t, s.bH, '-', 'Color', COL_E, 'LineWidth', LW);
        legend(a, [h1 h2], {'b_{true}', 'b_{hat}  (\pm\surdP_{55})'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'Box', 'on');
        if c == 1; ylabel(a, 'b', 'FontSize', FS, 'FontWeight', 'bold'); end
        % ---- row 4 : y2 share of the a update ----
        a = nexttile(tl, 6 + c); hold(a, 'on'); A(4, c) = a;
        plot(a, s.t, s.wa, '-', 'Color', [0.10 0.55 0.15], 'LineWidth', 2.0);
        if c == 1
            ylabel(a, {'y_2 share of', 'the a_z update'}, 'FontSize', FS, 'FontWeight', 'bold');
        end
        % ---- row 5 : y2 share of the b update ----
        a = nexttile(tl, 8 + c); hold(a, 'on'); A(5, c) = a;
        plot(a, s.t, s.wb, '-', 'Color', [0.55 0.30 0.75], 'LineWidth', 2.0);
        if c == 1
            ylabel(a, {'y_2 share of', 'the b update'}, 'FontSize', FS, 'FontWeight', 'bold');
        end
        % ---- row 6 : R2 vs measured var(y2) ----
        a = nexttile(tl, 10 + c); hold(a, 'on'); A(6, c) = a;
        h1 = plot(a, s.t, s.R2,  '-',  'Color', [0.85 0.33 0.10], 'LineWidth', 2.0);
        h2 = plot(a, s.t, s.vy2, '--', 'Color', [0.00 0.20 0.90], 'LineWidth', 1.6);
        set(a, 'YScale', 'log');
        legend(a, [h1 h2], {'R_2  (what the KF assumes)', ...
               sprintf('var(y_2) measured   ratio %.3f', s.ratio)}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'Box', 'on');
        xlabel(a, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        if c == 1
            ylabel(a, {'R_2   and', 'var(y_2)'}, 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    for row = 1:6
        for c = 1:2
            set(A(row, c), 'FontSize', FS, 'FontWeight', 'bold', ...
                'LineWidth', AXLW, 'Box', 'on');
            xlim(A(row, c), [d{1}.t(1) d{1}.t(end)]);
            if row < 6; set(A(row, c), 'XTickLabel', []); end
        end
        lo = min(cellfun(@(a) a(1), {ylim(A(row,1)), ylim(A(row,2))}));
        hi = max(cellfun(@(a) a(2), {ylim(A(row,1)), ylim(A(row,2))}));
        ylim(A(row, 1), [lo hi]); ylim(A(row, 2), [lo hi]);
        set(A(row, 2), 'YTickLabel', []);
    end
    exportgraphics(f, outfile, 'Resolution', 150);
    close(f);
end

% ---------------------------------------------------------------------
function b_of_w = local_b_true_curve()
%LOCAL_B_TRUE_CURVE  b_true(w) = a'_true / (1 - a_true)^2 from the published
%   truth curve, exactly as measure_b_tracking_and_sensitivity.m builds it.
    wq = linspace(1.05, 30, 4000);
    cp = zeros(size(wq));
    for i = 1:numel(wq)
        [~, cperp] = calc_correction_functions(wq(i));
        cp(i) = cperp;
    end
    a_tr = 1 ./ cp;
    b_tr = gradient(a_tr, wq) ./ (1 - a_tr).^2;
    b_of_w = @(w) interp1(wq, b_tr, w, 'linear', 'extrap');
end
