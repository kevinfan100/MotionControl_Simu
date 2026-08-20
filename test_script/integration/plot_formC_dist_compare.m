function plot_formC_dist_compare(oB, oD, opts)
%PLOT_FORMC_DIST_COMPARE  The with/without-delta-a comparison, one page per seed.
%
%   Left column  = derivation (a): no disturbance state, slot 5 locked at 0
%   Right column = derivation (b): additive disturbance delta a, Q55 = 0
%   Everything else identical, and the two arms run on the same random draws,
%   so each page is a paired comparison.
%
%   Rows (per formB_ws.tex figure style):
%     1  gain tracking      a_bar true (red) / estimate (blue) / measured (light blue)
%     2  relative gain error [%]
%     3  the free parameter: delta a with its +-sqrt(P55) band
%        (flat at zero on the left by construction -- that IS the arm)
%   Row y-limits are shared across the two columns so the arms compare by eye.
%
%   Usage: L = load('test_results/formC_dist_8seed_pair.mat');
%          plot_formC_dist_compare(L.oB, L.oD)
%
%   opts.suffix  ''  tag inserted into the file name, so a second scenario
%                    does not overwrite the canonical pages. '' reproduces
%                    the original formC_dist_cmp_s<seed>.png exactly.
%
%   Style: canonical (plot_var_ahat_6state.m): 18 pt bold sans, 2.0 pt axes,
%   no grid, box on, legend northoutside horizontal, no title, Resolution 150.

    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'Ts');     opts.Ts = 6.25e-4; end   % logged dt (1600 Hz)
    if ~isfield(opts, 'suffix'); opts.suffix = ''; end     % scenario tag
    if ~isfield(opts, 'names');  opts.names  = {'no \delta a', 'with \delta a'}; end
    % Row 3 plots STATE SLOT 5, whose meaning depends on the writing: the
    % additive disturbance in formC_state_dist, the law constant b in
    % formC_state_b. The label must follow the caller, not the file it was
    % forked from.
    if ~isfield(opts, 'slot5_name'); opts.slot5_name = '\delta a'; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AX = 3;  seeds = oB.seeds(:).';

    for q = 1:numel(seeds)
        d = {local_pull(oB.runs{q}, AX, opts.Ts), local_pull(oD.runs{q}, AX, opts.Ts)};
        out = fullfile(fig_dir, sprintf('formC_dist_cmp%s_s%02d.png', ...
                                        opts.suffix, seeds(q)));
        local_page(d, seeds(q), out, opts.names, opts.slot5_name);
        fprintf('seed %3d : no-da relerr RMS %6.3f %%  |  with-da %6.3f %%   da_end %+.3e\n', ...
                seeds(q), sqrt(mean(d{1}.e.^2)), sqrt(mean(d{2}.e.^2)), d{2}.da(end));
    end
    fprintf('[dist compare] wrote %d figures -> %s\n', numel(seeds), fig_dir);
end

% --------------------------------------------------------------------------
function s = local_pull(r, ax, Ts)
    ad = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);
    s.aT = r.a_true_out(:, ax) / ad;
    s.aH = r.a_bar_hat_out(:, ax);
    s.aM = r.a_xm_out(:, ax) / ad;
    s.e  = 100 * (s.aH - s.aT) ./ s.aT;
    s.da = r.b_hat_out(:, ax);          % slot 5 = the additive disturbance here
    s.sd = r.P_b_out(:, ax);            % already a std
    % Tracking error in the derivation's sign and units: S3 writes
    % w_bar[k] = w_bar_d[k] - dw3[k], so dw3 = commanded minus actual, and
    % R*dw3 is that in um.
    %
    % p_d_out and p_true_out are logged one sample apart: p_d_out[k] is the
    % command the step is driving TOWARDS, p_true_out[k] the position it starts
    % from. Differencing them at the same index adds one commanded step of bias
    % -- 28.455 nm on this trajectory, against a measured descent bias of
    % 27.70 nm, which is the whole of it. Aligned, the descent bias is
    % -0.74 nm and the overall RMS falls 29.12 -> 25.08 nm.
    % Caught 2026-08-18 by a cross-arm check: the bias did not move when the
    % gain error changed 8x across arms, which killed the gain-overshoot
    % explanation and left the indexing.
    s.dw = [NaN; r.p_d_out(2:end, ax) - r.p_true_out(1:end-1, ax)];
    % Row 3's reference. Slot 5 used to be an additive disturbance whose true
    % value is zero, so the row carried a yline at 0 labelled "zero". Slot 5 is
    % now the law constant b, whose reference is b_true AT THE PARTICLE'S OWN
    % HEIGHT -- a curve, not a level. Keeping the zero line was the same mistake
    % as drawing one placement's target on another placement's column: it made
    % the axis span 0..1 and hid the whole of b_hat's motion in the top decile.
    %   b_true(w_bar) = a_bar'_true / (1 - a_bar_true)^2
    pc   = physical_constants();
    s.bT = local_b_true(max(r.p_true_out(:, ax) / pc.R, 1.0005));
    s.t  = (0:numel(s.aH)-1).' * Ts;
end

function local_page(d, sq, out, NAME, SLOT5)
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_MEAS = [0.45 0.72 0.95];
    BANDC = [0.45 0.55 0.95];
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

    YL = cell(4, 1);
    YL{1} = local_lim([d{1}.aT; d{1}.aH; d{1}.aM; d{2}.aT; d{2}.aH; d{2}.aM], 0.05);
    YL{2} = local_lim([d{1}.e;  d{2}.e], 0.08);
    YL{3} = local_lim([d{1}.da + d{1}.sd; d{1}.da - d{1}.sd; ...
                       d{2}.da + d{2}.sd; d{2}.da - d{2}.sd; ...
                       d{1}.bT; d{2}.bT], 0.10);
    YL{4} = local_lim([d{1}.dw; d{2}.dw], 0.08);

    f = figure('Position', [40 40 1500 1380], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for row = 1:4
        for c = 1:2
            s = d{c};  a = nexttile(tl, (row-1)*2 + c);  hold(a, 'on');
            switch row
                case 1
                    h0 = plot(a, s.t, s.aM, '-', 'Color', COL_MEAS, 'LineWidth', 1.0, ...
                              'DisplayName', 'a_{m} readout (pre-whitening)');
                    h1 = plot(a, s.t, s.aT, '-', 'Color', COL_TRUE, 'LineWidth', LW + 0.6, ...
                              'DisplayName', 'a_{true}');
                    h2 = plot(a, s.t, s.aH, '-', 'Color', COL_HAT, 'LineWidth', LW, ...
                              'DisplayName', sprintf('a_{hat}  %s  seed %d', NAME{c}, sq));
                    legend(a, [h1 h2 h0], 'Location', 'northoutside', ...
                           'Orientation', 'horizontal', 'FontSize', LFS, ...
                           'FontWeight', 'bold', 'Box', 'on');
                    if c == 1
                        ylabel(a, 'a_z / a_o', 'FontSize', FS, 'FontWeight', 'bold');
                    end
                case 2
                    yline(a, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, ...
                          'HandleVisibility', 'off');
                    plot(a, s.t, s.e, '-', 'Color', COL_HAT, 'LineWidth', LW);
                    if c == 1
                        ylabel(a, 'a_z  error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
                    end
                case 3
                    fill(a, [s.t; flipud(s.t)], [s.da + s.sd; flipud(s.da - s.sd)], ...
                         BANDC, 'EdgeColor', 'none', 'FaceAlpha', 0.30, ...
                         'DisplayName', '\pm sqrt(P_{55})');
                    plot(a, s.t, s.bT, '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
                         'DisplayName', [SLOT5 '_{true} at the particle']);
                    plot(a, s.t, s.da, '-', 'Color', COL_HAT, 'LineWidth', LW, ...
                         'DisplayName', [SLOT5 '_{hat}']);
                    legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                    if c == 1
                        ylabel(a, SLOT5, 'FontSize', FS, 'FontWeight', 'bold');
                    end
                case 4
                    % tracking error, R*dw3 in um. Two decimals everywhere so
                    % the two columns' ticks line up.
                    plot(a, s.t, s.dw, '-', 'Color', COL_HAT, 'LineWidth', 0.8);
                    yline(a, 0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
                    if c == 1
                        % tex, like every other row. Using latex for this one
                        % label rendered it in Computer Modern while the rest of
                        % the page was the default sans bold -- one serif label
                        % among four sans ones. The bar over w is the price.
                        ylabel(a, 'R \delta w_3   [\mum]', ...
                               'FontSize', FS, 'FontWeight', 'bold');
                    end
            end
            if row == 4
                xlabel(a, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
            end
            xlim(a, [s.t(1) s.t(end)]);  ylim(a, YL{row});
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
                   'Box', 'on', 'TickLabelInterpreter', 'tex');
            if row < 4; set(a, 'XTickLabel', []); end
            if row == 4; ytickformat(a, '%.2f'); end
            if c == 2;  set(a, 'YTickLabel', []); end
            grid(a, 'off');
        end
    end
    exportgraphics(f, out, 'Resolution', 150);
    close(f);
end

function L = local_lim(v, pad)
    lo = min(v); hi = max(v); r = hi - lo;
    if r <= 0; r = 1; end
    L = [lo - pad*r, hi + pad*r];
end

% --------------------------------------------------------------------------
function b = local_b_true(w_bar)
%LOCAL_B_TRUE  The law constant the truth actually has at each height.
%   b_true = a_bar'_true / (1 - a_bar_true)^2, with a_bar_true = 1/c_perp.
%   Exactly 1 at contact and 8/9 in the far field, with an interior minimum
%   0.866978 at w_bar = 2.193 -- so a CONSTANT b cannot sit on it everywhere,
%   and the gap between the curve and the estimate is the shape error the
%   one-parameter law is making, drawn where it can be seen.
    b = zeros(size(w_bar));
    for i = 1:numel(w_bar)
        [~, cp, dd] = calc_correction_functions(w_bar(i), true);
        a = 1 / cp;
        b(i) = (-(1/cp^2) * dd.dc_perp_dh) / (1 - a)^2;
    end
end
