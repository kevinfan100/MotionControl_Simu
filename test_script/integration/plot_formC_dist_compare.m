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
    s.t  = (0:numel(s.aH)-1).' * Ts;
end

function local_page(d, sq, out, NAME, SLOT5)
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_MEAS = [0.45 0.72 0.95];
    BANDC = [0.45 0.55 0.95];
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

    YL = cell(3, 1);
    YL{1} = local_lim([d{1}.aT; d{1}.aH; d{1}.aM; d{2}.aT; d{2}.aH; d{2}.aM], 0.05);
    YL{2} = local_lim([d{1}.e;  d{2}.e], 0.08);
    YL{3} = local_lim([d{2}.da + d{2}.sd; d{2}.da - d{2}.sd; 0], 0.10);

    f = figure('Position', [40 40 1500 1050], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for row = 1:3
        for c = 1:2
            s = d{c};  a = nexttile(tl, (row-1)*2 + c);  hold(a, 'on');
            switch row
                case 1
                    h0 = plot(a, s.t, s.aM, '-', 'Color', COL_MEAS, 'LineWidth', 1.0, ...
                              'DisplayName', 'a_{m} readout');
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
                    plot(a, s.t, s.da, '-', 'Color', COL_HAT, 'LineWidth', LW, ...
                         'DisplayName', [SLOT5 '_{hat}']);
                    yline(a, 0, '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
                          'DisplayName', 'zero');
                    legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
                    if c == 1
                        ylabel(a, SLOT5, 'FontSize', FS, 'FontWeight', 'bold');
                    end
                    xlabel(a, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
            end
            xlim(a, [s.t(1) s.t(end)]);  ylim(a, YL{row});
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
                   'Box', 'on', 'TickLabelInterpreter', 'tex');
            if row < 3; set(a, 'XTickLabel', []); end
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
