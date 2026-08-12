function plot_formC_state_pairs(o, opts)
%PLOT_FORMC_STATE_PAIRS  Config B result pages, 3 rows x 2 seed columns per
%   page (seed left vs seed right); rows are gain tracking / relative gain
%   error / free parameter.
%
%   Pairing puts the two descent-peak branches on separate pages:
%       p1  27 | 31  (low branch)   p2   7 | 11  (high branch)
%       p3  23 | 42                 p4 101 | 777
%   Row y-limits are shared across the two columns so the seeds compare by eye.
%
%   Usage: L = load('test_results/run_formC_state_cfgB_y2on.mat');
%          plot_formC_state_pairs(L.out)
%
%   STYLE: copied from the canonical template plot_var_ahat_6state.m
%   (.claude/rules/figure-style.md): default (sans) font at FontSize 18 bold,
%   axis LineWidth 2.0, no grid, box on, legend northoutside horizontal bold
%   boxed, no title (stats to console), True = red / Estimate = blue /
%   Measured = light blue, exportgraphics(fig, ..., 'Resolution', 150).
%   Labels use MATLAB's tex interpreter (a_{true}, a_{hat}), NOT latex.
%
%   Time axis comes from the logged tout when present; opts.Ts is only the
%   fallback for logs without it.
%
%   Units: P_a_out / P_b_out already store the STANDARD DEVIATION (do not
%   sqrt again); a_disp = a_hat_out(1,3)/a_bar_hat_out(1,3) converts the
%   display-layer gain to a_bar; axis z = index 3.
%
%   See also: plot_var_ahat_6state, plot_formC_state_panels, run_formC_state

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'pairs'); opts.pairs = {[27 31], [7 11], [23 42], [101 777]}; end
    if ~isfield(opts, 'Ts');    opts.Ts    = 1/1612; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AX = 3;  seeds = o.seeds(:).';

    for pg = 1:numel(opts.pairs)
        pr = opts.pairs{pg};  d = cell(1, 2);
        for c = 1:2
            r = o.runs{find(seeds == pr(c), 1)};
            ad = r.a_hat_out(1, AX) / r.a_bar_hat_out(1, AX);
            s = struct();
            s.aT = r.a_true_out(:, AX) / ad;
            s.aH = r.a_bar_hat_out(:, AX);
            s.aM = r.a_xm_out(:, AX) / ad;      % measured gain readout (y2 channel)
            s.e  = 100 * (s.aH - s.aT) ./ s.aT;
            s.da = r.b_hat_out(:, AX);
            s.sd = r.P_b_out(:, AX);            % already a std, do not sqrt
            if isfield(r, 'tout') && numel(r.tout) == numel(s.aH)
                s.t = r.tout(:);                % logged grid (authoritative)
            else
                s.t = (0:numel(s.aH)-1).' * opts.Ts;
            end
            d{c} = s;
        end
        out = fullfile(fig_dir, sprintf('formC_state_cfgB_p%d.png', pg));
        local_page(d, pr, out);
        fprintf('p%d  seed %3d: relerr RMS %6.3f %%  da_end %+.5f  |  seed %3d: %6.3f %%  %+.5f\n', ...
            pg, pr(1), sqrt(mean(d{1}.e.^2)), d{1}.da(end), ...
                pr(2), sqrt(mean(d{2}.e.^2)), d{2}.da(end));
    end
    fprintf('[pairs] wrote %d figures -> %s\n', numel(opts.pairs), fig_dir);
end

% --------------------------------------------------------------------------
function local_page(d, pr, out)
    % house palette + canonical template geometry
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_MEAS = [0.45 0.72 0.95];
    BANDC = [0.45 0.55 0.95];
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;
    DA_ANCHOR = -1/9;                       % state-writing anchor value

    YL = cell(3, 1);
    YL{1} = local_lim([d{1}.aT; d{1}.aH; d{1}.aM; d{2}.aT; d{2}.aH; d{2}.aM], 0.05);
    YL{2} = local_lim([d{1}.e;  d{2}.e], 0.08);
    YL{3} = local_lim([d{1}.da + d{1}.sd; d{1}.da - d{1}.sd; ...
                       d{2}.da + d{2}.sd; d{2}.da - d{2}.sd], 0.05);

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
                              'DisplayName', sprintf('a_{hat}  seed %d', pr(c)));
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
                    hb = fill(a, [s.t; flipud(s.t)], [s.da + s.sd; flipud(s.da - s.sd)], ...
                              BANDC, 'EdgeColor', 'none', 'FaceAlpha', 0.30, ...
                              'DisplayName', '\pm sqrt(P_{55})');
                    hl = plot(a, s.t, s.da, '-', 'Color', COL_HAT, 'LineWidth', LW, ...
                              'DisplayName', '\delta a_{hat}');
                    ha = yline(a, DA_ANCHOR, '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
                               'DisplayName', '-1/9 anchor');
                    legend(a, [ha hl hb], 'Location', 'northoutside', ...
                           'Orientation', 'horizontal', 'FontSize', LFS, ...
                           'FontWeight', 'bold', 'Box', 'on');
                    if c == 1
                        ylabel(a, '\delta a_z', 'FontSize', FS, 'FontWeight', 'bold');
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
