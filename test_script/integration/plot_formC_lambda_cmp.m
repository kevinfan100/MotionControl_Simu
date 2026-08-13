function plot_formC_lambda_cmp(R, opts)
%PLOT_FORMC_LAMBDA_CMP  Forgetting-factor comparison, laid out to sit next to
%   the existing formC_dist_cmp_s*.png pages.
%
% STATUS: ACTIVE | scratch-grade figure for the Menq (4.15) forgetting-factor
%         probe. lambda_f is a DIAGNOSTIC, not an adopted fix.
%
%   R is a 2 x nLam cell: R{1,:} = arm 'base' (no delta a), R{2,:} = arm 'dist'
%   (with delta a), columns in the order of opts.lams.
%
%   Layout is the same 3 rows x 2 seed-free columns as plot_formC_dist_compare:
%       left  column = no delta a       right column = with delta a
%       row 1 gain tracking   row 2 relative gain error   row 3 delta a
%   Within each column, lambda_f = 1 and the probe value are overlaid, so the
%   page can be read directly against the earlier comparison figures.
%
%   A second figure carries the honesty ratio rho(t) = across-seed RMS of e_a
%   divided by the filter's own sqrt(P44); that is the curve that decides
%   whether the forgetting factor is doing what it claims.
%
%   Style: canonical (18 pt bold sans, 2.0 pt axes, no grid, box on, legend
%   northoutside horizontal, no title, exportgraphics 150).

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'lams');   opts.lams   = [1 0.9999 0.999 0.99]; end
    if ~isfield(opts, 'show');   opts.show   = [1 3];   end   % which lams to draw
    if ~isfield(opts, 'seed');   opts.seed   = 7;       end
    if ~isfield(opts, 'Ts');     opts.Ts     = 6.25e-4; end
    if ~isfield(opts, 'suffix'); opts.suffix = '';      end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AX = 3;  seeds = R{1,1}.seeds(:).';  q = find(seeds == opts.seed, 1);

    d = cell(2, numel(opts.show));
    for a = 1:2
        for c = 1:numel(opts.show)
            d{a, c} = local_pull(R{a, opts.show(c)}.runs{q}, AX, opts.Ts);
        end
    end
    out = fullfile(fig_dir, sprintf('formC_lambda_cmp%s_s%02d.png', opts.suffix, opts.seed));
    local_page(d, opts.lams(opts.show), opts.seed, out);

    % ---- rho(t), the deciding curve ------------------------------------
    out2 = fullfile(fig_dir, sprintf('formC_lambda_rho%s.png', opts.suffix));
    local_rho(R, opts, AX, out2);
    fprintf('[lambda cmp] wrote %s and %s\n', out, out2);
end

% --------------------------------------------------------------------------
function s = local_pull(r, ax, Ts)
    ad = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);
    s.aT = r.a_true_out(:, ax) / ad;
    s.aH = r.a_bar_hat_out(:, ax);
    s.e  = 100 * (s.aH - s.aT) ./ s.aT;
    s.da = r.b_hat_out(:, ax);
    s.t  = (0:numel(s.aH)-1).' * Ts;
end

function local_page(d, lams, sq, out)
    C_T = [0.8 0 0];  COL = [0 0.20 0.90; 0.10 0.60 0.30];  LS = {'-', '--'};
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;
    NAME = {'no \delta a', 'with \delta a'};

    all_a = []; all_e = []; all_d = [];
    for a = 1:2
        for c = 1:size(d, 2)
            all_a = [all_a; d{a,c}.aT; d{a,c}.aH];  %#ok<AGROW>
            all_e = [all_e; d{a,c}.e];              %#ok<AGROW>
            all_d = [all_d; d{a,c}.da];             %#ok<AGROW>
        end
    end
    YL = {local_lim(all_a, 0.05), local_lim(all_e, 0.08), local_lim([all_d; 0], 0.10)};

    f = figure('Position', [40 40 1500 1050], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:3
        for a = 1:2
            ax_ = nexttile(tl, (row-1)*2 + a);  hold(ax_, 'on');
            h = gobjects(1, size(d,2) + 1);
            if row == 1
                h(1) = plot(ax_, d{a,1}.t, d{a,1}.aT, '-', 'Color', C_T, ...
                            'LineWidth', LW + 0.6, 'DisplayName', 'a_{true}');
            end
            for c = 1:size(d, 2)
                s = d{a, c};
                switch row
                    case 1; y = s.aH;  case 2; y = s.e;  case 3; y = s.da;
                end
                hh = plot(ax_, s.t, y, LS{c}, 'Color', COL(c, :), 'LineWidth', LW, ...
                     'DisplayName', sprintf('%s  \\lambda_f = %g', NAME{a}, lams(c)));
                if row == 1; h(c+1) = hh; end
            end
            if row == 2
                yline(ax_, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, ...
                      'HandleVisibility', 'off');
            end
            if row == 1
                legend(ax_, h, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            end
            if a == 1
                lab = {'a_z / a_o', 'a_z  error  [%]', '\delta a_z'};
                ylabel(ax_, lab{row}, 'FontSize', FS, 'FontWeight', 'bold');
            end
            if row == 3
                xlabel(ax_, sprintf('time  [s]     seed %d', sq), ...
                       'FontSize', FS, 'FontWeight', 'bold');
            end
            xlim(ax_, [d{a,1}.t(1) d{a,1}.t(end)]);  ylim(ax_, YL{row});
            set(ax_, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
                     'Box', 'on', 'TickLabelInterpreter', 'tex');
            if row < 3; set(ax_, 'XTickLabel', []); end
            if a == 2;  set(ax_, 'YTickLabel', []); end
            grid(ax_, 'off');
        end
    end
    exportgraphics(f, out, 'Resolution', 150);  close(f);
end

function local_rho(R, opts, AX, out)
    COL = [0 0.20 0.90; 0.10 0.60 0.30];  LS = {'-', '--'};
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;
    nS = numel(R{1,1}.seeds);
    N  = size(R{1,1}.runs{1}.a_bar_hat_out, 1);  t = (0:N-1).' * opts.Ts;

    f = figure('Position', [60 60 1300 820], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    ax_ = axes(f); hold(ax_, 'on');
    h = gobjects(1, numel(opts.show));
    for c = 1:numel(opts.show)
        E = zeros(N, nS);  SP = zeros(N, nS);
        for q = 1:nS
            r  = R{1, opts.show(c)}.runs{q};
            ad = r.a_hat_out(1, AX) / r.a_bar_hat_out(1, AX);
            E(:, q)  = r.a_bar_hat_out(:, AX) - r.a_true_out(:, AX) / ad;
            SP(:, q) = r.P_a_out(:, AX) / ad;
        end
        rho = sqrt(mean(E.^2, 2)) ./ mean(SP, 2);
        h(c) = plot(ax_, t, rho, LS{c}, 'Color', COL(c, :), 'LineWidth', LW, ...
             'DisplayName', sprintf('no \\delta a,  \\lambda_f = %g', opts.lams(opts.show(c))));
    end
    yline(ax_, 1, '-', 'Color', [0.8 0 0], 'LineWidth', LW, 'DisplayName', 'honest');
    set(ax_, 'YScale', 'log');
    xlim(ax_, [0 t(end)]);
    xlabel(ax_, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax_, '\rho = RMS(e_a) / claimed sqrt(P_{44})', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax_, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(ax_, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
             'Box', 'on', 'TickLabelInterpreter', 'tex');
    grid(ax_, 'off');
    exportgraphics(f, out, 'Resolution', 150);  close(f);
end

function L = local_lim(v, pad)
    lo = min(v); hi = max(v); r = hi - lo;
    if r <= 0; r = 1; end
    L = [lo - pad*r, hi + pad*r];
end
