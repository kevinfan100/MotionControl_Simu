function S = plot_formC_seed_hist(o, opts)
%PLOT_FORMC_SEED_HIST  Per-seed histograms of the RAW random draws.
%
%   S = plot_formC_seed_hist(o)          % o = out struct of run_formC_state
%   S = plot_formC_seed_hist(o, opts)
%
%   Question these figures answer: does each seed produce a genuine, correct
%   random sequence? That is a property of the raw randn stream ALONE, so
%   nothing here is whitened or scaled -- no thermal variance model and no
%   measurement std enters. The red N(0,1) overlay is therefore a DEFINITION
%   (randn is standard normal by construction), not a fit: no parameter is
%   estimated from the data. Whether the correct per-step standard deviation
%   was later applied is a different question and belongs in a different
%   figure -- do not mix the two on one page.
%
%   STREAM REGENERATION (from test_script/scratch/audit_seed11_rng_stream.m,
%   where it was verified bit-exactly against the realized run):
%       rng(seed)                                 % driver's user seed
%       params = calc_simulation_params(config)   % draws randi -> thermal.seed
%       rng(params.thermal.seed)
%       Z = randn(6, N)                           % 6 normals per step,
%                                                 % thermal x/y/z then meas x/y/z
%   The re-derived thermal.seed is asserted against the one the run logged, so
%   the plotted stream is the one the run consumed.
%
%   opts fields (defaults first):
%       .seeds   o.seeds        which seeds to plot
%       .edges   -4:0.1:4       DISPLAY bin edges (80 bins). Symmetric +-4:
%                               for N = 7681 the expected max|z| is ~4.4, so
%                               +-4 frames the realized range, while +-1 would
%                               clip ~32% of the points and hide the tails --
%                               exactly where a bad stream would show. The
%                               N(0,1) overlay is 100*binwidth*pdf, so it
%                               follows this width automatically; the y limits
%                               autoscale to the bars.
%
%   Figures -> derivation/figures/formC_seedhist_s<NN>.png (2 x 3 panels:
%   row 1 = raw draws Z_1..Z_3, row 2 = Z_4..Z_6; y = 100*counts/N in [%]).
%   Statistics (N, mean, std, skew, kurtosis, min, max, %|z|>3, KS and
%   chi-square p vs N(0,1)) and the cross-seed overlap check go to the
%   console, never on the figure (house rule).
%
%   Style: canonical template plot_var_ahat_6state.m -- FontSize 18 bold,
%   axis LineWidth 2.0, no grid, box on, legend northoutside horizontal,
%   no title, exportgraphics(fig, ..., 'Resolution', 150).
%
%   See also: plot_formC_state_pairs, plot_var_ahat_6state

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds'); opts.seeds = o.seeds(:).'; end
    if ~isfield(opts, 'edges'); opts.edges = -4:0.1:4;     end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    CH = {'Z_1 thermal x', 'Z_2 thermal y', 'Z_3 thermal z', ...
          'Z_4 meas. x',   'Z_5 meas. y',   'Z_6 meas. z'};
    all_seeds = o.seeds(:).';
    ns = numel(opts.seeds);
    S = struct('seed', num2cell(opts.seeds), 'thermal_seed', [], ...
               'Z', [], 'stats', [], 'file', '');

    fprintf('\n%6s %14s %6s %8s %7s %8s %8s %7s %7s %8s %9s %9s\n', ...
            'seed', 'channel', 'N', 'mean', 'std', 'skew', 'kurt', ...
            'min', 'max', '|z|>3 %', 'KS p', 'chi2 p');
    fprintf('%s\n', repmat('-', 1, 112));

    for q = 1:ns
        r = o.runs{find(all_seeds == opts.seeds(q), 1)};
        N = size(r.p_true_out, 1);                 % run length in steps

        % --- driver's path: user seed -> params -> thermal.seed -> stream ---
        rng(opts.seeds(q));
        params = calc_simulation_params(r.meta.config);
        th_seed = params.Value.thermal.seed;
        assert(th_seed == r.meta.params_value.thermal.seed, ...
               'plot_formC_seed_hist: seed %d re-derived thermal.seed %d ~= logged %d', ...
               opts.seeds(q), th_seed, r.meta.params_value.thermal.seed);
        rng(th_seed);
        Z = randn(6, N);                           % RAW draws, no scaling

        st = zeros(6, 10);
        for c = 1:6
            z = Z(c, :).';  m = mean(z);  sd = std(z);  zc = (z - m) / sd;
            [ks_p, ch_p] = local_gof(z);
            st(c, :) = [N, m, sd, mean(zc.^3), mean(zc.^4), min(z), max(z), ...
                        100 * mean(abs(z) > 3), ks_p, ch_p];
            fprintf('%6d %14s %6d %+8.4f %7.4f %+8.4f %8.4f %+7.3f %+7.3f %8.3f %9.3f %9.3f\n', ...
                    opts.seeds(q), CH{c}, N, st(c, 2), st(c, 3), st(c, 4), st(c, 5), ...
                    st(c, 6), st(c, 7), st(c, 8), st(c, 9), st(c, 10));
        end

        S(q).thermal_seed = th_seed;
        S(q).Z     = Z;
        S(q).stats = st;
        S(q).file  = fullfile(fig_dir, sprintf('formC_seedhist_s%02d.png', opts.seeds(q)));
        local_hist_page(Z, opts.seeds(q), CH, opts.edges, S(q).file);
    end

    fprintf('%s\n', repmat('-', 1, 112));
    fprintf(['ideal N(0,1): mean 0, std 1, skew 0, kurt 3, |z|>3 = 0.270 %%; ', ...
             'p-values uniform on [0,1] (both tests are against the FIXED N(0,1), ', ...
             'no fitted parameter)\n']);

    local_overlap_report(S);
    fprintf('[seedhist] wrote %d figures -> %s\n', ns, fig_dir);
end

% --------------------------------------------------------------------------
function [ks_p, chi2_p] = local_gof(z)
%LOCAL_GOF  Goodness of fit against the FIXED standard normal (no toolbox).
%   KS: two-sided statistic + asymptotic Kolmogorov p (Stephens' correction);
%   binning-free.
%   chi-square: its OWN fixed binning (0.2 over +-4 plus the two open tails,
%   df = nbins - 1), deliberately independent of the display bin width so that
%   changing how the histogram looks cannot move the reported p-values.
%   Nothing is estimated from the data, so no df is subtracted for fitting.
    CHI2_EDGES = -4:0.2:4;                 % test binning, NOT the display bins
    N   = numel(z);
    Phi = @(x) 0.5 * erfc(-x / sqrt(2));

    zs = sort(z(:));
    F  = Phi(zs);
    i  = (1:N).';
    D  = max([max(i/N - F); max(F - (i-1)/N)]);
    lam = (sqrt(N) + 0.12 + 0.11/sqrt(N)) * D;
    k = (1:100).';
    ks_p = min(1, max(0, 2 * sum((-1).^(k-1) .* exp(-2 * k.^2 * lam^2))));

    e  = [-inf, CHI2_EDGES, inf];
    obs = histcounts(z, e).';
    p   = diff(Phi(e)).';
    exp_ = N * p;
    chi2 = sum((obs - exp_).^2 ./ exp_);
    df   = numel(obs) - 1;
    chi2_p = gammainc(chi2/2, df/2, 'upper');
end

% --------------------------------------------------------------------------
function local_overlap_report(S)
%LOCAL_OVERLAP_REPORT  No two seeds may share draws (or a thermal seed).
    ns = numel(S);
    ts = [S.thermal_seed];
    fprintf('\n--- cross-seed independence ---\n');
    fprintf('user seed -> thermal.seed:');
    for q = 1:ns; fprintf('  %d->%d', S(q).seed, ts(q)); end
    fprintf('\n');
    if numel(unique(ts)) == ns
        fprintf('thermal.seed values: all %d distinct\n', ns);
    else
        fprintf('*** thermal.seed COLLISION among the %d seeds ***\n', ns);
    end

    worst = 0;  worst_pair = [0 0];  n_ident = 0;
    for a = 1:ns-1
        for b = a+1:ns
            if isequal(S(a).Z, S(b).Z); n_ident = n_ident + 1; end
            nc = numel(intersect(S(a).Z(:), S(b).Z(:)));   % exact double matches
            if nc > worst; worst = nc; worst_pair = [S(a).seed S(b).seed]; end
        end
    end
    fprintf(['pairwise (%d pairs): identical streams %d; max shared draw values %d', ...
             ' (pair %d/%d) out of %d per seed\n'], ...
            ns*(ns-1)/2, n_ident, worst, worst_pair(1), worst_pair(2), numel(S(1).Z));
end

% --------------------------------------------------------------------------
function local_hist_page(Z, seed, CH, edges, out)
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9];
    FS = 18; LFS = 13; AXLW = 2.0;

    N  = size(Z, 2);
    bw = edges(2) - edges(1);
    ctr = edges(1:end-1) + bw/2;
    xg  = linspace(edges(1), edges(end), 600);
    ideal = 100 * bw * exp(-xg.^2 / 2) / sqrt(2*pi);   % % of points per bin

    pct = zeros(numel(ctr), 6);
    for c = 1:6
        pct(:, c) = 100 * histcounts(Z(c, :), edges).' / N;
    end
    ymax = 1.12 * max([pct(:); ideal(:)]);

    f = figure('Position', [40 40 1600 900], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    for c = 1:6
        a = nexttile(tl, c);  hold(a, 'on');
        hb = bar(a, ctr, pct(:, c), 1, 'FaceColor', COL_HAT, 'EdgeColor', 'none', ...
                 'FaceAlpha', 0.85, 'DisplayName', CH{c});
        hi = plot(a, xg, ideal, '-', 'Color', COL_TRUE, 'LineWidth', 2.5, ...
                  'DisplayName', 'N(0,1) exact');
        if c == 1
            set(hb, 'DisplayName', sprintf('%s  seed %d', CH{c}, seed));
        end
        legend(a, [hb hi], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        xlim(a, [edges(1) edges(end)]);  ylim(a, [0 ymax]);
        set(a, 'XTick', edges(1):2:edges(end), 'FontSize', FS, 'FontWeight', 'bold', ...
               'LineWidth', AXLW, 'Box', 'on', 'TickLabelInterpreter', 'tex');
        if c > 3
            xlabel(a, 'raw randn draw  (unscaled)', 'FontSize', FS, 'FontWeight', 'bold');
        end
        if mod(c, 3) == 1
            ylabel(a, 'raw draws  [%]', 'FontSize', FS, 'FontWeight', 'bold');
        else
            set(a, 'YTickLabel', []);
        end
        grid(a, 'off');
    end
    exportgraphics(f, out, 'Resolution', 150);
    close(f);
end
