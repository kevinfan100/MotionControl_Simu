function out = check_formC_var_am_vs_atrue(S, opts)
%CHECK_FORMC_VAR_AM_VS_ATRUE  The readout's variance next to the truth's, and
%   next to what R(2,2) says it should be.
%
%   S   = load('test_results/formC_cdpmr_var_check/raw_seeds.mat');  S = S.S400;
%   out = check_formC_var_am_vs_atrue(S);
%
% STATUS: ACTIVE | third of the trio with check_formC_atrue_var and
%                  check_formC_am_vs_atrue (same 400-seed stack)
%
% THREE VARIANCES, THREE DIFFERENT MEANINGS -- kept apart on purpose:
%
%   Var(a_true)  PHYSICS.    The gain really does move, because Brownian motion
%                moves the particle: sd = |a'|*sd(w_bar), verified to 3.5 % at
%                the trough by check_formC_atrue_var.
%   Var(a_m)     INSTRUMENT. The readout jitters even when a stands still. It
%                is an EWMA of squares, so its own sampling noise is large.
%   R2           MODEL.      What the filter BELIEVES Var(y2) is. Comparing the
%                first two gives the channel's signal-to-noise; comparing the
%                second with R2 audits R(2,2) itself.
%
% THE COMPARISON IS DONE AT THE y2 LEVEL, WHERE IT IS EXACT. R2 models the
% variance of the whitened increment, not of a_m, so the script rebuilds
%       y2[k] = a_m[k] - (1-a_cov)*a_m[k-1]
% from the logged a_m per seed -- the controller's own line 923 -- and compares
% var_seeds(y2) with the logged R2. Converting the other way (R2 -> sd(a_m))
% is done ONLY for the display panel and drops R2's delay term, which is
% stated where it happens rather than hidden.
%
% ERROR BARS. The across-seed distribution of a_m is NOT Gaussian (kurtosis
% 5.6-10.5, session "R22 am"), so sqrt(2/(N-1)) does not apply to its variance.
% Bars come from the project's arbiter: G disjoint seed groups, one bin value
% each, std(group values)/sqrt(G).
%
% INSTRUMENT CHECKS, written before the run:
%   C1  row 1 is the init-only zero row (asserted, then dropped); y2 needs two
%       valid rows, so the first usable sample is k = 3.
%   C2  sd(a_m)/a in the far-field hold must reproduce 41.7 % and 49.7 % at the
%       trough (check_formC_am_vs_atrue, 2026-08-24).
%   C3  var(innov_y2)/R2 must be >= 1 up to noise: the innovation carries
%       S2 = H2*P*H2' + R2 and only R2 is logged, so this ratio is a LOWER
%       bound comparison, never an equality test.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');       opts.ax    = 3;   end
    if ~isfield(opts, 'n_bin');    opts.n_bin = 18;  end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    out_dir = fullfile(root, 'test_results', 'formC_cdpmr_var_check');

    ax = opts.ax;  K = S.K;  ns = numel(S.seeds);  ac = K.a_cov;
    tt = S.t(:);

    am = squeeze(S.a_xm_out(:, ax, 1:ns)) / K.a_nom;
    at = squeeze(S.a_true_out(:, ax, 1:ns)) / K.a_nom;
    R2 = squeeze(S.R2_out(:, ax, 1:ns));
    iy2 = squeeze(S.innov_y2_out(:, ax, 1:ns));

    assert(all(am(1,:) == 0), 'C1: row 1 is not the init-only zero row');

    % the controller's own whitening, rebuilt per seed
    y2 = am(2:end,:) - (1 - ac) * am(1:end-1,:);
    y2(1,:) = NaN;                       % needs a_m[k-1] from a valid row
    kk = 2:numel(tt);
    tt = tt(kk); am = am(kk,:); at = at(kk,:); R2 = R2(kk,:); iy2 = iy2(kk,:);

    m_at  = mean(at, 2);
    sd_at = std(at, 0, 2);
    sd_am = std(am, 0, 2);
    m_R2  = mean(R2, 2);
    v_y2  = var(y2, 0, 2);
    v_iy2 = var(iy2, 0, 2);

    % display-only conversion: sd of a_m implied by R2, delay term dropped
    sd_am_model = sqrt(m_R2 / (ac * (2 - ac)));

    snr   = sd_am ./ sd_at;              % how much bigger the noise is
    r_R2  = v_y2  ./ m_R2;               % the R(2,2) audit
    r_nis = v_iy2 ./ m_R2;               % lower-bound consistency check

    far = tt > 0.05 & tt < 0.50;   nea = tt > 3.70;
    fprintf('\n[C2] sd(a_m)/a: far hold %.2f %% (expect 41.7)   trough hold %.2f %% (expect 49.7)\n', ...
            100*mean(sd_am(far)./m_at(far)), 100*mean(sd_am(nea)./m_at(nea)));
    fprintf('[C3] var(innov_y2)/R2: far %.3f   end hold %.3f   (>= 1 expected)\n', ...
            mean(r_nis(far)), mean(r_nis(nea)));

    SEG = {'hold start', tt > 0.05 & tt < 0.50; ...
           'descend',    tt > 0.55 & tt < 1.45; ...
           'oscillate',  tt > 1.60 & tt < 3.40; ...
           'hold end',   tt > 3.70};
    fprintf('\n%-11s %8s %10s %10s %10s %10s %10s\n', 'segment', 'a/a_o', ...
            'sd(a_true)', 'sd(a_m)', 'sd_m/sd_t', 'var y2/R2', 'innov/R2');
    for q = 1:size(SEG,1)
        m = SEG{q,2};
        fprintf('%-11s %8.4f %10.2e %10.2e %10.1f %10.3f %10.3f\n', SEG{q,1}, ...
                mean(m_at(m)), mean(sd_at(m)), mean(sd_am(m)), ...
                mean(snr(m)), mean(r_R2(m)), mean(r_nis(m)));
    end

    % ---- binned on a/a_o with seed-group bars ----------------------------
    G = 20;  grp = mod((1:ns) - 1, G) + 1;
    use = tt > 0.20;
    edges = linspace(min(m_at(use)), max(m_at(use)), opts.n_bin + 1);
    ib = discretize(m_at(use), edges);   tu = find(use);
    ba = nan(opts.n_bin,1);
    b_snr = nan(opts.n_bin,1); s_snr = nan(opts.n_bin,1);
    b_R2  = nan(opts.n_bin,1); s_R2  = nan(opts.n_bin,1);
    for q = 1:opts.n_bin
        m = tu(ib == q);
        if numel(m) < 5; continue; end
        ba(q) = mean(m_at(m));
        g1 = nan(G,1);  g2 = nan(G,1);
        for gg = 1:G
            sel = grp == gg;
            g1(gg) = mean(std(am(m,sel),0,2) ./ std(at(m,sel),0,2));
            g2(gg) = mean(var(y2(m,sel),0,2) ./ mean(R2(m,sel),2));
        end
        b_snr(q) = mean(g1);  s_snr(q) = std(g1)/sqrt(G);
        b_R2(q)  = mean(g2);  s_R2(q)  = std(g2)/sqrt(G);
    end
    fprintf('\n%8s %12s %12s\n', 'a/a_o', 'sd_m/sd_t', 'var y2 / R2');
    for q = 1:opts.n_bin
        if isnan(ba(q)); continue; end
        fprintf('%8.4f %7.1f+-%-4.1f %7.3f+-%-5.3f\n', ba(q), ...
                b_snr(q), s_snr(q), b_R2(q), s_R2(q));
    end

    out = struct('t', tt, 'm_at', m_at, 'sd_at', sd_at, 'sd_am', sd_am, ...
                 'sd_am_model', sd_am_model, 'snr', snr, 'r_R2', r_R2, ...
                 'r_nis', r_nis, 'ba', ba, 'b_snr', b_snr, 's_snr', s_snr, ...
                 'b_R2', b_R2, 's_R2', s_R2, 'ns', ns);

    if opts.save_fig
        local_fig(out, fullfile(out_dir, 'var_am_vs_atrue.png'));
        fprintf('\nfigure -> %s\n', out_dir);
    end
end

% =======================================================================
function local_fig(o, fpath)
%LOCAL_FIG  One panel: the three standard deviations against time, log scale.
%   Everything the page has to say is in the vertical distance between the
%   red line and the two blue/green ones; the ratio and R22 panels that used
%   to sit below are printed to the console instead (2026-08-25).
    FS = 18; AXLW = 2.0;
    RED = [0.85 0.1 0.1];  LBL = [0.35 0.65 0.95];  GRN = [0.1 0.6 0.2];
    f = figure('Position',[40 40 1400 700],'Color','w','Visible','off');
    a1 = axes(f); hold(a1,'on'); box(a1,'on');
    plot(a1, o.t, o.sd_am, '-', 'Color', LBL, 'LineWidth', 2.6, ...
         'DisplayName','sd(a_m)   instrument noise of the readout');
    plot(a1, o.t, o.sd_am_model, '--', 'Color', GRN, 'LineWidth', 2.2, ...
         'DisplayName','sd(a_m) implied by R_{22}   what the filter assumes');
    plot(a1, o.t, o.sd_at, '-', 'Color', RED, 'LineWidth', 2.6, ...
         'DisplayName','sd(a_{true})   physical wander of the gain');
    set(a1,'YScale','log'); ylim(a1,[1e-5 1]);
    yticks(a1, 10.^(-5:0));
    xlabel(a1,'t [s]'); ylabel(a1,'sd / a_o   (across 400 seeds, per sample)');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
