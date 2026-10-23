function out = check_formC_var_identity(S, opts)
%CHECK_FORMC_VAR_IDENTITY  Do the two closed-loop variance identities hold?
%
%   S   = run_formC_var_seed_stack(1:400);
%   out = check_formC_var_identity(S);
%
% STATUS: ACTIVE | diagnostic for the formC b-as-state arm, P0 hard plane
%
% THE TWO IDENTITIES, on recorded simulation data alone:
%
%   Var(delta_x)   = C_dx   * 4kBT*a + [(1-lc)/(1+lc)] * sigma2_n
%   Var(delta_x_r) = C_dpmr * 4kBT*a +  C_n            * sigma2_n
%
%   C_dx = 2 + 1/(1-lc^2) = 3.9608 at lc = 0.7 -- the RAW tracking error, the
%   quantity drawn in row 4 of the formC comparison pages (there in nm).
%   C_dpmr = 3.1610 -- the a_pd-dependent coefficient of the HIGH-PASS
%   RESIDUAL that the y2 readout squares. They are NOT interchangeable: the
%   high pass removes the ultra-low-frequency tail before squaring, which is
%   the whole 20% between them. predict_closed_loop_var_eq6.m calls the FIRST
%   one C_dpmr_design, which is the one place the naming lies.
%
% ESTIMATOR: ensemble variance ACROSS SEEDS at each sample. No smoothing, no
% windowing, no detrending. p_d is identical across seeds so it cancels
% exactly and takes the deterministic tracking lag with it. The per-sample
% scatter is NOT removed; it is compared against sqrt(2/(N-1)), which holds
% here because delta_x and delta_x_r are Gaussian. (It does NOT hold for the
% a_xm / y2 channels, which are EWMAs of squares with cross-seed kurtosis
% 5.6-10.5 -- those need sqrt((kurt-1)/N). See session "R22 am".)
%
% ARITHMETIC IS NORMALIZED -- kappa_T*a_bar, the expression the controller
% literally evaluates (motion_control_law_formC_b.m:706,827) -- and multiplied
% by R^2 only for display. Measured and theory get the same R^2, so the
% conversion cannot create or hide a mismatch.
%
% ALIGNMENT. a_true_k is evaluated at p_curr BEFORE step_dynamics while
% p_true_out(k) is written AFTER it, so a_true_out(k) belongs to
% p_true_out(k-1) -- the same off-by-one the 2026-08-18 audit found in the
% tracking-error bias, where it was the whole of a 27.70 nm "bias". dx_r
% additionally sees the d = 2 step measurement delay. Neither is asserted:
% the lag is SCANNED and the residual-minimising value reported next to the
% nominal one.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');       opts.ax       = 3;    end   % wall-normal
    if ~isfield(opts, 'lag_scan'); opts.lag_scan = 0:6;  end
    if ~isfield(opts, 'warmup_s'); opts.warmup_s = 0.05; end
    if ~isfield(opts, 'tag');      opts.tag      = '';   end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    % 'binned' follows the project's own prior art for this exact check
    % (verify_axm_cdpmr_6state.m -> fig_dx_var_time / fig_dx_var_scatter) and
    % the R22/a_m section's fig3_r22_raw_vs_a: raw samples against time, then
    % binned points WITH ERROR BARS against a. Binning along a is aggregation,
    % not a time-domain filter -- no sample is hidden and every bar is measured.
    % 'raw' plots all 7681 samples in both panels instead.
    if ~isfield(opts, 'style');    opts.style    = 'binned'; end
    if ~isfield(opts, 'n_bin');    opts.n_bin    = 28;   end
    % Evaluate the formula at the REALIZED loop pole instead of the designed
    % one. The identities are derived for a loop whose pole is lambda_c, but a
    % controller that places its pole using a_hat realizes
    %     lambda_eff = 1 - (a_true/a_hat)*(1 - lambda_c)
    % and a_hat is 19 % high at the trough on the deployed arm. Substituting
    % lambda_eff is not a fit -- g is measured per step from the log and there
    % is no free parameter. Negative control: at a_hat = a_true this returns
    % the stored constants to four decimals.
    if ~isfield(opts, 'use_lambda_eff'); opts.use_lambda_eff = false; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    out_dir = fullfile(root, 'test_results', ['formC_cdpmr_var_check' opts.tag]);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    ax = opts.ax;  K = S.K;  ns = numel(S.seeds);
    N  = size(S.p_true_out, 1);
    lc = K.lambda_c;

    C_dx   = 2 + 1/(1 - lc^2);
    C_n_fb = (1 - lc)/(1 + lc);
    C_dpmr = K.C_dpmr;
    C_n    = K.C_n;
    kappa_T = 4 * (K.kBT / K.R) * K.a_o;              % [-]
    sg2n_nd = K.sigma2_n_s(ax) / K.R^2;               % [-]

    % a_bar is deterministic (same trajectory every seed); take seed 1 and
    % check the spread across seeds is only the tracking error, not a
    % different trajectory.
    % ENSEMBLE-MEAN a, not seed 1's. The measured quantity is the variance
    % ACROSS seeds at a fixed k, so the a it belongs to is the ensemble's, not
    % any one member's. During a hold the command is fixed and seed 1's own
    % a wanders by its Brownian motion (sd 3.8 %) while the ensemble mean
    % barely moves (0.36 %), so binning on seed 1 selects times when THAT seed
    % happened to be deep and then evaluates the formula at that low a against
    % the ensemble's variance -- a ~5 %% inflation of the innermost bins,
    % purely constructed. Caught 2026-08-21.
    a_bar = mean(squeeze(S.a_true_out(:, ax, 1:ns)), 2) / K.a_nom;

    % --- the only two measured quantities, nothing applied to them ---------
    V_ptrue = var(squeeze(S.p_true_out(:, ax, 1:ns)), 0, 2) / K.R^2;   % [-]
    V_dxr   = var(squeeze(S.dx_r_out(:,   ax, 1:ns)), 0, 2) / K.R^2;   % [-]

    if opts.use_lambda_eff
        g_step = mean(squeeze(S.a_true_out(:, ax, 1:ns)) / K.a_nom ./ ...
                      squeeze(S.a_bar_hat_out(:, ax, 1:ns)), 2);
        lam_e  = 1 - g_step * (1 - lc);
        th_dx  = local_Cdx(lam_e)          .* (kappa_T * a_bar) + local_Cnfb(lam_e) * sg2n_nd;
        th_dxr = local_Cdpmr(lam_e, K.a_pd).* (kappa_T * a_bar) + local_Cn(lam_e, K.a_pd) * sg2n_nd;
        fprintf(['[var-check] theory evaluated at the REALIZED pole: lambda_eff range ' ...
                 '[%.4f, %.4f] over the run, %.4f in the final hold (designed %.4f)\n'], ...
                min(lam_e), max(lam_e), mean(lam_e(S.t > 3.70)), lc);
    else
        th_dx  = C_dx   * kappa_T * a_bar + C_n_fb * sg2n_nd;
        th_dxr = C_dpmr * kappa_T * a_bar + C_n    * sg2n_nd;
    end

    % --- lag scan ---------------------------------------------------------
    k0 = max(opts.lag_scan) + 1 + round(opts.warmup_s / K.Ts);
    kk = (k0:N).';
    sc = struct('lag', opts.lag_scan, 'rms_dx', zeros(1,numel(opts.lag_scan)), ...
                                      'rms_dxr', zeros(1,numel(opts.lag_scan)));
    for j = 1:numel(opts.lag_scan)
        Lg = opts.lag_scan(j);
        sc.rms_dx(j)  = sqrt(mean((V_ptrue(kk - Lg)./th_dx(kk)  - 1).^2));
        sc.rms_dxr(j) = sqrt(mean((V_dxr(kk   - Lg)./th_dxr(kk) - 1).^2));
    end
    [~, jx ] = min(sc.rms_dx);   L_dx  = opts.lag_scan(jx);
    [~, jxr] = min(sc.rms_dxr);  L_dxr = opts.lag_scan(jxr);

    Vm_dx  = V_ptrue(kk - L_dx);   Vt_dx  = th_dx(kk);
    Vm_dxr = V_dxr(kk   - L_dxr);  Vt_dxr = th_dxr(kk);
    ab = a_bar(kk);  tt = S.t(kk);

    % --- coefficients read straight back out of the data ------------------
    Chat_dx   = mean((Vm_dx  - C_n_fb*sg2n_nd) ./ (kappa_T * ab));
    Chat_dpmr = mean((Vm_dxr - C_n   *sg2n_nd) ./ (kappa_T * ab));
    % SE by DELETE-ONE-SEED JACKKNIFE. An earlier version discounted the
    % within-bin spread by a measured tau_eff, i.e. it treated k as the
    % resampling unit. That is the wrong frame: every k already uses all N
    % seeds, so the k's are not independent replicas of the measurement --
    % the SEED is the independent unit, and removing a whole trajectory keeps
    % its time correlation intact instead of discounting it by hand. Settled
    % 2026-08-20 against an assumption-free arbiter (split the seeds into G
    % disjoint groups, take std(group values)/sqrt(G)): jackknife matched it
    % to 0.98 median over bins, the tau_eff bar was 16.6x too large.
    [rel_dx,  tau_dx ] = local_jack_C(S.p_true_out, opts.ax, ns, kk - L_dx,  ...
                                      K.R^2, C_n_fb*sg2n_nd, kappa_T*ab, Chat_dx);
    [rel_dxr, tau_dxr] = local_jack_C(S.dx_r_out,   opts.ax, ns, kk - L_dxr, ...
                                      K.R^2, C_n   *sg2n_nd, kappa_T*ab, Chat_dpmr);

    fprintf('\n=== variance identity | z axis | %d seeds | arm %s | trough w_bar %.2f ===\n', ...
            ns, K.ap_src, K.h_bottom/K.R);
    fprintf('predicted per-sample scatter (Gaussian, chi2_{N-1}): %.1f %%\n', ...
            100*sqrt(2/(ns-1)));
    fprintf('observed  per-sample scatter: dx %.1f %%   dxr %.1f %%\n', ...
            100*std(Vm_dx./Vt_dx), 100*std(Vm_dxr./Vt_dxr));
    fprintf('lag scan  dx  :'); fprintf('  %d:%.4f', [opts.lag_scan; sc.rms_dx]);
    fprintf('   -> %d (nominal 1)\n', L_dx);
    fprintf('lag scan  dxr :'); fprintf('  %d:%.4f', [opts.lag_scan; sc.rms_dxr]);
    fprintf('   -> %d (nominal %d = 1 + d)\n', L_dxr, 1 + K.d);
    % The run-averaged coefficient is a MISLEADING summary and is reported
    % only for completeness. It is an unweighted mean over k of
    % (V - C_n*sigma2_n)/(kappa_T*a), so the near-wall samples -- where the
    % divisor is 11x smaller -- dominate it. Measured per bin, the implied
    % constant is 3.1628-3.1694 for a/a_o > 0.75 against a theory of 3.1610,
    % and climbs to 3.562 in the bottom bin: the CONSTANT is right and the
    % departure is height-dependent. Quoting the run average as "the constant
    % is 2.5% off" would be wrong, and the far-field figure below is the one
    % that tests the constant.
    far = ab > 0.4;
    Cfar_dx  = mean((Vm_dx(far)  - C_n_fb*sg2n_nd) ./ (kappa_T * ab(far)));
    Cfar_dpmr= mean((Vm_dxr(far) - C_n   *sg2n_nd) ./ (kappa_T * ab(far)));
    fprintf('far field only (a/a_o > 0.4):  C_dx %.4f (theory %.4f, %+.2f %%)   C_dpmr %.4f (theory %.4f, %+.2f %%)\n', ...
            Cfar_dx, C_dx, 100*(Cfar_dx/C_dx - 1), Cfar_dpmr, C_dpmr, 100*(Cfar_dpmr/C_dpmr - 1));
    fprintf('run average (near-wall dominated, NOT a test of the constant):\n');
    fprintf('C_dx    measured %.4f +- %.4f   theory %.4f   ratio %.4f   sigma %.1f\n', ...
            Chat_dx, rel_dx, C_dx, Chat_dx/C_dx, abs(Chat_dx-C_dx)/rel_dx);
    fprintf('C_dpmr  measured %.4f +- %.4f   theory %.4f   ratio %.4f   sigma %.1f\n', ...
            Chat_dpmr, rel_dxr, C_dpmr, Chat_dpmr/C_dpmr, abs(Chat_dpmr-C_dpmr)/rel_dxr);

    % segment table -- reported, not used for any of the numbers above
    SEG = {'hold start', tt > 0.05 & tt < 0.50; ...
           'descend',    tt > 0.55 & tt < 1.45; ...
           'oscillate',  tt > 1.60 & tt < 3.40; ...
           'hold end',   tt > 3.70};
    fprintf('%-11s %8s %10s %10s %10s %10s\n', 'segment', 'a/a_o', ...
            'dx meas', 'dx form', 'dxr meas', 'dxr form');
    for i = 1:size(SEG,1)
        m = SEG{i,2};
        fprintf('%-11s %8.4f %10.4g %10.4g %10.4g %10.4g   [um^2]\n', SEG{i,1}, ...
                mean(ab(m)), mean(Vm_dx(m))*K.R^2, mean(Vt_dx(m))*K.R^2, ...
                mean(Vm_dxr(m))*K.R^2, mean(Vt_dxr(m))*K.R^2);
    end

    R2 = K.R^2;
    fig = [];
    if opts.save_fig
        RAW = {squeeze(S.p_true_out(kk - L_dx,  opts.ax, 1:ns)), ...
               squeeze(S.dx_r_out(  kk - L_dxr, opts.ax, 1:ns))};
        fig = local_page(tt, ab, Vm_dx*R2, Vt_dx*R2, Vm_dxr*R2, Vt_dxr*R2, ns, ...
                         fullfile(out_dir, ['var_identity_' opts.style '.png']), ...
                         opts.style, opts.n_bin, RAW, R2);
    end

    out = struct('t', tt, 'a_bar', ab, ...
                 'Vm_dx', Vm_dx*R2, 'Vt_dx', Vt_dx*R2, ...
                 'Vm_dxr', Vm_dxr*R2, 'Vt_dxr', Vt_dxr*R2, ...
                 'C_dx', C_dx, 'C_n_fb', C_n_fb, 'C_dpmr', C_dpmr, 'C_n', C_n, ...
                 'Chat_dx', Chat_dx, 'Chat_dpmr', Chat_dpmr, ...
                 'rel_dx', rel_dx, 'rel_dxr', rel_dxr, ...
                 'tau_dx', tau_dx, 'tau_dxr', tau_dxr, ...
                 'lag_dx', L_dx, 'lag_dxr', L_dxr, 'lag_scan', sc, ...
                 'K', K, 'seeds', S.seeds, 'out_dir', out_dir);
    save(fullfile(out_dir, 'var_check.mat'), 'out', '-v7.3');
    fprintf('[var-check] saved var_check.mat -> %s\n', out_dir);
end

% ----------------------------------------------------------------------
function [rel, tau] = local_sem(x)
%LOCAL_SEM  Relative SEM of mean(x) with the effective sample size MEASURED
%   from the autocorrelation, not assumed. tau = 1 + 2*sum(rho), truncated at
%   the first non-positive lag (Geyer initial-positive rule).
    x = x(:);  mu = mean(x);  M = numel(x);
    y = x - mu;  c0 = mean(y.^2);
    maxlag = min(1000, floor(M/10));
    c = zeros(maxlag, 1);  nk = 0;
    for L = 1:maxlag
        rho = mean(y(1:end-L) .* y(1+L:end)) / c0;
        if rho <= 0; break; end
        nk = nk + 1;  c(nk) = rho;
    end
    tau = max(1 + 2*sum(c(1:nk)), 1);
    rel = (std(y) / sqrt(M / tau)) / abs(mu);
end

% ----------------------------------------------------------------------
function f = local_page(t, ab, Vm1, Vt1, Vm2, Vt2, nseed, png, style, nbin, RAW, R2)
%LOCAL_PAGE  Two rows (the two quantities) x two columns (the two x axes).
%   Colours follow this check's own prior art: green = theory, blue = measured
%   (verify_axm_cdpmr_6state.m, and the R22/a_m section's fig3).
    COL_TH = [0 0.6 0.2]; COL_M = [0 0.2 0.9];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [40 40 1500 950], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ROW = {Vm1, Vt1, 'var(\delta x)  [\mum^2]'; ...
           Vm2, Vt2, 'var(\delta x_r)  [\mum^2]'};
    for r = 1:2
        Vm = ROW{r,1}; Vt = ROW{r,2};
        % --- left: against time, every sample, nothing filtered -----------
        a1 = nexttile(tl, (r-1)*2 + 1); hold(a1, 'on');
        % A connected line, not a scatter: consecutive samples ARE consecutive
        % in time, so the line is the honest rendering. It is still every
        % sample -- thin, unsmoothed, drawn under the theory so the theory
        % stays readable through the band.
        h1 = plot(a1, t, Vm, '-', 'Color', COL_M, 'LineWidth', 0.5, ...
                  'DisplayName', sprintf('measured  (cross-seed var, N = %d)', nseed));
        h2 = plot(a1, t, Vt, '-', 'Color', COL_TH, 'LineWidth', 3.0, ...
                  'DisplayName', 'formula');
        xlabel(a1, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
        ylabel(a1, ROW{r,3}, 'FontSize', FS, 'FontWeight', 'bold');
        local_ax(a1, FS, AXLW);
        if r == 1
            legend(a1, [h2 h1], 'Location', 'northoutside', 'Orientation', ...
                   'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        end
        % --- right: against a/a_o -----------------------------------------
        a2 = nexttile(tl, (r-1)*2 + 2); hold(a2, 'on');
        [xl, yl] = local_theory_line(ab, Vt);
        g2 = plot(a2, xl, yl, '-', 'Color', COL_TH, 'LineWidth', 3.0, ...
                  'DisplayName', 'formula');
        if strcmpi(style, 'binned')
            [bx, by, be] = local_bin(ab, Vm, nbin, RAW{r}, R2);
            p2 = errorbar(a2, bx, by, be, 'o', 'Color', COL_M, ...
                          'MarkerFaceColor', COL_M, 'MarkerSize', 7, ...
                          'LineWidth', 1.8, 'LineStyle', 'none', ...
                          'DisplayName', sprintf('measured  (cross-seed var, N = %d)', nseed));
        else
            p2 = plot(a2, ab, Vm, '.', 'Color', COL_M, 'MarkerSize', 3, ...
                      'DisplayName', sprintf('measured  (cross-seed var, N = %d)', nseed));
        end
        xlabel(a2, 'a / a_o  [-]', 'FontSize', FS, 'FontWeight', 'bold');
        local_ax(a2, FS, AXLW);
        if r == 1
            legend(a2, [g2 p2], 'Location', 'northoutside', 'Orientation', ...
                   'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        end
    end
    exportgraphics(f, png, 'Resolution', 150);
    fprintf('[var-check] wrote %s\n', png);
end

% ----------------------------------------------------------------------
function local_ax(a, FS, AXLW)
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    axis(a, 'tight');
    yl = ylim(a); ylim(a, [0, yl(2)*1.05]);   % linear, anchored at zero:
end                                            % the identity has an intercept

% ----------------------------------------------------------------------
function [xl, yl] = local_theory_line(ab, Vt)
%LOCAL_THEORY_LINE  The formula is affine in a, so its trace against a is a
%   straight line; sort so it draws as one, not as a scribble.
    [xl, i] = sort(ab);  yl = Vt(i);
end

% ----------------------------------------------------------------------
function [bx, by, be] = local_bin(x, y, nbin, X, R2)
%LOCAL_BIN  Equal-count bins along a. The bar is a DELETE-ONE-SEED JACKKNIFE:
%   each replicate drops one seed's whole trajectory, so every within-seed
%   time correlation stays intact rather than being discounted by a factor.
%   The seed is the independent unit here; the k's are not, because each k
%   already uses all N seeds. Verified 2026-08-20 against an assumption-free
%   disjoint-group standard error (median ratio 0.98 over bins).
    [xs, i] = sort(x(:));  ys = y(i);  Xs = X(i, :);
    ns = size(Xs, 2);
    S1 = sum(Xs, 2);  S2 = sum(Xs.^2, 2);
    edges = round(linspace(1, numel(xs)+1, nbin+1));
    bx = zeros(nbin,1); by = zeros(nbin,1); be = zeros(nbin,1);
    for b = 1:nbin
        id = edges(b):edges(b+1)-1;
        bx(b) = mean(xs(id));  by(b) = mean(ys(id));
        xj = Xs(id, :);
        %   V_(-j)[k] = ((S2 - x_j^2) - (S1 - x_j)^2/(n-1)) / (n-2)
        Vloo = ((S2(id) - xj.^2) - (S1(id) - xj).^2 / (ns-1)) / (ns-2);
        th = mean(Vloo, 1) * R2;
        be(b) = sqrt((ns-1)/ns * sum((th - mean(th)).^2));
    end
end

% ----------------------------------------------------------------------
function [se, sig] = local_jack_C(CH, ax, ns, idx, R2, off, den, Chat)
%LOCAL_JACK_C  Delete-one-seed jackknife SE of the measured coefficient, and
%   the departure from theory in units of that SE.
    Xs = squeeze(CH(idx, ax, 1:ns));
    S1 = sum(Xs, 2);  S2 = sum(Xs.^2, 2);
    Vloo = ((S2 - Xs.^2) - (S1 - Xs).^2 / (ns-1)) / (ns-2) / R2;
    th = mean((Vloo - off) ./ den, 1);
    se = sqrt((ns-1)/ns * sum((th - mean(th)).^2));
    sig = mean(th);   % the jackknife-mean estimate, for a self-consistency check
    assert(abs(sig - Chat) < 1e-6 * abs(Chat), ...
           'jackknife mean %.6f disagrees with the direct estimate %.6f', sig, Chat);
end

% ----------------------------------------------------------------------
% The identity's coefficients as functions of the loop pole. Verified against
% the stored constants at lambda = lambda_c (build_eq17_6state_constants.m:107
% for the full a_pd pair, and the a_pd -> 0 limits for the raw pair).
function C = local_Cdpmr(l, apd)
    om = 1 - apd;  dp = 1 - om .* l;
    C = om^2 * ( 2*om*(1 - l)./dp + (2/(2 - apd)) ./ ((1 + l).*dp) );
end

function C = local_Cn(l, apd)
    om = 1 - apd;  dp = 1 - om .* l;
    C = (2*om^2/(2 - apd)) * ( 1 + om^2*apd*(1 - l)./dp + (1 - l).^2 ./ ((1 + l).*dp) );
end

function C = local_Cdx(l);   C = 2 + 1 ./ (1 - l.^2);      end
function C = local_Cnfb(l);  C = (1 - l) ./ (1 + l);       end
