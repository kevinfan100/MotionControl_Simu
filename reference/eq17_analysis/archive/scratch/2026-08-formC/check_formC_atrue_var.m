function out = check_formC_atrue_var(S, opts)
%CHECK_FORMC_ATRUE_VAR  How much does the TRUE gain a itself fluctuate, and is
%   that fluctuation just the slope a' times the Brownian wander in height?
%
%   S   = load('test_results/formC_cdpmr_var_check/raw_seeds.mat');  S = S.S;
%   out = check_formC_atrue_var(S);
%
% STATUS: ACTIVE | companion to check_formC_var_identity (same 400-seed stack)
%
% THE QUESTION. a_true(k) is evaluated at the particle's ACTUAL height, not at
% the commanded one, so it is a random variable: the same command trajectory
% gives a different a in every seed because Brownian motion puts the particle
% somewhere else. Two things follow and both are measured here:
%
%   (1) LINEAR PROPAGATION
%           delta_a = a'(w_bar) * delta_w_bar ,   delta_w_bar = delta_x / R
%       =>  sd(a_true)  =  |a'| * sd(w_bar)                             [I]
%       The prediction side of [I] uses a' taken from the PLANT's own map,
%       not from the controller's law: a_true is a deterministic function of
%       h_bar_true, so the pooled (h_bar_true, a_true) cloud over all seeds and
%       all samples IS that map, and its numerical derivative is a'. Nothing is
%       fitted and the controller's b_hat never enters.
%
%   (2) SECOND ORDER (Jensen) -- the part that can bias, not just spread:
%           E[a_true] - a(w_bar_d)  =  1/2 * a''(w_bar) * var(w_bar)     [II]
%
% WHY IT MATTERS. Every closed-loop variance identity in this project is
% derived with a held CONSTANT over the window. The leading candidate for the
% unexplained ~1/4 of the near-wall variance excess (memory
% project-formC-var-identity-deep-band-2026-08-20) is that a is instead
% modulated by the particle's own displacement -- multiplicative noise, which a
% constant-a Lyapunov equation cannot represent. The size of that effect is
% governed by
%           eps = sd(a_true) / E[a_true]
% and enters the variance at O(eps^2), because the O(eps) term is odd in
% delta_x and integrates to zero for a symmetric distribution. This script
% measures eps and therefore BOUNDS the candidate before the 5.5-minute frozen
% c_perp arm is spent on it. Registered before looking (rule 13): if eps^2 lands
% two orders below the 10.8 pp excess, the frozen-c_perp arm is predicted NULL.
%
% A third, less obvious channel is included because it is NOT O(eps^2) small a
% priori: the realized loop pole depends on a through g = a_true/a_hat, and
% C(lambda) = 2 + 1/(1-lambda^2) is strongly convex, so a FLUCTUATING g biases
% the variance upward by 1/2 * C''(lambda) * var(delta_lambda) with
% delta_lambda = (1-lambda_c)*g*eps. That is Jensen on the pole rather than on
% the gain, and it is reported separately.
%
% INSTRUMENT CHECKS, written before the run:
%   C1  a_true must be a SINGLE-VALUED function of h_bar_true. If the two are
%       logged at different instants (the project's known off-by-one), the
%       pooled cloud fattens and the within-bin spread stops being ~0.
%   C2  the empirical a' must agree with the controller's own a_prime_out in
%       the FAR field (where the one-parameter law is known to be right) and
%       may differ near the wall (measured 0.93-1.04 on 2026-08-21).
%   C3  sd(w_bar) must equal sd(p_true_z)/R -- the same wander seen two ways.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');       opts.ax    = 3;   end     % wall normal
    if ~isfield(opts, 'n_curve');  opts.n_curve = 600; end   % bins for the map
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'tag');      opts.tag   = '';   end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    out_dir = fullfile(root, 'test_results', ['formC_cdpmr_var_check' opts.tag]);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    ax = opts.ax;  K = S.K;  ns = numel(S.seeds);
    tt = S.t(:);   N = numel(tt);
    lc = K.lambda_c;

    % ---- the measured quantities ---------------------------------------
    abar = squeeze(S.a_true_out(:, ax, 1:ns)) / K.a_nom;    % N x ns  [-]
    wbar = squeeze(S.h_bar_true_out(:, 1, 1:ns));           % N x ns  [-]
    ptru = squeeze(S.p_true_out(:, ax, 1:ns));              % N x ns  [um]
    ahat = squeeze(S.a_bar_hat_out(:, ax, 1:ns));           % N x ns  [-]

    m_a = mean(abar, 2);   sd_a = std(abar, 0, 2);
    m_w = mean(wbar, 2);   sd_w = std(wbar, 0, 2);
    sd_p = std(ptru, 0, 2) / K.R;
    eps_rel = sd_a ./ max(m_a, eps);

    % ---- the plant's own map a(w_bar), from the data itself -------------
    keep = tt > 0.02;                       % drop the init-only row and warmup
    wp = wbar(keep, :);  ap = abar(keep, :);
    wp = wp(:);          ap = ap(:);
    edges = linspace(min(wp), max(wp), opts.n_curve + 1);
    ib = discretize(wp, edges);
    ok = ~isnan(ib);
    wc = accumarray(ib(ok), wp(ok), [opts.n_curve 1], @mean, NaN);
    ac = accumarray(ib(ok), ap(ok), [opts.n_curve 1], @mean, NaN);
    sc = accumarray(ib(ok), ap(ok), [opts.n_curve 1], @std,  NaN);   % C1
    g  = ~isnan(wc);
    wc = wc(g);  ac = ac(g);  sc = sc(g);
    ap_curve  = gradient(ac, wc);            % a'(w_bar)
    app_curve = gradient(ap_curve, wc);      % a''(w_bar)

    a_p  = interp1(wc, ap_curve,  m_w, 'linear', 'extrap');
    a_pp = interp1(wc, app_curve, m_w, 'linear', 'extrap');

    sd_pred = abs(a_p) .* sd_w;                       % identity [I]
    jensen  = 0.5 * a_pp .* sd_w.^2;                  % identity [II]

    % ---- the two O(eps^2) channels --------------------------------------
    g_mean  = mean(abar ./ max(ahat, eps), 2);        % realized pole factor
    lam_eff = 1 - g_mean * (1 - lc);
    Cl   = 2 + 1 ./ (1 - lam_eff.^2);
    Cpp  = (2*(1 - lam_eff.^2) + 8*lam_eff.^2) ./ (1 - lam_eff.^2).^3;
    dlam = (1 - lc) * g_mean .* eps_rel;
    pole_jensen = 0.5 * Cpp .* dlam.^2 ./ Cl;         % relative, [-]

    % ---- the excess this is a candidate for ------------------------------
    kappa_T = 4 * (K.kBT / K.R) * K.a_o;
    sg2n_nd = K.sigma2_n_s(ax) / K.R^2;
    C_dx    = 2 + 1/(1 - lc^2);
    C_n_fb  = (1 - lc)/(1 + lc);
    V_dx    = var(ptru, 0, 2) / K.R^2;
    th_dx   = C_dx * kappa_T * m_a + C_n_fb * sg2n_nd;
    ratio_dx = V_dx ./ th_dx;

    % ---- instrument checks ------------------------------------------------
    dwid = median(diff(wc));
    c1_within = median(sc(~isnan(sc)));
    c1_slope  = median(abs(ap_curve)) * dwid;
    fprintf('\n[C1] within-bin sd(a_bar) = %.3e ; slope*binwidth = %.3e -> ratio %.2f\n', ...
            c1_within, c1_slope, c1_within / c1_slope);
    if isfield(S, 'a_prime_out')
        apc = mean(squeeze(S.a_prime_out(:, ax, 1:ns)), 2);
        far = tt > 0.05 & tt < 0.50;   nea = tt > 3.70;
        fprintf('[C2] a''(empirical)/a''(controller law): far %.4f  near-wall %.4f\n', ...
                mean(a_p(far))/mean(apc(far)), mean(a_p(nea))/mean(apc(nea)));
    else
        fprintf('[C2] a_prime_out not in this stack -- skipped\n');
    end
    fprintf('[C3] median |sd(w_bar) - sd(p_true)/R| / sd(w_bar) = %.2e\n', ...
            median(abs(sd_w(keep) - sd_p(keep)) ./ sd_w(keep)));

    % ---- segment table ----------------------------------------------------
    SEG = {'hold start', tt > 0.05 & tt < 0.50; ...
           'descend',    tt > 0.55 & tt < 1.45; ...
           'oscillate',  tt > 1.60 & tt < 3.40; ...
           'hold end',   tt > 3.70};
    fprintf('\n%-11s %8s %9s %9s %9s %9s %9s %9s %9s\n', 'segment', 'a/a_o', ...
            'sd(a)', 'eps %', 'a''', 'sd(w)', 'meas/pred', 'eps^2 %', 'pole %');
    for q = 1:size(SEG, 1)
        m = SEG{q, 2};
        fprintf('%-11s %8.4f %9.2e %9.3f %9.4f %9.2e %9.4f %9.4f %9.4f\n', ...
                SEG{q,1}, mean(m_a(m)), mean(sd_a(m)), 100*mean(eps_rel(m)), ...
                mean(a_p(m)), mean(sd_w(m)), ...
                mean(sd_a(m))/mean(sd_pred(m)), 100*mean(eps_rel(m).^2), ...
                100*mean(pole_jensen(m)));
    end
    fprintf('\n%-11s %10s %10s %12s\n', 'segment', 'E[a]-a(w_d)', 'Jensen[II]', 'var excess');
    for q = 1:size(SEG, 1)
        m = SEG{q, 2};
        fprintf('%-11s %10.3e %10.3e %11.1f%%\n', SEG{q,1}, ...
                mean(m_a(m)) - interp1(wc, ac, mean(m_w(m)), 'linear', 'extrap'), ...
                mean(jensen(m)), 100*(mean(ratio_dx(m)) - 1));
    end

    out = struct('t', tt, 'm_a', m_a, 'sd_a', sd_a, 'sd_pred', sd_pred, ...
                 'eps', eps_rel, 'a_p', a_p, 'a_pp', a_pp, 'm_w', m_w, ...
                 'sd_w', sd_w, 'jensen', jensen, 'pole_jensen', pole_jensen, ...
                 'ratio_dx', ratio_dx, 'wc', wc, 'ac', ac, 'ap_curve', ap_curve, ...
                 'ns', ns, 'SEG', {SEG});

    if opts.save_fig
        local_fig_time(out, K, fullfile(out_dir, 'atrue_var_time.png'));
        local_fig_map(out, wbar, abar, tt, fullfile(out_dir, 'atrue_var_map.png'));
        fprintf('\nfigures -> %s\n', out_dir);
    end
end

% =======================================================================
function local_fig_time(o, K, fpath)
    FS = 18;  AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  GRY = [0.5 0.5 0.5];
    f = figure('Position',[40 40 1250 1000],'Color','w','Visible','off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing','compact','Padding','compact');

    kk = 2:numel(o.t);            % row 1 of every log is the init-only zero row
    o.t = o.t(kk); o.m_a = o.m_a(kk); o.sd_a = o.sd_a(kk); o.sd_pred = o.sd_pred(kk);
    o.eps = o.eps(kk); o.pole_jensen = o.pole_jensen(kk); o.ratio_dx = o.ratio_dx(kk);

    a1 = nexttile(tl, 1); hold(a1,'on'); box(a1,'on');
    hb = fill(a1, [o.t; flipud(o.t)], [o.m_a + 10*o.sd_a; flipud(o.m_a - 10*o.sd_a)], ...
              RED, 'FaceAlpha', 0.20, 'EdgeColor','none', 'DisplayName','\pm10 sd(a_{true})');
    h1 = plot(a1, o.t, o.m_a, '-', 'Color', RED, 'LineWidth', 2.2, ...
              'DisplayName','E[a_{true}] / a_o');
    ylabel(a1, 'a / a_o');
    legend(a1, [h1 hb], 'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl, 2); hold(a2,'on'); box(a2,'on');
    plot(a2, o.t, o.sd_a, '-', 'Color', RED, 'LineWidth', 2.2, ...
         'DisplayName','measured  sd(a_{true})');
    plot(a2, o.t, o.sd_pred, '--', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','predicted  |a''| \cdot sd(w)');
    set(a2,'YScale','log'); ylim(a2,[1e-5 1e-2]);
    ylabel(a2, 'sd(a) / a_o');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl, 3); hold(a3,'on'); box(a3,'on');
    plot(a3, o.t, 100*(movmean(o.ratio_dx, 201) - 1), '-', 'Color', GRY, 'LineWidth', 2.4, ...
         'DisplayName','var(\delta x) excess [%] (movmean 201)');
    plot(a3, o.t, 100*o.eps, '-', 'Color', RED, 'LineWidth', 2.2, ...
         'DisplayName','\epsilon = sd(a)/E[a] [%]');
    plot(a3, o.t, 100*o.eps.^2, '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','\epsilon^2 [%]');
    plot(a3, o.t, 100*o.pole_jensen, ':', 'Color', [0.1 0.6 0.2], 'LineWidth', 2.2, ...
         'DisplayName','pole Jensen [%]');
    set(a3,'YScale','log'); ylim(a3,[1e-4 1e2]);
    xlabel(a3, 't [s]'); ylabel(a3, '[%]');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end

% =======================================================================
function local_fig_map(o, wbar, abar, tt, fpath)
% Left: the plant's own map a(w), log-w so both operating points fit, with the
% two sampled instants marked. Middle/right: the 400-seed cloud at each of
% those instants, plotted as DEVIATIONS -- horizontal in w_bar, vertical in
% percent of a -- so the two panels are directly comparable. Same physics,
% same Brownian wander; only the local slope differs.
    FS = 16;  AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  LBL = [0.35 0.65 0.95];
    [~, k_tr]  = min(o.m_a);
    [~, k_far] = min(abs(tt - 0.30));

    f = figure('Position',[40 40 1600 560],'Color','w','Visible','off');
    tl = tiledlayout(f, 1, 3, 'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl, 1); hold(a1,'on'); box(a1,'on');
    plot(a1, o.wc, o.ac, '-', 'Color', RED, 'LineWidth', 2.4, 'DisplayName','a(w) plant map');
    plot(a1, o.m_w(k_far), o.m_a(k_far), 'o', 'Color', LBL, 'MarkerFaceColor', LBL, ...
         'MarkerSize', 12, 'DisplayName','far hold');
    plot(a1, o.m_w(k_tr), o.m_a(k_tr), 'o', 'Color', BLU, 'MarkerFaceColor', BLU, ...
         'MarkerSize', 12, 'DisplayName','trough');
    set(a1,'XScale','log');
    xlabel(a1,'w = h / R'); ylabel(a1,'a / a_o');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    KS = [k_far k_tr];  CS = {LBL, RED};  NM = {'far hold', 'trough'};
    for q = 1:2
        k = KS(q);
        aq = nexttile(tl, q+1); hold(aq,'on'); box(aq,'on');
        dw = wbar(k,:) - o.m_w(k);
        da = 100 * (abar(k,:) - o.m_a(k)) / o.m_a(k);
        plot(aq, dw, da, 'o', 'Color', CS{q}, 'MarkerFaceColor', CS{q}, ...
             'MarkerSize', 5, 'DisplayName', sprintf('400 seeds, %s', NM{q}));
        xt = [-1 1] * 4 * o.sd_w(k);
        plot(aq, xt, 100 * o.a_p(k) * xt / o.m_a(k), '--', 'Color', BLU, ...
             'LineWidth', 2.4, 'DisplayName', sprintf('slope a'' = %.4f', o.a_p(k)));
        % +-1 sd guides: the horizontal pair is how far the particle wanders,
        % the vertical pair is what that wander does to the gain. Same 400
        % dots, read on the two axes.
        sw = o.sd_w(k);  sa = 100 * o.sd_a(k) / o.m_a(k);
        for sgn = [-1 1]
            xline(aq, sgn*sw, ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0, ...
                  'HandleVisibility','off');
            yline(aq, sgn*sa, ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0, ...
                  'HandleVisibility','off');
        end
        text(aq, 0.04, 0.93, sprintf('\\pm1 sd(w) = %.4f  (%.1f nm)', sw, sw*2250), ...
             'Units','normalized','FontSize',13,'FontWeight','bold');
        text(aq, 0.04, 0.85, sprintf('\\pm1 sd(a)/a = %.3f %%', sa), ...
             'Units','normalized','FontSize',13,'FontWeight','bold');
        xlabel(aq,'\Delta w'); ylabel(aq,'\Delta a / a  [%]');
        legend(aq,'Location','northoutside','Orientation','horizontal','FontSize',12);
        set(aq,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    end

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
