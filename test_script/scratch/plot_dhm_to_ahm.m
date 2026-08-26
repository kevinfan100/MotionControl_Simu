% STATUS: ACTIVE (scratch figure) | PURPOSE: the delta h_m -> a_hm readout
%   chain of ONE complete run, one panel per equation, physical units (um).
%
% NOTATION -- reference/shared/writeup_architecture.tex S2.1/S2.2, the
%   cross-controller SSOT, with x -> h since the axis under test is the wall
%   normal. All three quantities are subscripted (no overline is needed, and
%   MATLAB's tex interpreter cannot draw one):
%       delta h_m   measured tracking error        (= y_1)
%       delta h_md  its IIR mean, pole a_pd
%       delta h_mr  the residual                   (C_dpmr is named after it)
%   pm_to_axm_derivation.tex writes the residual delta x_r instead; the mr
%   form is used here because it is the one the constant is named after and
%   because it keeps the three symbols in one family.
%
% ESTIMATE-HAT. The project figure convention (plot_var_ahat_6state.m) writes
%   a hat as \^ under the tex interpreter, which draws a caret, not a true
%   accent. Followed here for \^sigma^2 so the page matches the other figures.
%
% THE SECOND TERM OF THE VARIANCE. writeup_architecture S2.2 defines
%       sigma^2_{dx_mr}[k] = mean(dx_mr^2)[k] - (dx_mrd[k])^2
%   with a THIRD IIR (pole a_prd) on the residual's own mean. formC has no
%   a_prd -- the field does not exist in its ctrl_const; only the 23-state and
%   eq6 controllers carry it. Dropping the term is correct here: H_HP(z=1)=0
%   makes delta h_mr exactly zero-mean (measured -0.0067 nm over the ramp), so
%   the EWMA of the square is already unbiased, while subtracting (dh_mrd)^2
%   would introduce the finite-window bias the same SSOT calls beta (measured:
%   the omitted term is 8.04% of the EWMA, beta = 0.9196, matching the closed
%   form at calc_ctrl_params.m:130).
%
% EQUATION INDEXING. The panels are labelled in the paper's [k+1]-output form
%   (paper Eq.9: overline{dx_m}[k+1] = a_var dx_m[k] + (1-a_var) overline{dx_m}[k]).
%   The code matches that form letter for letter -- one line produces the new
%   mean from the current sample and the previous mean. The ONLY place where a
%   real difference between the paper and this code could hide is the residual:
%   this code subtracts the POST-update mean,
%       delta h_mr[k] = delta h_m[k] - delta h_md[k+1] ,
%   whereas subtracting the PRE-update mean (delta h_md[k]) is a different
%   filter. The two are the same signal scaled by (1-a_pd) -- verified pointwise
%   to 7e-18, ratio 0.9500000000 with sd 7.4e-14, corr 1.000000000000 -- and the
%   scale is absorbed by C_dpmr, whose closed form carries the matching
%   (1-a_pd)^2 prefactor (3.16095 here; 3.50244 for the pre-update variant).
%   Mixing the two costs 10.8% on a_hm. Nothing here mixes them.
%
% AXES. Panel 4 is logarithmic -- (delta h_mr)^2 really does span six decades.
%   Panels 5, 6, 7 are LINEAR and share their shape, because a_hm is an affine
%   map of \^sigma^2. A log axis on panel 5 hides that: the same 40% relative
%   scatter reads as a small wiggle on eight decades and as a large swing on a
%   linear one. Measured rel sd: 57.6% for \^sigma^2, 59.1% for a_hm.
%
%   The KF estimate is deliberately absent. This page is the readout chain and
%   it ends at a_hm; the filter is a separate machine.
%
%   The offline chain is asserted bit-exact against the controller's own a_xm
%   log before anything is drawn.
function out = plot_dhm_to_ahm(r, ax, opts)

    if nargin < 2 || isempty(ax); ax = 3; end
    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'tag');  opts.tag  = ''; end
    if ~isfield(opts, 'no_figure'); opts.no_figure = false; end   % data only
    % Default window = the Meng RAMP, t 0.5 -> 10.5 s, exactly 10 s. The run is
    % 12.5 s only because a 0.5 s hold precedes it and 2 s of hold follow it.
    if ~isfield(opts, 'tlim'); opts.tlim = [0.5 10.5]; end   % [] = whole run

    cc = r.ctrl_const;  P = r.meta.params_value;
    a_disp = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);   % [um/pN] per unit a_bar
    a_o    = a_disp / r.R;
    kT     = 4 * (P.ctrl.k_B * P.ctrl.T / r.R) * a_o;       % kappa_T (normalized)
    fourkT = kT * r.R / a_o;                                % 4 kB T   [pN um]

    % ---- the chain, in the controller's normalized internals -----------
    dh = r.dh_m_out(:, ax) / r.R;
    aM = r.a_xm_out(:, ax) / a_disp;
    s2n_nd = P.ctrl.sigma2_noise(ax) / r.R^2;
    N = numel(dh);  md = zeros(N,1);  s2 = zeros(N,1);
    md(1) = dh(1);  s2(1) = aM(1)*cc.C_dpmr*kT + cc.C_n*s2n_nd;
    for k = 2:N
        md(k) = (1-cc.a_pd)*md(k-1) + cc.a_pd*dh(k);
        s2(k) = (1-cc.a_cov)*s2(k-1) + cc.a_cov*(dh(k)-md(k))^2;
    end

    % ---- physical units: um ---------------------------------------------
    UM   = r.R;                                       % normalized -> um
    dhm  = dh * UM;                                   % [um]
    dhmd = md * UM;                                   % [um]
    dhmr = dhm - dhmd;                                % [um]
    s2p  = s2 * UM^2;                                 % [um^2]
    Cn_s2n = cc.C_n * P.ctrl.sigma2_noise(ax);        % [um^2]
    den    = cc.C_dpmr * fourkT;                      % [pN um]
    ahm  = (s2p - Cn_s2n) / den;                      % [um/pN]
    ah   = r.a_true_out(:, ax);                       % [um/pN]

    % ---- liveness: the offline chain must BE the controller ------------
    % The comparison must start far enough past the init that the seeded
    % sigma-hat^2(1) has been forgotten: the EWMA memory is 1/a_cov steps, so
    % 40 time constants (and at least 2000 steps). At a_cov = 0.0125 a fixed
    % k = 2000 start leaves a 1.5e-11 residue of the seed; from k = 5000 it is
    % 5.8e-16, i.e. machine precision. Not a defect -- a seeding artefact.
    i0 = max(2000, round(40/cc.a_cov)):N;
    rel = max(abs((ahm(i0) - r.a_xm_out(i0, ax)) ./ r.a_xm_out(i0, ax)));
    assert(rel < 1e-12, 'plot_dhm_to_ahm:chain', ...
        'offline chain does not reproduce the controller (max rel %.3e)', rel);
    fprintf('chain check: max|rel| vs controller a_xm = %.3e  (PASS < 1e-12)\n', rel);
    fprintf('constants  : 4kBT %.6g pN um | C_dpmr %.4f | C_n %.4f | C_dpmr*4kBT %.6g pN um\n', ...
            fourkT, cc.C_dpmr, cc.C_n, den);
    fprintf('             sigma_n %.5g um | C_n*sigma_n^2 %.5g um^2 | a_pd %.4g | a_cov %.4g\n', ...
            sqrt(P.ctrl.sigma2_noise(ax)), Cn_s2n, cc.a_pd, cc.a_cov);

    i = (2:N).';                                     % row 1 of the log is init-only
    if ~isempty(opts.tlim)
        i = i(r.tout(i) >= opts.tlim(1) & r.tout(i) <= opts.tlim(2));
    end
    t = r.tout(i);

    out = struct('t', t, 'i', i, 'dhm', dhm, 'dhmd', dhmd, 'dhmr', dhmr, ...
                 's2', s2p, 'ahm', ahm, 'ah', ah, ...
                 'Cn_s2n', Cn_s2n, 'den', den, 'fourkT', fourkT, 'R', r.R, ...
                 'a_pd', cc.a_pd, 'a_cov', cc.a_cov);
    if ~opts.no_figure; local_page(out, r, ax, cc, opts.tag); end
end

% ---------------------------------------------------------------------
function local_page(o, r, ax, cc, tag)
%LOCAL_PAGE  Project figure style: FS 18 bold, LFS 13 bold, AXLW 2.0, no grid,
%   box on, legend northoutside horizontal, no title, Resolution 150,
%   True = red / Estimate = blue / Measured = light blue.
    COL_TRUE = [0.8 0 0];  COL_HAT = [0 0.2 0.9];  COL_MEAS = [0.45 0.72 0.95];
    COL_LP   = [0.10 0.55 0.15];   % an IIR output: neither true nor measured
    COL_SQ   = [0.35 0.35 0.35];
    COL_S2   = [0.85 0.33 0.10];
    COL_CN   = [0.40 0.40 0.40];
    COL_BG   = [0.70 0.70 0.70];
    FS = 18; LFS = 13; AXLW = 2.0; LWD = 0.6;
    % A REAL hat. The project template writes \^a under the tex interpreter,
    % which draws a caret NEXT TO the symbol, not above it. U+03C3 followed by
    % the combining circumflex U+0302 renders a true sigma-hat in the same bold
    % sans font, so the estimate mark sits where it belongs.
    SH = [char(963) char(770)];
    i = o.i;  t = o.t;
    sq    = o.dhmr(i).^2;
    s2    = o.s2(i);
    ahT   = o.ah(i);
    ident = o.den*ahT + o.Cn_s2n;
    S2MAX = 2.6e-3;                                   % [um^2]

    f = figure('Position', [10 10 1700 2150], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 7, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    A = gobjects(7,1);
    LG = @(a,h,c) legend(a, h, c, 'Location', 'northoutside', ...
            'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

    % --- 1 : (1) ---------------------------------------------------------
    a = nexttile(tl); A(1)=a; hold(a,'on');
    yline(a, 0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off');
    h1 = plot(a, t, o.dhm(i), '-', 'Color', COL_MEAS, 'LineWidth', LWD);
    LG(a, h1, {'\delta h_m[k] = h_d[k] - h_m[k]        (1)'});
    ylabel(a, '\delta h_m  (\mum)', 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a, [-0.12 0.12]); set(a, 'YTick', -0.1:0.05:0.1);

    % --- 2 : (2) ---------------------------------------------------------
    a = nexttile(tl); A(2)=a; hold(a,'on');
    yline(a, 0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off');
    h0 = plot(a, t, o.dhm(i),  '-', 'Color', COL_BG, 'LineWidth', LWD);
    h1 = plot(a, t, o.dhmd(i), '-', 'Color', COL_LP, 'LineWidth', 2.4);
    LG(a, [h0 h1], {'\delta h_m', ...
        sprintf('\\delta h_{md}[k+1] = a_{pd} \\delta h_m[k] + (1-a_{pd}) \\delta h_{md}[k]        (2)        a_{pd} = %.3g', cc.a_pd)});
    ylabel(a, '\delta h_{md}  (\mum)', 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a, [-0.12 0.12]); set(a, 'YTick', -0.1:0.05:0.1);

    % --- 3 : (3) ---------------------------------------------------------
    a = nexttile(tl); A(3)=a; hold(a,'on');
    yline(a, 0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off');
    h1 = plot(a, t, o.dhmr(i), '-', 'Color', COL_HAT, 'LineWidth', LWD);
    LG(a, h1, {'\delta h_{mr}[k] = \delta h_m[k] - \delta h_{md}[k+1]        (3)'});
    ylabel(a, '\delta h_{mr}  (\mum)', 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a, [-0.12 0.12]); set(a, 'YTick', -0.1:0.05:0.1);

    % --- 4 : the argument of (4), log ------------------------------------
    a = nexttile(tl); A(4)=a; hold(a,'on');
    h1 = plot(a, t, sq, '-', 'Color', COL_SQ, 'LineWidth', 0.45);
    set(a, 'YScale', 'log'); ylim(a, [1e-7 3e-2]);
    set(a, 'YTick', 10.^(-7:-2));
    LG(a, h1, {'( \delta h_{mr}[k] )^2'});
    ylabel(a, '(\delta h_{mr})^2  (\mum^2)', 'FontSize', FS, 'FontWeight', 'bold');

    % --- 5 : (4),(5), LINEAR ---------------------------------------------
    a = nexttile(tl); A(5)=a; hold(a,'on');
    h1 = plot(a, t, s2, '-', 'Color', COL_S2, 'LineWidth', 1.6);
    h2 = yline(a, o.Cn_s2n, '--', 'Color', COL_CN, 'LineWidth', 2.0);
    LG(a, [h1 h2], { ...
        sprintf('%s^2_{\\delta h_{mr}}[k+1] = a_{cov} ( \\delta h_{mr}[k] )^2 + (1-a_{cov}) %s^2_{\\delta h_{mr}}[k]        (4)        a_{cov} = %.3g', SH, SH, cc.a_cov), ...
        sprintf('C_n \\sigma_n^2 = %.4g \\mum^2', o.Cn_s2n)});
    ylabel(a, [SH '^2_{\delta h_{mr}}  (\mum^2)'], 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a, [0 S2MAX]); set(a, 'YTick', 0:5e-4:2.5e-3);

    % --- 6 : (5) the identity, LINEAR ------------------------------------
    a = nexttile(tl); A(6)=a; hold(a,'on');
    h1 = plot(a, t, s2,    '-', 'Color', COL_S2,   'LineWidth', 1.0);
    h2 = plot(a, t, ident, '-', 'Color', COL_TRUE, 'LineWidth', 3.0);
    h3 = yline(a, o.Cn_s2n, '--', 'Color', COL_CN, 'LineWidth', 2.0);
    LG(a, [h1 h2 h3], {[SH '^2_{\delta h_{mr}}'], ...
        sprintf('C_{dpmr} 4k_BT a_h + C_n \\sigma_n^2        (5)        C_{dpmr}4k_BT = %.4g pN \\mum', o.den), ...
        'C_n \sigma_n^2'});
    ylabel(a, [SH '^2_{\delta h_{mr}}  (\mum^2)'], 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a, [0 S2MAX]); set(a, 'YTick', 0:5e-4:2.5e-3);

    % --- 7 : (6) ---------------------------------------------------------
    a = nexttile(tl); A(7)=a; hold(a,'on');
    h1 = plot(a, t, o.ahm(i), '-', 'Color', COL_MEAS, 'LineWidth', 0.9);
    h2 = plot(a, t, ahT,      '-', 'Color', COL_TRUE, 'LineWidth', 3.0);
    LG(a, [h1 h2], { ...
        ['a_{hm}[k] = ( ' SH '^2_{\delta h_{mr}}[k+1] - C_n \sigma_n^2 ) / ( C_{dpmr} 4k_BT )        (6)'], ...
        'a_h'});
    ylabel(a, 'a_h  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a, [0 S2MAX/o.den]); set(a, 'YTick', 0:0.01:0.04);
    xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');

    for q = 1:7
        set(A(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
        grid(A(q), 'off');
        xlim(A(q), [t(1) t(end)]);
        if q < 7; set(A(q), 'XTickLabel', []); end
    end

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if ~exist(od, 'dir'); mkdir(od); end
    fn = fullfile(od, sprintf('dhm_to_ahm%s.png', tag));
    exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure -> %s\n', fn);
end
