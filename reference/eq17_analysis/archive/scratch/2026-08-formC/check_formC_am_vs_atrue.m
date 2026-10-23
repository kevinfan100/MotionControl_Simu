function out = check_formC_am_vs_atrue(S, opts)
%CHECK_FORMC_AM_VS_ATRUE  The variance readout a_m against the true gain of the
%   true motion: how well do they track, and exactly where do they part?
%
%   S   = load('test_results/formC_cdpmr_var_check/raw_seeds.mat');  S = S.S400;
%   out = check_formC_am_vs_atrue(S);
%
% STATUS: ACTIVE | companion to check_formC_atrue_var (same 400-seed stack)
%
% THE TWO QUANTITIES
%   a_true(k)   the gain at the particle's ACTUAL height -- the thing the loop
%               really has, only visible because this is a simulation.
%   a_m(k)      the readout the controller computes with no knowledge of the
%               wall: a_bar_wm = (sigma2_dwr_hat - C_n*sigma2_n)/(C_dpmr*kappa_T),
%               logged as a_xm = a_bar_wm * a_disp, a_disp = a_o*R = a_nom.
%               Physically: measure how hard the residual is shaking and invert
%               the closed-loop variance identity for a.
%   a_hat(k)    what the EKF believes after fusing the law, y1 and y2. Included
%               so the readout can be seen next to what it is used for.
%
% TWO DIFFERENT QUESTIONS, DELIBERATELY SEPARATED -- conflating them is how the
% "does the readout work" argument goes in circles:
%
%   (Q1) DETERMINISTIC TRACKING. Does the ensemble mean of a_m follow the
%        commanded 11x sweep of a? Measured as E[a_m]/E[a_true] - 1 per sample
%        and binned against height.
%   (Q2) PER-SEED SKILL. At a fixed instant, a_true differs between seeds by
%        eps = 0.003..3.9 % (Brownian wander, check_formC_atrue_var). Does a_m
%        see THAT? Measured as the across-seed correlation rho_k between a_m
%        and a_true at fixed k. a_m is an EWMA of squares with a_cov ~ 0.05, so
%        its own sampling noise is large and rho is expected to be small; this
%        quantifies "small" instead of asserting it.
%
% The distinction matters because the estimator only ever uses (Q1): y2 enters
% as a whitened AR(1) increment of a slow readout, not as a per-step gain
% measurement.
%
% INSTRUMENT CHECKS, written before the run:
%   C1  row 1 of every log is the controller's init-only call -- a_xm must be
%       exactly 0 there, and that row is dropped.
%   C2  the gate fraction must be ~0 in the end hold (measured 0 % on 2026-08-20
%       for G2/G3); if it is not, y2 is off and the comparison changes meaning.
%   C3  E[a_m]/E[a_true]-1 in the far-field hold must reproduce -0.53 % and in
%       the end hold +6.09 % (independently measured twice on 2026-08-20/21).
%       This is a regression check on the whole chain, not a new result.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');       opts.ax    = 3;   end
    if ~isfield(opts, 'n_bin');    opts.n_bin = 22;  end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'tag');      opts.tag   = '';  end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    out_dir = fullfile(root, 'test_results', ['formC_cdpmr_var_check' opts.tag]);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    ax = opts.ax;  K = S.K;  ns = numel(S.seeds);
    tt = S.t(:);

    am = squeeze(S.a_xm_out(:, ax, 1:ns))     / K.a_nom;    % readout      [-]
    at = squeeze(S.a_true_out(:, ax, 1:ns))   / K.a_nom;    % truth        [-]
    ah = squeeze(S.a_bar_hat_out(:, ax, 1:ns));             % EKF belief   [-]
    wb = squeeze(S.h_bar_true_out(:, 1, 1:ns));
    gt = squeeze(S.gate_out(:, ax, 1:ns));

    assert(all(am(1, :) == 0), 'C1: row 1 is not the init-only zero row');
    kk = 2:numel(tt);
    tt = tt(kk); am = am(kk,:); at = at(kk,:); ah = ah(kk,:); wb = wb(kk,:); gt = gt(kk,:);

    m_am = mean(am, 2);  m_at = mean(at, 2);  m_ah = mean(ah, 2);  m_w = mean(wb, 2);
    se_am = std(am, 0, 2) / sqrt(ns);
    se_ah = std(ah, 0, 2) / sqrt(ns);
    gfrac = mean(double(gt), 2);

    err_m = m_am ./ m_at - 1;                 % (Q1) readout error
    err_h = m_ah ./ m_at - 1;                 % same for the belief
    se_em = se_am ./ m_at;
    se_eh = se_ah ./ m_at;

    % (Q2) across-seed correlation at each fixed instant
    da = am - m_am;   dt = at - m_at;
    rho = sum(da .* dt, 2) ./ sqrt(sum(da.^2, 2) .* sum(dt.^2, 2));

    % ---- instrument checks -----------------------------------------------
    far = tt > 0.05 & tt < 0.50;   nea = tt > 3.70;
    fprintf('\n[C1] init-only row dropped, %d samples x %d seeds\n', numel(tt), ns);
    fprintf('[C2] gate-active fraction: far hold %.3f %%   end hold %.3f %%\n', ...
            100*mean(gfrac(far)), 100*mean(gfrac(nea)));
    fprintf('[C3] E[a_m]/E[a_true]-1: far hold %+.2f %% (expect -0.53)   end hold %+.2f %% (expect +6.09)\n', ...
            100*mean(err_m(far)), 100*mean(err_m(nea)));

    % ---- segment table ----------------------------------------------------
    SEG = {'hold start', tt > 0.05 & tt < 0.50; ...
           'descend',    tt > 0.55 & tt < 1.45; ...
           'oscillate',  tt > 1.60 & tt < 3.40; ...
           'hold end',   tt > 3.70};
    fprintf('\n%-11s %8s %10s %10s %10s %8s %8s\n', 'segment', 'a/a_o', ...
            'a_m err %', 'a_hat err%', 'sd(a_m) %', 'rho', 'gate %');
    for q = 1:size(SEG, 1)
        m = SEG{q, 2};
        fprintf('%-11s %8.4f %+9.2f %+10.2f %10.2f %8.3f %8.2f\n', SEG{q,1}, ...
                mean(m_at(m)), 100*mean(err_m(m)), 100*mean(err_h(m)), ...
                100*mean(std(am(m,:), 0, 2) ./ m_at(m)), mean(rho(m)), ...
                100*mean(gfrac(m)));
    end

    % ---- binned against the gain, with a SEED-GROUP error bar -------------
    % Bin on a/a_o, not on height: it is the variable the identity's error was
    % first seen in and it spreads the near-wall samples out instead of piling
    % 90 % of the run into the far-field bins. The bar is the project's own
    % arbiter (2026-08-20): split the seeds into G disjoint groups, compute the
    % bin value once per group, take std(group values)/sqrt(G). Samples inside a
    % bin are NOT independent (a_m is an EWMA with pole 1-a_cov = 0.95), so
    % dividing by sqrt(n_samples) would understate the bar by ~an order.
    G = 20;
    grp = mod((1:ns) - 1, G) + 1;
    use = tt > 0.20;
    edges = linspace(min(m_at(use)), max(m_at(use)), opts.n_bin + 1);
    ib = discretize(m_at(use), edges);
    tu = find(use);
    bw = nan(opts.n_bin,1); be = nan(opts.n_bin,1); bs = nan(opts.n_bin,1);
    ba = nan(opts.n_bin,1);
    for q = 1:opts.n_bin
        m = tu(ib == q);
        if numel(m) < 5; continue; end
        ba(q) = mean(m_at(m));  bw(q) = mean(m_w(m));
        gv = nan(G,1);
        for gg = 1:G
            sel = grp == gg;
            gv(gg) = mean(mean(am(m, sel), 2) ./ mean(at(m, sel), 2)) - 1;
        end
        be(q) = mean(gv);  bs(q) = std(gv) / sqrt(G);
    end
    fprintf('\n%8s %8s %10s %8s\n', 'a/a_o', 'w_bar', 'a_m err %', '+-SEM');
    for q = 1:opts.n_bin
        if isnan(ba(q)); continue; end
        fprintf('%8.4f %8.3f %+9.2f %8.2f\n', ba(q), bw(q), 100*be(q), 100*bs(q));
    end

    out = struct('t', tt, 'm_am', m_am, 'm_at', m_at, 'm_ah', m_ah, 'm_w', m_w, ...
                 'err_m', err_m, 'err_h', err_h, 'se_em', se_em, 'se_eh', se_eh, ...
                 'rho', rho, 'gfrac', gfrac, 'bw', bw, 'ba', ba, 'be', be, ...
                 'bs', bs, 'ns', ns);

    if opts.save_fig
        local_fig_time(out, fullfile(out_dir, 'am_vs_atrue_time.png'));
        local_fig_map(out, am, at, m_am, m_at, tt, fullfile(out_dir, 'am_vs_atrue_map.png'));
        fprintf('\nfigures -> %s\n', out_dir);
    end
end

% =======================================================================
function local_fig_time(o, fpath)
    FS = 18; AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  LBL = [0.35 0.65 0.95];
    GRY = [0.5 0.5 0.5];
    W = 151;                                     % display smoothing, stated
    f = figure('Position',[40 40 1250 1000],'Color','w','Visible','off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1, o.t, movmean(o.m_am, W), '-', 'Color', LBL, 'LineWidth', 2.6, ...
         'DisplayName','a_m  measured (movmean 151)');
    plot(a1, o.t, o.m_ah, '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','a_{hat}  estimate');
    plot(a1, o.t, o.m_at, '-', 'Color', RED, 'LineWidth', 2.2, ...
         'DisplayName','a_{true}  truth');
    ylabel(a1,'a / a_o'); ylim(a1,[0 1.05]);
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2, 0, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
    plot(a2, o.t, 100*movmean(o.err_m, W), '-', 'Color', LBL, 'LineWidth', 2.6, ...
         'DisplayName','a_m / a_{true} - 1  [%]');
    plot(a2, o.t, 100*o.err_h, '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','a_{hat} / a_{true} - 1  [%]');
    ylabel(a2,'[%]'); ylim(a2,[-15 30]);
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    plot(a3, o.t, o.rho, '-', 'Color', LBL, 'LineWidth', 2.2, ...
         'DisplayName','\rho(a_m, a_{true}) across seeds');
    plot(a3, o.t, o.gfrac, '-', 'Color', [0.1 0.6 0.2], 'LineWidth', 2.2, ...
         'DisplayName','y_2 gate active (fraction)');
    ylim(a3,[-0.2 1.05]); xlabel(a3,'t [s]'); ylabel(a3,'[-]');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end

% =======================================================================
function local_fig_map(o, am, at, m_am, m_at, tt, fpath)
    FS = 16; AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  LBL = [0.35 0.65 0.95];
    GRY = [0.5 0.5 0.5];
    f = figure('Position',[40 40 1600 560],'Color','w','Visible','off');
    tl = tiledlayout(f, 1, 3, 'TileSpacing','compact','Padding','compact');

    % (1) does the readout track the 11x sweep at all
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    lim = [0.05 1.1];
    plot(a1, lim, lim, '-', 'Color', GRY, 'LineWidth', 2.0, 'DisplayName','1:1');
    plot(a1, m_at, movmean(m_am, 151), '.', 'Color', LBL, 'MarkerSize', 8, ...
         'DisplayName','ensemble mean, per sample');
    set(a1,'XScale','log','YScale','log'); xlim(a1,lim); ylim(a1,lim);
    xlabel(a1,'a_{true} / a_o'); ylabel(a1,'a_m / a_o');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    % (2) where exactly it parts, against height
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2, 0, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
    errorbar(a2, o.ba, 100*o.be, 100*o.bs, 'o-', 'Color', RED, ...
             'MarkerFaceColor', RED, 'MarkerSize', 7, 'LineWidth', 2.0, ...
             'DisplayName','a_m / a_{true} - 1  [%]');
    set(a2,'XScale','log');
    xlabel(a2,'a_{true} / a_o'); ylabel(a2,'[%]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    % (3) per-seed skill at the trough: does a_m see the 3.9 % wander
    [~, k_tr] = min(m_at);
    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    dx = 100*(at(k_tr,:) - m_at(k_tr))/m_at(k_tr);
    dy = 100*(am(k_tr,:) - m_am(k_tr))/m_at(k_tr);
    plot(a3, dx, dy, 'o', 'Color', LBL, 'MarkerFaceColor', LBL, 'MarkerSize', 5, ...
         'DisplayName', sprintf('400 seeds at trough, \\rho = %.3f', o.rho(k_tr)));
    xl = [-1 1]*max(abs(dx))*1.1;
    plot(a3, xl, xl, '--', 'Color', RED, 'LineWidth', 2.2, ...
         'DisplayName','perfect per-seed skill (1:1)');
    xlim(a3, xl);
    xlabel(a3,'\Delta a_{true} / a  [%]'); ylabel(a3,'\Delta a_m / a  [%]');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
