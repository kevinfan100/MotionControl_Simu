function out = check_formC_y2_instant(S, opts)
%CHECK_FORMC_Y2_INSTANT  Does the readout lag seen in a_m reach the filter?
%
%   S   = load('test_results/formC_cdpmr_var_check/raw_seeds.mat');  S = S.S400;
%   out = check_formC_y2_instant(S);
%
% STATUS: ACTIVE | follow-up to check_formC_am_vs_atrue (same 400-seed stack)
%
% THE QUESTION. a_m = a_bar_wm is an EWMA with pole 1-a_cov, so it lags by
% tau = 1/a_cov = 20 steps = 12.5 ms, and check_formC_am_vs_atrue measures that
% lag as a +30 % / -10 % swing around the trough. But the KF is NOT fed a_m. It
% is fed the WHITENED increment (motion_control_law_formC_b.m:923)
%
%       y2[k] = a_bar_wm[k] - (1-a_cov)*a_bar_wm[k-1] = a_cov * u[k]
%       u[k]  = (dw_r[k]^2 - C_n*sigma2_n) / (C_dpmr*kappa_T)
%
% with H2 scaled by a_cov to match (:924). The whitening is an EXACT inverse of
% the EWMA: y2 depends on dw_r[k] alone, so the a_cov lag cannot reach the
% filter. This script tests that claim on the recorded data instead of trusting
% the algebra, by rebuilding u[k] from the logged dx_r and comparing it with
% a_true the same way a_m was compared.
%
% WHAT SURVIVES IF THE CLAIM IS TRUE. Two other memories are NOT inverted:
%   (a) the a_pd = 0.05 high-pass: dw_r = delta_w_m - LPF(delta_w_m), and
%       C_dpmr / C_n are STATIONARY constants derived for a held constant.
%       While a sweeps, the LPF state carries old motion, and nothing in the
%       measurement model represents that transient.
%   (b) the d = 2 step measurement delay, which IS modelled (delay_steps in R2,
%       and the delayed state in H). It is scanned here, not assumed.
%
% PREDICTION, REGISTERED BEFORE RUNNING (rule 13):
%   P1  the +30 %/-10 % swing at the trough DISAPPEARS in u[k] if it is purely
%       the a_cov EWMA. Any residue is candidate (a).
%   P2  the near-wall bias (+6..9 %) SURVIVES unchanged -- it is the identity
%       failing, not a memory effect.

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');       opts.ax = 3;     end
    if ~isfield(opts, 'lag_scan'); opts.lag_scan = 0:6; end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    out_dir = fullfile(root, 'test_results', 'formC_cdpmr_var_check');

    ax = opts.ax;  K = S.K;  ns = numel(S.seeds);
    tt = S.t(:);
    kappa_T = 4 * (K.kBT / K.R) * K.a_o;
    sg2n_nd = K.sigma2_n_s(ax) / K.R^2;

    dwr = squeeze(S.dx_r_out(:, ax, 1:ns)) / K.R;           % normalized residual
    am  = squeeze(S.a_xm_out(:, ax, 1:ns)) / K.a_nom;       % the EWMA readout
    at  = squeeze(S.a_true_out(:, ax, 1:ns)) / K.a_nom;
    ah  = squeeze(S.a_bar_hat_out(:, ax, 1:ns));            % the EKF belief

    % u[k] -- the per-sample readout the whitened y2 actually carries
    u = (dwr.^2 - C_n_of(K) * sg2n_nd) / (K.C_dpmr * kappa_T);

    kk = 2:numel(tt);
    tt = tt(kk); u = u(kk,:); am = am(kk,:); at = at(kk,:); ah = ah(kk,:);
    m_u = mean(u, 2);  m_am = mean(am, 2);  m_at = mean(at, 2);  m_ah = mean(ah, 2);

    % measurement delay: scanned, not assumed
    fprintf('\n[lag scan] shift  rms(u/a_true - 1) over t>0.2\n');
    use = tt > 0.20;
    best = [inf 0];
    for L = opts.lag_scan
        ats = circshift(m_at, L);
        r = m_u(use) ./ ats(use) - 1;
        v = sqrt(mean(r.^2));
        fprintf('              %d      %.4f\n', L, v);
        if v < best(1); best = [v L]; end
    end
    L = best(2);
    fprintf('[lag scan] best shift = %d steps (nominal d = %d)\n', L, K.d);

    at_s = circshift(m_at, L);
    err_u  = m_u  ./ at_s - 1;
    err_am = m_am ./ at_s - 1;
    % a_hat is a state, not a reading of the delayed plant -- compare it with
    % the UNSHIFTED truth, the same convention check_formC_am_vs_atrue used.
    err_ah = m_ah ./ m_at - 1;

    SEG = {'hold start', tt > 0.05 & tt < 0.50; ...
           'descend',    tt > 0.55 & tt < 1.45; ...
           'oscillate',  tt > 1.60 & tt < 3.40; ...
           'hold end',   tt > 3.70};
    fprintf('\n%-11s %9s %10s %10s %12s %12s %12s\n', 'segment', 'a/a_o', ...
            'u err %', 'a_m err %', 'a_hat err %', 'u swing %', 'a_m swing %');
    for q = 1:size(SEG,1)
        m = SEG{q,2};
        fprintf('%-11s %9.4f %+9.2f %+10.2f %+12.2f %12.2f %12.2f\n', SEG{q,1}, ...
                mean(m_at(m)), 100*mean(err_u(m)), 100*mean(err_am(m)), ...
                100*mean(err_ah(m)), ...
                100*(max(err_u(m)) - min(err_u(m))), ...
                100*(max(err_am(m)) - min(err_am(m))));
    end

    % the trough crossings are where a lag shows up; report the peak there
    tr = tt > 1.35 & tt < 1.65;
    fprintf('\ntrough crossing 1.35-1.65 s:  peak u err %+.2f %%   peak a_m err %+.2f %%\n', ...
            100*max(err_u(tr)), 100*max(err_am(tr)));

    out = struct('t', tt, 'm_u', m_u, 'm_am', m_am, 'm_at', m_at, 'm_ah', m_ah, ...
                 'err_u', err_u, 'err_am', err_am, 'err_ah', err_ah, ...
                 'lag', L, 'ns', ns);

    if opts.save_fig
        local_fig(out, fullfile(out_dir, 'y2_instant_vs_am.png'));
        fprintf('\nfigure -> %s\n', out_dir);
    end
end

function c = C_n_of(K)
    c = K.C_n;
end

% =======================================================================
function local_fig(o, fpath)
    FS = 18; AXLW = 2.0;
    RED = [0.85 0.1 0.1];  BLU = [0 0.2 0.9];  LBL = [0.35 0.65 0.95];
    GRY = [0.5 0.5 0.5];   W = 151;
    f = figure('Position',[40 40 1250 780],'Color','w','Visible','off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1, o.t, movmean(o.m_u, W), '-', 'Color', [0.1 0.6 0.2], 'LineWidth', 2.4, ...
         'DisplayName','u  = what y_2 carries (movmean 151)');
    plot(a1, o.t, movmean(o.m_am, W), '-', 'Color', LBL, 'LineWidth', 2.4, ...
         'DisplayName','a_m  EWMA readout (movmean 151)');
    plot(a1, o.t, o.m_ah, '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','a_{hat}  EKF belief');
    plot(a1, o.t, o.m_at, '-', 'Color', RED, 'LineWidth', 2.2, 'DisplayName','a_{true}');
    ylabel(a1,'a / a_o'); ylim(a1,[0 1.05]);
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2, 0, '-', 'Color', GRY, 'LineWidth', 1.5, 'HandleVisibility','off');
    plot(a2, o.t, 100*movmean(o.err_am, W), '-', 'Color', LBL, 'LineWidth', 2.4, ...
         'DisplayName','a_m / a_{true} - 1  [%]');
    plot(a2, o.t, 100*movmean(o.err_u, W), '-', 'Color', [0.1 0.6 0.2], 'LineWidth', 2.4, ...
         'DisplayName','u / a_{true} - 1  [%]');
    plot(a2, o.t, 100*o.err_ah, '-', 'Color', BLU, 'LineWidth', 2.2, ...
         'DisplayName','a_{hat} / a_{true} - 1  [%]');
    ylim(a2,[-15 35]); xlabel(a2,'t [s]'); ylabel(a2,'[%]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    exportgraphics(f, fpath, 'Resolution', 150); close(f);
end
