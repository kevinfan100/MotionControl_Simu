% STATUS: ACTIVE (scratch instrument) | PURPOSE: rebuild the p_m -> a_m gain
%   readout chain OFFLINE from logged signals and prove it reproduces the
%   controller's a_bar_wm bit-exactly, so the chain can be reasoned about
%   without reading motion_control_law_formC_b.m each time.
%   Verified 2026-08-25 on the Meng 10 s ramp, formC_b arm best, seed 7:
%   max relative error 2.1e-16.
%
% TWO UNIT TRAPS this script exists to document (both cost a wrong answer):
%   1. dh_m_out is delta_w_m * R  [um]. It must be divided by R again.
%   2. a_hat_out(1)/a_bar_hat_out(1) is a_disp = a_o * R, NOT a_o.
%      kappa_T = 4*kB*T/R * a_o needs a_o = a_disp / R. Using a_disp inflates
%      kappa_T by R = 2.25 and the rebuilt a_m comes out 2.2x wrong while the
%      correlation with the true a_m stays 1.000000 -- i.e. the usual
%      "looks right, is wrong" failure. Correlation cannot catch a scale error.
function out = rebuild_am_chain(r, ax, k_show)
%REBUILD_AM_CHAIN  out = rebuild_am_chain(run_struct, axis, step_to_print)

    if nargin < 2 || isempty(ax);     ax = 3; end
    if nargin < 3 || isempty(k_show); k_show = 5000; end

    cc = r.ctrl_const;  P = r.meta.params_value;
    a_disp = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);   % = a_o * R
    a_o    = a_disp / r.R;                                  % trap 2
    kT     = 4 * (P.ctrl.k_B * P.ctrl.T / r.R) * a_o;       % kappa_T
    s2n    = P.ctrl.sigma2_noise(ax) / r.R^2;               % normalized sensor noise

    dw = r.dh_m_out(:, ax) / r.R;                           % trap 1
    aM = r.a_xm_out(:, ax) / a_disp;                        % controller's a_bar_wm
    N  = numel(dw);
    bar = zeros(N, 1);  s2 = zeros(N, 1);  am = nan(N, 1);
    bar(1) = dw(1);
    s2(1)  = aM(1) * cc.C_dpmr * kT + cc.C_n * s2n;         % seed from the log's row 1
    for k = 2:N
        bar(k) = (1 - cc.a_pd) * bar(k-1) + cc.a_pd * dw(k);
        s2(k)  = (1 - cc.a_cov) * s2(k-1) + cc.a_cov * (dw(k) - bar(k))^2;
        am(k)  = (s2(k) - cc.C_n * s2n) / (cc.C_dpmr * kT);
    end

    i = (2000:N).';  e = am(i) - aM(i);
    fprintf('\nrebuild vs controller a_m : max|rel| %.3e  rms %.3e  (a_m median %.3f)\n', ...
            max(abs(e ./ aM(i))), rms(e), median(aM(i)));
    fprintf('  PASS if max|rel| < 1e-12. Anything larger means a unit or an\n');
    fprintf('  ordering assumption above no longer matches the controller.\n');

    k = k_show;
    fprintf('\n--- the chain at k = %d (t = %.3f s), axis %d ---\n', k, r.tout(k), ax);
    fprintf('  1  p_d[k-d] - p_m[k]       %+.4e um    raw tracking error\n', r.dh_m_out(k, ax));
    fprintf('  2  dw_m   = 1 / R          %+.4e       dimensionless\n', dw(k));
    fprintf('  3  dw_bar = LP(a_pd) of 2  %+.4e       the DETERMINISTIC part\n', bar(k));
    fprintf('  4  dw_r   = 2 - 3          %+.4e       the JIGGLE only\n', dw(k) - bar(k));
    fprintf('  5  s2     = EWMA(a_cov) of dw_r^2      %.4e\n', s2(k));
    fprintf('  6  - C_n*sigma2_n          %.4e  (%.2f %% of 5)  sensor-noise share\n', ...
            cc.C_n * s2n, 100 * cc.C_n * s2n / s2(k));
    fprintf('  7  / (C_dpmr*kappa_T)      %.4e        = var(dw_r) if a_bar were 1\n', ...
            cc.C_dpmr * kT);
    fprintf('  => a_m %.4f   (controller %.4f)  a_true %.4f  a_hat %.4f\n', ...
            am(k), aM(k), r.a_true_out(k, ax) / a_disp, r.a_bar_hat_out(k, ax));
    fprintf('\n  constants: a_o %.6g  kappa_T %.6g  C_dpmr %.5g  C_n %.5g  a_pd %.4g  a_cov %.4g\n', ...
            a_o, kT, cc.C_dpmr, cc.C_n, cc.a_pd, cc.a_cov);

    out = struct('t', r.tout, 'dw_m', dw, 'dw_bar', bar, 'dw_r', dw - bar, ...
                 's2', s2, 'am', am, 'am_ctrl', aM, 'a_o', a_o, 'kappa_T', kT, ...
                 'C_dpmr', cc.C_dpmr, 'C_n', cc.C_n, 'sigma2_n_nd', s2n, ...
                 'max_rel_err', max(abs(e ./ aM(i))));
end
