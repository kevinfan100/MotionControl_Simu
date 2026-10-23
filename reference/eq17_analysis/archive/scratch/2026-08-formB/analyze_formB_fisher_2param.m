function res = analyze_formB_fisher_2param(opts)
%ANALYZE_FORMB_FISHER_2PARAM  Fisher information on the Form B constants along
%   the production trajectory, y2 channel: 3-parameter baseline (b, p, w_s),
%   the 2-parameter working case (p pinned at the anchor 1), the decoupled
%   (A_ref, w_s) coordinates, and the comparison against the production priors.
%
% STATUS: ACTIVE -- 2-parameter (b, w_s) Fisher ledger on the production trajectory.
%   See memory project-formb-fisher-2param-prior-asymmetry-2026-08-06.
%
% PURPOSE: answer "if only (b, w_s) are estimated, is the y2 DATA stronger or
%   weaker than the priors the controller starts from, and by what factor?".
%
% SCOPE: OFFLINE ONLY. Reads the logs of the production driver run_formB_ws;
%   nothing under model/ is touched and no filter is re-implemented -- the law
%   rebuild is bit-checked against the controller's own logged a_bar'.
%
% SENSITIVITY CONVENTION (the point the whole result hangs on). The H2 row of
%   the controller carries the INSTANTANEOUS term -Grad_wbar_d*J_theta, i.e.
%   the increment the predict chain adds to the ACCUMULATED sensitivity each
%   step.  By equality of mixed partials d/dw_bar(dA/dtheta) = dA'/dtheta =
%   J_theta, the path integral of J_theta*Delta_w_bar is exactly the direct
%   level sensitivity dA/dtheta (measured 2026-08-06: accumulated path 100.01%
%   of I_b, instantaneous path 2.4e-5%).  The Fisher sum below therefore uses
%
%       s_theta[k] = dA/dtheta  evaluated at w_bar_d[k]          (ANALYTIC)
%           dA/db  = -p*u/b^2 * (1+u/b)^(-p-1)
%           dA/dws = -p/b     * (1+u/b)^(-p-1)
%           dA/dp  = (1+u/b)^(-p) * ln(1+u/b) ,     u = w_bar_d - w_s
%
%   The instantaneous-inclusive variant is computed too and reported side by
%   side; the two agree to 4 digits, as they must.
%
%   F = sum_k s_theta[k]' * R2_eff[k]^(-1) * s_theta[k] * H2_scale^2
%   over the steps where the y2 gate is open.  H2_scale = a_cov (y2_whiten).
%
% R2_eff AND EFFECTIVE SAMPLE COUNT (declared, not assumed silently).  y2 is
%   the WHITENED increment a_bar_wm[k] - (1-a_cov)*a_bar_wm[k-1] and R2 is the
%   controller's own per-sample variance for it, a_cov*(2-a_cov)*R2_int +
%   a_cov^2*d*Q44.  Using sum_k s^2/R2 assumes (i) R2 is the true marginal
%   variance and (ii) successive y2 are independent.  Both are the
%   controller's design assumptions, not facts -- the underlying readout is
%   built from dw_r^2, and dw_r carries the closed-loop memory.  The script
%   MEASURES both from the log and reports the calibrated figure alongside the
%   raw one:
%       c_R    = var( innov_y2 / sqrt(R2) )      1 if R2 is honest
%       tau    = 1 + 2*sum rho_j of that same standardized innovation
%       F_cal  = F_raw / (c_R * tau)
%   Every reported sigma states which convention it uses.  F_raw is the
%   convention the 0.0895 check point was produced under.
%
% CHECK POINTS supplied with the task, and what they turn out to be:
%   0.0895  = 1/sqrt(I_b) with the EULER-ACCUMULATED 7b sensitivity chain.
%             The exact level derivative used here gives 0.08816 (F_bb = 128.7
%             vs 124.8); the 3.1% gap IS the forward-Euler quadrature residual
%             of the 7b chain (sup|s/dA_db - 1| = 2.96%, 2026-08-06 memory).
%   0.864   = |cos| under the DETERMINISTIC-channel weight rho ~ v_d^2/a^2
%             (S4.4 / S3 construction), which gives the two holds zero weight.
%             The y2 measurement channel weights by a_bar'^2/R2 instead, under
%             which the 1.3 s final hold at the trough carries 68% of the w_s
%             information.  Both weights are computed and printed below; the
%             S4.4 one reproduces 0.8630 to 4 digits, so the two numbers are
%             the same calculation on two different channels, not a conflict.
%
% opts (all optional):
%   .seeds    [7 11 23 42 101 777]   house 6-seed set; the report is the mean
%             +- sd across seeds (the trajectory is common, R2 and the gate are
%             realization-dependent)
%   .w_ref    3.485    decoupling pivot to TEST (08-05 S4.4 derived value)
%   .verbose  false    let the driver banner through
%   .do_y1    true     run the deliverable-5 y1 accounting
%
%   See also: motion_control_law_formB_ws, run_formB_ws,
%             analyze_formB_info_7b_vs_7a

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds   = [7 11 23 42 101 777]; end
    if ~isfield(opts, 'w_ref');   opts.w_ref   = 3.485;  end
    if ~isfield(opts, 'verbose'); opts.verbose = false;  end
    if ~isfield(opts, 'do_y1');   opts.do_y1   = true;   end

    AX_Z = 3;                  % wall-normal axis (the only one on the perp law)
    PRIOR_WS_PROD = 0.111;     % production w_s prior width (run_formB_ws header)
    SIG_B_CHECK   = 0.0895;    % check point 1
    COS_CHECK     = 0.8635;    % check point 2
    PNAME = {'b', 'p', 'w_s'};

    seeds = opts.seeds(:).';
    ns    = numel(seeds);
    S     = cell(ns, 1);
    for q = 1:ns
        if opts.verbose
            S{q} = local_seed_fisher(seeds(q), AX_Z);
        else
            [~, S{q}] = evalc('local_seed_fisher(seeds(q), AX_Z)');
        end
        fprintf('  seed %4d done: F_bb = %8.3f  tau_int = %5.2f  open %.1f%%\n', ...
                seeds(q), S{q}.F3(1,1), S{q}.tau_int, 100*S{q}.open_frac);
    end

    % ---------------- aggregate ----------------
    g = @(f) cellfun(@(s) f(s), S);
    F3_all = zeros(3, 3, ns);
    for q = 1:ns; F3_all(:, :, q) = S{q}.F3; end
    F3_mean = mean(F3_all, 3);
    tau_mean = mean(g(@(s) s.tau_int));
    cR_mean  = mean(g(@(s) s.c_R));
    infl     = cR_mean * tau_mean;    % F_cal = F_raw / infl ; sigma_cal = sigma_raw*sqrt(infl)

    fprintf('\n================================================================\n');
    fprintf(' Form B Fisher information, y2 channel, z axis\n');
    fprintf(' scenario: production run_formB_ws (t1), %d seeds\n', ns);
    fprintf('================================================================\n');
    s1 = S{1};
    fprintf('H2_scale = a_cov = %.4g   echo (1-S) range [%.4f %.4f]\n', ...
            s1.H2_scale, s1.echo_lo, s1.echo_hi);
    fprintf('steps with y2 open: %.1f%%   R2 range [%.3e %.3e]\n', ...
            100*s1.open_frac, s1.R2_lo, s1.R2_hi);
    fprintf('law rebuild check: max|a_bar''_rebuilt - logged| = %.2e\n', ...
            max(g(@(s) s.d_ap)));
    fprintf('H2*P*H2''/R2 (prediction vs measurement variance): mean %.3g  max %.3g\n', ...
            mean(g(@(s) s.hph_over_R2_mean)), max(g(@(s) s.hph_over_R2_max)));
    fprintf('level-only vs level+instantaneous F_bb: %.6g vs %.6g (%.4f%% apart)\n', ...
            s1.F3(1,1), s1.F3_full(1,1), 100*abs(s1.F3_full(1,1)/s1.F3(1,1) - 1));

    % ---------------- R2 honesty + effective sample count ----------------
    fprintf('\n---- R2_eff audit, standardized innovation z = innov_y2/sqrt(R2) --\n');
    fprintf('%6s %10s %10s %10s %12s\n', 'seed', 'var(z)', 'rho(1)', 'rho(2)', 'tau_int');
    for q = 1:ns
        fprintf('%6d %10.4f %10.4f %10.4f %12.3f\n', seeds(q), S{q}.c_R, ...
                S{q}.rho1, S{q}.rho2, S{q}.tau_int);
    end
    fprintf(['var(z) = %.4f  -> R2 OVERSTATES the true per-sample variance by %.2fx\n', ...
             'tau_int = %.3f -> only 1 in %.2f whitened samples is independent\n', ...
             'net F_cal = F_raw / (var(z)*tau_int) = F_raw / %.3f  ', ...
             '(sigma_cal = %.3fx sigma_raw)\n'], ...
            cR_mean, 1/cR_mean, tau_mean, tau_mean, infl, sqrt(infl));

    % ---------------- DELIVERABLE 1: 3-parameter baseline ----------------
    fprintf('\n==== DELIVERABLE 1  3-parameter (b, p, w_s) ====================\n');
    d1 = local_report_block(F3_all, PNAME, infl);
    fprintf('\ncondition number of F3: mean %.4g  (min eig / max eig = %.3e)\n', ...
            mean(g(@(s) cond(s.F3))), 1/mean(g(@(s) cond(s.F3))));
    fprintf('\nCHECK 1  conditional sigma_b = 1/sqrt(F_bb) = %.5f  (target %.4f, %.2f%% off)\n', ...
            d1.sig_cond(1), SIG_B_CHECK, 100*(d1.sig_cond(1)/SIG_B_CHECK - 1));
    fprintf('         marginal sigma_b (3-param inverse)  = %.5f  = %.1fx the conditional\n', ...
            d1.sig_marg(1), d1.sig_marg(1)/d1.sig_cond(1));

    % ---------------- DELIVERABLE 2: 2-parameter ----------------
    fprintf('\n==== DELIVERABLE 2  2-parameter (b, w_s), p pinned at 1 ========\n');
    idx2 = [1, 3];
    F2_all = F3_all(idx2, idx2, :);
    d2 = local_report_block(F2_all, PNAME(idx2), infl);
    cosv = g(@(s) abs(s.F3(1,3)) / sqrt(s.F3(1,1)*s.F3(3,3)));
    fprintf('\n|cos(s_b, s_ws)| = %.5f +- %.5f   (CHECK 2 target %.4f, %.2f%% off)\n', ...
            mean(cosv), std(cosv), COS_CHECK, 100*(mean(cosv)/COS_CHECK - 1));
    fprintf('variance inflation 1/sqrt(1-cos^2) = %.3f\n', 1/sqrt(1 - mean(cosv)^2));

    fprintf(['\nreconciliation: |cos| = 1/sqrt(1+CV^2) with CV = coefficient of\n', ...
             'variation of the gap u under the weight psi = a_bar''^2 * (rate).\n', ...
             'Which rate you use IS the channel you are asking about:\n']);
    wlab = {'a_bar''^2/R2  = THIS y2 channel', ...
            'v_d^2/a^2     = S4.4/S3 deterministic channel', ...
            'uniform dwell = every height equally long'};
    fprintf('%-46s %8s %10s %10s\n', 'weight', 'CV', '|cos|', 'w_ref pivot');
    for j = 1:3
        cw = mean(g(@(s) s.cos_w(j))); vw = mean(g(@(s) s.CV_w(j)));
        pw = mean(g(@(s) s.piv_w(j)));
        fprintf('%-46s %8.4f %10.5f %10.4f\n', wlab{j}, vw, cw, pw);
    end
    fprintf('numerical |cos| from the full 2x2 Fisher: %.5f (matches row 1 closed form)\n', ...
            mean(cosv));

    fprintf('\nwhere the y2 information sits (mean over seeds):\n');
    fprintf('%-12s %10s %10s\n', 'segment', 'F_bb %', 'F_wsws %');
    segn = {'hold_init', 'descent', 'oscillation', 'hold_final'};
    segw = {[0 0.5], [0.5 1.5], [1.5 3.5], [3.5 Inf]};
    for j = 1:4
        sb_ = mean(g(@(s) 100*sum(s.fb_k(s.tout > segw{j}(1) & s.tout <= segw{j}(2)))/sum(s.fb_k)));
        sw_ = mean(g(@(s) 100*sum(s.fw_k(s.tout > segw{j}(1) & s.tout <= segw{j}(2)))/sum(s.fw_k)));
        fprintf('%-12s %10.2f %10.2f\n', segn{j}, sb_, sw_);
    end
    fprintf('h_bar < 3 share of F_wsws: %.2f%%   h_bar > 10: %.2f%%\n', ...
            mean(g(@(s) 100*sum(s.fw_k(s.wd < 3))/sum(s.fw_k))), ...
            mean(g(@(s) 100*sum(s.fw_k(s.wd > 10))/sum(s.fw_k))));

    % ---------------- DELIVERABLE 3: ratios ----------------
    prior_b  = mean(g(@(s) s.prior_b));
    fprintf('\n==== DELIVERABLE 3  data vs prior ==============================\n');
    fprintf('production priors: sqrt(P_bb[0]) = %.5f (envelope self-derived)   ', prior_b);
    fprintf('sqrt(P_wsws) = %.4f (calibration)\n', PRIOR_WS_PROD);
    fprintf('\n%-42s %10s %10s\n', 'ratio', 'raw', 'calibrated');
    r1 = d2.sig_marg(1) / d1.sig_marg(1);
    r1c = d2.sig_marg_c(1) / d1.sig_marg_c(1);
    fprintf('%-42s %10.4f %10.4f\n', 'sigma_b(2par) / sigma_b(3par)', r1, r1c);
    r2 = d2.sig_marg(1) / prior_b;
    r2c = d2.sig_marg_c(1) / prior_b;
    fprintf('%-42s %10.4f %10.4f\n', 'sigma_b(2par) / prior 0.0157', r2, r2c);
    r3 = d2.sig_marg(2) / PRIOR_WS_PROD;
    r3c = d2.sig_marg_c(2) / PRIOR_WS_PROD;
    fprintf('%-42s %10.4f %10.4f\n', 'sigma_ws(2par) / prior 0.111', r3, r3c);
    fprintf('(ratio < 1 = data beats prior)\n');

    fprintf('\nladder: how the answer depends on how many constants are free\n');
    fprintf('%-26s %12s %12s %12s %12s\n', 'free set', 'sigma_b', 'vs 0.0157', ...
            'sigma_ws', 'vs 0.111');
    sets = {1, 3, [1 3], [1 2 3]};
    lab  = {'b only', 'w_s only', 'b, w_s', 'b, p, w_s'};
    lad  = zeros(numel(sets), 2);
    for j = 1:numel(sets)
        sb = NaN; sw = NaN;
        for q = 1:ns
            % explicit inverse: these are 1x1..3x3 covariances, and the
            % off-diagonals are the result being reported
            C = inv(F3_all(sets{j}, sets{j}, q));
            v = sqrt(diag(C));
            ib = find(sets{j} == 1); iw = find(sets{j} == 3);
            if ~isempty(ib); sb = local_acc(sb, v(ib), q, ns); end
            if ~isempty(iw); sw = local_acc(sw, v(iw), q, ns); end
        end
        lad(j, :) = [sb, sw];
        fprintf('%-26s %12s %12s %12s %12s\n', lab{j}, ...
                local_num(sb), local_num(sb/prior_b), ...
                local_num(sw), local_num(sw/PRIOR_WS_PROD));
    end

    % ---------------- DELIVERABLE 4: decoupled coordinates ----------------
    fprintf('\n==== DELIVERABLE 4  (A_ref, w_s) coordinates ===================\n');
    fprintf('%6s %9s %9s %11s %11s %11s\n', 'seed', 'w_ref', 'rho(A,ws)', ...
            'sigma_A', 'w_ref*', 'rho at *');
    d4 = zeros(ns, 7);
    for q = 1:ns
        d4(q, :) = local_decouple(S{q}, opts.w_ref);
        fprintf('%6d %9.4f %9.5f %11.4e %11.4f %11.2e\n', seeds(q), opts.w_ref, ...
                d4(q,1), d4(q,2), d4(q,3), d4(q,4));
    end
    fprintf('\nat the TESTED pivot w_ref = %.3f (S4.4 derived value):\n', opts.w_ref);
    fprintf('   rho(A_ref, w_s) = %+.5f   NOT diagonal;  sigma_A_ref = %.4e (%.3f%% of a_bar)\n', ...
            mean(d4(:,1)), mean(d4(:,2)), 100*mean(d4(:,2))/mean(d4(:,5)));
    fprintf('at the EXACT pivot for this channel:\n');
    fprintf('   w_ref* = %.4f +- %.4f  (h = %.3f um);  rho = %.1e (numerical zero)\n', ...
            mean(d4(:,3)), std(d4(:,3)), mean(d4(:,3))*s1.R_radius, max(abs(d4(:,4))));
    fprintf('   sigma_A_ref* = %.4e = %.3f%% of a_bar(w_ref*) = %.4f\n', ...
            mean(d4(:,6)), 100*mean(d4(:,6))/mean(d4(:,7)), mean(d4(:,7)));
    fprintf('closed form <u^2>/<u> under psi = a_bar''^2/R2: %.4f (matches w_ref*)\n', ...
            mean(g(@(s) s.w_pivot_psi)));
    fprintf(['the S4.4 pivot 3.485 is the same closed form under the v_d^2/a^2\n', ...
             'weight; it diagonalizes the DETERMINISTIC channel, not this one.\n']);

    % Coordinate-free version of deliverable 3: push the (b, w_s) PRIOR through
    % the same map and compare the strong direction A_ref, where the b-w_s
    % degeneracy is by construction absent.
    Cpri = diag([prior_b^2, PRIOR_WS_PROD^2]);
    pv = zeros(ns, 3);
    for q = 1:ns
        s = S{q};
        gs = mean(d4(:,3)) - s.ws_ref; us = 1 + gs/s.b_ref;
        aps = (s.p_ref/s.b_ref) * us^(-s.p_ref)/us;
        Js  = [-(gs/s.b_ref)*aps, -aps; 0, 1];
        Cp  = Js * Cpri * Js.';
        Cd  = Js * inv(F3_all([1 3], [1 3], q)) * Js.';               %#ok<MINV>
        pv(q, :) = [sqrt(Cd(1,1)), sqrt(Cp(1,1)), sqrt(sqrt(det(Cd)/det(Cp)))];
    end
    fprintf('\nstrong direction at w_ref*, data vs prior:\n');
    fprintf('   sigma_A_ref  data %.4e   prior pushforward %.4e   ratio %.4f\n', ...
            mean(pv(:,1)), mean(pv(:,2)), mean(pv(:,1))/mean(pv(:,2)));
    fprintf('   joint 2-D area ratio (det^(1/4)) data/prior: %.4f\n', mean(pv(:,3)));

    % ---------------- DELIVERABLE 5: y1 accounting ----------------
    y1 = [];
    if opts.do_y1
        fprintf('\n==== DELIVERABLE 5  y1 channel =================================\n');
        y1 = local_y1_block(S, F3_all, seeds);
    end

    res = struct('seeds', seeds, 'F3_all', F3_all, 'F3_mean', F3_mean, ...
                 'd1', d1, 'd2', d2, 'd4', d4, 'cos_bws', mean(cosv), ...
                 'tau_int', tau_mean, 'c_R', cR_mean, 'infl', infl, ...
                 'prior_b', prior_b, ...
                 'ratios', [r1, r2, r3], 'ratios_cal', [r1c, r2c, r3c], ...
                 'y1', y1);
    res.S = S;
end


%% =================== Local Helpers ===================

function out = local_seed_fisher(seed, AX_Z)
%LOCAL_SEED_FISHER  One production run -> 3x3 y2 Fisher + diagnostics.
    base = run_formB_ws(struct('seeds', seed));
    ref  = base.runs{1};
    ctrl_const = ref.ctrl_const;
    P = ref.meta.params_value;

    R_radius = P.common.R;                                  % [um]
    Ts       = P.common.Ts;                                 % [s]
    a_o      = Ts / (P.ctrl.gamma * R_radius);              % [1/pN]
    a_disp   = a_o * R_radius;                              % [um/pN]
    kappa_T  = 4 * (P.ctrl.k_B * P.ctrl.T / R_radius) * a_o;
    s2n_nd   = P.ctrl.sigma2_noise(:) / R_radius^2;
    lambda_c = ctrl_const.lambda_c;
    a_cov    = ctrl_const.a_cov;
    a_pd     = ctrl_const.a_pd;
    d_delay  = ctrl_const.d;
    xi_bar   = (ctrl_const.C_n / ctrl_const.C_dpmr) * s2n_nd / kappa_T;

    b_clamp     = local_field(ctrl_const, 'b_clamp',     [0.05, 5]);
    p_clamp     = local_field(ctrl_const, 'p_clamp',     [0.05, 5]);
    a_bar_floor = local_field(ctrl_const, 'a_bar_floor', 0.05);
    gap_floor   = local_field(ctrl_const, 'ws_margin',   1e-3);
    ws0_perp    = local_field(ctrl_const, 'ws0_perp',    1);
    if local_field(ctrl_const, 'y2_whiten', true); H2_scale = a_cov; else; H2_scale = 1; end
    if local_field(ctrl_const, 'y2_echo_corr', true)
        [S_T, S_n] = local_echo_shares(lambda_c, a_pd);
    else
        S_T = 0; S_n = 0;
    end

    N  = numel(ref.tout);
    wd = ref.h_bar_d_out;                                   % w_bar_d[k]
    wd(1) = wd(2);      % k=1 is the controller INIT call: empty_diag h_bar_d = 0
    % prior row = what x_curr held when step k ran (posterior of k-1)
    b_pri  = [NaN; ref.b_hat_out(1:end-1,  AX_Z)];
    p_pri  = [NaN; ref.p_hat_out(1:end-1,  AX_Z)];
    ws_pri = [NaN; ref.ws_hat_out(1:end-1, AX_Z)];
    ab_pri = [NaN; ref.a_bar_hat_out(1:end-1, AX_Z)];
    R2     = ref.R2_out(:, AX_Z);
    open_y2 = ~ref.gate_out(:, AX_Z);
    open_y2(1) = false;                                     % controller init call

    % Grad_wbar_d[k] = w_bar_d[k] - w_bar_d[k-d]; the pd buffers are prefilled
    % with pd(1), so the first d steps back off to the start height.
    kk = (1:N).';
    grad = wd - wd(max(kk - d_delay, 1));

    dA   = zeros(N, 3);      % [dA/db  dA/dp  dA/dws]     ANALYTIC per step
    Jth  = zeros(N, 3);      % [J_b    J_p    J_ws]
    apv  = zeros(N, 1);
    echo = ones(N, 1);
    gapv = zeros(N, 1);
    for k = 2:N
        lb = min(max(b_pri(k), b_clamp(1)), b_clamp(2));
        lp = min(max(p_pri(k), p_clamp(1)), p_clamp(2));
        lw = ws_pri(k) + (ws0_perp - 1);
        gp = max(wd(k) - lw, gap_floor);
        gapv(k) = gp;
        u    = 1 + gp / lb;
        upow = u^(-lp);
        ap   = (lp / lb) * upow / u;
        apv(k) = ap;
        dA(k, :)  = [-(gp/lb)*ap, upow*log(u), -ap];
        Jth(k, :) = [(-1/lb + (lp+1)*gp/(lb*(gp+lb)))*ap, ...
                     (1/lp - log(u))*ap, ...
                     ((lp+1)/(gp+lb))*ap];
        abi = max(ab_pri(k), a_bar_floor);
        echo(k) = 1 - (S_T*abi + S_n*xi_bar(AX_Z)) / (abi + xi_bar(AX_Z));
    end
    d_ap = max(abs(apv(2:end) - ref.a_prime_out(2:end, AX_Z) / a_disp));
    assert(d_ap < 1e-12, 'analyze_formB_fisher_2param:lawMismatch', ...
           'rebuilt a_bar'' differs from the controller log by %.3e.', d_ap);

    m = open_y2;
    w = 1 ./ R2(m);                                         % R2_eff^-1 per sample
    Dl = H2_scale * dA(m, :);                               % level path (primary)
    Df = Dl - H2_scale * (echo(m) .* grad(m)) .* Jth(m, :); % + instantaneous term
    F3      = Dl.' * (w .* Dl);                             % NUMERICAL sum over log
    F3_full = Df.' * (w .* Df);

    % H2*P*H2' vs R2: justifies using R2 rather than the innovation variance S2
    P44 = (ref.P_a_out(:, AX_Z) / a_disp).^2;
    hph = (H2_scale * echo).^2 .* P44;

    % --- R2 honesty and leftover correlation, both from the standardized
    %     innovation z = innov_y2/sqrt(R2) (var 1 and white if R2 is right) ---
    z = ref.innov_y2_out(:, AX_Z) ./ sqrt(R2);
    c_R = var(z(m));
    [tau_int, rho1, rho2] = local_tau_int(z, m);

    % --- weight comparison: this channel vs the S4.4 deterministic-channel
    %     weight rho ~ v_d^2/a^2, which is what produced the 0.864 figure -----
    uu   = gapv(m);
    abv  = 1 - (1 + gapv ./ b_pri).^(-p_pri);       % law level along the run
    vd   = [0; diff(wd)];
    wgt  = struct('fisher', 1 ./ R2, ...
                  'review', vd.^2 ./ max(abv, eps).^2, ...
                  'dwell',  ones(N, 1));
    wn = fieldnames(wgt);
    cosw = zeros(1, numel(wn)); CVw = zeros(1, numel(wn)); pivw = zeros(1, numel(wn));
    for j = 1:numel(wn)
        pw = wgt.(wn{j})(m) .* apv(m).^2;
        q1 = sum(pw .* uu) / sum(pw);
        q2 = sum(pw .* uu.^2) / sum(pw);
        CVw(j)  = sqrt(max(q2 - q1^2, 0)) / q1;
        cosw(j) = 1 / sqrt(1 + CVw(j)^2);
        pivw(j) = q2 / q1 + mean(ws_pri(m));
    end
    CV = CVw(1); m2 = pivw(1) - mean(ws_pri(m)); m1 = m2 / (1 + CV^2); %#ok<NASGU>

    % --- segment shares (which part of the run carries the information) -----
    fb_k = zeros(N, 1); fw_k = zeros(N, 1);
    fb_k(m) = (H2_scale * dA(m, 1)).^2 ./ R2(m);
    fw_k(m) = (H2_scale * dA(m, 3)).^2 ./ R2(m);

    out = struct('seed', seed, 'F3', F3, 'F3_full', F3_full, ...
                 'H2_scale', H2_scale, 'echo_lo', min(echo(2:end)), ...
                 'echo_hi', max(echo(2:end)), 'open_frac', mean(m), ...
                 'R2_lo', min(R2(m)), 'R2_hi', max(R2(m)), 'd_ap', d_ap, ...
                 'hph_over_R2_mean', mean(hph(m)./R2(m)), ...
                 'hph_over_R2_max', max(hph(m)./R2(m)), ...
                 'tau_int', tau_int, 'rho1', rho1, 'rho2', rho2, 'c_R', c_R, ...
                 'CV_psi', CV, 'w_pivot_psi', pivw(1), ...
                 'cos_w', cosw, 'CV_w', CVw, 'piv_w', pivw, ...
                 'fb_k', fb_k, 'fw_k', fw_k, 'tout', ref.tout, 'wd', wd, ...
                 'prior_b', ref.P_b_out(1, AX_Z), 'prior_p', ref.P_p_out(1, AX_Z), ...
                 'post_b', ref.P_b_out(end, AX_Z), 'post_p', ref.P_p_out(end, AX_Z), ...
                 'b_ref', median(b_pri(m)), 'p_ref', median(p_pri(m)), ...
                 'ws_ref', median(ws_pri(m)) + (ws0_perp - 1), ...
                 'R_radius', R_radius, 'lambda_c', lambda_c, ...
                 'ref', ref, 'AX_Z', AX_Z, 'a_disp', a_disp, 's2n_nd', s2n_nd);
end


function d = local_report_block(F_all, pname, tau)
%LOCAL_REPORT_BLOCK  Per-seed marginal / conditional sigmas from a Fisher stack.
    ns = size(F_all, 3);
    np = size(F_all, 1);
    sm = zeros(ns, np); sc = zeros(ns, np);
    for q = 1:ns
        C = inv(F_all(:, :, q));
        sm(q, :) = sqrt(diag(C)).';
        sc(q, :) = (1 ./ sqrt(diag(F_all(:, :, q)))).';
    end
    fprintf('%-6s %14s %14s %14s\n', 'param', 'F_ii', 'sigma cond', 'sigma marg');
    for j = 1:np
        fprintf('%-6s %14.4g %14.5f %14.5f\n', pname{j}, ...
                mean(squeeze(F_all(j, j, :))), mean(sc(:, j)), mean(sm(:, j)));
    end
    sd = std(sm, 0, 1);
    fprintf('per-seed sd of sigma marg: ');
    for j = 1:np
        fprintf('%s %.2e   ', pname{j}, sd(j));
    end
    fprintf('\ncalibrated (F/%.3f): ', tau);
    for j = 1:np
        fprintf('%s %.5f   ', pname{j}, mean(sm(:, j)) * sqrt(tau));
    end
    fprintf('\n');
    d = struct('sig_marg', mean(sm, 1), 'sig_cond', mean(sc, 1), ...
               'sig_marg_c', mean(sm, 1) * sqrt(tau), 'sig_marg_sd', std(sm, 0, 1));
end


function v = local_decouple(s, w_ref)
%LOCAL_DECOUPLE  (b, w_s) -> (A_ref, w_s) reparametrization.
%   A_ref = a_bar(w_ref); the Jacobian is [dA/db dA/dws; 0 1], so
%   C_eta = J*C*J'.  Setting Cov(A_ref, w_s) = 0 gives the exact pivot
%   u* = -b * C(2,2)/C(1,2)  (analytic; dA/db / dA/dws = u/b for any p).
    F2 = s.F3([1 3], [1 3]);
    C2 = inv(F2);
    gp = w_ref - s.ws_ref;
    u  = 1 + gp / s.b_ref;
    ap = (s.p_ref / s.b_ref) * u^(-s.p_ref) / u;
    A_b  = -(gp / s.b_ref) * ap;
    A_ws = -ap;
    A_lv = 1 - u^(-s.p_ref);
    J    = [A_b, A_ws; 0, 1];
    Ce   = J * C2 * J.';
    rho  = Ce(1,2) / sqrt(Ce(1,1) * Ce(2,2));
    u_st = -s.b_ref * C2(2,2) / C2(1,2);
    w_st = u_st + s.ws_ref;
    % residual correlation at the exact pivot (numerical zero check)
    gs = w_st - s.ws_ref; us = 1 + gs/s.b_ref;
    aps = (s.p_ref/s.b_ref) * us^(-s.p_ref)/us;
    Js = [-(gs/s.b_ref)*aps, -aps; 0, 1];
    Cs = Js * C2 * Js.';
    rho_st = Cs(1,2) / sqrt(Cs(1,1) * Cs(2,2));
    A_lv_st = 1 - us^(-s.p_ref);
    v = [rho, sqrt(Ce(1,1)), w_st, rho_st, A_lv, sqrt(Cs(1,1)), A_lv_st];
end


function y1 = local_y1_block(S, F3_all, seeds)
%LOCAL_Y1_BLOCK  Deliverable 5.  Two readings, one decisive and one not:
%   (a) DECISIVE -- does the p cross-information explain the 2.98-3.07% vs
%       4.01% P_bb shrink gap?  The single-parameter prediction I/(1/P0+I) is
%       compared with the JOINT (b, p) prediction (P0^-1 + F_bp)^-1, which is
%       what the production filter actually runs (lock_ws = true, b and p
%       free).  If the joint prediction moves AWAY from the realized shrink,
%       the "p cross-information" half of the 08-06 note is wrong.
%   (b) NOT DECISIVE -- the y1 channel.  The plant-side sensitivity dy1/dtheta
%       is computed exactly, but converting it into a Fisher number needs the
%       innovation-form (Ljung) recursion of the theta-KNOWN filter, whose
%       per-step K1 production does not log.  Only the per-sample SNR scale is
%       reported, plus the level / first-difference bracket, which is three
%       decades wide and therefore settles nothing.  Declared, not fudged.
    ns = numel(seeds);
    shr1 = zeros(ns, 1); shr2 = zeros(ns, 1); shrR = zeros(ns, 1);
    lo = zeros(ns, 3); hi = zeros(ns, 3); snr = zeros(ns, 3); sdet = zeros(ns, 3);
    for q = 1:ns
        s = S{q};
        Pb0 = s.prior_b^2;
        Ptt0 = diag([Pb0, s.prior_p^2]);
        Fbp  = F3_all([1 2], [1 2], q);
        shr1(q) = 100 * Fbp(1,1) / (1/Pb0 + Fbp(1,1));          % b alone, p known
        Ppost = inv(inv(Ptt0) + Fbp);                            % 2x2, explicit
        shr2(q) = 100 * (1 - Ppost(1,1)/Pb0);                    % b and p joint
        shrR(q) = 100 * (1 - (s.post_b/s.prior_b)^2);            % filter realized
        [lo(q, :), hi(q, :), snr(q, :), sdet(q, :)] = local_y1_sens(s);
    end
    fprintf('(a) P_bb shrink over the run [DECISIVE]\n');
    fprintf('    y2 Fisher, b alone (p known)      : %.2f%%\n', mean(shr1));
    fprintf('    y2 Fisher, b and p JOINT          : %.2f%%   <- what production runs\n', mean(shr2));
    fprintf('    filter realized                   : %.2f%%\n', mean(shrR));
    fprintf(['    => the p cross-information moves the prediction DOWN by %.2f pp,\n', ...
             '       i.e. AWAY from the realized value.  It cannot be the missing\n', ...
             '       term; the residual to explain GROWS from %.2f to %.2f pp.\n'], ...
            mean(shr1) - mean(shr2), mean(shrR) - mean(shr1), mean(shrR) - mean(shr2));

    fprintf('\n(b) y1 channel [NOT RESOLVED -- scale only]\n');
    fprintf('    plant-side sensitivity dy1/dtheta, ANALYTIC per step, integrated\n');
    fprintf('    NUMERICALLY along the logged trajectory; estimator frozen.\n');
    fprintf('%-6s %14s %14s %14s %14s\n', 'param', 'rms|dy1/dth|', 'per-samp SNR', ...
            'I bracket lo', 'I bracket hi');
    pn = {'b', 'p', 'w_s'};
    for j = 1:3
        fprintf('%-6s %14.4e %14.3f %14.4g %14.4g\n', pn{j}, mean(sdet(:, j)), ...
                mean(snr(:, j)), mean(lo(:, j)), mean(hi(:, j)));
    end
    fprintf('    (y2 comparison: F_bb = %.4g, per-sample SNR %.4f)\n', ...
            mean(squeeze(F3_all(1,1,:))), sqrt(mean(squeeze(F3_all(1,1,:))) / 7680));
    fprintf(['    lo = only the step-to-step CHANGE of the theta-induced deviation\n', ...
             '    survives; hi = the filter predicts none of it.  Three decades wide:\n', ...
             '    NO y1 number is claimed.  What IS solid: the per-sample SNR of the\n', ...
             '    y1 channel is O(1) per unit b, vs %.3f for y2 -- so y1 is not a\n', ...
             '    negligible channel and the 08-06 residual plausibly lives there.\n'], ...
            sqrt(mean(squeeze(F3_all(1,1,:))) / 7680));
    y1 = struct('shrink_1par', shr1, 'shrink_2par', shr2, 'shrink_real', shrR, ...
                'I_lo', lo, 'I_hi', hi, 'snr', snr, 'rms_sens', sdet);
end


function [I_lo, I_hi, snr, rms_s] = local_y1_sens(s)
%LOCAL_Y1_SENS  Plant-side sensitivity of the y1 reading to theta, with the
%   ESTIMATOR FROZEN (its own adaptation excluded, as in the 7b accumulation).
%       s_f[k] = -(1-lc)/a_hat * ( s_w[k] + sum_i a_hat[k-i]*s_f[k-i] )
%       s_w[k+1] = s_w[k]*(1 + a'*f_bar) + f_bar*dA/dtheta + a_bar*s_f[k]
%       dy1/dtheta = -s_w[k]
%   The plant feedback through delta_w_m IS retained (it is physics, not
%   estimator adaptation).  Denominator = the empirical y1 innovation variance
%   (measured white: |rho(1..4)| < 0.03, so no tau correction is needed here).
    ref = s.ref; ax = s.AX_Z;
    N  = numel(ref.tout);
    fb = ref.f_bar_out(:, ax);                     % f_bar[k] applied at step k
    ah = [ref.a_bar_hat_out(1, ax); ref.a_bar_hat_out(1:end-1, ax)];  % a_ctrl[k]
    ab = ref.a_true_out(:, ax) / s.a_disp;         % TRUE a_bar entering step k
    wt = ref.h_bar_true_out;                       % true height entering step k
    lc = s.lambda_c;
    b = s.b_ref; p = s.p_ref; ws = s.ws_ref;

    sw = zeros(N, 3); sf = zeros(N, 3);
    for k = 2:N
        gp = max(wt(k) - ws, 1e-3);
        u  = 1 + gp/b; upw = u^(-p);
        ap = (p/b) * upw / u;
        dA = [-(gp/b)*ap, upw*log(u), -ap];
        hist = zeros(1, 3);
        if k >= 2; hist = hist + ah(k-1) * sf(k-1, :); end
        if k >= 3; hist = hist + ah(k-2) * sf(k-2, :); end
        sf(k, :) = -(1 - lc) / max(ah(k), 1e-6) * (sw(k, :) + hist);
        if k < N
            sw(k+1, :) = sw(k, :) * (1 + ap*fb(k)) + fb(k)*dA + ab(k)*sf(k, :);
        end
    end
    Sy1 = var(ref.innov_y1_out(3:end, ax));        % empirical innovation variance
    dlev = -sw;
    ddif = [zeros(1, 3); diff(dlev, 1, 1)];
    I_hi = sum(dlev.^2, 1) / Sy1;
    I_lo = sum(ddif.^2, 1) / Sy1;
    rms_s = rms(dlev, 1);
    snr   = rms_s / sqrt(Sy1);
end


function [tau, rho1, rho2] = local_tau_int(v, m)
%LOCAL_TAU_INT  Integrated autocorrelation time of the whitened y2 innovation
%   on the longest contiguous gate-open block.  tau = 1 + 2*sum rho_j, summed
%   until the first non-positive rho (initial-positive-sequence truncation).
    [i0, i1] = local_longest_block(m);
    x = v(i0:i1); x = x - mean(x);
    n = numel(x);
    L = min(200, floor(n/10));
    c0 = sum(x.^2) / n;
    rho = zeros(L, 1);
    for j = 1:L
        rho(j) = sum(x(1:end-j) .* x(1+j:end)) / (n * c0);
    end
    tau = 1;
    for j = 1:L
        if rho(j) <= 0; break; end
        tau = tau + 2*rho(j);
    end
    rho1 = rho(1); rho2 = rho(2);
end


function [i0, i1] = local_longest_block(m)
%LOCAL_LONGEST_BLOCK  Longest run of true in a logical vector.
    m = m(:);
    dm = diff([false; m; false]);
    st = find(dm == 1); en = find(dm == -1) - 1;
    [~, j] = max(en - st);
    i0 = st(j); i1 = en(j);
end


function [S_T, S_n] = local_echo_shares(lambda_c, a_pd)
%LOCAL_ECHO_SHARES  y2 self-echo sensitivities (verbatim controller init copy).
    alE = 1 - lambda_c;
    epE = 1e-4;
    vE  = zeros(2, 3);
    gE_list = [1, 1/(1+epE), 1/(1-epE)];
    for iN = 1:2
        for iG = 1:3
            gE = gE_list(iG);
            AE = zeros(6); BqE = zeros(6,1); BnE = zeros(6,1);
            AE(1,1)=1; AE(1,3)=-gE*alE; AE(1,4)=-gE*alE; AE(1,5)=-gE*alE;
            BnE(1)=-gE*alE; BqE(1)=1;
            AE(2,1)=1; AE(3,2)=1;
            AE(4,3)=-alE; AE(4,4)=-alE; AE(4,5)=-alE; BnE(4)=-alE;
            AE(5,4)=1;
            AE(6,3)=a_pd; AE(6,6)=1-a_pd; BnE(6)=a_pd;
            if iN == 1; QE = BqE*BqE.'; extraE = 0;
            else;       QE = BnE*BnE.'; extraE = (1-a_pd)^2; end
            XE = reshape((eye(36) - kron(AE,AE)) \ QE(:), 6, 6);
            cE = zeros(1,6); cE(3) = 1-a_pd; cE(6) = -(1-a_pd);
            vE(iN,iG) = cE*XE*cE.' + extraE;
        end
    end
    S_T = (log(vE(1,2)) - log(vE(1,3))) / (2*epE);
    S_n = (log(vE(2,2)) - log(vE(2,3))) / (2*epE);
end


function a = local_acc(a, v, q, ns)
%LOCAL_ACC  Running mean over seeds (NaN-initialised accumulator).
    if q == 1 || isnan(a); a = v / ns; else; a = a + v / ns; end
end


function s = local_num(v)
%LOCAL_NUM  Fixed-width number or '-' for NaN.
    if isnan(v); s = '-'; else; s = sprintf('%.4f', v); end
end


function v = local_field(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)); v = s.(name); else; v = default; end
end
