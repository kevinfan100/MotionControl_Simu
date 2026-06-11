function gates_part5()
%GATES_PART5 Dual gates for packaging part 5 (EKF recursion, D6 form).
%
%   E1  DARE init structure: replicate the controller's [0G] DARE
%       STRUCTURE (same F_e_ss/H/Q-form; R22 uses a placeholder IF scale,
%       exact IF dual-source lands in part 6) and verify the fixed point
%       (residual < 1e-11 -- the unit-eigenvalue Jordan slots with zero Q
%       converge ~1/k, so the residual floor is above machine eps),
%       posterior PSD, and PD of the a-priori advance F*P*F' + Q.
%   E2  Closed-loop h50 with the LIVE EKF, 5 seeds x 5 s (the mother
%       repo's own verification thresholds -- valid here because the
%       mother's guards never trigger at h50, so guard removal is a no-op
%       in this scenario): tracking std < 40 nm; aggregate a_hat bias vs
%       physics ground truth < 5 percent; aggregate rel-std < 5 percent.
%   E3  L2 envelope (ramp 50 -> 2.7 um, h_bar = 1.2, full 20 s, NO
%       guards): numerically stable (all finite), tracking < 40 nm,
%       a_hat > 0 throughout. Near-wall a_hat ACCURACY is deliberately
%       not asserted -- the documented L2 limitation.
%
%   Part-6 gates (Q/R content: Q55 emp/closed, IF dual-source) follow in
%   the next round; part-7 adds the closed-loop equivalence vs mother.

    here = fileparts(mfilename('fullpath'));
    sa_root = fileparts(here);

    orig_path = path; orig_dir = pwd;
    cleanup = onCleanup(@() local_restore(orig_path, orig_dir));

    addpath(sa_root, fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim'));

    % ================= E1: DARE init dual-source =================
    rng(1); params = config('h50');
    lc = params.ctrl.lambda_c; apd = params.ctrl.a_pd; acov = params.ctrl.a_cov;
    kBT = params.ctrl.k_B * params.ctrl.T;
    Ts = params.common.Ts; a_nom = Ts / params.common.gamma_N;
    hb0 = params.traj.h_init / params.common.R;
    [cpa0, cpe0, dv0] = wall_corrections(hb0, true);
    a_init = [a_nom/cpa0; a_nom/cpa0; a_nom/cpe0];
    K_h0   = [dv0.K_h_para; dv0.K_h_para; dv0.K_h_perp];

    one_m_apd = 1 - apd; denom_pole = 1 - one_m_apd * lc;
    C_dpmr = one_m_apd^2 * (2*one_m_apd*(1-lc)/denom_pole ...
             + (2/(2-apd)) * 1/((1+lc)*denom_pole));
    C_n = (2*one_m_apd^2/(2-apd)) * (1 + one_m_apd^2*apd*(1-lc)/denom_pole ...
             + (1-lc)^2/((1+lc)*denom_pole));
    K_var = 2*acov/(2-acov);
    fac_inc = 2/(1+lc);
    xi = (C_n/C_dpmr) * params.ctrl.sigma2_noise / (4*kBT);

    Fe_ss = [0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 lc -1 0 0; ...
             0 0 0 1 0 0; 0 0 0 0 1 1; 0 0 0 0 0 1];
    H = [1 0 0 0 0 0; 0 0 0 0 1 -2];
    sigma2_dh0 = 4*kBT*(a_nom/cpe0);
    olc = 1 - lc;
    for ax = 1:3
        var_da0 = fac_inc * (a_init(ax)*K_h0(ax)/params.common.R)^2 * sigma2_dh0;
        Q = zeros(6);
        Q(3,3) = 4*kBT*a_init(ax)*(1 + olc^2*2) + olc^2*params.ctrl.sigma2_noise(ax);
        Q(5,5) = var_da0;
        % IF via independent geometric reference values is overkill here;
        % use the controller-identical structure through a long fixed-point
        % run instead: the point of E1 is the DARE, not IF (IF has its own
        % self-check assert inside the controller init).
        IF0 = 4.2;                                   % conservative placeholder scale
        R22 = K_var * IF0 * (a_init(ax) + xi(ax))^2 + 2*var_da0;
        Rm = [params.ctrl.sigma2_noise(ax), 0; 0, R22];

        P = eye(6);
        for it = 1:20000
            Pp = Fe_ss*P*Fe_ss' + Q; Pp = 0.5*(Pp+Pp');
            S = H*Pp*H' + Rm; K = (Pp*H')/S;
            Pn = (eye(6)-K*H)*Pp; Pn = 0.5*(Pn+Pn');
            if max(abs(Pn(:)-P(:))) < 1e-13, P = Pn; break; end
            P = Pn;
        end
        % fixed-point residual + PD of posterior and a-priori advance
        Pp = Fe_ss*P*Fe_ss' + Q; S = H*Pp*H' + Rm; K = (Pp*H')/S;
        Pchk = (eye(6)-K*H)*Pp; Pchk = 0.5*(Pchk+Pchk');
        res = max(abs(Pchk(:)-P(:)));
        assert(res < 1e-11, 'E1 DARE residual %g (ax %d)', res, ax);
        assert(all(eig(0.5*(P+P')) > -1e-20), 'E1 posterior not PSD (ax %d)', ax);
        Pf0 = Fe_ss*P*Fe_ss' + Q;
        assert(all(eig(0.5*(Pf0+Pf0')) > 0), 'E1 a-priori P_f0 not PD (ax %d)', ax);
    end
    fprintf('E1 PASS  DARE fixed point (residual < 1e-11), posterior PSD, a-priori P_f0 PD\n');

    % ================= E2: closed-loop h50, live EKF, 5 seeds =============
    seeds = 1:5;
    n_warm = round(0.5 / Ts);
    trk = zeros(numel(seeds), 3); ab = zeros(numel(seeds), 3); as = zeros(numel(seeds), 3);
    for i = 1:numel(seeds)
        out = run_simulation('h50', struct('seed', seeds(i)));
        N = numel(out.tout); idx = (n_warm+1):N;
        trk(i,:) = std(out.p_d_out(idx,:) - out.p_m_out(idx,:), 0, 1) * 1e3;
        % physics ground truth from the noise-free probe
        h_true = out.p_true_out(idx,3) / params.common.R;
        a_tr = zeros(numel(idx), 3);
        for k = 1:numel(idx)
            [ca, ce] = wall_corrections(max(h_true(k), 1.001));
            a_tr(k,:) = [a_nom/ca, a_nom/ca, a_nom/ce];
        end
        ah = out.ekf_out(idx, 1:3);
        ab(i,:) = (mean(ah,1) - mean(a_tr,1)) ./ mean(a_tr,1) * 100;
        as(i,:) = std(ah,0,1) ./ mean(ah,1) * 100;
    end
    trk_m = mean(trk,1); ab_m = mean(ab,1); as_m = mean(as,1);
    assert(all(trk_m < 40), 'E2 tracking %s nm', mat2str(trk_m,4));
    assert(all(abs(ab_m) < 5), 'E2 a_hat bias %s pct', mat2str(ab_m,3));
    assert(all(as_m < 5), 'E2 a_hat rel-std %s pct', mat2str(as_m,3));
    fprintf(['E2 PASS  h50 live EKF (5 seeds): tracking [%.1f %.1f %.1f] nm, ' ...
             'bias [%.2f %.2f %.2f] pct, rel-std [%.2f %.2f %.2f] pct\n'], ...
            trk_m, ab_m, as_m);

    % ================= E3: L2 envelope (ramp to h_bar = 1.2, no guards) ====
    out = run_simulation('ramp2p7', struct('seed', 1));
    N = numel(out.tout); idx = (n_warm+1):N;
    assert(all(isfinite(out.p_m_out(:))) && all(isfinite(out.ekf_out(:))), 'E3 non-finite');
    trk3 = std(out.p_d_out(idx,:) - out.p_m_out(idx,:), 0, 1) * 1e3;
    assert(all(trk3 < 40), 'E3 tracking %s nm', mat2str(trk3,4));
    assert(all(out.ekf_out(idx,1:3) > 0, 'all'), 'E3 a_hat went non-positive');
    assert(abs(out.ekf_out(end,4) - 1.2) < 0.15, 'E3 final h_bar %.3f', out.ekf_out(end,4));
    fprintf(['E3 PASS  ramp to h_bar = 1.2 (20 s, NO guards): stable, tracking ' ...
             '[%.1f %.1f %.1f] nm, a_hat positive throughout\n'], trk3);

    fprintf('\n=== gates_part5: ALL PASS (E1 DARE, E2 h50 live EKF, E3 L2 envelope) ===\n');
end


function local_restore(orig_path, orig_dir)
%LOCAL_RESTORE  Undo path/cwd mutations on exit (incl. on error).
    path(orig_path);
    cd(orig_dir);
end
