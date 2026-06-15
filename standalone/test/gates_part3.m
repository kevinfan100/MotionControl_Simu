function gates_part3()
%GATES_PART3 Dual gates for packaging part 3 (measurement chain).
%
%   STATUS: C1/C4 live; C2/C3 SNAPSHOT since pack(6) (probe retired; see
%   the notice in the body -- successor = part-7 closed-loop equivalence).
%
%   C1  Constants dual-source: C_dpmr / C_n full a_pd closed form
%       transcribed here independently == mother build_eq17_6state_constants
%       output (bit-exact) == reference values 3.1610 / 1.1093.
%   C2  d=2 delay-chain k-table (deferred from part 2): on a moving (osc)
%       run the controller's logged delta_x_m_z (z = wall normal, the axis
%       the trajectory moves -- so both the pd and p_m delay buffers are
%       exercised) must satisfy, exactly for k >= 4,
%           ekf_out(k,1) == p_d_out(k-2,3) - p_m_out(k-3,3)
%       LOG-ROW COORDINATES NOTE: p_d_out row j stores the sample-j
%       reference pd[j], but p_m_out row j stores the measurement produced
%       AFTER step j's integration, i.e. the sample-(j+1) measurement.
%       In SAMPLE coordinates both terms are at sample k-2 (a clean d=2
%       delay); the k-2 / k-3 row asymmetry is purely the logging offset.
%       IC steps (k = 1 init, k = 2,3 buffer IC -> 0) asserted
%       individually, plus the IIR prefill value visible at k = 1.
%   C3  Synthetic AR(1) theory gate: drive the controller directly with
%       delta_x_m = AR(1)(lambda_s, q) + white(r); expected Var(dx_r) is
%       computed EXACTLY from filter theory (impulse responses of the
%       high-pass (1-a_pd)(1-z^-1)/(1-(1-a_pd)z^-1) and the AR(1) shaping),
%       and the controller's time-averaged sigma2_dxr_hat must match
%       within 4 percent (time-average estimator noise ~1.2 percent at
%       N=45k). Also asserts the a_xm linear inversion identity bit-exact
%       from the logged probes.
%   C4  d = 2 contract: cfg.d == 2 and mother ctrl_const.d == 2.
%
%   NOTE: the closed-loop calibration check (sigma2_dxr == C_dpmr*4kBT*a
%   + C_n*sigma2_nx with emp/closed ~ 1) requires the full loop and lands
%   in part 7/8 gates; C3 here validates the chain DYNAMICS exactly.

    here = fileparts(mfilename('fullpath'));
    sa_root = fileparts(here);
    repo    = fileparts(sa_root);

    orig_path = path; orig_dir = pwd;
    cleanup = onCleanup(@() local_restore(orig_path, orig_dir));

    addpath(sa_root, fullfile(sa_root, 'physics'), fullfile(sa_root, 'sim'), ...
            fullfile(repo, 'model', 'controller'));

    % ================= C1: constants dual-source =================
    lc = 0.7; apd = 0.05;
    one_m_apd  = 1 - apd;
    denom_pole = 1 - one_m_apd * lc;
    C_dpmr_gate = one_m_apd^2 * ( ...
                    2 * one_m_apd * (1 - lc) / denom_pole ...
                  + (2 / (2 - apd)) * 1 / ((1 + lc) * denom_pole) );
    C_n_gate = (2 * one_m_apd^2 / (2 - apd)) * ( ...
                    1 ...
                  + one_m_apd^2 * apd * (1 - lc) / denom_pole ...
                  + (1 - lc)^2 / ((1 + lc) * denom_pole) );

    opts_cc = struct('lambda_c', lc, 'a_pd', apd, 'a_cov', 0.05, 'd', 2, ...
                     'sigma2_n_s', [0.00062; 0.00057; 0.00331].^2, ...
                     'kBT', 1.3806503e-5 * 310.15);
    cc = build_eq17_6state_constants(opts_cc);
    assert(C_dpmr_gate == cc.C_dpmr && C_n_gate == cc.C_n, ...
           'C1 dual-source mismatch: gate (%.6f, %.6f) vs builder (%.6f, %.6f)', ...
           C_dpmr_gate, C_n_gate, cc.C_dpmr, cc.C_n);
    assert(abs(C_dpmr_gate - 3.1610) < 5e-4 && abs(C_n_gate - 1.1093) < 5e-4, ...
           'C1 reference values drifted');
    fprintf('C1 PASS  C_dpmr = %.4f, C_n = %.4f (gate == mother builder, bit-exact)\n', ...
            C_dpmr_gate, C_n_gate);

    % ================= C4: d = 2 contract =================
    rng(1); [~, cfg] = config('h50');
    assert(cfg.d == 2 && cc.d == 2, 'C4 d contract violated');
    fprintf('C4 PASS  d = 2 (config + mother builder)\n');

    % ================= C2/C3: SNAPSHOT (retired at pack(6)) =================
    % Established at pack(4)/pack(5) using the INTERIM measurement-chain
    % probe in ekf_out, which the part-5 EKF build replaced with the final
    % [a_hat_xyz; h_bar] output. The facts they pinned (delay k-table row
    % mapping; IIR chain == exact filter theory; a_xm inversion) are part
    % of the frozen [1] section text; their successor is the part-7
    % closed-loop equivalence gate vs the mother repo.
    fprintf(['C2/C3 SNAPSHOT  established pack(4)-pack(5); probe retired; ' ...
             'superseded by part-7 closed-loop equivalence\n']);
    fprintf('\n=== gates_part3: ALL PASS (C1 constants, C4 d=2; C2/C3 snapshot) ===\n');
    return;

    % ----- retired probe-dependent sections below (kept for the record) -----
    out = run_simulation('osc1hz', struct('seed', 9, 'T_sim', 3)); %#ok<UNRCH>
    N = numel(out.tout);

    % k = 1: init return -- delta_x_m probe = 0, prefill visible in slot 2
    rng(9); params9 = config('osc1hz');             % same params as the run
    Ts9 = params9.common.Ts; a_nom9 = Ts9 / params9.common.gamma_N;
    hb0 = params9.traj.h_init / params9.common.R;
    [~, cpe0] = wall_corrections(hb0);
    a_seed_z = a_nom9 / cpe0;                       % z axis uses c_perp
    kBT9 = params9.ctrl.k_B * params9.ctrl.T;
    prefill_z = 4 * kBT9 * a_seed_z * C_dpmr_gate + C_n_gate * params9.ctrl.sigma2_noise(3);
    assert(out.ekf_out(1, 1) == 0, 'C2 k=1 probe nonzero');
    assert(abs(out.ekf_out(1, 2) - prefill_z) < 1e-15 * prefill_z, ...
           'C2 prefill mismatch: logged %.6e vs expected %.6e', ...
           out.ekf_out(1, 2), prefill_z);

    % k = 2, 3: pd buffer IC and p_m buffer IC are both p0 -> delta_x_m = 0
    % (p_m_buffer holds d+1 = 3 columns of p0; rows 2 and 3 still read IC)
    assert(out.ekf_out(2, 1) == 0, 'C2 k=2 IC step nonzero');
    assert(out.ekf_out(3, 1) == 0, 'C2 k=3 IC step nonzero');

    % k >= 4: delta_x_m[k] = pd[k-2] - m[k-2] in SAMPLE coordinates, which
    % in log-row coordinates is p_d_out(k-2) - p_m_out(k-3) (see header)
    lhs = out.ekf_out(4:N, 1);
    rhs = out.p_d_out(2:N-2, 3) - out.p_m_out(1:N-3, 3);
    dmax = max(abs(lhs - rhs));
    assert(dmax == 0, 'C2 k-table max|diff| = %g (not exact)', dmax);
    fprintf('C2 PASS  delay k-table exact for k=4..%d (z axis: pd AND p_m sides) + IC k=1..3 + prefill\n', N);

    % ================= C3: synthetic AR(1) theory gate =================
    clear controller_6state;
    rng(21); params = config('h50');
    p0 = params.common.p0;
    lam_s = 0.7;
    sig_s = 0.030;                      % [um] target AR(1) stationary std
    q = sig_s^2 * (1 - lam_s^2);        % AR(1) innovation variance
    sig_r = 0.001;                      % [um] white component std
    r = sig_r^2;

    Nsyn = 5e4; burn = 5e3;
    e_ar  = filter(sqrt(q), [1, -lam_s], randn(Nsyn, 3));   % AR(1) per axis
    s_tot = e_ar + sig_r * randn(Nsyn, 3);                  % + white

    probe = zeros(Nsyn, 4);
    for k = 1:Nsyn
        p_m_k = p0 - s_tot(k, :)';      % so delta_x_m = s_tot (pd const p0)
        [~, probe(k, :)] = controller_6state(zeros(3, 1), p0, p_m_k, params);
    end

    % Exact expected Var(dx_r) via impulse responses:
    %   HP(z) = (1-a_pd)(1-z^-1) / (1-(1-a_pd)z^-1)
    %   Var = q*sum(h(HP*AR1)^2) + r*sum(h(HP)^2)
    apd_c = params.ctrl.a_pd;
    M = 20000; imp = [1; zeros(M-1, 1)];
    num_hp = (1 - apd_c) * [1, -1];
    den_hp = [1, -(1 - apd_c)];
    h_hp    = filter(num_hp, den_hp, imp);
    h_hp_ar = filter(num_hp, conv(den_hp, [1, -lam_s]), imp);
    var_expect = q * sum(h_hp_ar.^2) + r * sum(h_hp.^2);

    sigma2_avg = mean(probe(burn+1:end, 2));
    rel = abs(sigma2_avg - var_expect) / var_expect;
    assert(rel < 0.04, 'C3 Var(dx_r): emp %.4e vs theory %.4e (rel %.3f)', ...
           sigma2_avg, var_expect, rel);

    % a_xm linear-inversion identity from the logged probes (bit-exact)
    kBTs = params.ctrl.k_B * params.ctrl.T;
    a_xm_expect = (probe(burn+1:end, 2) - C_n_gate * params.ctrl.sigma2_noise(3)) ...
                  / (C_dpmr_gate * 4 * kBTs);
    assert(isequal(probe(burn+1:end, 3), a_xm_expect), 'C3 a_xm inversion identity');
    fprintf(['C3 PASS  synthetic AR(1): sigma2_dxr emp/theory = %.4f (tol 4%%); ' ...
             'a_xm inversion bit-exact\n'], sigma2_avg / var_expect);

    fprintf('\n=== gates_part3: ALL PASS (C1 constants, C2 k-table, C3 AR(1) theory, C4 d=2) ===\n');
end


function local_restore(orig_path, orig_dir)
%LOCAL_RESTORE  Undo path/cwd mutations on exit (incl. on error).
    path(orig_path);
    cd(orig_dir);
end
