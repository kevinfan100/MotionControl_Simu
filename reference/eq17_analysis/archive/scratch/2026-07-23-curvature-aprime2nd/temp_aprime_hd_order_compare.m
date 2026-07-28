function out = temp_aprime_hd_order_compare(varargin)
%TEMP_APRIME_HD_ORDER_COMPARE  3-arm oracle-vs-closure test of 6state_curvature_hd.tex.
%
%   out = temp_aprime_hd_order_compare()                 % seed 1, manuscript defaults
%   out = temp_aprime_hd_order_compare('seed',3, ...)
%
%   -------------------------------------------------------------------------
%   WHAT THIS TESTS
%   -------------------------------------------------------------------------
%   The manuscript reference/eq17_analysis/derivation/6state_curvature_hd.tex
%   models the gain slope a'_hd as a DETERMINISTIC 2nd-order (curvature) object:
%       a_h[k]        = a_h(h_d[k]) - a'_hd[k]*delta_h[k]         (Taylor gain)
%       a'_hd[k+1]    = a'_hd[k] + delta_a'_hd[k]*Dh_d[k]          (pure SWEEP)
%       delta_a'_hd[k+1] = delta_a'_hd[k]                          (FROZEN curvature)
%   The manuscript q-vector is q=[0,0,q3,q4,0,q6], q3=-eps_h, q4=a'_hd*eps_h
%   (thermal, rank-1) and q6 = a'''_hd*Dh_d (the TRUE 3rd-order sweep the frozen
%   delta_a'_hd estimator misses).  So Q33=sigma_eps^2, Q44=a'_hd^2*sigma_eps^2,
%   Q55=0, and the ONE open question is Q66 = a'''^2*Dh^2.  Since a''' needs
%   a'''' -> a^(5) ..., the .tex proposes a TOP-STATE CLOSURE a''' ~ delta_a'/R:
%       Q66 ~ (delta_a'_hd * Dh_d / R)^2   (uses the ESTIMATE, no higher deriv).
%
%   This script runs THREE arms on the SAME trajectory / seed / true wall curve
%   (z-axis / c_perp, calc_correction_functions) to answer: (1) does the 2nd-order
%   STRUCTURE help when Q66 is correct? (2) is the closure good enough? (3) does
%   the 2nd-order model corrupt a_hat / tracking?
%
%   -------------------------------------------------------------------------
%   THE THREE ARMS  (identical trajectory, seed, plant, control law, F_dh, H(2,5))
%   -------------------------------------------------------------------------
%   ARM 1  (1st-order RW, 5-state):  x = [dh1 dh2 dh3 a_h a'_hd]
%       a'_hd[k+1] = a'_hd[k] + w_a,   F_e row5 = [0 0 0 0 1],  Q55 > 0.
%       Q55 = thermal-grounded a''^2 * Var(Delta x_ram) (measured h-bar),
%       scaled by 'Q55_arm1_scale'  <-- ARM 1's free knob.
%   ARM 2a (2nd-order + ORACLE Q66, 6-state):  x = [dh1 dh2 dh3 a_h a'_hd delta_a'_hd]
%       manuscript F_e (row3 = [0 0 lc -F_dh F_dh^{DH} 0], the a'-into-dh3 coupling
%       the production 4/5-state controllers drop) + manuscript raw-a_hm H.
%       Q33=sigma_eps^2, Q44=a'_hd^2*sigma_eps^2, Q55=0 (per .tex), and
%       Q66[k] = a'''_true(h_d[k])^2 * Dh_d[k]^2, a'''_true = REAL 3rd derivative
%       of the gain curve (central FD of a''(h) from calc_correction_functions at
%       the COMMANDED height h_d).  This is the ONLY cheat: true a''' feeds Q66
%       ONLY; a, a', a'' stay estimated states, never read off the curve.
%   ARM 2b (2nd-order + CLOSURE Q66, 6-state):  identical to 2a but HONEST --
%       Q66[k] = (delta_a'_hd_hat[k] * Dh_d[k] / R)^2 (top-state closure, uses the
%       ESTIMATE delta_a'_hd_hat and known R, Dh_d; no a''').  Everything else 2a.
%
%   -------------------------------------------------------------------------
%   SCENARIO / FIDELITY
%   -------------------------------------------------------------------------
%   Canonical 1 Hz: hold(0.5s) -> cosine descent(1.0s) -> 4-cycle osc, h-bar
%   spanning ~[2, 22].  Thermal ON, measurement noise ON, single canonical seed.
%   Gamma_inv is DIAGONAL for w_hat = z_hat (z-mobility = 1/(gamma_N*c_perp)), so
%   the wall-normal z-axis decouples from x/y -> exact clean 1-D reimplementation
%   of the manuscript's 1-D plant.  Plant = ode4 (10 us substep), thermal variance
%   4*kBT*gamma*c_perp/Ts, measurement d=2 step delay.  Offline R22/IF/C_dpmr
%   constants from build_eq17_6state_constants.  All arms share the SAME thermal /
%   measurement noise draws.
%
%   KNOBS (name/value):
%       'seed'            RNG seed (shared thermal+meas draws)        default 1
%       'Q66_floor'       tiny design floor added to Q66 (both 2a/2b) default 0
%       'Q55_arm1_scale'  ARM 1 thermal-Q55 multiplier                default 1
%       'P55_0_frac'      a' prior sd = frac*a_nom/R                   default 0.01
%       'P66_0_frac'      delta_a' prior sd = frac*a_nom/R^2           default 1e-3
%       'make_fig'        save the comparison figure                  default true
%       'verbose'         print per-step progress                     default false
%
%   TEMP diagnostic (chat 2026-07-23); gitignored (temp_*.m), delete after use.

    % ---- knobs -----------------------------------------------------------
    PAR.seed           = 1;
    PAR.Q66_floor      = 0;       % tiny design floor added to BOTH 2a and 2b Q66
    PAR.Q55_arm1_scale = 1;       % ARM 1's process-noise knob (thermal-grounded default)
    PAR.P55_0_frac     = 0.01;    % a' prior sd fraction (far-field a'~=0 with confidence)
    PAR.P66_0_frac     = 1e-3;    % delta_a' prior sd fraction
    PAR.make_fig       = true;
    PAR.verbose        = false;
    for i = 1:2:numel(varargin); PAR.(varargin{i}) = varargin{i+1}; end

    % ---- paths (self-contained deps: physical_constants, calc_correction_functions,
    %      build_eq17_6state_constants) ------------------------------------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'controller'));

    % ---- fixed physical parameters (physical_constants) ------------------
    pc  = physical_constants();
    pm.Ts    = pc.Ts;                 % [s]
    pm.R     = pc.R;                  % [um]
    pm.gamma = pc.gamma_N;            % [pN*s/um]
    pm.kBT   = pc.k_B * pc.T;         % [pN*um]
    pm.a_nom = pm.Ts / pm.gamma;      % [um/pN] bulk Stokes gain
    pm.lc    = 0.7;                   % lambda_c
    pm.d     = 2;                     % measurement delay
    pm.a_pd  = 0.05;                  % IIR LP weight (dx_bar_m)
    pm.a_cov = 0.05;                  % EWMA variance weight (sigma2_dxr)
    pm.t_warmup = 0;                  % KF warm-up (prefill init -> 0)
    pm.hbar_safe = 1;                 % Guard 3 threshold (canonical scenario)
    meas_std = [0.00062; 0.00057; 0.00331];   % [um] per-axis sensor noise (canonical)
    pm.sig2n_z = meas_std(3)^2;       % z-axis measurement-noise variance [um^2]

    % ---- offline constants (production builder) for a faithful R22 floor --
    eq17.lambda_c   = pm.lc;
    eq17.sigma2_n_s = meas_std.^2;    % 3x1 [um^2]
    eq17.kBT        = pm.kBT;
    eq17.a_cov      = pm.a_cov;
    eq17.a_pd       = pm.a_pd;
    eq17.d          = pm.d;
    cst_full = build_eq17_6state_constants(eq17);
    cst.C_dpmr = cst_full.C_dpmr;
    cst.C_n    = cst_full.C_n;
    cst.K_var  = cst_full.K_var;
    cst.IF_abc = cst_full.IF_abc(:);
    cst.xi_z   = cst_full.xi_per_axis(3);
    cst.var_da_increment_factor = cst_full.var_da_increment_factor;
    cst.r22_delay_factor        = cst_full.r22_delay_sum_factor;

    % ---- trajectory (canonical 1 Hz hold->descent->osc), h-bar ~ [2,22] ---
    tr.h_init    = 50;                 % [um] -> h-bar_init = 22.2
    tr.h_bottom  = 2.0 * pm.R;         % [um] -> h-bar_bottom = 2.0 (osc trough)
    tr.amplitude = 2.5;                % [um] osc half-amplitude
    tr.freq      = 1;                  % [Hz]
    tr.t_hold    = 0.5;                % [s]
    tr.t_descend = 1.0;                % [s]
    tr.n_cycles  = 4;
    tr.t1 = tr.t_hold;                       % end hold   = 0.5
    tr.t2 = tr.t1 + tr.t_descend;            % end descent = 1.5
    tr.t3 = tr.t2 + tr.n_cycles / tr.freq;   % end osc     = 5.5
    T_sim = tr.t3 + 0.5;                     % 6.0
    N = round(T_sim / pm.Ts) + 1;
    tout = (0:N-1)' * pm.Ts;
    tr.N = N;

    % desired-height sequences (unit-delay convention: controller pd_k = H(t_k))
    h_d_seq     = zeros(N, 1);
    h_d_kp1_seq = zeros(N, 1);
    for k = 1:N
        h_d_seq(k)     = traj_height(tout(k),          tr);
        h_d_kp1_seq(k) = traj_height(tout(k) + pm.Ts,  tr);
    end
    tr.h_d_seq = h_d_seq;
    tr.h_d_kp1_seq = h_d_kp1_seq;

    % ---- oracle TRUTH: a, a', a'', a''' at the commanded height h_d[k] --------
    %   a, a', a'' are metric targets (mirror verify_aprime_Qprime_assumptions /
    %   a_gain_chain.tex). a''' is the ONLY cheat: it feeds arm 2a's Q66 (central
    %   FD of a''(h) from calc_correction_functions along h_d).
    oracle_a   = zeros(N, 1);
    oracle_ap  = zeros(N, 1);
    oracle_app = zeros(N, 1);
    oracle_a3  = zeros(N, 1);
    for k = 1:N
        hb = max(h_d_seq(k) / pm.R, 1.001);
        [~, cp, drv] = calc_correction_functions(hb, true);
        oracle_a(k)   = pm.a_nom / cp;
        oracle_ap(k)  = -(pm.a_nom / cp) * drv.K_h_perp / pm.R;                       % da/dh   [um/pN/um]
        oracle_app(k) =  (pm.a_nom / cp) / pm.R^2 * (drv.K_h_perp^2 - drv.K_h_prime_perp);  % d2a/dh2
        oracle_a3(k)  = a3prime_at(h_d_seq(k), pm);                                   % d3a/dh3 (central FD)
    end
    % analytic a''' cross-check (a_gain_chain D''' expansion) -> report max rel err
    a3_ana = zeros(N,1);
    for k = 1:N; a3_ana(k) = a3prime_analytic(h_d_seq(k), pm); end
    a3_fd_relerr = max(abs(oracle_a3 - a3_ana)) / max(abs(a3_ana));

    % ---- shared noise draws (fair: both arms see the SAME thermal + meas randn) ----
    rng(PAR.seed);
    noise.z_th   = randn(N, 1);
    noise.z_meas = randn(N, 1);

    knobs = struct('Q66_floor', PAR.Q66_floor, 'Q55_arm1_scale', PAR.Q55_arm1_scale, ...
                   'P55_0_frac', PAR.P55_0_frac, 'P66_0_frac', PAR.P66_0_frac);

    % ---- arm specs -------------------------------------------------------
    specs = { ...
        struct('id', '1',  'label', '1st-order',            'order', 1, 'q66_mode', 'na',      'a3_hd', zeros(N,1)); ...
        struct('id', '2a', 'label', '2nd-order oracle-Q66', 'order', 2, 'q66_mode', 'oracle',  'a3_hd', oracle_a3); ...
        struct('id', '2b', 'label', '2nd-order closure-Q66','order', 2, 'q66_mode', 'closure', 'a3_hd', zeros(N,1)) };
    ids    = cellfun(@(s) s.id,    specs, 'UniformOutput', false);
    labels = cellfun(@(s) s.label, specs, 'UniformOutput', false);

    % ---- run all arms (independent closed loops, shared noise) -----------
    warning('off', 'MATLAB:nearlySingularMatrix');
    res = cell(numel(specs), 1);
    for a = 1:numel(specs)
        res{a} = run_arm(specs{a}, cst, pm, tr, noise, knobs, PAR.verbose);
    end
    warning('on', 'MATLAB:nearlySingularMatrix');
    res1 = res{1};  res2a = res{2};  res2b = res{3};

    % =====================================================================
    % Reporting
    % =====================================================================
    seg.hold = tout >= 0.1        & tout <= tr.t1;
    seg.desc = tout >  tr.t1      & tout <  tr.t2;
    seg.osc  = tout >= tr.t2 + 0.5 & tout <= tr.t3;   % skip first half-cycle settling
    per_samp = round((1 / tr.freq) / pm.Ts);

    nm = 1e3;    % um/pN[/um...] -> nm/pN[/um...] display scale
    fprintf('\n==================================================================\n');
    fprintf(' a'' MODELING: 3-arm oracle-vs-closure  (6state_curvature_hd.tex)\n');
    fprintf('   ARM 1 = 1st-order RW  |  ARM 2a = 2nd-order + ORACLE Q66  |  ARM 2b = 2nd-order + CLOSURE Q66\n');
    fprintf('   canonical 1Hz, z-axis (c_perp), seed %d\n', PAR.seed);
    fprintf('==================================================================\n');
    fprintf(' segments (s):  hold[0.1,%.2f]  descent(%.2f,%.2f)  osc[%.2f,%.2f]\n', ...
            tr.t1, tr.t1, tr.t2, tr.t2 + 0.5, tr.t3);
    fprintf(' T_sim=%.2f s, N=%d, Ts=%.4e s, lambda_c=%.2f, d=%d, sigma_n_z=%.3f nm\n', ...
            T_sim, N, pm.Ts, pm.lc, pm.d, sqrt(pm.sig2n_z)*nm);
    fprintf(' oracle a'''''' (arm 2a Q66): central-FD vs analytic D'''''''' max rel.err = %.2e\n', a3_fd_relerr);

    % ---- true a / a' / a'' / a''' ranges along the trajectory -------------
    fprintf('\n TRUE curve along trajectory (z-axis / c_perp):\n');
    fprintf('   a    = a_nom/c_perp  : [%.3f , %.3f] nm/pN            (a_nom = %.3f nm/pN)\n', ...
            min(oracle_a)*nm, max(oracle_a)*nm, pm.a_nom*nm);
    fprintf('   a''   = da/dh         : [%.3f , %.3f] nm/pN/um\n', min(oracle_ap)*nm, max(oracle_ap)*nm);
    fprintf('   a''''  = d2a/dh2       : [%.3f , %.3f] nm/pN/um^2\n', min(oracle_app)*nm, max(oracle_app)*nm);
    fprintf('   a''''''  = d3a/dh3       : [%.3e , %.3e] um/pN/um^3\n', min(oracle_a3), max(oracle_a3));

    % ---- Q / R actually used (osc-mean) -----------------------------------
    om = seg.osc;
    fprintf('\n PROCESS / MEASUREMENT NOISE used (osc-mean unless noted):\n');
    fprintf('   ARM 1  (RW)      : Q33=%.3e  Q55=%.3e (thermal x%.2g)  R22=%.3e\n', ...
            mean(res1.Q33(om)), mean(res1.Q55(om)), PAR.Q55_arm1_scale, mean(res1.R22(om)));
    fprintf('   ARM 2a (oracle)  : Q33=%.3e  Q55=0  Q66=%.3e            R22=%.3e\n', ...
            mean(res2a.Q33(om)), mean(res2a.Q66(om)), mean(res2a.R22(om)));
    fprintf('   ARM 2b (closure) : Q33=%.3e  Q55=0  Q66=%.3e            R22=%.3e\n', ...
            mean(res2b.Q33(om)), mean(res2b.Q66(om)), mean(res2b.R22(om)));

    % ---- Q66 oracle-vs-closure gap (min/median/max over osc + all) --------
    print_q66_table(res2a, res2b, seg);

    % ---- per-segment / whole-run metric tables ----------------------------
    print_metric_table('a''-hat vs a''_true(h_d)', res, ids, oracle_ap, seg, per_samp, pm.Ts, 'aprime');
    print_metric_table('a-hat  vs a_true(h)',      res, ids, oracle_a,  seg, per_samp, pm.Ts, 'again');
    print_ratio_table(res, ids, seg);
    print_track_table(res, ids, seg);

    % ---- verdict ----------------------------------------------------------
    m1  = seg_metric(res1.ap_hat,  oracle_ap, om, per_samp, pm.Ts, true);
    m2a = seg_metric(res2a.ap_hat, oracle_ap, om, per_samp, pm.Ts, true);
    m2b = seg_metric(res2b.ap_hat, oracle_ap, om, per_samp, pm.Ts, true);
    r1  = mean(res1.a_hat(om)  ./ res1.a_true(om));
    r2a = mean(res2a.a_hat(om) ./ res2a.a_true(om));
    r2b = mean(res2b.a_hat(om) ./ res2b.a_true(om));
    fprintf('\n VERDICT (osc-segment a'' vs true a''(h_d)):\n');
    fprintf('   %-24s %-10s %-8s %-8s %-9s %-9s\n', 'arm', 'RMS', 'rel[%]', 'corr', 'lag[ms]', 'a_hat/a');
    fprintf('   %-24s %-10.4f %-8.1f %-8.3f %-9s %-9.4f\n', '1  1st-order',   m1.rms*nm,  m1.rel,  m1.corr,  fmt_num(m1.lag_ms,'%+.1f'),  r1);
    fprintf('   %-24s %-10.4f %-8.1f %-8.3f %-9s %-9.4f\n', '2a oracle-Q66',  m2a.rms*nm, m2a.rel, m2a.corr, fmt_num(m2a.lag_ms,'%+.1f'), r2a);
    fprintf('   %-24s %-10.4f %-8.1f %-8.3f %-9s %-9.4f\n', '2b closure-Q66', m2b.rms*nm, m2b.rel, m2b.corr, fmt_num(m2b.lag_ms,'%+.1f'), r2b);
    fprintf('\n   Q1  2a-oracle beats 1st-order on osc a''?   RMS %s , corr %s\n', ...
            better(m2a.rms, m1.rms), cmp_hi(m2a.corr, m1.corr));
    fprintf('   Q2  2b-closure ~= 2a-oracle?              RMS %.1f%% of oracle , corr %.3f vs %.3f\n', ...
            100*m2b.rms/max(m2a.rms,realmin), m2b.corr, m2a.corr);
    fprintf('   Q3  did 2nd-order corrupt a_hat/tracking? a_hat/a: %.3f(1) %.3f(2a) %.3f(2b) ; osc trk std: %.1f/%.1f/%.1f nm\n', ...
            r1, r2a, r2b, std(res1.dh_track(om))*nm, std(res2a.dh_track(om))*nm, std(res2b.dh_track(om))*nm);
    fprintf('==================================================================\n\n');

    % ---- figure -----------------------------------------------------------
    fig_path = '';
    if PAR.make_fig
        fig_path = make_comparison_figure(tout, tr, seg, oracle_a, oracle_ap, ...
                                          res, labels, project_root);
        fprintf(' FIGURE saved: %s\n\n', fig_path);
    end

    out = struct('tout', tout, 'tr', tr, 'pm', pm, 'cst', cst, 'PAR', PAR, ...
                 'oracle_a', oracle_a, 'oracle_ap', oracle_ap, 'oracle_app', oracle_app, ...
                 'oracle_a3', oracle_a3, 'res1', res1, 'res2a', res2a, 'res2b', res2b, ...
                 'seg', seg, 'fig_path', fig_path);
end


%% ==================================================================
%  Closed-loop 1-D (z-axis) sim + inline manuscript EKF for one arm
%% ==================================================================
function res = run_arm(spec, cst, pm, tr, noise, knobs, verbose)
    order = spec.order;  q66_mode = spec.q66_mode;  a3_hd = spec.a3_hd;
    N = tr.N; Ts = pm.Ts; R = pm.R; gamma = pm.gamma; kBT = pm.kBT;
    a_nom = pm.a_nom; lc = pm.lc; d = pm.d; sig2n = pm.sig2n_z; omlc = 1 - lc;
    C_dpmr = cst.C_dpmr; C_n = cst.C_n; K_var = cst.K_var; IF_abc = cst.IF_abc;
    xi = cst.xi_z; vdaf = cst.var_da_increment_factor; r22f = cst.r22_delay_factor;
    a_pd = pm.a_pd; a_cov = pm.a_cov;
    n_aug = 4 + order;               % order 1 -> 5, order 2 -> 6

    % ---- init (wall-aware seed at h_init; three-pillar prefill + Riccati) ----
    hbar0 = tr.h_init / R;
    [~, cp0, drv0] = calc_correction_functions(hbar0, true);
    a_z0 = a_nom / cp0;  K_h0 = drv0.K_h_perp;

    x = zeros(n_aug, 1);  x(4) = a_z0;      % a'_hd = 0, delta_a'_hd = 0 (far-field)

    % Riccati DARE on the 4-state core at the init operating point (f_d=0, hold):
    % at Dh_d=DH_d=0 the manuscript F_e/H reduce exactly to the core below.
    Fss = [0 1 0 0; 0 0 1 0; 0 0 lc 0; 0 0 0 1];
    Hss = [1 0 0 0; 0 0 0 1];
    sig2dh0  = 4 * kBT * a_z0;
    var_da0  = vdaf * (a_z0 * K_h0 / R)^2 * sig2dh0;
    Q33_0    = 4 * kBT * a_z0 * (1 + omlc^2 * d) + omlc^2 * sig2n;
    Qss = zeros(4);  Qss(3,3) = Q33_0;  Qss(4,4) = var_da0;
    IF0 = if_eff(IF_abc, C_dpmr, C_n, kBT, a_z0, sig2n);
    R22_0 = K_var * IF0 * (a_z0 + xi)^2 + r22f * d * var_da0;
    Rss = [sig2n 0; 0 R22_0];
    Pcore = dare_fp(Fss, Hss, Qss, Rss);
    Pc = zeros(n_aug);  Pc(1:4,1:4) = Pcore;
    Pc(5,5) = (knobs.P55_0_frac * a_nom / R)^2;          % a' prior (far-field a'~=0)
    if order == 2
        Pc(6,6) = (knobs.P66_0_frac * a_nom / R^2)^2;    % curvature prior
    end

    % ---- IIR prefill + delay buffers ----
    dx_bar_m   = 0;
    sigma2_dxr = 4 * kBT * a_z0 * C_dpmr + C_n * sig2n;
    pd_km1 = tr.h_init;  pd_km2 = tr.h_init;   % h_d[k-1], h_d[k-2]
    f_km1  = 0;          f_km2  = 0;           % f_dh[k-1], f_dh[k-2]
    a_km1  = a_z0;       a_km2  = a_z0;        % a_hat[k-1], a_hat[k-2] (Q33 + ctrl law)
    vda_km1 = var_da0;   vda_km2 = var_da0;    % Var(delta_a_ram)[k-1], [k-2] (R22 delay)
    h_curr = tr.h_init;                        % TRUE height above wall (pre-integration)
    hnoisy_hist = tr.h_init * ones(N, 1);      % arm-specific noisy true height per step

    % ---- logs ----
    ap_hat  = zeros(N, 1);  app_hat = zeros(N, 1);  a_hat = zeros(N, 1);
    a_true  = zeros(N, 1);  dh_track = zeros(N, 1);  hbar_meas_log = zeros(N, 1);
    Q33L = zeros(N, 1);  Q55L = zeros(N, 1);  Q66L = zeros(N, 1);  R22L = zeros(N, 1);

    for k = 1:N
        t_k = (k - 1) * Ts;

        % -- desired-height terms --
        h_d_k   = tr.h_d_seq(k);
        h_d_kp1 = tr.h_d_kp1_seq(k);
        dh_d_step  = h_d_k - pd_km1;      % Dh_d[k] = h_d[k]-h_d[k-1]   (backward, predict k-1->k)
        dH_d_dspan = h_d_k - pd_km2;      % DH_d[k] = h_d[k]-h_d[k-d]

        % -- d-step delayed measurement: dh_m[k] = dh[k-d] + n[k-d] --
        if k > d
            dh_m = tr.h_d_seq(k - d) - hnoisy_hist(k - d);
        else
            dh_m = 0;                     % sensor-buffer IC (hold, dh~=0)
        end

        % -- control gain (posterior[k-1]); guard the division only --
        a_ctrl = max(x(4), 1e-6);

        % -- Eq.17 (d-step delay compensated) control effort --
        sum_a_fd = a_km1 * f_km1 + a_km2 * f_km2;      % Sum_i a_hat[k-i] f_dh[k-i]  (d=2)
        f_dz = (1 / a_ctrl) * ( h_d_kp1 - lc * h_d_k - omlc * pd_km2 ...
                                + omlc * dh_m - omlc * sum_a_fd );

        % -- IIR variance channel -> a_hm (paper 2025 Eq.9-13; raw, y2) --
        dx_bar_m   = (1 - a_pd) * dx_bar_m + a_pd * dh_m;
        dx_r       = dh_m - dx_bar_m;
        sigma2_dxr = (1 - a_cov) * sigma2_dxr + a_cov * dx_r^2;
        a_hm = (sigma2_dxr - C_n * sig2n) / (C_dpmr * 4 * kBT);

        % -- wall curve at the MEASURED (delayed) h-bar for Q/R (no cheating) --
        if k > d
            hbar_meas = max(hnoisy_hist(k - d) / R, 1.001);
        else
            hbar_meas = hbar0;
        end
        [~, cp_m, drv_m] = calc_correction_functions(hbar_meas, true);
        K_h_m  = drv_m.K_h_perp;  K_hp_m = drv_m.K_h_prime_perp;
        a_perp_meas = a_nom / cp_m;
        sig2dh = 4 * kBT * a_perp_meas;

        a_hat_i = x(4);  ap_i = x(5);

        % -- Q (manuscript): rank-1 thermal Q(3:4) from eps_h; Q55/Q66 per arm --
        var_da_inc = vdaf * (a_hat_i * K_h_m / R)^2 * sig2dh;         % Var(delta_a_ram)[k] (R22 delay)
        eps_var = 4 * kBT * (a_hat_i + omlc^2 * (a_km1 + a_km2)) + omlc^2 * sig2n;  % Var(eps_h)
        Q = zeros(n_aug);
        Q(3,3) = eps_var;                 % q3 = -eps_h                    (= sigma_eps^2)
        Q(4,4) = ap_i^2 * eps_var;        % q4 = a'_hd * eps_h             (= a'_hd^2 sigma_eps^2)
        Q(3,4) = -ap_i * eps_var;  Q(4,3) = -ap_i * eps_var;   % shared sample (rank 1)
        Q55 = 0;  Q66_used = 0;
        if order == 1
            % 1st-order RW: thermal-grounded, time-varying Q55 (measured h-bar).
            a_pp_m     = -(a_hat_i / R^2) * (K_hp_m - K_h_m^2);     % a''(h) at measured h-bar
            var_dx_ram = (2 / (1 + lc)) * 4 * kBT * a_hat_i;        % Var(Delta x_ram)
            Q55 = knobs.Q55_arm1_scale * a_pp_m^2 * var_dx_ram;
            Q(5,5) = Q55;
        else
            % 2nd-order manuscript: Q55=0; Q66 per arm (oracle vs closure).
            Q(5,5) = 0;
            switch q66_mode
                case 'oracle'    % a'''_true(h_d[k])^2 * Dh_d[k]^2  (the ONE cheat)
                    Q66_used = (a3_hd(k) * dh_d_step)^2;
                case 'closure'   % (delta_a'_hd_hat[k] * Dh_d[k] / R)^2  (honest top-state closure)
                    Q66_used = (x(6) * dh_d_step / R)^2;
                otherwise        % 'floor' / 'na'
                    Q66_used = 0;
            end
            Q66_used = Q66_used + knobs.Q66_floor;     % optional tiny design floor (default 0)
            Q(6,6) = Q66_used;
        end

        % -- R (a_hm intrinsic color-inflation floor + d-step delay term) --
        IF_eff  = if_eff(IF_abc, C_dpmr, C_n, kBT, a_hat_i, sig2n);
        R2_int  = K_var * IF_eff * (a_hat_i + xi)^2;
        R22     = R2_int + r22f * (vda_km1 + vda_km2);

        % -- guards (canonical: G1 off since t_warmup=0, G3 off since h-bar>=2) --
        G1 = t_k < pm.t_warmup;
        G2 = (sigma2_dxr - C_n * sig2n) <= 0;      % NaN/low-SNR guard
        G3 = hbar_meas < pm.hbar_safe;
        gate_off = G1 || G2 || G3;

        % -- F_e (manuscript, time-varying) --
        F_dh  = f_dz + omlc * (f_km1 + f_km2);
        F_dhH = omlc * (dh_d_step * f_km1 + dH_d_dspan * f_km2);   % a'-into-dh3 coupling (row3 col5)
        Fe = build_Fe(order, lc, F_dh, F_dhH, dh_d_step, ap_i);

        % -- EKF predict (nonlinear mean; Fe propagates covariance) --
        xp = zeros(n_aug, 1);
        xp(1) = x(2);
        xp(2) = x(3);
        xp(3) = lc * x(3);
        xp(4) = x(4) + ap_i * dh_d_step + omlc * ap_i * x(3);      % Taylor gain advance
        if order == 1
            xp(5) = x(5);                                          % a' random walk
        else
            xp(5) = x(5) + dh_d_step * x(6);                       % a' pure sweep
            xp(6) = x(6);                                          % delta_a' frozen
        end
        Pp = Fe * Pc * Fe' + Q;  Pp = 0.5 * (Pp + Pp');

        % -- EKF update (manuscript H: raw a_hm, -DH_d*a'_hd kept in H(2,5)) --
        Hfull = zeros(2, n_aug);
        Hfull(1,1) = 1;  Hfull(2,4) = 1;  Hfull(2,5) = -dH_d_dspan;
        if gate_off
            Huse = Hfull(1,:);  yuse = dh_m;          Ruse = sig2n;
        else
            Huse = Hfull;       yuse = [dh_m; a_hm];  Ruse = [sig2n 0; 0 R22];
        end
        innov = yuse - Huse * xp;
        S = Huse * Pp * Huse' + Ruse;  S = 0.5 * (S + S');
        K = (Pp * Huse') / S;
        if G1;  K(4,:) = 0;  end
        x = xp + K * innov;
        ImKH = eye(n_aug) - K * Huse;
        Pc = ImKH * Pp * ImKH' + K * Ruse * K';  Pc = 0.5 * (Pc + Pc');

        % -- PLANT (exact 1-D z: z-mobility = 1/(gamma*c_perp); ode4 10us; thermal) --
        hbar_true = max(h_curr / R, 1.001);
        [~, cp_t] = calc_correction_functions(hbar_true);
        a_true(k) = a_nom / cp_t;                                  % gain at controlled position
        f_Tz  = sqrt(4 * kBT * gamma * cp_t / Ts) * noise.z_th(k); % Var matches calc_thermal_force
        h_curr = ode4_step_1d(h_curr, f_dz + f_Tz, pm);
        n_meas = sqrt(sig2n) * noise.z_meas(k);
        hnoisy_hist(k) = h_curr + n_meas;

        % -- logs --
        a_hat(k)  = x(4);  ap_hat(k) = x(5);
        if order == 2;  app_hat(k) = x(6);  end
        dh_track(k) = h_d_k - h_curr;
        hbar_meas_log(k) = hbar_meas;
        Q33L(k) = eps_var;  Q55L(k) = Q55;  Q66L(k) = Q66_used;  R22L(k) = R22;

        % -- shift buffers --
        pd_km2 = pd_km1;  pd_km1 = h_d_k;
        f_km2  = f_km1;   f_km1  = f_dz;
        a_km2  = a_km1;   a_km1  = a_ctrl;
        vda_km2 = vda_km1;  vda_km1 = var_da_inc;

        if verbose && mod(k, max(1, round(N/10))) == 0
            fprintf('   [arm %s] step %d/%d  t=%.2fs  a_hat=%.4f  a''_hat=%.4f nm/pN[/um]\n', ...
                    spec.id, k, N, t_k, x(4)*1e3, x(5)*1e3);
        end
    end

    res = struct('id', spec.id, 'label', spec.label, 'order', order, ...
                 'a_hat', a_hat, 'ap_hat', ap_hat, 'app_hat', app_hat, ...
                 'a_true', a_true, 'dh_track', dh_track, 'hbar_meas', hbar_meas_log, ...
                 'Q33', Q33L, 'Q55', Q55L, 'Q66', Q66L, 'R22', R22L);
end


%% ==================================================================
%  Local helpers
%% ==================================================================
function Fe = build_Fe(order, lc, F_dh, F_dhH, dh_d, ap)
%BUILD_FE  Manuscript 6state_curvature_hd.tex boxed F_e (order 2), or its
%   5-state (order 1) restriction (drop col/row 6). Cols: dh1 dh2 dh3 a_h a'_hd [delta_a'_hd].
    omlc = 1 - lc;
    if order == 1
        Fe = [0 1 0        0            0    ; ...
              0 0 1        0            0    ; ...
              0 0 lc      -F_dh         F_dhH; ...
              0 0 omlc*ap  1+ap*F_dh    dh_d ; ...
              0 0 0        0            1    ];
    else
        Fe = [0 1 0        0            0     0   ; ...
              0 0 1        0            0     0   ; ...
              0 0 lc      -F_dh         F_dhH 0   ; ...
              0 0 omlc*ap  1+ap*F_dh    dh_d  0   ; ...
              0 0 0        0            1     dh_d; ...
              0 0 0        0            0     1   ];
    end
end

function app = a2prime_at(h, pm)
%A2PRIME_AT  a''(h) = (a/R^2)(K_h^2 - K_h') at height h [um] (z-axis / c_perp).
    hb = max(h / pm.R, 1.001);
    [~, cp, drv] = calc_correction_functions(hb, true);
    app = (pm.a_nom / cp) / pm.R^2 * (drv.K_h_perp^2 - drv.K_h_prime_perp);
end

function a3 = a3prime_at(h, pm)
%A3PRIME_AT  a'''(h) via clean central finite difference of a''(h) along h.
    delta = 1e-3 * pm.R;                       % [um] small step vs R
    a3 = (a2prime_at(h + delta, pm) - a2prime_at(h - delta, pm)) / (2 * delta);
end

function a3 = a3prime_analytic(h, pm)
%A3PRIME_ANALYTIC  a'''(h) = (a_nom/R^3) D'''(h_bar), D''' = d/dh_bar of the
%   D_perp'' polynomial in calc_correction_functions (cross-check for the FD).
%   d(u^n)/dh_bar = -n u^(n+1); D_perp'' = -(9/4)u^3 +6u^5 -(57/5)u^6 +6u^7
%   +(231/50)u^13 -(156/25)u^14.
    hb = max(h / pm.R, 1.001);
    u  = 1 / hb;
    D3 = (27/4)*u^4 - 30*u^6 + (342/5)*u^7 - 42*u^8 ...
         - (3003/50)*u^14 + (2184/25)*u^15;
    a3 = (pm.a_nom / pm.R^3) * D3;
end

function IF = if_eff(abc, C_dpmr, C_n, kBT, a, snx)
%IF_EFF  Exact color-inflation factor for R22 (R22_derivation S4-S6).
    sxT = 4 * kBT * a;
    num = sxT^2 * abc(1) + 2 * sxT * snx * abc(2) + snx^2 * abc(3);
    den = (C_dpmr * sxT + C_n * snx)^2;
    IF  = 1 + 2 * num / den;
end

function P = dare_fp(F, H, Q, R)
%DARE_FP  Discrete KF Riccati steady state (fixed-point iteration).
    n = size(F, 1);  P = eye(n);
    for it = 1:20000
        Pp = F * P * F' + Q;  Pp = 0.5 * (Pp + Pp');
        S  = H * Pp * H' + R;
        K  = (Pp * H') / S;
        Pn = (eye(n) - K * H) * Pp;  Pn = 0.5 * (Pn + Pn');
        if max(abs(Pn(:) - P(:))) < 1e-14;  P = Pn;  return;  end
        P = Pn;
    end
end

function h = ode4_step_1d(h, F, pm)
%ODE4_STEP_1D  One Ts of dh/dt = F/(gamma*c_perp(h/R)); ode4, ~10us substep.
%   Exact 1-D reduction of step_dynamics for w_hat = z_hat (z-mobility diagonal).
    Ninner = max(1, round(pm.Ts / 10e-6));
    hh = pm.Ts / Ninner;
    for j = 1:Ninner
        k1 = z_rate(h,            F, pm);
        k2 = z_rate(h + hh/2*k1,  F, pm);
        k3 = z_rate(h + hh/2*k2,  F, pm);
        k4 = z_rate(h + hh*k3,    F, pm);
        h  = h + hh/6 * (k1 + 2*k2 + 2*k3 + k4);
    end
end

function v = z_rate(h, F, pm)
    hb = max(h / pm.R, 1.001);
    [~, cp] = calc_correction_functions(hb);
    v = F / (pm.gamma * cp);
end

function h = traj_height(tau, tr)
%TRAJ_HEIGHT  Wall-normal desired height [um] (hold->cosine descent->cosine osc->hold).
%   Mirrors trajectory_generator's phase formula (evaluated at absolute time tau).
    if tau <= tr.t1
        h = tr.h_init;
    elseif tau <= tr.t2
        td = tau - tr.t1;
        h = tr.h_bottom + (tr.h_init - tr.h_bottom) * (1 + cos(pi * td / (tr.t2 - tr.t1))) / 2;
    elseif tau <= tr.t3
        to = tau - tr.t2;
        h = (tr.h_bottom + tr.amplitude) - tr.amplitude * cos(2 * pi * tr.freq * to);
    else
        h = tr.h_bottom;
    end
end

function m = seg_metric(est, truth, win, per_samp, Ts, do_lag)
%SEG_METRIC  RMS / rel% / corr / lag for est vs truth over a window.
    e   = est(win) - truth(win);
    m.rms = rms(e);
    m.rel = m.rms / max(rms(truth(win)), realmin) * 100;
    if numel(unique(truth(win))) < 2 || numel(unique(est(win))) < 2
        m.corr = NaN;
    else
        cc = corrcoef(est(win), truth(win));  m.corr = cc(1,2);
    end
    if do_lag && sum(win) > 8
        L = xcorr_lag(truth(win), est(win), per_samp);   % + = est lags truth
        m.lag_ms = L * Ts * 1e3;
    else
        m.lag_ms = NaN;
    end
end

function L = xcorr_lag(ref, sig, maxlag)
    ref = ref(:) - mean(ref);  sig = sig(:) - mean(sig);
    best = 0;  bestc = -inf;
    for LL = -maxlag:maxlag
        if LL >= 0
            a = ref(1:end-LL);  b = sig(1+LL:end);
        else
            a = ref(1-LL:end);  b = sig(1:end+LL);
        end
        if numel(a) < 8;  continue;  end
        c = sum(a .* b) / sqrt(sum(a.^2) * sum(b.^2) + realmin);
        if c > bestc;  bestc = c;  best = LL;  end
    end
    L = best;
end

function print_metric_table(title_str, arms, ids, truth, seg, per_samp, Ts, kind)
%PRINT_METRIC_TABLE  Per-segment + whole-run RMS/rel/corr/lag for all arms.
    nm = 1e3;  segn = {'hold', 'desc', 'osc', 'all'};
    win_all = true(size(truth));
    fprintf('\n %s   (RMS/rel in %s, corr, lag ms; lag on osc only):\n', title_str, unit_of(kind));
    fprintf('  %-5s %-16s %-11s %-9s %-8s %-9s\n', 'seg', 'arm', 'RMS', 'rel[%]', 'corr', 'lag[ms]');
    for s = 1:numel(segn)
        switch segn{s}
            case 'hold';  win = seg.hold;  dl = false;
            case 'desc';  win = seg.desc;  dl = false;
            case 'osc';   win = seg.osc;   dl = true;
            otherwise;    win = win_all;   dl = true;
        end
        for a = 1:numel(arms)
            if strcmp(kind, 'again')
                tr_use = arms{a}.a_true;      % per-arm TRUE gain a(actual h)
            else
                tr_use = truth;               % shared a'(h_d) estimation target
            end
            m = seg_metric(get_est(arms{a}, kind), tr_use, win, per_samp, Ts, dl);
            fprintf('  %-5s %-16s %-11.4f %-9.1f %-8s %-9s\n', segn{s}, ids{a}, m.rms*nm, m.rel, ...
                    fmt_num(m.corr, '%.3f'), fmt_num(m.lag_ms, '%+.1f'));
        end
    end
end

function print_ratio_table(arms, ids, seg)
%PRINT_RATIO_TABLE  a_hat/a_true ratio per segment per arm (parity check).
    segn = {'hold', 'desc', 'osc', 'all'};
    fprintf('\n a-hat / a-true ratio (mean per segment):\n');
    fprintf('  %-5s', 'seg');
    for a = 1:numel(arms);  fprintf(' %-10s', ids{a});  end
    fprintf('\n');
    for s = 1:numel(segn)
        switch segn{s}
            case 'hold';  win = seg.hold;
            case 'desc';  win = seg.desc;
            case 'osc';   win = seg.osc;
            otherwise;    win = true(size(arms{1}.a_true));
        end
        fprintf('  %-5s', segn{s});
        for a = 1:numel(arms)
            r = mean(arms{a}.a_hat(win) ./ arms{a}.a_true(win));
            fprintf(' %-10.4f', r);
        end
        fprintf('\n');
    end
end

function print_track_table(arms, ids, seg)
%PRINT_TRACK_TABLE  Tracking-error std per segment (nm), all arms.
    segn = {'hold', 'desc', 'osc', 'all'};
    fprintf('\n tracking error  delta_h  std (nm):\n');
    fprintf('  %-5s', 'seg');
    for a = 1:numel(arms);  fprintf(' %-10s', ids{a});  end
    fprintf('\n');
    for s = 1:numel(segn)
        switch segn{s}
            case 'hold';  win = seg.hold;
            case 'desc';  win = seg.desc;
            case 'osc';   win = seg.osc;
            otherwise;    win = true(size(arms{1}.dh_track));
        end
        fprintf('  %-5s', segn{s});
        for a = 1:numel(arms)
            fprintf(' %-10.1f', std(arms{a}.dh_track(win))*1e3);
        end
        fprintf('\n');
    end
end

function print_q66_table(res2a, res2b, seg)
%PRINT_Q66_TABLE  Q66 used by 2a (oracle) vs 2b (closure): min/median/max.
    fprintf('\n Q66 used (oracle 2a vs closure 2b), min/median/max:\n');
    fprintf('  %-6s %-14s %-14s\n', 'seg', '2a oracle', '2b closure');
    segn = {'osc', 'all'};
    for s = 1:numel(segn)
        if strcmp(segn{s}, 'osc'); win = seg.osc; else; win = true(size(res2a.Q66)); end
        qa = res2a.Q66(win);  qb = res2b.Q66(win);
        fprintf('  %-6s [%.2e %.2e %.2e]  [%.2e %.2e %.2e]\n', segn{s}, ...
                min(qa), median(qa), max(qa), min(qb), median(qb), max(qb));
    end
    fprintf('  (ratio oracle/closure median on osc = %.2f)\n', ...
            median(res2a.Q66(seg.osc)) / max(median(res2b.Q66(seg.osc)), realmin));
end

function e = get_est(res, kind)
    if strcmp(kind, 'aprime');  e = res.ap_hat;  else;  e = res.a_hat;  end
end

function u = unit_of(kind)
    if strcmp(kind, 'aprime');  u = 'nm/pN/um';  else;  u = 'nm/pN';  end
end

function s = fmt_num(x, f)
    if isnan(x);  s = '--';  else;  s = sprintf(f, x);  end
end

function s = better(a2, a1)
    if a2 < a1;  s = 'YES (2a<1)';  else;  s = 'no (2a>=1)';  end
end

function s = cmp_hi(a, b)
    if a > b;  s = 'YES (2a>1)';  else;  s = 'no (2a<=1)';  end
end


%% ==================================================================
%  Comparison figure (lab convention: no grid/title, legend northoutside,
%  bold FS, true = thick black, arms distinct colors)
%% ==================================================================
function fig_path = make_comparison_figure(tout, tr, seg, oracle_a, oracle_ap, arms, labels, project_root)
    nm = 1e3;  FS = 17;
    col_true = [0 0 0];
    col = { [0 0.45 0.74], [0.85 0.33 0.10], [0.20 0.62 0.20] };   % 1st / oracle / closure
    lw_arm = 1.4;
    fig_dir = fullfile(project_root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if ~exist(fig_dir, 'dir');  mkdir(fig_dir);  end
    fig_path = fullfile(fig_dir, 'aprime_hd_oracle_compare.png');

    fig = figure('Color', 'w', 'Position', [80 80 1000 1080]);

    % ---- Panel a: a'_hat vs true a'(h_d) --------------------------------
    ax1 = subplot(3,1,1);  hold(ax1, 'on');
    h_true = plot(ax1, tout, oracle_ap*nm, 'Color', col_true, 'LineWidth', 2.8);
    h_arm = gobjects(numel(arms),1);
    for a = 1:numel(arms)
        h_arm(a) = plot(ax1, tout, arms{a}.ap_hat*nm, 'Color', col{a}, 'LineWidth', lw_arm);
    end
    mark_segments(ax1, tr);
    ylabel(ax1, 'a''_{hd}  [nm/pN/\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    style_axis(ax1, FS);
    yl = [min(oracle_ap*nm) max(oracle_ap*nm)];  pad = 0.15*max(diff(yl),eps);
    ylim(ax1, [yl(1)-pad, yl(2)+pad]);
    lg = legend([h_true; h_arm], [{'true a''(h_d)'}, labels(:)'], ...
                'Orientation', 'horizontal', 'Location', 'northoutside', 'FontSize', FS-3);
    lg.Box = 'off';
    text(ax1, 0.008, 0.90, '(a)', 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');

    % ---- Panel b: a_hat vs true a ---------------------------------------
    ax2 = subplot(3,1,2);  hold(ax2, 'on');
    plot(ax2, tout, oracle_a*nm, 'Color', col_true, 'LineWidth', 2.8);
    for a = 1:numel(arms)
        plot(ax2, tout, arms{a}.a_hat*nm, 'Color', col{a}, 'LineWidth', lw_arm);
    end
    mark_segments(ax2, tr);
    ylabel(ax2, 'a_{h}  [nm/pN]', 'FontSize', FS, 'FontWeight', 'bold');
    style_axis(ax2, FS);
    text(ax2, 0.008, 0.90, '(b)', 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');

    % ---- Panel c: tracking-error running std ----------------------------
    ax3 = subplot(3,1,3);  hold(ax3, 'on');
    win = max(1, round(0.15 / (tout(2)-tout(1))));   % ~0.15 s running window
    for a = 1:numel(arms)
        plot(ax3, tout, movstd(arms{a}.dh_track, win)*nm, 'Color', col{a}, 'LineWidth', lw_arm+0.2);
    end
    mark_segments(ax3, tr);
    ylabel(ax3, '\delta h  running std  [nm]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax3, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    style_axis(ax3, FS);
    text(ax3, 0.008, 0.90, '(c)', 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');

    linkaxes([ax1 ax2 ax3], 'x');  xlim(ax1, [0 tout(end)]);
    drawnow;
    exportgraphics(fig, fig_path, 'Resolution', 150);
    close(fig);
end

function mark_segments(ax, tr)
%MARK_SEGMENTS  Light dashed verticals at hold/descent/osc boundaries (no legend).
    yl = ylim(ax);
    for tb = [tr.t1, tr.t2, tr.t3]
        plot(ax, [tb tb], yl, '--', 'Color', [0.55 0.55 0.55], ...
             'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    ylim(ax, yl);
end

function style_axis(ax, FS)
    set(ax, 'FontSize', FS-2, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
end
