% DRAFT-NOT-RUN | text-only 稿, 從未執行過 (memory: project_taylor_ahat_dx3hat_verify_2026-07-13)
% 依「測試通過才 commit」規則: 跑通 Phase 1 之後才入版控。綁 C2/audit 線 (block C 待辦)。
function out = verify_audit_L1_theta(opts)
%VERIFY_AUDIT_L1_THETA  L1 layer of the C1/C2 unknown-c(h_bar) theta-fit audit.
%
%   out = verify_audit_L1_theta()
%   out = verify_audit_L1_theta(struct('seeds',1:3,'force_rerun',false))
%
%   OPEN-LOOP theta-fit verification: fit a parametric drag curve
%   a(h_bar; theta_hat) offline on the a_xm training stream harvested from the
%   KNOWN-c production 4-state controller (use_q44_ar1=true, osc_1hz). The fit
%   never feeds back (L1 is open-loop), so ablations are free and safe.
%
%   Layers (audit plan reference/eq17_analysis/c1c2_unknownc_audit_plan.md S4;
%   spec reference/eq17_analysis/derivation/4state_del_hd_thetafit.tex S6/S9;
%   basis oracle reference/eq17_analysis/L0_basis_oracle/):
%     PHASE A  collect (or load) 3 seeds of the known-c production log.
%     PHASE B  reconstruct a_xm offline from the logged delta_x_m (mirrors the
%              controller Eq.9-13 at motion_control_law_eq17_4state.m L332-335),
%              cross-check vs the logged diag.a_xm, and build the beta-corrected
%              variant a_xm/(1+beta_pred(h_bar)).
%     PHASE C  offline theta-RLS variants (basis / pairing / smoothing /
%              forgetting / kill-test-D / P_tilt).
%     PHASE D  the seven fig_L1_* figures + L1_notes.md + CSV export.
%
%   TEXT-ONLY authoring: this file is NOT run by the drafting session; the main
%   thread runs it via MATLAB/MCP. Every sim run clears the controller first
%   (persistent-state pollution lesson); the driver also clears internally.
%
%   Output dir: test_results/audit_c1c2/L1/ (gitignored). Idempotent: PHASE A
%   skips seeds whose .mat already exists unless opts.force_rerun=true.
%
%   See also: run_pure_simulation, motion_control_law_eq17_4state,
%             build_eq17_6state_constants, calc_correction_functions,
%             verify_eq17_4state

    if nargin < 1 || isempty(opts); opts = struct(); end
    K = l1_constants();
    if isfield(opts, 'seeds') && ~isempty(opts.seeds); K.SEEDS = opts.seeds(:).'; end
    if ~isfield(opts, 'force_rerun'); opts.force_rerun = false; end
    if ~isfield(opts, 'make_figs');   opts.make_figs   = true;  end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    paths = l1_paths(proj);
    ensure_dir(paths.root); ensure_dir(paths.data);

    % ------------------------------------------------------------------
    % PHASE A : data collection (idempotent per seed)
    % ------------------------------------------------------------------
    cfg = build_l1_cfg(K);
    D = struct([]);
    for si = 1:numel(K.SEEDS)
        seed = K.SEEDS(si);
        Ds = collect_or_load_seed(cfg, seed, paths, opts.force_rerun);
        if si == 1; D = Ds; else; D(si) = Ds; end %#ok<AGROW>
    end
    out.n_seeds = numel(D);
    out.diverged = arrayfun(@(x) x.diverged, D);

    % ------------------------------------------------------------------
    % PHASE B : offline a_xm reconstruction + cross-check + beta variant
    %   Build a fresh array (phase_b adds fields; in-place D(si)=... would
    %   trip "dissimilar structures" on the first element).
    % ------------------------------------------------------------------
    Db = struct([]);
    for si = 1:numel(D)
        ds = phase_b_reconstruct(D(si), K);
        if si == 1; Db = ds; else; Db(si) = ds; end %#ok<AGROW>
    end
    D = Db;
    out.recon_max_reldiff = max(arrayfun(@(x) x.recon_max_reldiff, D));

    % ------------------------------------------------------------------
    % PHASE C : theta-RLS variants (per axis)
    % ------------------------------------------------------------------
    R = run_all_variants(D, K);
    out.gateG1 = R.gateG1;

    % ------------------------------------------------------------------
    % PHASE D : figures + notes + CSV
    % ------------------------------------------------------------------
    if opts.make_figs
        make_all_figures(D, R, K, paths);
    end
    write_l1_notes(R, K, paths, out);
    write_l1_csv(D, R, K, paths);

    out.paths = paths;
    fprintf('[verify_audit_L1_theta] done. seeds=%d  recon max reldiff=%.2e  notes=%s\n', ...
            numel(D), out.recon_max_reldiff, paths.notes);
end


%% ====================================================================
%  Constants + paths + config
%% ====================================================================

function K = l1_constants()
%L1_CONSTANTS  Named constants (no magic numbers downstream).
    K = struct();
    % --- scenario (mirrors verify_eq17_4state build_cfg, the known-c prod cfg) ---
    K.SEEDS          = 1:3;
    K.H_INIT_UM      = 50;
    K.H_BOTTOM_UM    = 2.7;          % -> h_bar trough 1.2
    K.AMPLITUDE_UM   = 2.5;          % h_bar in [1.2, 3.42]
    K.FREQ_HZ        = 1.0;
    K.T_HOLD_S       = 0.5;
    K.T_DESCEND_S    = 1.0;
    K.T_SIM_S        = 4.0;
    K.N_CYCLES       = 2.0;          % 2 s osc at 1 Hz
    K.LAMBDA_C       = 0.7;
    K.A_PD           = 0.05;
    K.A_COV          = 0.05;
    K.MEAS_NOISE_STD = [0.00062; 0.00057; 0.00331];   % [um] per axis
    K.H_MIN_SCALE    = 1.05;         % h_min = 1.05*R
    K.H_BAR_SAFE_PROD = 1.0;         % production G3 unreachable (trough 1.2 > 1)

    % --- offline RLS training gate (audit S6.2 in-band discipline) ---
    K.H_GATE_LO      = 1.5;          % G3-style lower gate (h_bar >= 1.5)
    K.H_TRAIN_MAX    = 5.0;          % far-band cap h_train
    K.T_WARMUP_S     = 0.2;          % warm-up gate

    % --- measurement chain geometry ---
    K.D_DELAY        = 2;            % sensor delay [steps]
    K.L_EFF          = 19;           % EWMA window-centre lag = (1-a_cov)/a_cov

    % --- forgetting sweep (audit C4; S6 Ruling 4 working range) ---
    K.LAMBDA_F_SWEEP = [1, 0.9999, 0.999, 0.99];
    K.LAMBDA_F_BASE  = 0.999;        % baseline working value

    % --- bases + L0 dwell theta* targets + priors ---
    K.AXIS_KIND      = {'para','para','perp'};   % x,y -> c_para ; z -> c_perp
    K.AXIS_NAME      = {'x','y','z'};
    K.BASIS          = {'M2','M2','Z2'};          % xy=M2 ; z=Z2
    K.BASIS_FALLBACK = {'M1','M1','Z1'};          % ladder fallbacks
    K.THETA_STAR     = {[0.5393;0.0884],[0.5393;0.0884],[1.0620;0.1162]};
    K.THETA_STAR_FB  = {0.5770, 0.5770, 1.1249};  % M1 b1 / Z1 theta dwell
    % priors (S6.2 / S7.1): Z2 sum tight + split wide ; M2 b1 Faxen + b3 shrink
    K.PRIOR_Z2_SUM_MEAN = 9/8;       % 1.125 Brenner
    K.PRIOR_Z2_SUM_STD  = 0.075;
    K.PRIOR_Z2_SPLIT_STD = 0.15;
    K.PRIOR_M2_B1_MEAN  = 9/16;      % 0.5625 Faxen
    K.PRIOR_M2_B1_STD   = 0.05;
    K.PRIOR_M2_B3_STD   = 0.05;      % shrinkage toward 0
    K.PRIOR_Z1_MEAN     = 9/8;
    K.PRIOR_Z1_STD      = 0.10;
    K.PRIOR_M1_STD      = 0.05;

    % --- beta prediction (S9 / compute_L0.py S7) ---
    K.E_J2           = (1 - K.A_COV) * (2 - K.A_COV) / K.A_COV^2;   % = 741
    K.P_TILT_EPS     = 0.05;         % synthetic scale for P_tilt (C6)
    K.P_TILT_ASSUMED = 0.9;          % S9 flag v assumed value

    % --- scoring heights + grid ---
    K.HB_SCORE       = [1.2, 1.5, 2.0, 3.0, 3.42];
    K.HB_KEY         = [1.2, 1.5, 3.42];      % trough / gate / peak
    K.HB_TROUGH      = 1.2;
    K.HB_GRID        = linspace(1.2, 3.4222, 400);
    K.HB_GRID_FAR    = linspace(1.2, 22.2, 600);

    % --- kill-test-D window ---
    K.KILLD_WIN_S    = 0.25;

    % --- Gate G1 thresholds (audit S4) ---
    K.G1_LEVEL_TROUGH_MAX = 0.005;   % level@trough parameter contribution < 0.5%
    K.G1_SMOOTH_CORR_FACTOR = 2.0;   % smoothing-off corr same-sign, within 2x

    % --- figure style ---
    K.FS             = 18;
    K.LFS            = 13;
    K.AXLW           = 2.0;
    K.COL_TRUE       = [0.85 0.10 0.10];   % red
    K.COL_EST        = [0.00 0.30 0.75];   % blue
    K.COL_MEAS       = [0.40 0.70 0.90];   % light blue
    K.COL_ALT        = [0.90 0.55 0.10];   % orange (ablation "other")
    K.COL_GATE       = [0.90 0.90 0.90];   % gate shading
    K.DPI            = 150;
    K.SCATTER_STRIDE = 40;
end


function paths = l1_paths(proj)
    paths.root  = fullfile(proj, 'test_results', 'audit_c1c2', 'L1');
    paths.data  = fullfile(paths.root, 'data');
    paths.notes = fullfile(paths.root, 'L1_notes.md');
end


function cfg = build_l1_cfg(K)
%BUILD_L1_CFG  Known-c production 4-state osc_1hz config (use_q44_ar1=true).
%   Mirrors verify_eq17_4state.build_cfg + the AR(1) production gain model.
    cfg = user_config();
    cfg.eq17_variant       = '4state';
    cfg.use_q44_ar1        = true;               % AR(1) reverting-gain production
    cfg.trajectory_type    = 'osc';
    cfg.h_init             = K.H_INIT_UM;
    cfg.h_bottom           = K.H_BOTTOM_UM;
    cfg.amplitude          = K.AMPLITUDE_UM;
    cfg.frequency          = K.FREQ_HZ;
    cfg.n_cycles           = K.N_CYCLES;
    cfg.t_hold             = K.T_HOLD_S;
    cfg.t_descend_override = K.T_DESCEND_S;
    cfg.T_sim              = K.T_SIM_S;
    pc = physical_constants();
    cfg.h_min              = K.H_MIN_SCALE * pc.R;
    cfg.ctrl_enable        = true;
    cfg.thermal_enable     = true;
    cfg.meas_noise_enable  = true;
    cfg.lambda_c           = K.LAMBDA_C;
    cfg.a_pd               = K.A_PD;
    cfg.a_cov              = K.A_COV;
    cfg.meas_noise_std     = K.MEAS_NOISE_STD;
    cfg.suppress_xD        = true;
    cfg.h_bar_safe         = K.H_BAR_SAFE_PROD;  % EKF y2 stays on through trough
    cfg.use_am_lpf         = false;
    cfg.a_det              = 0.005;              % a_m_det EWMA weight (S6 C3)
end


%% ====================================================================
%  PHASE A : data collection
%% ====================================================================

function Ds = collect_or_load_seed(cfg, seed, paths, force_rerun)
    matfile_path = fullfile(paths.data, sprintf('L1_data_seed%02d.mat', seed));
    if exist(matfile_path, 'file') && ~force_rerun
        S = load(matfile_path);
        Ds = S.Ds;
        return;
    end

    % persistent-state hygiene (driver also clears; belt-and-suspenders)
    clear motion_control_law_eq17_4state trajectory_generator calc_thermal_force;

    ro = struct('seed', seed, 'verbose', false, 'collect_diag', true, ...
                'use_true_gain', false);
    s = run_pure_simulation(cfg, ro);

    P = s.meta.params_value;
    w_hat = P.wall.w_hat(:);  pz = P.wall.pz;  R = P.common.R;
    a_nom = P.ctrl.Ts / P.ctrl.gamma;

    Ds = struct();
    Ds.seed   = seed;
    Ds.t      = s.tout(:);                        % [N x 1]
    Ds.p_d    = s.p_d_out;                        % [N x 3]
    Ds.p_m    = s.p_m_out;                        % [N x 3]
    Ds.a_true = s.a_true_out;                     % [N x 3] oracle (scoring only)
    Ds.a_hat  = s.diag.a_hat;                     % [N x 3] known-c EKF posterior
    Ds.a_xm   = s.diag.a_xm;                      % [N x 3] raw a_xm (SSOT log)
    Ds.a_m_det = s.diag.a_m_det;                  % [N x 3] EWMA(0.005) smoothed
    Ds.a_x_det = s.diag.a_x_det;                  % [N x 3] true-c feedforward level
    Ds.delta_x_m = s.diag.delta_x_m;              % [N x 3] p_d[k-d]-p_m[k]
    Ds.hbar_meas = s.diag.h_bar(:);               % [N x 1] from p_m (production G3)
    Ds.gate_prod = s.diag.gate_active(:,1) | s.diag.gate_active(:,2) | s.diag.gate_active(:,3);
    Ds.hbar_d = (s.p_d_out * w_hat - pz) / R;     % [N x 1] desired height
    % physical desired wall-normal position h_d [um] and its acceleration
    Ds.h_d    = s.p_d_out * w_hat - pz;           % [N x 1] um
    Ds.a_nom  = a_nom;
    Ds.R      = R;
    Ds.Ts     = P.common.Ts;
    Ds.p0     = P.common.p0(:);
    Ds.w_hat  = w_hat; Ds.pz = pz;
    Ds.ctrl_const = s.ctrl_const;                 % C_dpmr,C_n,K_var,IF_abc,xi,kBT,...

    % divergence flag (tracking error blow-up)
    e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);
    Ds.max_abs_err_um = max(abs(e(:)));
    Ds.diverged = Ds.max_abs_err_um > 0.5;

    save(matfile_path, 'Ds', '-v7.3');
    if Ds.diverged
        warning('verify_audit_L1_theta:diverged', ...
                'seed %d diverged (|e|_max=%.3g um); data saved but flagged.', ...
                seed, Ds.max_abs_err_um);
    end
end


%% ====================================================================
%  PHASE B : offline a_xm reconstruction + beta variant
%% ====================================================================

function Ds = phase_b_reconstruct(Ds, K)
%PHASE_B_RECONSTRUCT  Mirror controller Eq.9-13 (L332-335) from logged
%   delta_x_m; cross-check vs diag.a_xm; build beta-corrected stream.
    cc = Ds.ctrl_const;
    N  = size(Ds.delta_x_m, 1);
    a_pd = cc.a_pd; a_cov = cc.a_cov; C_dpmr = cc.C_dpmr; C_n = cc.C_n; kBT = cc.kBT;
    s2n  = cc.sigma2_n_s(:);

    % prefill init identical to controller L220/L226 (dx_bar_m=0 ; sigma2_dxr
    % prefilled at wall-aware a_x_init). a_x_init = a_nom/c(h_bar_init).
    a_x_init = wall_aware_a_init(Ds);
    dx_bar_m   = zeros(3,1);
    sigma2_dxr = 4*kBT*a_x_init*C_dpmr + C_n*s2n;   % [3x1] prefill

    a_xm_recon = zeros(N,3);
    % controller returns a_xm=0 on the first (init) call; per-step from k=2.
    for k = 2:N
        dxm = Ds.delta_x_m(k,:).';                  % logged delta_x_m[k]
        dx_bar_m   = (1-a_pd)*dx_bar_m + a_pd*dxm;   % L332
        dx_r       = dxm - dx_bar_m;                 % L333
        sigma2_dxr = (1-a_cov)*sigma2_dxr + a_cov*dx_r.^2;   % L334
        a_xm_recon(k,:) = ((sigma2_dxr - C_n*s2n) / (C_dpmr*4*kBT)).';   % L335
    end
    Ds.a_xm_recon = a_xm_recon;

    % cross-check vs logged diag.a_xm, over the settled region (k > 2/a_cov)
    settle = min(N, round(2/a_cov) + K.D_DELAY);
    idx = settle:N;
    denom = max(abs(Ds.a_xm(idx,:)), 1e-30);
    reld  = abs(a_xm_recon(idx,:) - Ds.a_xm(idx,:)) ./ denom;
    Ds.recon_max_reldiff = max(reld(:));

    % ---- beta-corrected variant (uses running-fit K_h computed in PHASE C;
    %      here store the physics inputs so PHASE C can build beta per basis) ----
    Ds.hddot_d = second_diff(Ds.h_d, Ds.Ts);        % [N x 1] um/s^2 (desired accel)
end


function a_init = wall_aware_a_init(Ds)
    h_bar_init = max((dot(Ds.p0, Ds.w_hat) - Ds.pz) / Ds.R, 1.001);
    [cp0, cq0] = calc_correction_functions(h_bar_init);
    a_init = [Ds.a_nom/cp0; Ds.a_nom/cp0; Ds.a_nom/cq0];
end


function y = second_diff(x, Ts)
%SECOND_DIFF  central second difference d2x/dt2 ; endpoints zero.
    x = x(:);  N = numel(x);  y = zeros(N,1);
    if N >= 3
        y(2:N-1) = (x(3:N) - 2*x(2:N-1) + x(1:N-2)) / Ts^2;
    end
end


%% ====================================================================
%  PHASE C : theta-RLS variants
%% ====================================================================

function R = run_all_variants(D, K)
%RUN_ALL_VARIANTS  Baseline + C1..C6 ablations per axis, all seeds.
    nS = numel(D);
    nA = 3;
    R = struct();
    R.axis = struct([]);

    for ax = 1:nA
        basis = K.BASIS{ax};
        basis_fb = K.BASIS_FALLBACK{ax};
        kind  = K.AXIS_KIND{ax};
        [theta0, P0]     = make_prior(basis, K);
        [theta0_fb, P0_fb] = make_prior(basis_fb, K);
        rc = axis_rls_const(D(1).ctrl_const, ax);

        AX = struct();
        AX.name = K.AXIS_NAME{ax};
        AX.basis = basis; AX.kind = kind;
        AX.theta_star = K.THETA_STAR{ax};

        % per-seed baseline (P2 window pairing, raw a_xm, lambda_f base)
        base = struct([]);
        for si = 1:nS
            v = fit_variant(D(si), ax, basis, theta0, P0, K.LAMBDA_F_BASE, ...
                            'a_xm', K.D_DELAY + K.L_EFF, 'all', K, rc);
            if si==1; base = v; else; base(si) = v; end %#ok<AGROW>
        end
        AX.base = base;

        % ---- beta stream (seed 1) from baseline converged theta_hat ----
        beta1 = build_beta(D(1), ax, base(1).theta_final, basis, K);
        AX.beta1 = beta1;

        % ---- theta_hat(t) convergence time (seed 1, param combo within 5%) ----
        AX.t_conv = conv_time(base(1).theta_hist, base(1).theta_final, base(1).t, basis);

        % ---- C3 smoothing ablation (raw vs a_m_det) seed 1 ----
        v_raw  = base(1);
        v_smooth = fit_variant(D(1), ax, basis, theta0, P0, K.LAMBDA_F_BASE, ...
                               'a_m_det', K.D_DELAY + K.L_EFF, 'all', K, rc);
        AX.smooth = struct('raw', v_raw, 'smooth', v_smooth);
        AX.corr_raw    = stream_true_corr(D(1), ax, 'a_xm',    K.D_DELAY + K.L_EFF, K);
        AX.corr_smooth = stream_true_corr(D(1), ax, 'a_m_det', K.D_DELAY + K.L_EFF, K);

        % ---- C2 pairing ablation x segment (seed 1) ----
        pair_lags  = [K.D_DELAY, K.D_DELAY + K.L_EFF, K.D_DELAY + K.L_EFF];
        pair_stream = {'a_xm','a_xm','a_xm_beta'};
        pair_name  = {'naive_k-d','window_k-d-19','beta+recentred'};
        seg_name   = {'all','descent','osc'};
        pair = struct();
        pair.name = pair_name;
        pair.seg  = seg_name;
        pair.theta = nan(numel(pair_name), numel(seg_name), numel(theta0));
        for pp = 1:numel(pair_name)
            Duse = D(1);
            if strcmp(pair_stream{pp}, 'a_xm_beta')
                Duse.a_xm_beta = D(1).a_xm ./ (1 + beta1);
                strm = 'a_xm_beta';
            else
                strm = 'a_xm';
            end
            for ss = 1:numel(seg_name)
                v = fit_variant(Duse, ax, basis, theta0, P0, K.LAMBDA_F_BASE, ...
                                strm, pair_lags(pp), seg_name{ss}, K, rc);
                pair.theta(pp,ss,:) = v.theta_final(:).';
            end
        end
        AX.pair = pair;

        % ---- C4 forgetting sweep (all seeds) ----
        nF = numel(K.LAMBDA_F_SWEEP);
        fbias = nan(nF, numel(theta0));
        fspread = nan(nF, numel(theta0));
        fpred  = nan(nF, 1);
        for fi = 1:nF
            lf = K.LAMBDA_F_SWEEP(fi);
            tf = nan(nS, numel(theta0));  Pf = nan(nS,1);
            for si = 1:nS
                v = fit_variant(D(si), ax, basis, theta0, P0, lf, ...
                                'a_xm', K.D_DELAY + K.L_EFF, 'all', K, rc);
                tf(si,:) = v.theta_final(:).';
                Pf(si)   = v.P_final(1,1);   % lead-parameter posterior var
            end
            fbias(fi,:)   = mean(tf,1) - AX.theta_star(:).';
            fspread(fi,:) = std(tf,0,1);
            fpred(fi)     = sqrt(mean(Pf));  % S6 closed-form posterior std proxy
        end
        AX.forget = struct('lambda_f',K.LAMBDA_F_SWEEP, 'bias',fbias, ...
                           'spread',fspread, 'pred',fpred);

        % ---- C1 basis fallback (seed 1) ----
        v_fb = fit_variant(D(1), ax, basis_fb, theta0_fb, P0_fb, K.LAMBDA_F_BASE, ...
                           'a_xm', K.D_DELAY + K.L_EFF, 'all', K, rc);
        AX.fallback = v_fb; AX.basis_fb = basis_fb; AX.theta_star_fb = K.THETA_STAR_FB{ax};

        % ---- C5 kill-test D (windowed local regression) seed 1 ----
        AX.killD = killtest_D(D(1), ax, kind, K);

        % ---- C6 P_tilt (inject +eps scale, refit) seed 1 ----
        Dtilt = D(1); Dtilt.a_xm = D(1).a_xm * (1 + K.P_TILT_EPS);
        v_tilt = fit_variant(Dtilt, ax, basis, theta0, P0, K.LAMBDA_F_BASE, ...
                             'a_xm', K.D_DELAY + K.L_EFF, 'all', K, rc);
        AX.ptilt = measure_ptilt(base(1).theta_final, v_tilt.theta_final, basis, ...
                                 D(1).a_nom, K);

        % ---- scoring + Gate G1 (baseline, seed-mean theta) ----
        tf_mean = mean(cell2mat(arrayfun(@(v) v.theta_final(:).', base, 'uni', 0).'), 1).';
        AX.theta_mean = tf_mean;
        AX.score = score_fit(basis, tf_mean, AX.theta_star, kind, D(1).a_nom, K);
        AX.theta_spread = std(cell2mat(arrayfun(@(v) v.theta_final(:).', base, 'uni', 0).'), 0, 1).';
        AX.posterior_std = sqrt(diag(base(1).P_final));

        % ---- reference curves for figures ----
        AX.grid = K.HB_GRID;
        AX.a_true_grid  = arrayfun(@(h) true_a(kind,h,D(1).a_nom), K.HB_GRID);
        AX.a_model_grid = arrayfun(@(h) model_a(basis,tf_mean,h,D(1).a_nom), K.HB_GRID);
        AX.a_star_grid  = arrayfun(@(h) model_a(basis,AX.theta_star,h,D(1).a_nom), K.HB_GRID);
        % beta-shifted theta* (offline dwell fit to (1+beta)*a_true)
        AX.theta_star_beta = fit_beta_shifted_target(D(1), ax, basis, kind, ...
                                                     base(1).theta_final, theta0, K);

        if ax==1; R.axis = AX; else; R.axis(ax) = AX; end %#ok<AGROW>
    end

    % ---- Gate G1 verdict per axis ----
    R.gateG1 = eval_gate_g1(R, K);
end


function rc = axis_rls_const(cc, ax)
%AXIS_RLS_CONST  per-axis scalar bundle for R_RLS (S6.2).
    rc = struct('IF_abc', cc.IF_abc(:), 'C_dpmr', cc.C_dpmr, 'C_n', cc.C_n, ...
                'kBT', cc.kBT, 'K_var', cc.K_var, ...
                'xi', cc.xi_per_axis(ax), 'sigma2_nx', cc.sigma2_n_s(ax));
end


function [theta0, P0] = make_prior(basis, K)
%MAKE_PRIOR  Gaussian prior mean + covariance as pseudo-observations (S6.2).
%   Priors are axis-independent (xy shares c_para, z uses c_perp) so only the
%   basis selects the prior. Z2 uses the sum-tight + split-wide pseudo-obs;
%   M2 uses Faxen b1 + b3 shrinkage; Z1/M1 are the 1-param fallbacks.
    switch basis
        case 'Z2'   % sum tight + split wide
            Hpr = [1 1; 0 1];
            varpr = [K.PRIOR_Z2_SUM_STD^2; K.PRIOR_Z2_SPLIT_STD^2];
            Lam = Hpr' * diag(1./varpr) * Hpr;
            P0  = Lam \ eye(size(Lam,1));   % prior covariance = inv(precision)
            theta0 = [K.PRIOR_Z2_SUM_MEAN; 0];   % sum=9/8, split=0
        case 'M2'
            P0 = diag([K.PRIOR_M2_B1_STD^2, K.PRIOR_M2_B3_STD^2]);
            theta0 = [K.PRIOR_M2_B1_MEAN; 0];
        case 'Z1'
            P0 = K.PRIOR_Z1_STD^2;
            theta0 = K.PRIOR_Z1_MEAN;
        case 'M1'
            P0 = K.PRIOR_M1_STD^2;
            theta0 = K.PRIOR_M2_B1_MEAN;
        otherwise
            error('verify_audit_L1_theta:badBasis', 'unknown basis %s', basis);
    end
    theta0 = theta0(:);   % keep column
end


function v = fit_variant(Ds, ax, basis, theta0, P0, lambda_f, stream_name, ...
                         lag, segment, K, rc)
%FIT_VARIANT  One causal theta-RLS pass. Returns theta_hist / final / P_final.
    N = size(Ds.a_xm, 1);
    a_stream = get_stream(Ds, ax, stream_name);
    hbar_reg = shifted_hbar(Ds.hbar_d, lag);            % [N x 1] regressor height
    mask = train_mask(Ds, hbar_reg, segment, K);
    v = run_theta_rls(a_stream, hbar_reg, mask, basis, theta0, P0, lambda_f, ...
                      rc, Ds.a_nom);
    v.t = Ds.t;
    v.mask = mask;
    v.hbar_reg = hbar_reg;
    v.basis = basis;
end


function a_stream = get_stream(Ds, ax, stream_name)
    switch stream_name
        case 'a_xm';      a_stream = Ds.a_xm(:,ax);
        case 'a_m_det';   a_stream = Ds.a_m_det(:,ax);
        case 'a_xm_beta'; a_stream = Ds.a_xm_beta(:,ax);
        otherwise; error('verify_audit_L1_theta:badStream','%s', stream_name);
    end
end


function hbr = shifted_hbar(hbar_d, lag)
    N = numel(hbar_d);
    hbr = nan(N,1);
    if lag < N; hbr(lag+1:N) = hbar_d(1:N-lag); end
end


function mask = train_mask(Ds, hbar_reg, segment, K)
%TRAIN_MASK  in-band training discipline: warm-up + [H_GATE_LO, H_TRAIN_MAX]
%   on the paired regressor height, restricted to the requested segment.
    N = numel(hbar_reg);
    m = (Ds.t >= K.T_WARMUP_S) & (hbar_reg >= K.H_GATE_LO) & (hbar_reg <= K.H_TRAIN_MAX);
    m(~isfinite(hbar_reg)) = false;
    t_osc0 = K.T_HOLD_S + K.T_DESCEND_S;
    switch segment
        case 'all';     seg = true(N,1);
        case 'descent'; seg = (Ds.t >= K.T_HOLD_S) & (Ds.t < t_osc0);
        case 'osc';     seg = (Ds.t >= t_osc0);
        otherwise; error('verify_audit_L1_theta:badSeg','%s', segment);
    end
    mask = m & seg;
end


function v = run_theta_rls(a_stream, hbar_reg, use_mask, basis, theta0, P0, ...
                           lambda_f, rc, a_nom)
%RUN_THETA_RLS  Exponential-forgetting linearized (EKF-form) RLS, Gaussian
%   prior init, R_RLS = K_var*IF_eff*(a_pred+xi)^2 (S6.2). Forgetting applied
%   only on used samples (event-based); a_pred is the RLS OWN estimate (never
%   reads the EKF a_hat -- S6.5 forbidden inputs).
    N = numel(a_stream);
    nth = numel(theta0);
    theta = theta0(:);  P = P0;
    theta_hist = nan(N, nth);
    n_upd = 0;  t_first = NaN;
    for k = 1:N
        if use_mask(k) && isfinite(a_stream(k)) && isfinite(hbar_reg(k)) && hbar_reg(k) > 1.0
            h = hbar_reg(k);
            [a_pred, ~, jac] = model_eval(basis, theta, h, a_nom);
            a_for_R = max(a_pred, eps);
            IF = if_eff_eval(rc.IF_abc, rc.C_dpmr, rc.C_n, rc.kBT, a_for_R, rc.sigma2_nx);
            Rk = max(rc.K_var * IF * (max(a_pred,0) + rc.xi)^2, eps);
            Ppred = P / lambda_f;
            phi = jac(:);
            S = phi' * Ppred * phi + Rk;
            Kg = (Ppred * phi) / S;
            innov = a_stream(k) - a_pred;
            theta = theta + Kg * innov;
            P = (eye(nth) - Kg*phi') * Ppred;
            P = 0.5*(P + P');
            n_upd = n_upd + 1;
            if isnan(t_first); t_first = k; end
        end
        theta_hist(k,:) = theta.';
    end
    v = struct('theta_hist',theta_hist, 'theta_final',theta, 'P_final',P, ...
               'n_upd',n_upd, 'first_upd',t_first);
end


function c = stream_true_corr(Ds, ax, stream_name, lag, K)
%STREAM_TRUE_CORR  Pearson corr between the (paired) stream and a_true along
%   the in-band osc samples (smoothing-off sign check, S4 Gate G1).
    a_stream = get_stream(Ds, ax, stream_name);
    hbar_reg = shifted_hbar(Ds.hbar_d, lag);
    mask = train_mask(Ds, hbar_reg, 'all', K);
    kind = K.AXIS_KIND{ax};
    at = arrayfun(@(h) true_a(kind, max(h,1.001), Ds.a_nom), hbar_reg);
    idx = mask & isfinite(a_stream) & isfinite(at);
    if nnz(idx) < 10; c = NaN; return; end
    cc = corrcoef(a_stream(idx), at(idx));
    c = cc(1,2);
end


function beta = build_beta(Ds, ax, theta_ref, basis, K)
%BUILD_BETA  beta_pred[k] = 0.5*|K_h(h_paired)|/R*|hddot(t_paired)|*E_j2*Ts^2,
%   K_h from the pass-1 converged fitted curve (weak self-reference; L1
%   open-loop so safe). Paired index = k-d-L_eff (window centre).
    N = size(Ds.a_xm,1);
    lag = K.D_DELAY + K.L_EFF;
    beta = zeros(N,1);
    for k = lag+1:N
        j = k - lag;
        h = max(Ds.hbar_d(j), 1.001);
        [~,~,~,Kh] = model_eval(basis, theta_ref, h, Ds.a_nom);
        beta(k) = 0.5 * abs(Kh)/Ds.R * abs(Ds.hddot_d(j)) * K.E_J2 * Ds.Ts^2;
    end
end


function kd = killtest_D(Ds, ax, kind, K)
%KILLTEST_D  Windowed local linear regression of a_xm on h_bar_meas -> local
%   slope vs a'_true(h_bar) (da/dh_bar). Expected far-field failure.
    Ts = Ds.Ts;
    win = max(5, round(K.KILLD_WIN_S/Ts));
    step = max(1, round(win/4));
    a_xm = Ds.a_xm(:,ax);
    hbm  = Ds.hbar_meas;
    t    = Ds.t;
    t_osc0 = K.T_HOLD_S;   % include descent + osc (moving h)
    idx0 = find(t >= t_osc0, 1, 'first');
    N = numel(a_xm);
    centers = []; slopes = []; aptrue = [];
    for k0 = idx0:step:(N-win)
        seg = k0:(k0+win-1);
        x = hbm(seg); y = a_xm(seg);
        good = isfinite(x) & isfinite(y);
        if nnz(good) < win/2 || (max(x(good)) - min(x(good))) < 1e-6; continue; end
        p = polyfit(x(good), y(good), 1);   % local d(a_xm)/d(h_bar)
        hc = mean(x(good));
        centers(end+1,1) = hc;               %#ok<AGROW>
        slopes(end+1,1)  = p(1);             %#ok<AGROW>
        [~, apt] = true_a(kind, max(hc,1.001), Ds.a_nom);
        aptrue(end+1,1)  = apt;              %#ok<AGROW>
    end
    kd = struct('hbar', centers, 'slope_local', slopes, 'aprime_true', aptrue);
    if ~isempty(centers)
        near = centers < 2.0; far = centers >= 2.0;
        kd.sign_agree_near = mean(sign(slopes(near)) == sign(aptrue(near)));
        kd.sign_agree_far  = mean(sign(slopes(far))  == sign(aptrue(far)));
    else
        kd.sign_agree_near = NaN; kd.sign_agree_far = NaN;
    end
end


function pt = measure_ptilt(theta_base, theta_tilt, basis, a_nom, K)
%MEASURE_PTILT  induced level change per unit injected scale (resolves S9 v).
    hk = K.HB_KEY;
    ptvals = nan(size(hk));
    for i = 1:numel(hk)
        a_b = model_a(basis, theta_base, hk(i), a_nom);
        a_t = model_a(basis, theta_tilt, hk(i), a_nom);
        ptvals(i) = (a_t/a_b - 1) / K.P_TILT_EPS;
    end
    pt = struct('hbar', hk, 'P_tilt', ptvals, 'assumed', K.P_TILT_ASSUMED);
end


function theta_beta = fit_beta_shifted_target(Ds, ax, basis, kind, theta_ref, theta0, K)
%FIT_BETA_SHIFTED_TARGET  offline dwell fit of the model to (1+beta)*a_true.
%   Predicts where a bias-uncorrected RLS should land (fig_L1_ablation_beta ref).
    t_osc0 = K.T_HOLD_S + K.T_DESCEND_S;
    idx = (Ds.t >= t_osc0) & (Ds.hbar_d >= K.H_GATE_LO) & (Ds.hbar_d <= K.H_TRAIN_MAX);
    hbv = Ds.hbar_d(idx);
    if numel(hbv) < 10; theta_beta = theta_ref; return; end
    at = arrayfun(@(h) true_a(kind, max(h,1.001), Ds.a_nom), hbv);
    % beta at these dwell heights (same physics as build_beta, height-only)
    beta = zeros(size(hbv));
    for i = 1:numel(hbv)
        [~,~,~,Kh] = model_eval(basis, theta_ref, max(hbv(i),1.001), Ds.a_nom);
        hddot = K.AMPLITUDE_UM * (2*pi*K.FREQ_HZ)^2;   % dwell |hddot| envelope
        beta(i) = 0.5*abs(Kh)/Ds.R * hddot * K.E_J2 * Ds.Ts^2;
    end
    target = (1 + beta) .* at;
    theta_beta = fit_offline(basis, hbv, target, ones(size(hbv)), Ds.a_nom, theta0);
end


%% ====================================================================
%  Scoring + Gate G1
%% ====================================================================

function sc = score_fit(basis, theta_final, theta_star, kind, a_nom, K)
%SCORE_FIT  level / slope / parameter-channel error at HB_SCORE.
    hk = K.HB_SCORE;  n = numel(hk);
    sc = struct('hbar', hk, 'level', nan(1,n), 'slope', nan(1,n), ...
                'param', nan(1,n), 'struct_floor', nan(1,n));
    for i = 1:n
        h = hk(i);
        [am, apm] = model_curve_pt(basis, theta_final, h, a_nom);
        [as, ~]   = model_curve_pt(basis, theta_star, h, a_nom);
        [at, apt] = true_a(kind, h, a_nom);
        sc.level(i)        = am/at - 1;
        sc.slope(i)        = apm/apt - 1;
        sc.param(i)        = am/as - 1;         % parameter channel
        sc.struct_floor(i) = as/at - 1;         % L0 structure floor
    end
end


function g = eval_gate_g1(R, K)
%EVAL_GATE_G1  audit S4 Gate G1 criteria per axis.
    nA = numel(R.axis);
    g = struct([]);
    for ax = 1:nA
        AX = R.axis(ax);
        sc = AX.score;
        it = find(abs(sc.hbar - K.HB_TROUGH) < 1e-6, 1);
        param_trough = abs(sc.param(it));
        floor_trough = abs(sc.struct_floor(it));
        crit_param_le_floor = param_trough <= floor_trough + 1e-9;
        crit_level_trough   = param_trough < K.G1_LEVEL_TROUGH_MAX;
        corr_ok = same_sign_within(AX.corr_raw, AX.corr_smooth, K.G1_SMOOTH_CORR_FACTOR);
        % spread consistent with posterior (lead param)
        spread_ok = AX.theta_spread(1) <= K.G1_SMOOTH_CORR_FACTOR * AX.posterior_std(1) + 1e-12;
        gi = struct('axis',AX.name, ...
                    'param_trough',param_trough, 'floor_trough',floor_trough, ...
                    'crit_param_le_floor',crit_param_le_floor, ...
                    'crit_level_trough',crit_level_trough, ...
                    'corr_raw',AX.corr_raw, 'corr_smooth',AX.corr_smooth, ...
                    'crit_corr',corr_ok, ...
                    'theta_spread',AX.theta_spread(1), 'posterior_std',AX.posterior_std(1), ...
                    'crit_spread',spread_ok, ...
                    'pass', crit_param_le_floor && crit_level_trough && corr_ok && spread_ok);
        if ax==1; g = gi; else; g(ax) = gi; end %#ok<AGROW>
    end
end


function ok = same_sign_within(a, b, factor)
    if ~isfinite(a) || ~isfinite(b); ok = false; return; end
    ok = (sign(a) == sign(b)) && (abs(b) <= factor*abs(a)) && (abs(a) <= factor*abs(b));
end


function tc = conv_time(theta_hist, theta_final, t, basis)
%CONV_TIME  time after which the plotted param combo stays within 5% of final.
    yf = param_combo_scalar(basis, theta_final);
    y  = param_combo(basis, theta_hist);
    within = abs(y - yf) <= 0.05*abs(yf) + eps;
    bad = find(~within);
    if isempty(bad); tc = t(1); return; end
    li = bad(end);
    if li >= numel(t); tc = NaN; else; tc = t(li+1); end
end


%% ====================================================================
%  Model + truth closures
%% ====================================================================

function [a, ap, jac, Kh] = model_eval(basis, theta, hb, a_nom)
%MODEL_EVAL  a(h;theta), da/dh_bar, da/dtheta (jac, nth x 1), K_h=-a'/a.
    hb = max(hb, 1.001);
    switch basis
        case 'Z2'
            p1 = 1/(hb-1); p2 = 1/hb;
            c  = 1 + theta(1)*p1 + theta(2)*p2;
            a  = a_nom/c;
            jac = [-a_nom*p1/c^2; -a_nom*p2/c^2];
            cp = theta(1)*(-1/(hb-1)^2) + theta(2)*(-1/hb^2);
            ap = -a_nom*cp/c^2;
        case 'Z1'
            p1 = 1/(hb-1);
            c  = 1 + theta(1)*p1;
            a  = a_nom/c;
            jac = -a_nom*p1/c^2;
            cp = theta(1)*(-1/(hb-1)^2);
            ap = -a_nom*cp/c^2;
        case 'M2'
            u = 1/hb;
            a = a_nom*(1 - theta(1)*u - theta(2)*u^3);
            jac = [-a_nom*u; -a_nom*u^3];
            ap = a_nom/hb^2*(theta(1) + 3*theta(2)*u^2);
        case 'M1'
            u = 1/hb;
            a = a_nom*(1 - theta(1)*u);
            jac = -a_nom*u;
            ap = a_nom*theta(1)/hb^2;
        otherwise
            error('verify_audit_L1_theta:badBasis','%s', basis);
    end
    Kh = -ap/a;
end


function a = model_a(basis, theta, hb, a_nom)
    [a,~,~] = model_eval(basis, theta, hb, a_nom);
end


function [a, ap] = model_curve_pt(basis, theta, hb, a_nom)
    [a, ap, ~] = model_eval(basis, theta, hb, a_nom);
end


function [a, ap] = true_a(kind, hb, a_nom)
%TRUE_A  analytic a_true / a'_true (da/dh_bar) via calc_correction_functions SSOT.
    hb = max(hb, 1.001);
    [cp, cq, dv] = calc_correction_functions(hb, true);
    if strcmp(kind,'perp'); c = cq; Kh = dv.K_h_perp; else; c = cp; Kh = dv.K_h_para; end
    a  = a_nom/c;
    ap = -Kh*a;
end


function theta = fit_offline(basis, hbv, target, w, a_nom, theta0)
%FIT_OFFLINE  weighted Gauss-Newton a-space fit (theta* verification / beta ref).
    theta = theta0(:);  nth = numel(theta);
    for it = 1:60
        A = zeros(nth); b = zeros(nth,1);
        for i = 1:numel(hbv)
            [am,~,jac] = model_eval(basis, theta, hbv(i), a_nom);
            r = am - target(i);
            A = A + w(i)*(jac*jac');
            b = b + w(i)*jac*r;
        end
        dth = A \ b;
        theta = theta - dth;
        if norm(dth) < 1e-13; break; end
    end
end


function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_EVAL  exact color-inflation factor (mirrors 4state controller L683-689).
    sxT = 4*kBT*a;
    num = sxT^2*IF_abc(1) + 2*sxT*sigma2_nx*IF_abc(2) + sigma2_nx^2*IF_abc(3);
    den = (C_dpmr*sxT + C_n*sigma2_nx)^2;
    IF  = 1 + 2*num/den;
end


%% ====================================================================
%  PHASE D : figures + notes + CSV
%% ====================================================================

function make_all_figures(D, R, K, paths)
    fig_theta_conv(D, R, K, paths);
    fig_afit_overlay(D, R, K, paths);
    fig_ablation_smoothing(R, K, paths);
    fig_ablation_beta(D, R, K, paths);
    fig_pairing_ablation(R, K, paths);
    fig_forgetting_sweep(R, K, paths);
    fig_killtest_D(R, K, paths);
end


function f = newfig()
    f = figure('Color','w','NumberTitle','off','Visible','off', ...
               'Position',[80 80 1000 780]);
end

function style_axes(K)
    set(gca,'FontSize',K.FS,'FontWeight','bold','LineWidth',K.AXLW,'Box','on');
    grid off;
end

function export_fig(f, K, fpath)
    exportgraphics(f, fpath, 'Resolution', K.DPI);
    close(f);
end


function fig_theta_conv(D, R, K, paths)
%FIG_L1_THETA_CONV  theta_hat(t) per seed + theta* + beta-shifted + gate shade.
%   z tile: theta1+theta2 (identifiable sum) ; xy tile: b1.
    f = newfig();
    ax_ids = [3 1];  labels = {'z: \theta_1+\theta_2 (Z2)','x: b_1 (M2)'};
    for tt = 1:2
        subplot(2,1,tt); hold on;
        ax = ax_ids(tt);
        AX = R.axis(ax);
        for si = 1:numel(D)
            th = AX.base(si).theta_hist;
            y = param_combo(AX.basis, th);
            plot(D(si).t, y, '-', 'Color', K.COL_EST, 'LineWidth', 1.3, ...
                 'HandleVisibility', ternary(si==1,'on','off'), 'DisplayName','\theta_{hat}(t)');
        end
        ystar = param_combo_scalar(AX.basis, AX.theta_star);
        ybeta = param_combo_scalar(AX.basis, AX.theta_star_beta);
        yline(ystar, '--', 'Color', K.COL_TRUE, 'LineWidth', 2.0, 'DisplayName','\theta^*');
        yline(ybeta, ':',  'Color', K.COL_ALT,  'LineWidth', 2.0, 'DisplayName','\theta^*_{\beta}');
        shade_gate(D(1), AX.base(1).mask, K);   % after plotting so ylim is set
        ylabel(labels{tt}, 'FontSize', K.FS, 'FontWeight','bold');
        if tt==2; xlabel('Time (sec)','FontSize',K.FS,'FontWeight','bold'); end
        legend('Location','northoutside','Orientation','horizontal', ...
               'FontSize',K.LFS,'FontWeight','bold','Box','on');
        style_axes(K);
    end
    export_fig(f, K, fullfile(paths.root, 'fig_L1_theta_conv.png'));
end


function fig_afit_overlay(D, R, K, paths)
%FIG_L1_AFIT_OVERLAY  a(h;theta_final) vs a_true vs a_xm scatter, per axis.
    f = newfig();
    ax_ids = [3 1]; names = {'z','x'};
    for tt = 1:2
        subplot(2,1,tt); hold on;
        ax = ax_ids(tt); AX = R.axis(ax);
        plot(AX.grid, AX.a_true_grid*1e3, '-', 'Color', K.COL_TRUE, 'LineWidth', 2.5, 'DisplayName','a_{true}');
        plot(AX.grid, AX.a_model_grid*1e3, '-', 'Color', K.COL_EST, 'LineWidth', 1.8, 'DisplayName','a(\theta_{hat})');
        % a_xm scatter (seed 1, in-band, window-paired regressor height)
        hbr = shifted_hbar(D(1).hbar_d, K.D_DELAY + K.L_EFF);
        mask = train_mask(D(1), hbr, 'all', K);
        idx = find(mask); idx = idx(1:K.SCATTER_STRIDE:end);
        scatter(hbr(idx), D(1).a_xm(idx,ax)*1e3, 6, K.COL_MEAS, 'filled', ...
                'MarkerFaceAlpha', 0.35, 'DisplayName','a_{xm}');
        ylabel(sprintf('a_%s (nm/pN)', names{tt}), 'FontSize', K.FS, 'FontWeight','bold');
        if tt==2; xlabel('h_{bar}','FontSize',K.FS,'FontWeight','bold'); end
        xlim([1.15 3.5]);
        legend('Location','northoutside','Orientation','horizontal', ...
               'FontSize',K.LFS,'FontWeight','bold','Box','on');
        style_axes(K);
    end
    export_fig(f, K, fullfile(paths.root, 'fig_L1_afit_overlay.png'));
end


function fig_ablation_smoothing(R, K, paths)
%FIG_L1_ABLATION_SMOOTHING  raw a_xm vs a_m_det theta trajectories (mandatory).
    f = newfig();
    ax_ids = [3 1]; labels = {'z: \theta_1+\theta_2','x: b_1'};
    for tt = 1:2
        subplot(2,1,tt); hold on;
        ax = ax_ids(tt); AX = R.axis(ax);
        yr = param_combo(AX.basis, AX.smooth.raw.theta_hist);
        ys = param_combo(AX.basis, AX.smooth.smooth.theta_hist);
        plot(AX.smooth.raw.t,    yr, '-', 'Color', K.COL_EST, 'LineWidth', 1.6, 'DisplayName','raw a_{xm}');
        plot(AX.smooth.smooth.t, ys, '-', 'Color', K.COL_ALT, 'LineWidth', 1.6, 'DisplayName','a_{m,det} (EWMA 0.005)');
        ystar = param_combo_scalar(AX.basis, AX.theta_star);
        yline(ystar, '--', 'Color', K.COL_TRUE, 'LineWidth', 1.8, 'DisplayName','\theta^*');
        title_txt = sprintf('corr(stream,a_{true}): raw %.2f | smooth %.2f', AX.corr_raw, AX.corr_smooth);
        ylabel(labels{tt}, 'FontSize', K.FS, 'FontWeight','bold');
        text(0.02, 0.06, title_txt, 'Units','normalized','FontSize',K.LFS,'FontWeight','bold');
        if tt==2; xlabel('Time (sec)','FontSize',K.FS,'FontWeight','bold'); end
        legend('Location','northoutside','Orientation','horizontal', ...
               'FontSize',K.LFS,'FontWeight','bold','Box','on');
        style_axes(K);
    end
    export_fig(f, K, fullfile(paths.root, 'fig_L1_ablation_smoothing.png'));
end


function fig_ablation_beta(D, R, K, paths)
%FIG_L1_ABLATION_BETA  de-bias on/off theta trajectories (G3 / T3-1 check).
    f = newfig();
    ax_ids = [3 1]; labels = {'z: \theta_1+\theta_2','x: b_1'};
    for tt = 1:2
        subplot(2,1,tt); hold on;
        ax = ax_ids(tt); AX = R.axis(ax);
        rc = axis_rls_const(D(1).ctrl_const, ax);
        [theta0, P0] = make_prior(AX.basis, K);
        % off = raw ; on = beta-corrected
        Db = D(1);
        Db.a_xm_beta = D(1).a_xm ./ (1 + AX.beta1);
        v_off = AX.base(1);
        v_on  = fit_variant(Db, ax, AX.basis, theta0, P0, K.LAMBDA_F_BASE, ...
                            'a_xm_beta', K.D_DELAY + K.L_EFF, 'all', K, rc);
        plot(v_off.t, param_combo(AX.basis, v_off.theta_hist), '-', ...
             'Color', K.COL_ALT, 'LineWidth', 1.6, 'DisplayName','de-bias OFF');
        plot(v_on.t,  param_combo(AX.basis, v_on.theta_hist), '-', ...
             'Color', K.COL_EST, 'LineWidth', 1.6, 'DisplayName','de-bias ON');
        yline(param_combo_scalar(AX.basis, AX.theta_star), '--', 'Color', K.COL_TRUE, ...
              'LineWidth', 1.8, 'DisplayName','\theta^*');
        yline(param_combo_scalar(AX.basis, AX.theta_star_beta), ':', 'Color', [0.4 0.4 0.4], ...
              'LineWidth', 1.8, 'DisplayName','\theta^*_{\beta}');
        ylabel(labels{tt}, 'FontSize', K.FS, 'FontWeight','bold');
        if tt==2; xlabel('Time (sec)','FontSize',K.FS,'FontWeight','bold'); end
        legend('Location','northoutside','Orientation','horizontal', ...
               'FontSize',K.LFS,'FontWeight','bold','Box','on');
        style_axes(K);
    end
    export_fig(f, K, fullfile(paths.root, 'fig_L1_ablation_beta.png'));
end


function fig_pairing_ablation(R, K, paths)
%FIG_L1_PAIRING_ABLATION  final theta (param combo) x pairing x segment.
    f = newfig();
    ax_ids = [3 1]; labels = {'z: \theta_1+\theta_2','x: b_1'};
    for tt = 1:2
        subplot(2,1,tt); hold on;
        ax = ax_ids(tt); AX = R.axis(ax);
        np = numel(AX.pair.name);
        segs = {'all','descent','osc'};
        M = nan(np, numel(segs));
        for pp = 1:np
            for ss = 1:numel(segs)
                th = squeeze(AX.pair.theta(pp,ss,:));
                M(pp,ss) = param_combo_scalar(AX.basis, th);
            end
        end
        b = bar(M, 'grouped');
        cols = {K.COL_EST, K.COL_ALT, [0.4 0.4 0.4]};
        for ii = 1:numel(b)
            b(ii).FaceColor = cols{ii};
            b(ii).DisplayName = segs{ii};
        end
        yline(param_combo_scalar(AX.basis, AX.theta_star), '--', 'Color', K.COL_TRUE, ...
              'LineWidth', 2.0, 'DisplayName','\theta^*');
        set(gca,'XTick',1:np,'XTickLabel',AX.pair.name);
        ylabel(labels{tt}, 'FontSize', K.FS, 'FontWeight','bold');
        legend('Location','northoutside','Orientation','horizontal', ...
               'FontSize',K.LFS,'FontWeight','bold','Box','on');
        style_axes(K);
    end
    export_fig(f, K, fullfile(paths.root, 'fig_L1_pairing_ablation.png'));
end


function fig_forgetting_sweep(R, K, paths)
%FIG_L1_FORGETTING_SWEEP  bias + spread vs lambda_f + S6 posterior-std pred.
    f = newfig();
    ax_ids = [3 1]; names = {'z','x'};
    for tt = 1:2
        subplot(2,1,tt); hold on;
        ax = ax_ids(tt); AX = R.axis(ax);
        lf = AX.forget.lambda_f;
        xi = 1:numel(lf);
        bias = abs(AX.forget.bias(:,1));       % lead-param bias magnitude
        spread = AX.forget.spread(:,1);
        pred = AX.forget.pred;
        plot(xi, bias, '-o', 'Color', K.COL_TRUE, 'LineWidth', 2.0, 'MarkerFaceColor',K.COL_TRUE, 'DisplayName','|bias|');
        plot(xi, spread, '-s', 'Color', K.COL_EST, 'LineWidth', 2.0, 'MarkerFaceColor',K.COL_EST, 'DisplayName','seed spread');
        plot(xi, pred, '--^', 'Color', K.COL_ALT, 'LineWidth', 2.0, 'DisplayName','S6 posterior std');
        set(gca,'XTick',xi,'XTickLabel',arrayfun(@(x) sprintf('%.4g',x), lf,'uni',0));
        ylabel(sprintf('%s lead-param', names{tt}), 'FontSize', K.FS, 'FontWeight','bold');
        if tt==2; xlabel('\lambda_f','FontSize',K.FS,'FontWeight','bold'); end
        legend('Location','northoutside','Orientation','horizontal', ...
               'FontSize',K.LFS,'FontWeight','bold','Box','on');
        style_axes(K);
    end
    export_fig(f, K, fullfile(paths.root, 'fig_L1_forgetting_sweep.png'));
end


function fig_killtest_D(R, K, paths)
%FIG_L1_KILLTEST_D  windowed local slope vs a'_true (expected far-field fail).
    f = newfig();
    ax = 3; AX = R.axis(ax); kd = AX.killD;
    subplot(1,1,1); hold on;
    if ~isempty(kd.hbar)
        scatter(kd.hbar, kd.slope_local*1e3, 22, K.COL_MEAS, 'filled', ...
                'MarkerFaceAlpha',0.5, 'DisplayName','local slope (a_{xm})');
        [hs, is] = sort(kd.hbar);
        plot(hs, kd.aprime_true(is)*1e3, '-', 'Color', K.COL_TRUE, 'LineWidth', 2.5, ...
             'DisplayName','a''_{true}');
    end
    ylabel('da/dh_{bar}  (nm/pN)', 'FontSize', K.FS, 'FontWeight','bold');
    xlabel('h_{bar} (window centre)','FontSize',K.FS,'FontWeight','bold');
    txt = sprintf('sign-agree near %.2f | far %.2f', kd.sign_agree_near, kd.sign_agree_far);
    text(0.02, 0.06, txt, 'Units','normalized','FontSize',K.LFS,'FontWeight','bold');
    legend('Location','northoutside','Orientation','horizontal', ...
           'FontSize',K.LFS,'FontWeight','bold','Box','on');
    style_axes(K);
    export_fig(f, K, fullfile(paths.root, 'fig_L1_killtest_D.png'));
end


%% ---- figure helpers ----

function y = param_combo(basis, theta_hist)
%PARAM_COMBO  the plotted parameter combination (Z2 sum ; M2/M1/Z1 first param).
    if strcmp(basis,'Z2')
        y = theta_hist(:,1) + theta_hist(:,2);
    else
        y = theta_hist(:,1);
    end
end

function y = param_combo_scalar(basis, theta)
    if strcmp(basis,'Z2'); y = theta(1) + theta(2); else; y = theta(1); end
end

function shade_gate(Ds, mask, K)
%SHADE_GATE  shade time regions where the training gate is OFF during osc.
    t_osc0 = K.T_HOLD_S + K.T_DESCEND_S;
    off = (~mask) & (Ds.t >= t_osc0);
    yl = ylim; ylim(yl);
    d = diff([0; off(:); 0]);
    starts = find(d==1); stops = find(d==-1)-1;
    for i = 1:numel(starts)
        x0 = Ds.t(starts(i)); x1 = Ds.t(min(stops(i),numel(Ds.t)));
        p = patch([x0 x1 x1 x0],[yl(1) yl(1) yl(2) yl(2)], K.COL_GATE, ...
                  'EdgeColor','none','FaceAlpha',0.4,'HandleVisibility','off');
        uistack(p,'bottom');
    end
end

function v = ternary(cond, a, b)
    if cond; v = a; else; v = b; end
end


%% ====================================================================
%  Notes + CSV
%% ====================================================================

function write_l1_notes(R, K, paths, out)
    fid = fopen(paths.notes, 'w');
    if fid < 0; warning('verify_audit_L1_theta:notes','cannot open %s', paths.notes); return; end
    cleanup_fid = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '# L1 theta-fit verification notes\n\n');
    fprintf(fid, 'Auto-generated by `verify_audit_L1_theta.m`. Open-loop offline theta-fit on\n');
    fprintf(fid, 'the known-c 4-state production log (use_q44_ar1, osc_1hz).\n\n');
    fprintf(fid, '- seeds: %s\n', mat2str(K.SEEDS));
    fprintf(fid, '- diverged seeds: %s\n', mat2str(find(out.diverged)));
    fprintf(fid, '- PHASE B a_xm reconstruction max rel diff vs diag.a_xm: %.3e (expect ~machine eps)\n\n', out.recon_max_reldiff);

    % scoring table
    fprintf(fid, '## Scoring (baseline, seed-mean theta)\n\n');
    fprintf(fid, '| axis | basis | theta_hat | theta* | level@1.2 | level@1.5 | slope@1.2 | param@1.2 | floor@1.2 |\n');
    fprintf(fid, '|---|---|---|---|---|---|---|---|---|\n');
    for ax = 1:numel(R.axis)
        AX = R.axis(ax); sc = AX.score;
        it = 1;  % HB_SCORE(1)=1.2
        i15 = find(abs(sc.hbar-1.5)<1e-6,1);
        fprintf(fid, '| %s | %s | %s | %s | %+.2f%% | %+.2f%% | %+.2f%% | %+.2f%% | %+.2f%% |\n', ...
            AX.name, AX.basis, vec2str(AX.theta_mean), vec2str(AX.theta_star), ...
            100*sc.level(it), 100*sc.level(i15), 100*sc.slope(it), ...
            100*sc.param(it), 100*sc.struct_floor(it));
    end
    fprintf(fid, '\nConvergence time (seed 1, param combo within 5%% of final): ');
    for ax = 1:numel(R.axis)
        fprintf(fid, '%s=%.2fs  ', R.axis(ax).name, R.axis(ax).t_conv);
    end
    fprintf(fid, '\n');

    % Gate G1 verdict
    fprintf(fid, '\n## Gate G1 verdict\n\n');
    fprintf(fid, '| axis | param@trough | floor@trough | param<=floor | level<0.5%% | corr raw/smooth | spread<=2x post | PASS |\n');
    fprintf(fid, '|---|---|---|---|---|---|---|---|\n');
    for ax = 1:numel(R.gateG1)
        g = R.gateG1(ax);
        fprintf(fid, '| %s | %.2f%% | %.2f%% | %s | %s | %.2f/%.2f (%s) | %.2e/%.2e (%s) | **%s** |\n', ...
            g.axis, 100*g.param_trough, 100*g.floor_trough, ...
            yn(g.crit_param_le_floor), yn(g.crit_level_trough), ...
            g.corr_raw, g.corr_smooth, yn(g.crit_corr), ...
            g.theta_spread, g.posterior_std, yn(g.crit_spread), yn(g.pass));
    end

    % pairing ablation (z + xy)
    fprintf(fid, '\n## Pairing ablation (param combo; z=sum, xy=b1)\n\n');
    fprintf(fid, '| axis | segment | naive[k-d] | window[k-d-19] | beta+recentred | theta* |\n');
    fprintf(fid, '|---|---|---|---|---|---|\n');
    segs = {'all','descent','osc'};
    for ax = [3 1 2]
        AX = R.axis(ax);
        for ss = 1:3
            vals = arrayfun(@(pp) param_combo_scalar(AX.basis, squeeze(AX.pair.theta(pp,ss,:))), 1:3);
            fprintf(fid, '| %s | %s | %.4f | %.4f | %.4f | %.4f |\n', ...
                AX.name, segs{ss}, vals(1), vals(2), vals(3), param_combo_scalar(AX.basis, AX.theta_star));
        end
    end

    % forgetting sweep
    fprintf(fid, '\n## Forgetting sweep (lead-param |bias| / seed spread / S6 posterior std)\n\n');
    fprintf(fid, '| axis | lambda_f | |bias| | spread | S6 pred |\n|---|---|---|---|---|\n');
    for ax = [3 1]
        AX = R.axis(ax);
        for fi = 1:numel(AX.forget.lambda_f)
            fprintf(fid, '| %s | %.4g | %.4f | %.4f | %.4f |\n', AX.name, ...
                AX.forget.lambda_f(fi), abs(AX.forget.bias(fi,1)), ...
                AX.forget.spread(fi,1), AX.forget.pred(fi));
        end
    end

    % kill-test D + P_tilt
    fprintf(fid, '\n## Kill-test D (windowed local slope sign-agreement)\n\n');
    for ax = [3 1]
        AX = R.axis(ax);
        fprintf(fid, '- %s: sign-agree near(h<2)=%.2f  far(h>=2)=%.2f  (far~0.5 = noise-dominated => global basis needed)\n', ...
            AX.name, AX.killD.sign_agree_near, AX.killD.sign_agree_far);
    end
    fprintf(fid, '\n## P_tilt (S9 flag v; assumed 0.9)\n\n');
    fprintf(fid, '| axis | h=1.2 | h=1.5 | h=3.42 | assumed |\n|---|---|---|---|---|\n');
    for ax = 1:numel(R.axis)
        AX = R.axis(ax); pt = AX.ptilt;
        fprintf(fid, '| %s | %.3f | %.3f | %.3f | %.2f |\n', AX.name, ...
            pt.P_tilt(1), pt.P_tilt(2), pt.P_tilt(3), pt.assumed);
    end

    % resolved flags
    fprintf(fid, '\n## S9 flags resolved by L1\n\n');
    ptz = R.axis(3).ptilt;
    fprintf(fid, '- **flag v (|P_tilt|~0.9)**: measured z P_tilt = [%.3f %.3f %.3f] at h={1.2,1.5,3.42}; xy(M2) = [%.3f %.3f %.3f]. Compare vs assumed 0.9.\n', ...
        ptz.P_tilt(1),ptz.P_tilt(2),ptz.P_tilt(3), ...
        R.axis(1).ptilt.P_tilt(1),R.axis(1).ptilt.P_tilt(2),R.axis(1).ptilt.P_tilt(3));
    fprintf(fid, '- **flag viii (FF-enable 0.3-0.7s)**: partial. See fig_L1_theta_conv for the RLS convergence rate; FF-enable timing follows once P_theta collapse crosses the S6.4 threshold (quantified at L2).\n');
    fprintf(fid, '\n(Figures: fig_L1_theta_conv, fig_L1_afit_overlay, fig_L1_ablation_smoothing, fig_L1_ablation_beta, fig_L1_pairing_ablation, fig_L1_forgetting_sweep, fig_L1_killtest_D.)\n');
end


function write_l1_csv(D, R, K, paths)
%WRITE_L1_CSV  theta trajectories (baseline, per axis/seed) + final fits table.
    % (1) theta trajectories
    for ax = 1:numel(R.axis)
        AX = R.axis(ax);
        for si = 1:numel(D)
            th = AX.base(si).theta_hist;
            M = [D(si).t, th];
            fp = fullfile(paths.root, sprintf('L1_theta_traj_%s_seed%02d.csv', AX.name, D(si).seed));
            hdr = [{'t'}, arrayfun(@(j) sprintf(',theta%d',j), 1:size(th,2), 'uni', 0)];
            write_csv_matrix(fp, strjoin(hdr,''), M);
        end
    end
    % (2) final fits table
    fp = fullfile(paths.root, 'L1_final_fits.csv');
    fid = fopen(fp, 'w');
    if fid < 0; return; end
    cleanup_fid = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'axis,basis,seed,theta1,theta2,level_1p2,level_1p5,level_3p42,slope_1p2,param_1p2\n');
    for ax = 1:numel(R.axis)
        AX = R.axis(ax);
        for si = 1:numel(D)
            th = AX.base(si).theta_final;
            sc = score_fit(AX.basis, th, AX.theta_star, AX.kind, D(si).a_nom, K);
            t2 = NaN; if numel(th) >= 2; t2 = th(2); end
            i15 = find(abs(sc.hbar-1.5)<1e-6,1);
            i342 = find(abs(sc.hbar-3.42)<1e-6,1);
            fprintf(fid, '%s,%s,%d,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f\n', ...
                AX.name, AX.basis, D(si).seed, th(1), t2, ...
                sc.level(1), sc.level(i15), sc.level(i342), sc.slope(1), sc.param(1));
        end
    end
end


function write_csv_matrix(fp, header, M)
    fid = fopen(fp, 'w');
    if fid < 0; return; end
    cleanup_fid = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s\n', header);
    fmt = [repmat('%.8g,', 1, size(M,2)-1), '%.8g\n'];
    for i = 1:size(M,1)
        fprintf(fid, fmt, M(i,:));
    end
end


%% ---- small utils ----

function ensure_dir(d)
    if ~exist(d, 'dir'); mkdir(d); end
end

function s = vec2str(v)
    v = v(:);
    if numel(v)==1; s = sprintf('%.4f', v(1));
    else; s = sprintf('[%.4f, %.4f]', v(1), v(2)); end
end

function s = yn(b)
    if b; s = 'Y'; else; s = 'N'; end
end
