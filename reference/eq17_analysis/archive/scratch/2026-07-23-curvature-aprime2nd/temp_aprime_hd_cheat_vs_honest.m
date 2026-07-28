function out = temp_aprime_hd_cheat_vs_honest(varargin)
%TEMP_APRIME_HD_CHEAT_VS_HONEST  Oracle-FF ("cheat") vs honest a' arms, ONE engine.
%
%   out = temp_aprime_hd_cheat_vs_honest()               % seed 1, manuscript defaults
%   out = temp_aprime_hd_cheat_vs_honest('seed',3, ...)
%
%   -------------------------------------------------------------------------
%   WHAT THIS TESTS  (derived from temp_aprime_hd_order_compare.m)
%   -------------------------------------------------------------------------
%   Puts the "oracle-cheat" a' feedforward arms next to the honest arms, all in the
%   SAME 1-D closed-loop z-axis simulator (real wall curve, calc_correction_functions),
%   with the SAME seed / trajectory / plant / control law / measurement -> a FAIR
%   upper-bound-vs-achievable comparison of a'_hd(t) tracking.
%
%   The "cheat" mechanism (reproduced from temp_aprime_order_compare.m's
%   cfg.aprime_ff_oracle) is an ORACLE a' FEEDFORWARD in the EKF predict:
%       xp(5) = x(5) + a''_true(h_d[k]) * Dh_d[k]         (uses the TRUE curvature)
%   a''_true = d2 a_h / dh2 read off the real gain curve at the COMMANDED height
%   h_d[k]. This is "with the map": it pushes a' along the true curve, so a' tracks
%   without ever estimating curvature. When the toggle is OFF, the honest recursions
%   run (1st-order pure random walk, or 2nd-order sweep by the ESTIMATED curvature).
%
%   -------------------------------------------------------------------------
%   THE FOUR ARMS  (identical trajectory, seed, plant, control law, F_dh, H(2,5))
%   -------------------------------------------------------------------------
%   ARM 1  1st oracle-FF (cheat)   order 1, ff_oracle=ON.  a'-predict = oracle FF
%          a''_true*Dh_d; NO curvature state.  Q55 thermal-grounded (measured h-bar).
%   ARM 2  2nd oracle-FF (cheat)   order 2, ff_oracle=ON.  Curvature state present
%          (slot 6) + oracle a''*Dh_d feedforward on a' (F_e(5,6)=0 since a' now uses
%          the known oracle input, not the estimate).  Q66 oracle (a'''_true^2 Dh^2).
%   ARM 3  2nd honest closure      order 2, ff_oracle=OFF.  The REAL one: a' swept by
%          the ESTIMATED curvature delta_a'_hat, Q66 = (delta_a'_hat*Dh/R)^2, NO oracle.
%   ARM 4  1st honest              order 1, ff_oracle=OFF.  a' pure random walk, no FF
%          (the noise floor for contrast). Q55 thermal-grounded (same as ARM 1).
%
%   Cheat arms (1,2) = upper bound "with the map". Honest arms (3,4) = what we can
%   actually do with c(h) unknown. The gap between honest-2nd (3) and the cheat arms
%   is the remaining a'-observability deficit.
%
%   -------------------------------------------------------------------------
%   SCENARIO / FIDELITY  (unchanged from temp_aprime_hd_order_compare.m)
%   -------------------------------------------------------------------------
%   Canonical 1 Hz: hold(0.5s) -> cosine descent(1.0s) -> 4-cycle osc, h-bar ~ [2,22].
%   Thermal ON, measurement noise ON, single canonical seed. z-mobility diagonal ->
%   exact 1-D plant (ode4 10 us). Offline R22/IF/C_dpmr from build_eq17_6state_constants.
%   All arms share the SAME thermal/measurement noise draws.
%
%   KNOBS (name/value):  seed(1) Q66_floor(0) Q55_arm1_scale(1) P55_0_frac(0.01)
%                        P66_0_frac(1e-3) make_fig(true) verbose(false)
%
%   TEMP diagnostic (chat 2026-07-23); gitignored (temp_*.m), delete after use.

    % ---- knobs -----------------------------------------------------------
    PAR.seed           = 1;
    PAR.Q66_floor      = 0;
    PAR.Q55_arm1_scale = 1;
    PAR.P55_0_frac     = 0.01;
    PAR.P66_0_frac     = 1e-3;
    PAR.make_fig       = true;
    PAR.verbose        = false;
    for i = 1:2:numel(varargin); PAR.(varargin{i}) = varargin{i+1}; end

    % ---- paths -----------------------------------------------------------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'controller'));

    % ---- fixed physical parameters ---------------------------------------
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
    pm.t_warmup = 0;                  % KF warm-up
    pm.hbar_safe = 1;                 % Guard 3 threshold
    meas_std = [0.00062; 0.00057; 0.00331];   % [um] per-axis sensor noise
    pm.sig2n_z = meas_std(3)^2;       % z-axis measurement-noise variance [um^2]

    % ---- offline constants (production builder) --------------------------
    eq17.lambda_c   = pm.lc;
    eq17.sigma2_n_s = meas_std.^2;
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

    % ---- trajectory (canonical 1 Hz hold->descent->osc) ------------------
    tr.h_init    = 50;                 % [um]
    tr.h_bottom  = 2.0 * pm.R;         % [um] h-bar_bottom = 2.0
    tr.amplitude = 2.5;                % [um]
    tr.freq      = 1;                  % [Hz]
    tr.t_hold    = 0.5;                % [s]
    tr.t_descend = 1.0;                % [s]
    tr.n_cycles  = 4;
    tr.t1 = tr.t_hold;
    tr.t2 = tr.t1 + tr.t_descend;
    tr.t3 = tr.t2 + tr.n_cycles / tr.freq;
    T_sim = tr.t3 + 0.5;
    N = round(T_sim / pm.Ts) + 1;
    tout = (0:N-1)' * pm.Ts;
    tr.N = N;

    h_d_seq     = zeros(N, 1);
    h_d_kp1_seq = zeros(N, 1);
    for k = 1:N
        h_d_seq(k)     = traj_height(tout(k),          tr);
        h_d_kp1_seq(k) = traj_height(tout(k) + pm.Ts,  tr);
    end
    tr.h_d_seq = h_d_seq;
    tr.h_d_kp1_seq = h_d_kp1_seq;

    % ---- oracle TRUTH: a, a', a'', a''' at commanded height h_d[k] -------
    oracle_a   = zeros(N, 1);
    oracle_ap  = zeros(N, 1);
    oracle_app = zeros(N, 1);          % <- a2_true FF source (the "map")
    oracle_a3  = zeros(N, 1);
    for k = 1:N
        hb = max(h_d_seq(k) / pm.R, 1.001);
        [~, cp, drv] = calc_correction_functions(hb, true);
        oracle_a(k)   = pm.a_nom / cp;
        oracle_ap(k)  = -(pm.a_nom / cp) * drv.K_h_perp / pm.R;
        oracle_app(k) =  (pm.a_nom / cp) / pm.R^2 * (drv.K_h_perp^2 - drv.K_h_prime_perp);
        oracle_a3(k)  = a3prime_at(h_d_seq(k), pm);
    end

    % ---- shared noise draws ---------------------------------------------
    rng(PAR.seed);
    noise.z_th   = randn(N, 1);
    noise.z_meas = randn(N, 1);

    knobs = struct('Q66_floor', PAR.Q66_floor, 'Q55_arm1_scale', PAR.Q55_arm1_scale, ...
                   'P55_0_frac', PAR.P55_0_frac, 'P66_0_frac', PAR.P66_0_frac);

    % ---- arm specs (order, oracle-FF toggle, Q66 mode) -------------------
    z = zeros(N,1);
    specs = { ...
        struct('id','1ffo','label','1st oracle-FF (cheat)', 'order',1,'q66_mode','na',     'ff_oracle',true,  'a3_hd',z,        'a2_hd',oracle_app); ...
        struct('id','2ffo','label','2nd oracle-FF (cheat)', 'order',2,'q66_mode','oracle', 'ff_oracle',true,  'a3_hd',oracle_a3,'a2_hd',oracle_app); ...
        struct('id','2hon','label','2nd honest closure',    'order',2,'q66_mode','closure','ff_oracle',false, 'a3_hd',z,        'a2_hd',z) };
    ids    = cellfun(@(s) s.id,    specs, 'UniformOutput', false);
    labels = cellfun(@(s) s.label, specs, 'UniformOutput', false);

    % ---- run all arms ----------------------------------------------------
    warning('off', 'MATLAB:nearlySingularMatrix');
    res = cell(numel(specs), 1);
    for a = 1:numel(specs)
        res{a} = run_arm(specs{a}, cst, pm, tr, noise, knobs, PAR.verbose);
    end
    warning('on', 'MATLAB:nearlySingularMatrix');

    % =====================================================================
    % Reporting
    % =====================================================================
    seg.hold = tout >= 0.1         & tout <= tr.t1;
    seg.desc = tout >  tr.t1       & tout <  tr.t2;
    seg.osc  = tout >= tr.t2 + 0.5 & tout <= tr.t3;   % skip first half-cycle settling
    per_samp = round((1 / tr.freq) / pm.Ts);

    nm = 1e3;
    fprintf('\n==================================================================\n');
    fprintf(' a''_hd CHEAT vs HONEST : oracle-FF arms next to honest arms (SAME engine)\n');
    fprintf('   ARM 1 = 1st oracle-FF (cheat) | ARM 2 = 2nd oracle-FF (cheat)\n');
    fprintf('   ARM 3 = 2nd honest closure    | ARM 4 = 1st honest\n');
    fprintf('   canonical 1Hz, z-axis (c_perp), seed %d\n', PAR.seed);
    fprintf('==================================================================\n');
    fprintf(' segments (s):  hold[0.1,%.2f]  descent(%.2f,%.2f)  osc[%.2f,%.2f]\n', ...
            tr.t1, tr.t1, tr.t2, tr.t2 + 0.5, tr.t3);
    fprintf(' T_sim=%.2f s, N=%d, Ts=%.4e s, lambda_c=%.2f, d=%d, sigma_n_z=%.3f nm\n', ...
            T_sim, N, pm.Ts, pm.lc, pm.d, sqrt(pm.sig2n_z)*nm);
    fprintf(' oracle-FF mechanism: xp(5) = x(5) + a2_true(h_d[k])*Dh_d[k]  (a2_true = a''''(h_d), the map)\n');

    fprintf('\n TRUE curve along trajectory (z-axis / c_perp):\n');
    fprintf('   a    = a_nom/c_perp  : [%.3f , %.3f] nm/pN            (a_nom = %.3f nm/pN)\n', ...
            min(oracle_a)*nm, max(oracle_a)*nm, pm.a_nom*nm);
    fprintf('   a''   = da/dh         : [%.3f , %.3f] nm/pN/um\n', min(oracle_ap)*nm, max(oracle_ap)*nm);
    fprintf('   a''''  = d2a/dh2       : [%.3f , %.3f] nm/pN/um^2  (oracle-FF source)\n', ...
            min(oracle_app)*nm, max(oracle_app)*nm);

    % ---- Q / R actually used (osc-mean) ----------------------------------
    om = seg.osc;
    fprintf('\n PROCESS / MEASUREMENT NOISE used (osc-mean):\n');
    for a = 1:numel(res)
        if res{a}.order == 1
            fprintf('   %-22s : Q33=%.3e  Q55=%.3e (thermal x%.2g)  R22=%.3e\n', ...
                    labels{a}, mean(res{a}.Q33(om)), mean(res{a}.Q55(om)), PAR.Q55_arm1_scale, mean(res{a}.R22(om)));
        else
            fprintf('   %-22s : Q33=%.3e  Q55=0  Q66=%.3e   R22=%.3e\n', ...
                    labels{a}, mean(res{a}.Q33(om)), mean(res{a}.Q66(om)), mean(res{a}.R22(om)));
        end
    end

    % ---- per-segment / whole-run metric tables ---------------------------
    print_metric_table('a''-hat vs a''_true(h_d)', res, ids, oracle_ap, seg, per_samp, pm.Ts, 'aprime');
    print_metric_table('a-hat  vs a_true(h)',      res, ids, oracle_a,  seg, per_samp, pm.Ts, 'again');
    print_ratio_table(res, ids, seg);
    print_track_table(res, ids, seg);

    % ---- verdict ---------------------------------------------------------
    M = cell(numel(res),1);
    for a = 1:numel(res);  M{a} = seg_metric(res{a}.ap_hat, oracle_ap, om, per_samp, pm.Ts, true);  end
    rr = zeros(numel(res),1);
    for a = 1:numel(res);  rr(a) = mean(res{a}.a_hat(om) ./ res{a}.a_true(om));  end
    fprintf('\n VERDICT (osc-segment a'' vs true a''(h_d)):\n');
    fprintf('   %-24s %-10s %-8s %-8s %-9s %-9s %-9s\n', 'arm', 'RMS[nm]', 'rel[%]', 'corr', 'lag[ms]', 'a_hat/a', 'trk[nm]');
    for a = 1:numel(res)
        fprintf('   %-24s %-10.4f %-8.1f %-8.3f %-9s %-9.4f %-9.1f\n', ...
                labels{a}, M{a}.rms*nm, M{a}.rel, M{a}.corr, fmt_num(M{a}.lag_ms,'%+.1f'), ...
                rr(a), std(res{a}.dh_track(om))*nm);
    end
    % arm indices: 1=1ffo 2=2ffo 3=2hon 4=1hon
    cheat_rel = [M{1}.rel, M{2}.rel];  cheat_corr = [M{1}.corr, M{2}.corr];
    fprintf('\n   with the MAP (oracle-FF, upper bound):\n');
    fprintf('     1st oracle-FF : rel %.1f%% , corr %.3f , lag %+.1f ms\n', M{1}.rel, M{1}.corr, M{1}.lag_ms);
    fprintf('     2nd oracle-FF : rel %.1f%% , corr %.3f , lag %+.1f ms\n', M{2}.rel, M{2}.corr, M{2}.lag_ms);
    fprintf('   c UNKNOWN (honest, achievable):\n');
    fprintf('     2nd honest    : rel %.1f%% , corr %.3f , lag %+.1f ms\n', M{3}.rel, M{3}.corr, M{3}.lag_ms);
    fprintf('   GAP honest-2nd vs cheat: rel %.1f%% vs %.1f-%.1f%% (%.1fx worse) ; corr %.3f vs %.3f-%.3f\n', ...
            M{3}.rel, min(cheat_rel), max(cheat_rel), M{3}.rel/max(min(cheat_rel),realmin), ...
            M{3}.corr, min(cheat_corr), max(cheat_corr));
    fprintf('==================================================================\n\n');

    % ---- figure ----------------------------------------------------------
    fig_path = '';
    if PAR.make_fig
        fig_path = make_cheat_honest_figure(tout, tr, oracle_a, oracle_ap, res, labels, project_root);
        fprintf(' FIGURE saved: %s\n\n', fig_path);
    end

    out = struct('tout', tout, 'tr', tr, 'pm', pm, 'cst', cst, 'PAR', PAR, ...
                 'oracle_a', oracle_a, 'oracle_ap', oracle_ap, 'oracle_app', oracle_app, ...
                 'oracle_a3', oracle_a3, 'res', {res}, 'labels', {labels}, 'ids', {ids}, ...
                 'seg', seg, 'metrics', {M}, 'a_ratio', rr, 'fig_path', fig_path);
end


%% ==================================================================
%  Closed-loop 1-D (z-axis) sim + inline manuscript EKF for one arm
%  (extends temp_aprime_hd_order_compare.m/run_arm with an oracle-FF toggle)
%% ==================================================================
function res = run_arm(spec, cst, pm, tr, noise, knobs, verbose)
    order = spec.order;  q66_mode = spec.q66_mode;  a3_hd = spec.a3_hd;
    ff_oracle = spec.ff_oracle;  a2_hd = spec.a2_hd;   % <- oracle a'' feedforward source
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
    Pc(5,5) = (knobs.P55_0_frac * a_nom / R)^2;
    if order == 2
        Pc(6,6) = (knobs.P66_0_frac * a_nom / R^2)^2;
    end

    % ---- IIR prefill + delay buffers ----
    dx_bar_m   = 0;
    sigma2_dxr = 4 * kBT * a_z0 * C_dpmr + C_n * sig2n;
    pd_km1 = tr.h_init;  pd_km2 = tr.h_init;
    f_km1  = 0;          f_km2  = 0;
    a_km1  = a_z0;       a_km2  = a_z0;
    vda_km1 = var_da0;   vda_km2 = var_da0;
    h_curr = tr.h_init;
    hnoisy_hist = tr.h_init * ones(N, 1);

    % ---- logs ----
    ap_hat  = zeros(N, 1);  app_hat = zeros(N, 1);  a_hat = zeros(N, 1);
    a_true  = zeros(N, 1);  dh_track = zeros(N, 1);  hbar_meas_log = zeros(N, 1);
    Q33L = zeros(N, 1);  Q55L = zeros(N, 1);  Q66L = zeros(N, 1);  R22L = zeros(N, 1);

    for k = 1:N
        t_k = (k - 1) * Ts;

        h_d_k   = tr.h_d_seq(k);
        h_d_kp1 = tr.h_d_kp1_seq(k);
        dh_d_step  = h_d_k - pd_km1;      % Dh_d[k]
        dH_d_dspan = h_d_k - pd_km2;      % DH_d[k]

        if k > d
            dh_m = tr.h_d_seq(k - d) - hnoisy_hist(k - d);
        else
            dh_m = 0;
        end

        a_ctrl = max(x(4), 1e-6);

        sum_a_fd = a_km1 * f_km1 + a_km2 * f_km2;
        f_dz = (1 / a_ctrl) * ( h_d_kp1 - lc * h_d_k - omlc * pd_km2 ...
                                + omlc * dh_m - omlc * sum_a_fd );

        dx_bar_m   = (1 - a_pd) * dx_bar_m + a_pd * dh_m;
        dx_r       = dh_m - dx_bar_m;
        sigma2_dxr = (1 - a_cov) * sigma2_dxr + a_cov * dx_r^2;
        a_hm = (sigma2_dxr - C_n * sig2n) / (C_dpmr * 4 * kBT);

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

        % -- Q (manuscript): rank-1 thermal Q(3:4); Q55/Q66 per arm --
        var_da_inc = vdaf * (a_hat_i * K_h_m / R)^2 * sig2dh;
        eps_var = 4 * kBT * (a_hat_i + omlc^2 * (a_km1 + a_km2)) + omlc^2 * sig2n;
        Q = zeros(n_aug);
        Q(3,3) = eps_var;
        Q(4,4) = ap_i^2 * eps_var;
        Q(3,4) = -ap_i * eps_var;  Q(4,3) = -ap_i * eps_var;
        Q55 = 0;  Q66_used = 0;
        if order == 1
            a_pp_m     = -(a_hat_i / R^2) * (K_hp_m - K_h_m^2);
            var_dx_ram = (2 / (1 + lc)) * 4 * kBT * a_hat_i;
            Q55 = knobs.Q55_arm1_scale * a_pp_m^2 * var_dx_ram;
            Q(5,5) = Q55;
        else
            Q(5,5) = 0;
            switch q66_mode
                case 'oracle'
                    Q66_used = (a3_hd(k) * dh_d_step)^2;
                case 'closure'
                    Q66_used = (x(6) * dh_d_step / R)^2;
                otherwise
                    Q66_used = 0;
            end
            Q66_used = Q66_used + knobs.Q66_floor;
            Q(6,6) = Q66_used;
        end

        IF_eff  = if_eff(IF_abc, C_dpmr, C_n, kBT, a_hat_i, sig2n);
        R2_int  = K_var * IF_eff * (a_hat_i + xi)^2;
        R22     = R2_int + r22f * (vda_km1 + vda_km2);

        G1 = t_k < pm.t_warmup;
        G2 = (sigma2_dxr - C_n * sig2n) <= 0;
        G3 = hbar_meas < pm.hbar_safe;
        gate_off = G1 || G2 || G3;

        F_dh  = f_dz + omlc * (f_km1 + f_km2);
        F_dhH = omlc * (dh_d_step * f_km1 + dH_d_dspan * f_km2);
        Fe = build_Fe(order, lc, F_dh, F_dhH, dh_d_step, ap_i, ff_oracle);

        % -- EKF predict (nonlinear mean; Fe propagates covariance) --
        xp = zeros(n_aug, 1);
        xp(1) = x(2);
        xp(2) = x(3);
        xp(3) = lc * x(3);
        xp(4) = x(4) + ap_i * dh_d_step + omlc * ap_i * x(3);      % Taylor gain advance
        if ff_oracle
            % ORACLE feedforward: a' pushed along the TRUE curve by a''_true*Dh_d
            xp(5) = x(5) + a2_hd(k) * dh_d_step;
        elseif order == 1
            xp(5) = x(5);                                          % honest random walk
        else
            xp(5) = x(5) + dh_d_step * x(6);                       % honest: swept by ESTIMATED curvature
        end
        if order == 2
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

        % -- PLANT (exact 1-D z; ode4 10us; thermal) --
        hbar_true = max(h_curr / R, 1.001);
        [~, cp_t] = calc_correction_functions(hbar_true);
        a_true(k) = a_nom / cp_t;
        f_Tz  = sqrt(4 * kBT * gamma * cp_t / Ts) * noise.z_th(k);
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
                 'ff_oracle', ff_oracle, 'a_hat', a_hat, 'ap_hat', ap_hat, 'app_hat', app_hat, ...
                 'a_true', a_true, 'dh_track', dh_track, 'hbar_meas', hbar_meas_log, ...
                 'Q33', Q33L, 'Q55', Q55L, 'Q66', Q66L, 'R22', R22L);
end


%% ==================================================================
%  Local helpers
%% ==================================================================
function Fe = build_Fe(order, lc, F_dh, F_dhH, dh_d, ap, ff_oracle)
%BUILD_FE  Manuscript F_e. For the order-2 oracle-FF arm the a' predict uses the
%   known oracle input a''_true*Dh (not the estimated curvature), so F_e(5,6)=0.
    omlc = 1 - lc;
    if order == 1
        Fe = [0 1 0        0            0    ; ...
              0 0 1        0            0    ; ...
              0 0 lc      -F_dh         F_dhH; ...
              0 0 omlc*ap  1+ap*F_dh    dh_d ; ...
              0 0 0        0            1    ];
    else
        c56 = dh_d;  if ff_oracle;  c56 = 0;  end   % oracle FF decouples a' from x(6)
        Fe = [0 1 0        0            0     0   ; ...
              0 0 1        0            0     0   ; ...
              0 0 lc      -F_dh         F_dhH 0   ; ...
              0 0 omlc*ap  1+ap*F_dh    dh_d  0   ; ...
              0 0 0        0            1     c56 ; ...
              0 0 0        0            0     1   ];
    end
end

function app = a2prime_at(h, pm)
    hb = max(h / pm.R, 1.001);
    [~, cp, drv] = calc_correction_functions(hb, true);
    app = (pm.a_nom / cp) / pm.R^2 * (drv.K_h_perp^2 - drv.K_h_prime_perp);
end

function a3 = a3prime_at(h, pm)
    delta = 1e-3 * pm.R;
    a3 = (a2prime_at(h + delta, pm) - a2prime_at(h - delta, pm)) / (2 * delta);
end

function IF = if_eff(abc, C_dpmr, C_n, kBT, a, snx)
    sxT = 4 * kBT * a;
    num = sxT^2 * abc(1) + 2 * sxT * snx * abc(2) + snx^2 * abc(3);
    den = (C_dpmr * sxT + C_n * snx)^2;
    IF  = 1 + 2 * num / den;
end

function P = dare_fp(F, H, Q, R)
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
    e   = est(win) - truth(win);
    m.rms = rms(e);
    m.rel = m.rms / max(rms(truth(win)), realmin) * 100;
    if numel(unique(truth(win))) < 2 || numel(unique(est(win))) < 2
        m.corr = NaN;
    else
        cc = corrcoef(est(win), truth(win));  m.corr = cc(1,2);
    end
    if do_lag && sum(win) > 8
        % cap search at half a period: avoids the full-period wrap degeneracy
        % (for a corr~1 periodic signal, 0 and +/-1 period are indistinguishable)
        L = xcorr_lag(truth(win), est(win), max(4, round(per_samp/2)));
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

function print_metric_table(title_str, arms, ~, truth, seg, per_samp, Ts, kind)
    nm = 1e3;  segn = {'hold', 'desc', 'osc', 'all'};
    win_all = true(size(truth));
    fprintf('\n %s   (RMS/rel in %s, corr, lag ms; lag on osc only):\n', title_str, unit_of(kind));
    fprintf('  %-5s %-22s %-11s %-9s %-8s %-9s\n', 'seg', 'arm', 'RMS', 'rel[%]', 'corr', 'lag[ms]');
    for s = 1:numel(segn)
        switch segn{s}
            case 'hold';  win = seg.hold;  dl = false;
            case 'desc';  win = seg.desc;  dl = false;
            case 'osc';   win = seg.osc;   dl = true;
            otherwise;    win = win_all;   dl = true;
        end
        for a = 1:numel(arms)
            if strcmp(kind, 'again')
                tr_use = arms{a}.a_true;
            else
                tr_use = truth;
            end
            m = seg_metric(get_est(arms{a}, kind), tr_use, win, per_samp, Ts, dl);
            fprintf('  %-5s %-22s %-11.4f %-9.1f %-8s %-9s\n', segn{s}, arms{a}.label, m.rms*nm, m.rel, ...
                    fmt_num(m.corr, '%.3f'), fmt_num(m.lag_ms, '%+.1f'));
        end
    end
end

function print_ratio_table(arms, ids, seg)
    segn = {'hold', 'desc', 'osc', 'all'};
    fprintf('\n a-hat / a-true ratio (mean per segment):\n');
    fprintf('  %-5s', 'seg');
    for a = 1:numel(arms);  fprintf(' %-8s', ids{a});  end
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
            fprintf(' %-8.4f', r);
        end
        fprintf('\n');
    end
end

function print_track_table(arms, ids, seg)
    segn = {'hold', 'desc', 'osc', 'all'};
    fprintf('\n tracking error  delta_h  std (nm):\n');
    fprintf('  %-5s', 'seg');
    for a = 1:numel(arms);  fprintf(' %-8s', ids{a});  end
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
            fprintf(' %-8.1f', std(arms{a}.dh_track(win))*1e3);
        end
        fprintf('\n');
    end
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

function s = tern(c, a, b)
    if c;  s = a;  else;  s = b;  end
end


%% ==================================================================
%  Comparison figure (lab convention: no grid/title, legend northoutside,
%  bold FS, true = thick RED, arms distinct solid colors)
%% ==================================================================
function fig_path = make_cheat_honest_figure(tout, tr, oracle_a, oracle_ap, arms, labels, project_root)
    nm = 1e3;  FS = 18;
    col_true = [0.85 0 0];                                   % thick red = truth (lab convention)
    col = { [0 0.45 0.74], ...    % 1st oracle-FF (cheat)  blue
            [0.93 0.50 0.06], ... % 2nd oracle-FF (cheat)  orange
            [0.20 0.62 0.20], ... % 2nd honest closure     green
            [0.55 0.20 0.60] };   % 1st honest             purple
    lw_arm = [2.0 1.6 2.0 1.6];   % cheat/honest pairs; keep both visible under overlap
    fig_dir = fullfile(project_root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if ~exist(fig_dir, 'dir');  mkdir(fig_dir);  end
    fig_path = fullfile(fig_dir, 'aprime_hd_cheat_vs_honest.png');

    fig = figure('Color', 'w', 'Position', [80 60 1120 1120]);

    % ---- Panel a: a'_hat vs true a'(h_d)  (KEY PANEL) --------------------
    ax1 = subplot(3,1,1);  hold(ax1, 'on');
    h_true = plot(ax1, tout, oracle_ap*nm, 'Color', col_true, 'LineWidth', 3.0);
    h_arm = gobjects(numel(arms),1);
    for a = 1:numel(arms)
        h_arm(a) = plot(ax1, tout, arms{a}.ap_hat*nm, 'Color', col{a}, 'LineWidth', lw_arm(a));
    end
    mark_segments(ax1, tr);
    ylabel(ax1, 'a''_{hd}  [nm/pN/\mum]', 'FontSize', FS, 'FontWeight', 'bold');
    style_axis(ax1, FS);
    yl = [min(oracle_ap*nm) max(oracle_ap*nm)];  pad = 0.18*max(diff(yl),eps);
    ylim(ax1, [yl(1)-pad, yl(2)+pad]);
    lg = legend([h_true; h_arm], [{'true a''(h_d)'}, labels(:)'], ...
                'Location', 'northoutside', 'NumColumns', 3, 'FontSize', FS-4);
    lg.Box = 'off';
    text(ax1, 0.008, 0.90, '(a)', 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');

    % ---- Panel b: a_hat vs true a ---------------------------------------
    ax2 = subplot(3,1,2);  hold(ax2, 'on');
    plot(ax2, tout, oracle_a*nm, 'Color', col_true, 'LineWidth', 3.0);
    for a = 1:numel(arms)
        plot(ax2, tout, arms{a}.a_hat*nm, 'Color', col{a}, 'LineWidth', lw_arm(a));
    end
    mark_segments(ax2, tr);
    ylabel(ax2, 'a_{h}  [nm/pN]', 'FontSize', FS, 'FontWeight', 'bold');
    style_axis(ax2, FS);
    text(ax2, 0.008, 0.90, '(b)', 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');

    % ---- Panel c: tracking error delta_h --------------------------------
    ax3 = subplot(3,1,3);  hold(ax3, 'on');
    for a = 1:numel(arms)
        plot(ax3, tout, arms{a}.dh_track*nm, 'Color', col{a}, 'LineWidth', 1.0);
    end
    mark_segments(ax3, tr);
    ylabel(ax3, 'tracking error \delta h  [nm]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax3, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    style_axis(ax3, FS);
    text(ax3, 0.008, 0.90, '(c)', 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');

    linkaxes([ax1 ax2 ax3], 'x');  xlim(ax1, [0 tout(end)]);
    drawnow;
    exportgraphics(fig, fig_path, 'Resolution', 150);
    close(fig);
end

function mark_segments(ax, tr)
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
