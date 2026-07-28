function out = temp_var_centered_verify(varargin)
%TEMP_VAR_CENTERED_VERIFY  Simulation check of 5state_aprime_var_centered.tex.
%
%   out = temp_var_centered_verify()                 % run all levels C1-C5
%   out = temp_var_centered_verify('levels', 1:4)    % subset
%   out = temp_var_centered_verify('T_sim', 8, ...)  % override any PAR field
%
%   Verifies the REFERENCE-CENTERED a'_hdm channel of
%   reference/eq17_analysis/derivation/5state_aprime_var_centered.tex, whose
%   claim is that subtracting a MAINTAINED deterministic reference
%   a_det_hat[k+1] = a_det_hat[k] + ahat'*Dh_d (init a_nom) -- instead of the
%   blind EWMA mean-tracker of the companion 5state_aprime_var_esti.tex --
%   removes the deterministic-swing leak (the W1 dwell-only restriction), so
%   the channel is valid for ANY trajectory (here a 1 Hz sweep).
%
%   Simplified ladder C1-C5 (all on the 1 Hz canonical scenario):
%     C1  W1-kill main proof: s = a_h_true - a_h(h_d[k]) (true numerator,
%         ORACLE reference), denom true dh, sliding-window sqrt(Var s/Var dh)
%         vs oracle a'(h_d(t)) across the FULL run. Old (esti blind) = 13.5x.
%     C2  reference implementability: swap oracle ref for the maintained
%         recursion a_det_hat (ahat'=known lookup); e_det effect; + arm with
%         ahat'=0.8x oracle (self-repair-term magnitude).
%     C3  measurement source + kappa: s = a_xm - a_det_hat[k-d] (d=2 align),
%         freeze arms {0.5,1,2}x oracle via apscale; kappa_emp + var^2-ahat'^2
%         floor discriminator; noise floor.
%     C4  shadow EWMA: centered a'_hdm -> slow beta-EWMA (tau=0.35 s), shadow.
%     C5  closed loop: feed the centered a'_hdm back to the 3 a' use points
%         (temp_motion_control_law_eq17_4state_centered, aprime_source
%         ='varchan_centered').
%
%   TEMP diagnostic (chat 2026-07-16); kept for user review per task rules.
%   Nothing committed; no existing file modified.

    % ------------------------------------------------------------------
    % Paths
    % ------------------------------------------------------------------
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);
    out_dir = fullfile(project_root, 'test_results', 'temp_var_centered_figs');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % ------------------------------------------------------------------
    % Parameters
    % ------------------------------------------------------------------
    PAR.levels   = 1:5;
    PAR.a_cov    = 0.05;          % fast EWMA weight (sig2_s / sig2_dh)
    PAR.win_s    = 0.5;           % [s] sliding window for C1/C2
    PAR.tau      = 0.35;          % [s] slow beta-EWMA time constant (C4/C5)
    PAR.t_warm   = 0.5;           % [s] slow-EWMA warm-up freeze
    PAR.scales   = [0.5 1.0 2.0]; % C3 frozen a' arms (x oracle)
    PAR.c1_seeds = 1:2;
    PAR.c3_seeds = 1:4;
    for i = 1:2:numel(varargin)
        PAR.(varargin{i}) = varargin{i+1};
    end

    % ------------------------------------------------------------------
    % 1 Hz canonical scenario (= temp_var1hz_figs)
    % ------------------------------------------------------------------
    pc = physical_constants(); R = pc.R;
    cfg = user_config();
    cfg.eq17_variant='4state'; cfg.trajectory_type='osc';
    cfg.h_init=50; cfg.h_bottom=2.0*R; cfg.amplitude=2.5;
    cfg.frequency=1; cfg.n_cycles=4; cfg.t_hold=0.5; cfg.t_descend_override=1.0;
    cfg.h_min=1.05*R; cfg.T_sim = cfg.t_hold + cfg.t_descend_override + cfg.n_cycles/cfg.frequency + 0.5; % 6.0
    cfg.ctrl_enable=true; cfg.thermal_enable=true; cfg.meas_noise_enable=true;
    cfg.lambda_c=0.7; cfg.a_pd=0.05; cfg.a_cov=0.05;
    cfg.meas_noise_std=[0.00062;0.00057;0.00331];
    cfg.suppress_xD=true; cfg.h_bar_safe=1;
    cfg.use_taylor_gain=true; cfg.aprime_source='known';

    Pp = calc_simulation_params(cfg); Pp = Pp.Value;
    a_nom = Pp.common.Ts / Pp.ctrl.gamma;
    Ts = Pp.common.Ts;  w = Pp.wall.w_hat;  pz = Pp.wall.pz;
    S.cfg = cfg; S.R = R; S.a_nom = a_nom; S.Ts = Ts; S.w = w; S.pz = pz;
    S.a_cov = PAR.a_cov; S.win = round(PAR.win_s / Ts);
    S.beta = 1 - Ts / PAR.tau; S.t_warm = PAR.t_warm; S.tau = PAR.tau;
    S.out_dir = out_dir;
    % window indices (shared)
    S.t1 = cfg.t_hold; S.t2 = S.t1 + cfg.t_descend_override;
    S.t3 = S.t2 + cfg.n_cycles/cfg.frequency;

    fprintf('==================================================================\n');
    fprintf(' 5state_aprime_var_centered.tex verification (C1-C5)\n');
    fprintf(' scenario: 1 Hz canonical, h_bar 22.2->2.0, hold->descent->osc, T=%.1fs\n', cfg.T_sim);
    fprintf(' a_nom=%.4e, a_cov=%.3f, win=%.2fs (%d smp), tau=%.2fs (beta=%.4f)\n', ...
            a_nom, PAR.a_cov, PAR.win_s, S.win, PAR.tau, S.beta);
    fprintf(' esti baseline (blind mean-tracker): 1 Hz shadow ratio 13.5, corr 0.29\n');
    fprintf('==================================================================\n\n');

    out.PAR = PAR; out.S = S;
    if any(PAR.levels==1); out.c1 = run_C1(S, PAR); end
    if any(PAR.levels==2); out.c2 = run_C2(S, PAR); end
    if any(PAR.levels==3); out.c3 = run_C3(S, PAR); end
    if any(PAR.levels==4); out.c4 = run_C4(S, PAR); end
    if any(PAR.levels==5); out.c5 = run_C5(S, PAR); end
    if any(PAR.levels==6); out.d  = run_D(S, PAR);  end   % restoring-force experiment (hold)
    if any(PAR.levels==7); out.e  = run_E(S, PAR);  end   % closed-loop convergence in a hold
    if any(PAR.levels==8); out.e100 = run_E100(S, PAR); end % charter-legal far-field hold (h=100)
    if any(PAR.levels==9); out.e1s  = run_E1S(S, PAR);  end % a'-as-state 5-state KF on 1 Hz
    if any(PAR.levels==11); out.e1so = run_E1SO(S, PAR); end % a'-as-state + selfmod: FULL-PATH estimation incl. holds

    % merge-save: keep results of levels not run this invocation
    mat_path = fullfile(script_dir, 'temp_var_centered_results.mat');
    if exist(mat_path, 'file')
        old = load(mat_path);
        if isfield(old, 'out')
            fn = fieldnames(old.out);
            for i = 1:numel(fn)
                if ~isfield(out, fn{i}); out.(fn{i}) = old.out.(fn{i}); end
            end
        end
    end
    save(mat_path, 'out');
    fprintf('\nsaved: %s\n', mat_path);
end


%% ==================== C1 ====================
function c1 = run_C1(S, PAR)
    fprintf('--- C1: W1-kill main proof (true numerator + ORACLE reference) ---\n');
    nS = numel(PAR.c1_seeds);
    c1.ratio_osc = zeros(nS,1); c1.corr_osc = zeros(nS,1);
    c1.ratio_full = zeros(nS,1); c1.corr_full = zeros(nS,1);
    for si = 1:nS
        so = run_pure_simulation(S.cfg, struct('seed',PAR.c1_seeds(si),'collect_diag',true));
        [t, h_d, h_tru, ~, dh_true, ~, a_tru, a_det_oracle, ~, ~, ~, oap] = extract(so, S);
        % s = a_h(h_true) - a_h(h_d) = a_r  (oracle reference removes det swing)
        s = a_tru - a_det_oracle;
        ap_win = sqrt( movvar(s, S.win, 0, 'Endpoints','shrink') ./ ...
                       max(movvar(dh_true, S.win, 0, 'Endpoints','shrink'), realmin) );
        [c1.ratio_osc(si), c1.corr_osc(si), c1.ratio_full(si), c1.corr_full(si)] = ...
            metrics(ap_win, oap, t, S);
        if si == 1
            c1.t = t; c1.ap_win = ap_win; c1.oap = oap; c1.dh = dh_true;
        end
    end
    fprintf('  osc-window  : mean ratio=%.3f +/- %.3f, corr=%.3f (n=%d seeds)\n', ...
            mean(c1.ratio_osc), std(c1.ratio_osc), mean(c1.corr_osc), nS);
    fprintf('  full (descent+osc, t>t_hold): mean ratio=%.3f, corr=%.3f\n', ...
            mean(c1.ratio_full), mean(c1.corr_full));
    fprintf('  vs esti blind mean-tracker: ratio 13.5 -> %.2f  (W1 leak removed)\n\n', mean(c1.ratio_osc));
    % figure: full-run sliding-window a' vs oracle
    fig_overlay(c1.t, c1.oap, c1.ap_win, S, ...
        sprintf('C1: osc ratio=%.2f corr=%.2f', mean(c1.ratio_osc), mean(c1.corr_osc)), ...
        '\^a''_{hd} (sliding-window, true+oracle)', 'C1_window_overlay');
end


%% ==================== C2 ====================
function c2 = run_C2(S, PAR)
    fprintf('--- C2: reference implementability (maintained a_det recursion) ---\n');
    so = run_pure_simulation(S.cfg, struct('seed',1,'collect_diag',true));
    [t, h_d, h_tru, ~, dh_true, ~, a_tru, a_det_oracle, ~, ~, ~, oap] = extract(so, S);
    arms = struct('name',{'ahat''=oracle','ahat''=0.8xoracle'}, 'scale',{1.0, 0.8});
    c2.arm = arms;
    for a = 1:numel(arms)
        apser = arms(a).scale * oap;                 % known-lookup slope (scaled)
        adet  = build_adet(h_d, apser, S.a_nom);     % maintained reference
        s = a_tru - adet;
        ap_win = sqrt( movvar(s, S.win, 0, 'Endpoints','shrink') ./ ...
                       max(movvar(dh_true, S.win, 0, 'Endpoints','shrink'), realmin) );
        [ro, co, rf, cf] = metrics(ap_win, oap, t, S);
        c2.ratio_osc(a) = ro; c2.corr_osc(a) = co; c2.ratio_full(a) = rf;
        c2.e_det_osc(a) = std_osc(adet - a_det_oracle, t, S);   % reference error std [um/pN]
        if a==1; c2.t=t; c2.oap=oap; c2.ap_win1=ap_win; end
        if a==2; c2.ap_win2=ap_win; end
        fprintf('  %-16s: osc ratio=%.3f corr=%.3f | full ratio=%.3f | e_det std=%.2e\n', ...
                arms(a).name, ro, co, rf, c2.e_det_osc(a));
    end
    fprintf('  (C1 oracle-ref osc ratio was ~1.0; e_det from Euler recursion is the extra term)\n\n');
end


%% ==================== C3 ====================
function c3 = run_C3(S, PAR)
    fprintf('--- C3: measurement source + kappa  (s = a_xm - a_det[k-d], d=2) ---\n');
    nS = numel(PAR.c3_seeds); nSc = numel(PAR.scales);
    meanhdm = zeros(nSc,nS); mean_ap_frozen = zeros(nSc,1); mean_oracle = 0;
    for ci = 1:nSc
        sc = PAR.scales(ci);
        for si = 1:nS
            cfg2 = S.cfg; cfg2.aprime_scale = sc;
            so = temp_run_pure_sim_apscale(cfg2, struct('seed',PAR.c3_seeds(si),'collect_diag',true));
            [t, h_d, ~, ~, ~, dh_m, ~, ~, a_xm, ~, ~, oap] = extract(so, S);
            apser = sc * oap;                              % frozen slope used by controller
            adet  = build_adet(h_d, apser, S.a_nom);       % maintained reference
            adet_d = [adet(1); adet(1); adet(1:end-2)];    % delay by d=2 (a_xm[k] ~ a_h[k-d])
            s = a_xm - adet_d;
            [aphat_raw] = centered_ewma(s, dh_m, S.a_cov); % sqrt(sig2_s/sig2_dh)
            meanhdm(ci,si) = mean_osc(aphat_raw, t, S);
            if ci==1 && si==1; mean_oracle = mean_osc(oap, t, S); end
        end
        mean_ap_frozen(ci) = sc * mean_oracle;
    end
    c3.scales = PAR.scales; c3.meanhdm = meanhdm; c3.ap_frozen = mean_ap_frozen;
    c3.oracle = mean_oracle;
    fprintf('  osc-window oracle a''(h_d) mean = %.3e\n', mean_oracle);
    fprintf('  %-7s %-13s %-13s %-13s %-16s\n','scale','mean(a''hdm)','ahat''(frozen)','a''_true','kappa_emp');
    kacc = [];
    for ci = 1:nSc
        sc = PAR.scales(ci);
        mv = mean(meanhdm(ci,:)); sv = std(meanhdm(ci,:));
        ap = mean_ap_frozen(ci);
        if abs(mean_oracle - ap) < 1e-30
            fprintf('  %-7.2f %.4e   %.4e   %.4e   -- (denom 0)\n', sc, mv, ap, mean_oracle);
        else
            kk = (meanhdm(ci,:) - ap) / (mean_oracle - ap);
            kacc = [kacc, kk]; %#ok<AGROW>
            fprintf('  %-7.2f %.4e   %.4e   %.4e   %+.3f +/- %.3f\n', sc, mv, ap, mean_oracle, mean(kk), std(kk));
        end
        c3.floor(ci) = mv^2 - ap^2;
    end
    fprintf('  pooled kappa_emp (arms 0.5 & 2.0): %+.3f +/- %.3f\n', mean(kacc), std(kacc));
    fprintf('  floor discriminator (var^2 - ahat''^2 vs kappa=1 cross-term 2*ahat''*e_a''):\n');
    for ci = 1:nSc
        ap = mean_ap_frozen(ci);
        fprintf('    scale %.2f: var^2-ahat''^2 = %+.3e  [kappa=1 would give %+.3e]\n', ...
                PAR.scales(ci), c3.floor(ci), 2*ap*(mean_oracle-ap));
    end
    c3.kappa_pooled = mean(kacc); c3.kappa_std = std(kacc);
    fprintf('\n');
    % figure: three arms mean(a'hdm) vs frozen ahat' and oracle
    fig_c3_arms(c3, S);
end


%% ==================== C4 ====================
function c4 = run_C4(S, PAR)
    fprintf('--- C4: shadow EWMA (centered a''_hdm, slow beta-EWMA) ---\n');
    so = run_pure_simulation(S.cfg, struct('seed',1,'collect_diag',true));
    [t, h_d, h_tru, ~, dh_true, dh_m, a_tru, a_det_oracle, a_xm, a_hat, dxh3, oap] = extract(so, S);
    N = numel(t);
    % (A) self-consistent maintained reference (the true online recursion: a_det
    %     built from the slow a'_hdm EWMA), measurement source a_xm (delay d=2)
    %     and estimator source a_hat (dxhat3 denom).
    [ap_meas, ~, tdiv_m] = centered_online(a_xm, dh_m, h_d, S, oap, 2);
    [ap_esti, ~, tdiv_e] = centered_online(a_hat, dxh3, h_d, S, oap, 0);
    % (B) decoupled ORACLE reference (best case: a_det = a_h(h_d) exact) -- stable,
    %     isolates the numerator quality of each source.
    adet_d = [a_det_oracle(1); a_det_oracle(1); a_det_oracle(1:end-2)];   % d=2 delayed
    ap_meas_dec = slow_beta(centered_ewma(a_xm - adet_d, dh_m, S.a_cov), oap, t, S);
    ap_esti_dec = slow_beta(centered_ewma(a_hat - a_det_oracle, dxh3, S.a_cov), oap, t, S);
    c4.t=t; c4.oap=oap; c4.ap_meas=ap_meas_dec; c4.ap_esti=ap_esti_dec;
    c4.a_tru=a_tru; c4.a_hat=a_hat; c4.dxz=(h_d-h_tru)*1e3;
    c4.tdiv_meas=tdiv_m; c4.tdiv_esti=tdiv_e;
    [c4.ratio_meas, c4.corr_meas] = metrics(ap_meas_dec, oap, t, S);
    [c4.ratio_esti, c4.corr_esti] = metrics(ap_esti_dec, oap, t, S);
    fprintf('  (A) self-consistent online reference (true online recursion):\n');
    fprintf('      measurement source: reference DIVERGES at t=%.3f s (positive feedback in descent)\n', tdiv_m);
    fprintf('      estimator   source: reference DIVERGES at t=%.3f s\n', tdiv_e);
    fprintf('  (B) decoupled ORACLE reference (best-case numerator, stable):\n');
    fprintf('      measurement source (a_xm): osc ratio=%.2f corr=%.2f  (a_xm noise floor)\n', c4.ratio_meas, c4.corr_meas);
    fprintf('      estimator   source (a_hat): osc ratio=%.2f corr=%.2f  (circular: reads back known a'')\n', c4.ratio_esti, c4.corr_esti);
    fprintf('  vs esti blind mean-tracker shadow: ratio 13.5, corr 0.29\n\n');
    fig_c4_combo(c4, S);
end


%% ==================== C5 ====================
function c5 = run_C5(S, PAR)
    fprintf('--- C5: closed loop (centered a''_hdm fed back to 3 a'' use points) ---\n');
    % shadow reference (known a')
    soS = run_pure_simulation(S.cfg, struct('seed',1,'collect_diag',true));
    [tS, ~, hS_tru, ~, ~, ~, ~, ~, ~, ~, ~, oapS] = extract(soS, S);
    dxz_s = (soS.p_d_out*S.w - S.pz - hS_tru)*1e3;
    osc = (tS>=S.t2+1)&(tS<=S.t3);
    trk_std_s = std(dxz_s(osc));

    % closed-loop centered varchan
    cfgV = S.cfg; cfgV.aprime_source='varchan_centered';
    cfgV.varchan_tau = S.tau; cfgV.varchan_warm = S.t_warm;
    warning('off','MATLAB:nearlySingularMatrix');
    soV = temp_run_pure_sim_centered(cfgV, struct('seed',1,'collect_diag',true));
    warning('on','MATLAB:nearlySingularMatrix');
    t = soV.tout; N=numel(t); w=S.w; pz=S.pz; R=S.R;
    az_true = soV.a_true_out(:,3); az_hat = soV.diag.a_hat(:,3);
    ap_cl = soV.diag.a_prime_used(:,3);
    hbar_cl = (soV.p_true_out*w-pz)/R;
    dxz_cl = (soV.p_d_out*w - pz - (soV.p_true_out*w-pz))*1e3;
    oracle_ap = zeros(N,1);
    for k=1:N
        [~,cp,drv]=calc_correction_functions(max((soV.p_d_out(k,:)*w-pz)/R,1.001),true);
        oracle_ap(k)=-(S.a_nom/cp)*drv.K_h_perp/R;
    end
    bad = (t>S.t_warm) & (hbar_cl>40 | hbar_cl<1.05 | abs(dxz_cl)>1000 | ~isfinite(dxz_cl) | ~isfinite(az_hat));
    idiv = find(bad,1);
    if isempty(idiv); t_div=t(end); diverged=false; iend=N; else; t_div=t(idiv); diverged=true; iend=idiv; end
    c5.diverged=diverged; c5.t_div=t_div; c5.trk_std_shadow=trk_std_s;
    if ~diverged
        osc = (t>=S.t2+1)&(t<=S.t3);
        c5.trk_std_cl = std(dxz_cl(osc));
        c5.ap_ratio = mean(ap_cl(osc)./oracle_ap(osc));
        cc=corrcoef(ap_cl(osc),oracle_ap(osc)); c5.ap_corr=cc(1,2);
        c5.a_ratio = mean(az_hat(osc)./az_true(osc));
    else
        c5.trk_std_cl=NaN; c5.ap_ratio=NaN; c5.ap_corr=NaN; c5.a_ratio=NaN;
    end
    fprintf('  shadow tracking std (osc) = %.1f nm\n', trk_std_s);
    if diverged
        fprintf('  CLOSED LOOP DIVERGES at t=%.3f s (h_bar/dxz out of bounds)\n', t_div);
        fprintf('  a''_hd peak before div = %.2e (oracle ~ %.2e)\n', max(ap_cl(t<=t_div)), oracle_ap(1));
    else
        fprintf('  CLOSED LOOP STABLE. osc tracking std=%.1f nm; a''_hd ratio=%.2f corr=%.2f; a_hat ratio=%.2f\n', ...
                c5.trk_std_cl, c5.ap_ratio, c5.ap_corr, c5.a_ratio);
    end
    fprintf('\n');
    fig_c5_combo(t, oracle_ap, ap_cl, az_true, az_hat, dxz_cl, iend, diverged, t_div, S);
end


%% ==================== E (closed-loop convergence in a hold) ====================
function e = run_E(S, PAR)
    fprintf('--- E: closed-loop varchan (esti source) in a HOLD @ h_bar=2.0, T=20s, seed 1 ---\n');
    R = S.R; a_nom = S.a_nom;
    cfg = S.cfg;
    cfg.trajectory_type = 'positioning';   % pure hold, Dh_d=0 (taylor FF silent)
    cfg.h_init = 2.0 * R; cfg.T_sim = 20.0;
    [~, cperp_d, drv_d] = calc_correction_functions(2.0, true);
    a_true_hd = a_nom / cperp_d;
    oracle_ap = -a_true_hd * drv_d.K_h_perp / R;
    e.oracle_ap = oracle_ap; e.a_true_hd = a_true_hd;
    arms = struct('name',{'E-A: init=oracle (from truth)','E-B: init=0.5x oracle'}, 'scale',{1.0, 0.5});
    e.arm = arms;
    for a = 1:numel(arms)
        cfgv = cfg; cfgv.aprime_source='varchan';
        cfgv.varchan_tau = S.tau; cfgv.varchan_warm = S.t_warm;
        cfgv.varchan_ap_init_scale = arms(a).scale;
        warning('off','MATLAB:nearlySingularMatrix');
        so = temp_run_pure_sim_centered(cfgv, struct('seed',1,'collect_diag',true));
        warning('on','MATLAB:nearlySingularMatrix');
        t = so.tout; N = numel(t);
        ap  = so.diag.a_prime_used(:,3);
        az_hat = so.diag.a_hat(:,3);
        az_true = so.a_true_out(:,3);
        h_d = so.p_d_out*S.w - S.pz; h_tru = so.p_true_out*S.w - S.pz;
        dxz = (h_d - h_tru)*1e3;
        % divergence check
        bad = (t>S.t_warm) & (~isfinite(az_hat) | ~isfinite(ap) | abs(dxz)>1000 | az_hat<=0);
        idiv = find(bad,1); diverged = ~isempty(idiv);
        % metrics
        last = t >= (cfg.T_sim - 2);
        ap_final = mean(ap(last));
        % drift rate: linear slope of ap over t in [1, T-1] s
        win = (t>=1) & (t<=cfg.T_sim-1) & isfinite(ap);
        pf = polyfit(t(win), ap(win), 1); drift_per_s = pf(1);      % [um/pN/um per s]
        drift_pct_per_s = 100 * drift_per_s / oracle_ap;            % %truth per s
        std_early = std(dxz((t>=2)&(t<=4)));
        std_late  = std(dxz((t>=cfg.T_sim-4)&(t<=cfg.T_sim-2)));
        e.res(a) = struct('t',t,'ap',ap,'az_hat',az_hat,'az_true',az_true,'dxz',dxz, ...
                          'ap_final',ap_final,'drift_per_s',drift_per_s, ...
                          'drift_pct_per_s',drift_pct_per_s,'std_early',std_early, ...
                          'std_late',std_late,'diverged',diverged);
        fprintf('  %-28s: ap_final/truth=%.2fx | drift=%.2f%%truth/s | trk std %.1f->%.1f nm | %s\n', ...
                arms(a).name, ap_final/oracle_ap, drift_pct_per_s, std_early, std_late, ...
                ternary(diverged, sprintf('DIVERGES t=%.2fs', t(idiv)), 'stable'));
    end
    % verdict
    convA = abs(e.res(1).ap_final/oracle_ap - 1) < 0.10 && abs(e.res(1).drift_pct_per_s) < 1;
    fprintf('  VERDICT: from-truth arm (E-A) ap_final=%.2fx truth, drift=%.2f%%/s -> %s\n', ...
            e.res(1).ap_final/oracle_ap, e.res(1).drift_pct_per_s, ...
            ternary(convA, 'truth ~ stationary (stays put)', 'truth NOT a fixed point (drifts)'));
    fprintf('\n');
    fig_e_hold(e, S, oracle_ap, a_true_hd);
end

function s = ternary(c, a, b)
    if c; s = a; else; s = b; end
end

function fig_e_hold(e, S, oracle_ap, a_true_hd)
    GRN=[0 0.6 0]; RED=[0.8 0 0]; BLU=[0 0.2 0.8]; ORG=[0.9 0.45 0]; FS=15; LFS=11; LW=1.8;
    t = e.res(1).t; T = t(end);
    f=figure('Position',[80 80 1100 820],'Color','w','Visible','off');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    % top: a'(t) both arms + oracle
    nexttile; hold on;
    h1=plot(t, e.res(1).ap*1e3,'-','Color',RED,'LineWidth',1.7,'DisplayName','\^a''_{hd} (E-A: init=oracle)');
    h2=plot(t, e.res(2).ap*1e3,'-','Color',ORG,'LineWidth',1.7,'DisplayName','\^a''_{hd} (E-B: init=0.5\times)');
    ho=yline(oracle_ap*1e3,'--','Color',GRN,'LineWidth',2.4,'DisplayName','oracle a''(h_d)');
    xlim([0 T]); ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    legend([h1 h2 ho],'Location','northoutside','Orientation','horizontal','FontSize',LFS-1,'FontWeight','bold','Box','on');
    text(0.985,0.90,sprintf('E-A drift %.2f%%/s, final %.2fx | E-B final %.2fx',e.res(1).drift_pct_per_s,e.res(1).ap_final/oracle_ap,e.res(2).ap_final/oracle_ap), ...
         'Units','normalized','HorizontalAlignment','right','FontSize',LFS-1,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % bottom: a_z hat quality both arms
    nexttile; hold on;
    g0=plot(t, e.res(1).az_true*1e3,'-','Color',GRN,'LineWidth',2.6,'DisplayName','a_{z,true}');
    g1=plot(t, e.res(1).az_hat*1e3,'-','Color',RED,'LineWidth',1.4,'DisplayName','\^a_z (E-A)');
    g2=plot(t, e.res(2).az_hat*1e3,'-','Color',ORG,'LineWidth',1.4,'DisplayName','\^a_z (E-B)');
    xlim([0 T]); ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([g0 g1 g2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'E1_closedloop_hold.png'),'Resolution',200); close(f);
    fprintf('  [E1_closedloop_hold]\n');
end


%% ==================== E1S (a'-as-state 5-state KF, 1 Hz) ====================
function e = run_E1S(S, PAR)
%RUN_E1S  a' promoted to state 5 (aprime_source='state', z axis), 1 Hz canonical.
%   Information route: y2 via F_e(4,5)=Dh_d coupling and P45 (no y3, no var
%   chain). Slot 5 inits at 0 and must learn from sweep excitation.
%   Baseline arm: aprime_source='known' (lookup a'), same scenario/seed.
    fprintf('--- E1S: a''-as-state 5-state KF on 1 Hz canonical (seed 1) ---\n');
    R = S.R; a_nom = S.a_nom; w = S.w; pz = S.pz;

    % state arm
    cfgS = S.cfg; cfgS.aprime_source = 'state';
    warning('off','MATLAB:nearlySingularMatrix');
    soS = temp_run_pure_sim_centered(cfgS, struct('seed',1,'collect_diag',true));
    warning('on','MATLAB:nearlySingularMatrix');
    % known arm
    soK = temp_run_pure_sim_centered(S.cfg, struct('seed',1,'collect_diag',true));

    t = soS.tout; N = numel(t);
    ap_state = soS.diag.a_prime_used(:, 3);      % slot-5 posterior[k-1] (control-time value)
    P55      = soS.diag.P_aprime(:, 3);
    azS_hat = soS.diag.a_hat(:, 3);  azS_true = soS.a_true_out(:, 3);
    azK_hat = soK.diag.a_hat(:, 3);
    dxzS = (soS.p_d_out*w - pz - (soS.p_true_out*w - pz))*1e3;
    dxzK = (soK.p_d_out*w - pz - (soK.p_true_out*w - pz))*1e3;
    oracle_ap = zeros(N, 1);
    for k = 1:N
        hbd = max((soS.p_d_out(k,:)*w - pz)/R, 1.001);
        [~, cp, drv] = calc_correction_functions(hbd, true);
        oracle_ap(k) = -(a_nom/cp) * drv.K_h_perp / R;
    end

    % metrics
    osc  = (t >= S.t2 + 1) & (t <= S.t3);
    near = osc & (oracle_ap > 0.5*max(oracle_ap));   % near-wall half of each cycle
    cc = corrcoef(ap_state(osc), oracle_ap(osc));
    e.corr_osc   = cc(1,2);
    e.ratio_near = mean(ap_state(near) ./ oracle_ap(near));
    e.err_rms    = rms(ap_state(osc) - oracle_ap(osc));
    e.p55_rms    = rms(sqrt(P55(osc)));
    e.honesty    = e.p55_rms / e.err_rms;            % <1 = overconfident
    e.trk_std_S  = std(dxzS(osc));  e.trk_std_K = std(dxzK(osc));
    hold_win = t <= S.t1;
    e.ap_hold_max = max(abs(ap_state(hold_win)));
    e.t=t; e.ap_state=ap_state; e.P55=P55; e.oracle_ap=oracle_ap;
    e.azS_hat=azS_hat; e.azK_hat=azK_hat; e.azS_true=azS_true;
    e.dxzS=dxzS; e.dxzK=dxzK;

    % var/var statistic (doc esti chain) computed OFFLINE on this run's logs,
    % for direct comparison with the state-5 estimate and truth (W1 check)
    dh3S = soS.diag.delta_x_hat_3(:, 3);
    e.dh3S = dh3S;
    e.varr_raw = local_var_channel_esti(azS_hat, dh3S, S.a_cov, S.a_cov);
    varr_beta = zeros(N, 1); vb = 0;
    for k = 1:N
        if k > 42                                    % fixed-step guard (E100 convention)
            vb = S.beta*vb + (1 - S.beta)*e.varr_raw(k);
        end
        varr_beta(k) = vb;
    end
    e.varr_beta = varr_beta;
    e.var_osc_mean = mean(varr_beta(osc));
    e.var_vs_oracle = e.var_osc_mean / mean(oracle_ap(osc));
    fprintf('  var/var (esti chain, offline): osc mean = %.3e = %.1fx oracle mean (W1 leak)\n', ...
            e.var_osc_mean, e.var_vs_oracle);

    fprintf('  hold (0-%.1fs): |a''| max = %.2e (init 0, expect frozen)\n', S.t1, e.ap_hold_max);
    fprintf('  osc window: corr=%.3f | near-wall ratio=%.2f | err rms=%.3e\n', ...
            e.corr_osc, e.ratio_near, e.err_rms);
    fprintf('  P55 honesty: sqrt(P55) rms / err rms = %.2f (1=honest, <1 overconfident)\n', e.honesty);
    fprintf('  tracking std (osc): state=%.1f nm | known=%.1f nm\n\n', e.trk_std_S, e.trk_std_K);
end

%% ==================== E1SO (a'-as-state + selfmod, full-path estimation) ====================
function e = run_E1SO(S, PAR)
%RUN_E1SO  Full-path 1 Hz with a' estimated in EVERY segment (holds included):
%   aprime_source='state' + aprime_state_selfmod=true completes the F(4,5)
%   Jacobian (sweep Dh_d + deviation-correction (1-lc)*dxhat3 couplings), so
%   the thermal fluctuation keeps slot 5 weakly observable when Dh_d=0.
%   Baseline: the plain state arm (E1S red, sweep coupling only).
    fprintf('--- E1SO: a''-as-state + selfmod (full-path estimation), 1 Hz canonical, seed 1 ---\n');
    R = S.R; a_nom = S.a_nom; w = S.w; pz = S.pz;
    cfgO = S.cfg; cfgO.aprime_source = 'state'; cfgO.aprime_state_selfmod = true;
    warning('off','MATLAB:nearlySingularMatrix');
    soO = temp_run_pure_sim_centered(cfgO, struct('seed',1,'collect_diag',true));
    warning('on','MATLAB:nearlySingularMatrix');
    cfgK = S.cfg;   % known arm baseline
    soK = temp_run_pure_sim_centered(cfgK, struct('seed',1,'collect_diag',true));
    t = soO.tout; N = numel(t);
    e.t = t;
    e.ap  = soO.diag.a_prime_used(:, 3);
    e.P55 = soO.diag.P_aprime(:, 3);
    e.az_hat = soO.diag.a_hat(:, 3);  e.az_true = soO.a_true_out(:, 3);
    e.dxz  = (soO.p_d_out*w - pz - (soO.p_true_out*w - pz))*1e3;
    e.azK_hat = soK.diag.a_hat(:, 3);
    e.dxzK = (soK.p_d_out*w - pz - (soK.p_true_out*w - pz))*1e3;
    e.dh3 = soO.diag.delta_x_hat_3(:, 3);
    e.varr_raw = local_var_channel_esti(e.az_hat, e.dh3, S.a_cov, S.a_cov);
    oracle_ap = zeros(N, 1);
    for k = 1:N
        hbd = max((soO.p_d_out(k,:)*w - pz)/R, 1.001);
        [~, cp, drv] = calc_correction_functions(hbd, true);
        oracle_ap(k) = -(a_nom/cp) * drv.K_h_perp / R;
    end
    e.oracle_ap = oracle_ap;
    % segment stats
    segs = {'hold(far)', 0, S.t1; 'descent', S.t1, S.t2; 'osc', S.t2, S.t3; 'hold(near)', S.t3, t(end)};
    for i = 1:4
        m = (t >= segs{i,2}) & (t < segs{i,3});
        fprintf('  %-11s: a''_hat mean=%7.3f std=%6.3f | truth mean=%6.3f | sqrt(P55) mean=%6.3f [nm/pN/um]\n', ...
            segs{i,1}, mean(e.ap(m))*1e3, std(e.ap(m))*1e3, mean(oracle_ap(m))*1e3, mean(sqrt(e.P55(m)))*1e3);
    end
    osc = (t >= S.t2+1) & (t <= S.t3);
    cc = corrcoef(e.ap(osc), oracle_ap(osc));
    e.corr_osc = cc(1,2);
    e.trk_std_O = std(e.dxz(osc)); e.trk_std_K = std(e.dxzK(osc));
    fprintf('  osc corr=%.3f | trk std %.1f vs %.1f nm\n\n', e.corr_osc, e.trk_std_O, e.trk_std_K);
    fig_e1so_trace(e, S);
end

function fig_e1so_trace(e, S)
%FIG_E1SO_TRACE  Same 3-tile segment-annotated format as E1S_trace_segments.
    GRN=[0 0.6 0]; RED=[0.8 0 0]; BLU=[0 0.2 0.8]; ORG=[0.9 0.45 0]; FS=14; LFS=10; LW=1.6;
    t = e.t; T = t(end);
    osc = (t >= S.t2+1) & (t <= S.t3);
    f=figure('Position',[60 60 1100 1100],'Color','w','Visible','off');
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    nexttile; hold on;
    h3=plot(t, e.varr_raw*1e3,'-','Color',ORG,'LineWidth',0.8,'DisplayName','\^a''_{hdm} raw = sqrt(\sigma^2_{a,r}/\sigma^2_{\deltah,r})');
    h1=plot(t, e.ap*1e3,'-','Color',RED,'LineWidth',1.6,'DisplayName','\^a''_{hd} (state 5 + selfmod)');
    h2=plot(t, e.oracle_ap*1e3,'-','Color',GRN,'LineWidth',2.2,'DisplayName','a''_{true}(h_d(t))');
    for x=[S.t1 S.t2 S.t3]; xline(x,'--','Color',[.5 .5 .5],'LineWidth',1.2,'HandleVisibility','off'); end
    ylim([min(-4, 1.1*min(e.ap*1e3)), 1.15*prctile(e.varr_raw(osc),95)*1e3]);
    yl=ylim;
    text(S.t1/2, yl(2)*0.95,'hold','FontSize',LFS,'FontWeight','bold','HorizontalAlignment','center');
    text((S.t1+S.t2)/2, yl(2)*0.95,'descent','FontSize',LFS,'FontWeight','bold','HorizontalAlignment','center');
    text((S.t2+S.t3)/2, yl(2)*0.95,'osc','FontSize',LFS,'FontWeight','bold','HorizontalAlignment','center');
    text((S.t3+T)/2, yl(2)*0.95,'hold','FontSize',LFS,'FontWeight','bold','HorizontalAlignment','center');
    xlim([0 T]); ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    legend([h2 h1 h3],'Location','northoutside','Orientation','horizontal','FontSize',LFS-1,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    nexttile; hold on;
    g0=plot(t, e.az_true*1e3,'-','Color',GRN,'LineWidth',2.4,'DisplayName','a_{z,true}');
    g1=plot(t, e.az_hat*1e3,'-','Color',RED,'LineWidth',1.2,'DisplayName','\^a_z (selfmod arm)');
    g2=plot(t, e.azK_hat*1e3,'-','Color',BLU,'LineWidth',1.0,'DisplayName','\^a_z (known arm)');
    for x=[S.t1 S.t2 S.t3]; xline(x,'--','Color',[.5 .5 .5],'LineWidth',1.2,'HandleVisibility','off'); end
    xlim([0 T]); ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold');
    legend([g0 g1 g2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    nexttile; hold on;
    d1=plot(t, e.dxz,'-','Color',RED,'LineWidth',0.8,'DisplayName','\deltax_z (selfmod arm)');
    d2=plot(t, e.dxzK,'-','Color',BLU,'LineWidth',0.8,'DisplayName','\deltax_z (known arm)');
    for x=[S.t1 S.t2 S.t3]; xline(x,'--','Color',[.5 .5 .5],'LineWidth',1.2,'HandleVisibility','off'); end
    xlim([0 T]); ylabel('\deltax_z (nm)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([d1 d2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    text(0.985,0.90,sprintf('osc std: %.1f vs %.1f nm',e.trk_std_O,e.trk_std_K),'Units','normalized', ...
         'HorizontalAlignment','right','FontSize',LFS,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'E1SO_trace.png'),'Resolution',200); close(f);
    fprintf('  [E1SO_trace]\n');
end


%% ==================== E100 (charter-legal far-field hold) ====================
function e = run_E100(S, PAR)
%RUN_E100  Closed-loop varchan (esti source) hold @ h=100 um, charter-legal init.
%   Init basis (a_nom, a'=0): init_from_anom (production knob semantics),
%   varchan a' seed = 0 (K_h_init=0), P44 a_nom-tolerance prior (5% a_nom),
%   no time warm-up (varchan_warm=0 -> generic 2-step only).
%   At h_bar=44.4 truth is a_true ~ 0.975 a_nom, a'_true ~ 0 (1/500 of h_bar=2),
%   so this tests whether the channel DEGRADES a near-correct (a_hat, a'_hd):
%   var[k] >= 0 -> predicted pull-up of a'_hd to the rectification floor.
%   Two runs: main (varchan closed loop) + REF (aprime_source='known', same
%   init compliance) whose logs give the open-loop floor line.
    fprintf('--- E100: closed-loop varchan hold @ h=100 um (h_bar=44.4), charter-legal init, T=20s, seed 1 ---\n');
    R = S.R; a_nom = S.a_nom;
    cfg = S.cfg;
    cfg.trajectory_type = 'positioning';
    cfg.h_init = 100;  cfg.T_sim = 20.0;
    cfg.init_from_anom = true;  cfg.p44_prior_frac = 0.05;
    h_bar0 = 100 / R;
    [~, cperp_d, drv_d] = calc_correction_functions(h_bar0, true);
    a_true_hd = a_nom / cperp_d;
    oracle_ap = -a_true_hd * drv_d.K_h_perp / R;
    e.oracle_ap = oracle_ap; e.a_true_hd = a_true_hd; e.h_bar0 = h_bar0;
    fprintf('  truth @ h_bar=%.1f: a_true=%.4e (=%.4fx a_nom), a''_true=%.3e\n', ...
            h_bar0, a_true_hd, a_true_hd/a_nom, oracle_ap);

    % ---- REF arm: known a', same compliant init ----
    cfgR = cfg; cfgR.aprime_source = 'known';
    soR = temp_run_pure_sim_centered(cfgR, struct('seed',1,'collect_diag',true));
    tR = soR.tout;
    azR_hat = soR.diag.a_hat(:,3);  azR_true = soR.a_true_out(:,3);
    dxzR = (soR.p_d_out*S.w - S.pz - (soR.p_true_out*S.w - S.pz))*1e3;
    dh3R = soR.diag.delta_x_hat_3(:,3);
    % open-loop floor: run the esti var chain offline on REF logs
    varr_R = local_var_channel_esti(azR_hat, dh3R, S.a_cov, S.a_cov);
    floor_ol = mean(varr_R(tR >= 10));            % last 10 s
    e.floor_ol = floor_ol;

    % ---- main arm: varchan closed loop ----
    cfgV = cfg; cfgV.aprime_source = 'varchan';
    cfgV.varchan_tau = S.tau; cfgV.varchan_warm = 0;
    cfgV.varchan_warm_steps = 42;   % fixed-step guard: d + 2/a_cov = 2 + 40
    warning('off','MATLAB:nearlySingularMatrix');
    soV = temp_run_pure_sim_centered(cfgV, struct('seed',1,'collect_diag',true));
    warning('on','MATLAB:nearlySingularMatrix');
    t = soV.tout;
    ap = soV.diag.a_prime_used(:,3);
    az_hat = soV.diag.a_hat(:,3);  az_true = soV.a_true_out(:,3);
    hbar_cl = (soV.p_true_out*S.w - S.pz)/R;
    dxz = (soV.p_d_out*S.w - S.pz - (soV.p_true_out*S.w - S.pz))*1e3;
    bad = (t > 0.1) & (hbar_cl > 80 | hbar_cl < 1.05 | abs(dxz) > 1000 | ...
                       ~isfinite(dxz) | ~isfinite(az_hat) | az_hat <= 0);
    idiv = find(bad, 1); diverged = ~isempty(idiv);
    if diverged; iend = idiv; t_div = t(idiv); else; iend = numel(t); t_div = t(end); end

    % ---- metrics ----
    last = t >= (cfg.T_sim - 2);
    e.ap_final = mean(ap(last));
    win = (t >= 1) & (t <= cfg.T_sim - 1) & isfinite(ap);
    pf = polyfit(t(win), ap(win), 1);
    e.drift_per_s = pf(1);
    e.a_ratio_cl  = mean(az_hat(last) ./ az_true(last));
    e.a_ratio_ref = mean(azR_hat(tR >= cfg.T_sim - 2) ./ azR_true(tR >= cfg.T_sim - 2));
    e.trk_std_cl  = std(dxz((t >= 10) & (t <= 18)));
    e.trk_std_ref = std(dxzR((tR >= 10) & (tR <= 18)));
    e.diverged = diverged; e.t_div = t_div;
    e.t = t; e.ap = ap; e.az_hat = az_hat; e.az_true = az_true; e.dxz = dxz;
    e.tR = tR; e.azR_hat = azR_hat; e.dxzR = dxzR; e.varr_R = varr_R;
    % full varchan chain internals (z axis) -- saved for offline re-analysis
    e.dh3  = soV.diag.delta_x_hat_3(:, 3);
    e.dh3R = dh3R;
    e.vc = struct('s2ar', soV.diag.vc_s2ar, 's2dh', soV.diag.vc_s2dh, ...
                  'abar', soV.diag.vc_abar, 'dbar', soV.diag.vc_dbar, ...
                  'ap_hat', soV.diag.vc_ap_hat, 'var_k', soV.diag.vc_var_k);

    fprintf('  REF  (known a''): a_hat/truth last2s = %.4f | trk std = %.1f nm\n', ...
            e.a_ratio_ref, e.trk_std_ref);
    fprintf('  open-loop floor (var chain on REF logs, last 10s) = %.4e (= %.1fx a''_true)\n', ...
            floor_ol, floor_ol / oracle_ap);
    if diverged
        fprintf('  MAIN (varchan):  DIVERGES at t=%.3f s\n', t_div);
    else
        fprintf('  MAIN (varchan):  ap_final = %.4e (= %.1fx a''_true, = %.2fx floor) | drift %.2e /s\n', ...
                e.ap_final, e.ap_final / oracle_ap, e.ap_final / floor_ol, e.drift_per_s);
        fprintf('                   a_hat/truth last2s = %.4f | trk std = %.1f nm\n', ...
                e.a_ratio_cl, e.trk_std_cl);
    end
    fprintf('\n');
    fig_e100(e, S, iend);
    fig_e100_chain(e, S, [0 0.2],  'E100_chain_zoom');
    fig_e100_chain(e, S, [0 20],   'E100_chain_full');
end

function fig_e100_chain(e, S, twin, name)
%FIG_E100_CHAIN  varchan chain internals, top-to-bottom in signal order.
%   All-linear axes (user 2026-07-17); ylim clips the startup spike. The two
%   variance EWMAs get separate tiles (units/scales differ by ~200x).
%   (1) numerator input  a_hat_z + mean tracker abar
%   (2) denominator input dxhat3_z + mean tracker dbar
%   (3) numerator EWMA sigma2_a_r
%   (4) denominator EWMA sigma2_dh_r
%   (5) raw statistic a'_hdm and slow-EWMA state a'_hd (spike clipped)
    GRN=[0 0.6 0]; RED=[0.8 0 0]; BLU=[0 0.2 0.8]; ORG=[0.9 0.45 0]; FS=14; LFS=10; LW=1.6;
    t = e.t; m = (t >= twin(1)) & (t <= twin(2));
    stab = m & (t >= twin(1) + 0.6*(twin(2)-twin(1)));   % late-window for ylim scaling
    f=figure('Position',[60 60 1100 1500],'Color','w','Visible','off');
    tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
    % (1) numerator source
    nexttile; hold on;
    p1=plot(t(m), e.az_hat(m)*1e3,'-','Color',RED,'LineWidth',1.0,'DisplayName','\^a_z (numerator input)');
    p2=plot(t(m), e.vc.abar(m)*1e3,'-','Color',BLU,'LineWidth',1.6,'DisplayName','\^a_z mean tracker (a_{var})');
    p3=yline(e.a_true_hd*1e3,'--','Color',GRN,'LineWidth',1.8,'DisplayName','a_{z,true}');
    xlim(twin); ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold');
    legend([p1 p2 p3],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % (2) denominator source
    nexttile; hold on;
    q1=plot(t(m), e.dh3(m)*1e3,'-','Color',RED,'LineWidth',1.0,'DisplayName','\delta\^h_3 (denominator input)');
    q2=plot(t(m), e.vc.dbar(m)*1e3,'-','Color',BLU,'LineWidth',1.6,'DisplayName','\delta\^h_3 mean tracker (a_{var})');
    xlim(twin); ylabel('\delta\^h_3 (nm)','FontSize',FS,'FontWeight','bold');
    legend([q1 q2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % (3) numerator variance EWMA (linear, own scale)
    nexttile; hold on;
    r1=plot(t(m), e.vc.s2ar(m),'-','Color',RED,'LineWidth',1.4,'DisplayName','\sigma^2_{a,r} numerator EWMA [(\mum/pN)^2]');
    ylim([0, 1.3*max(e.vc.s2ar(stab))]);
    xlim(twin); ylabel('\sigma^2_{a,r}','FontSize',FS,'FontWeight','bold');
    legend(r1,'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % (4) denominator variance EWMA (linear, own scale)
    nexttile; hold on;
    r2=plot(t(m), e.vc.s2dh(m),'-','Color',BLU,'LineWidth',1.4,'DisplayName','\sigma^2_{\deltah,r} denominator EWMA [\mum^2]');
    ylim([0, 1.3*max(e.vc.s2dh(stab))]);
    xlim(twin); ylabel('\sigma^2_{\deltah,r}','FontSize',FS,'FontWeight','bold');
    legend(r2,'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % (5) statistic and state (linear, spike clipped by ylim)
    nexttile; hold on;
    s1=plot(t(m), e.vc.var_k(m)*1e3,'-','Color',ORG,'LineWidth',1.0,'DisplayName','\^a''_{hdm} = sqrt(ratio) (raw statistic)');
    s2=plot(t(m), e.vc.ap_hat(m)*1e3,'-','Color',RED,'LineWidth',1.8,'DisplayName','\^a''_{hd} (slow \beta-EWMA state)');
    s3=yline(e.floor_ol*1e3,'--','Color',BLU,'LineWidth',1.8,'DisplayName','open-loop floor');
    s4=yline(e.oracle_ap*1e3,'--','Color',GRN,'LineWidth',1.8,'DisplayName','truth');
    ymax5 = 1.3 * max([e.vc.var_k(stab); e.vc.ap_hat(stab); e.floor_ol]) * 1e3;
    ylim([0 ymax5]);
    xlim(twin);
    ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([s1 s2 s3 s4],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,[name '.png']),'Resolution',200); close(f);
    fprintf('  [%s]\n', name);
end

function fig_e100(e, S, iend)
    GRN=[0 0.6 0]; RED=[0.8 0 0]; BLU=[0 0.2 0.8]; FS=15; LFS=11; LW=1.8;
    t = e.t; T = t(end);
    f=figure('Position',[80 80 1100 1100],'Color','w','Visible','off');
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    % (1) a'_hd: closed loop vs truth (~0) vs open-loop floor (linear y,
    % ylim clips any startup spike; user 2026-07-17)
    nexttile; hold on;
    h1=plot(t(1:iend), e.ap(1:iend)*1e3,'-','Color',RED,'LineWidth',1.7,'DisplayName','\^a''_{hd} (closed loop, init 0)');
    hf=yline(e.floor_ol*1e3,'--','Color',BLU,'LineWidth',2.0,'DisplayName','open-loop floor (REF logs)');
    ho=yline(e.oracle_ap*1e3,'--','Color',GRN,'LineWidth',2.4,'DisplayName','a''_{hd} truth (\approx 0)');
    ymax1 = 1.2 * max([e.ap_final, e.floor_ol]) * 1e3;
    ylim([0 ymax1]);
    xlim([0 T]); ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    legend([h1 hf ho],'Location','northoutside','Orientation','horizontal','FontSize',LFS-1,'FontWeight','bold','Box','on');
    text(0.985,0.10,sprintf('final = %.1f\\times truth = %.2f\\times floor', ...
         e.ap_final/e.oracle_ap, e.ap_final/e.floor_ol),'Units','normalized', ...
         'HorizontalAlignment','right','FontSize',LFS,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % (2) a_z: closed loop vs REF vs truth (a_nom init -> y2 pulls 2.5% gap)
    nexttile; hold on;
    g0=plot(t, e.az_true*1e3,'-','Color',GRN,'LineWidth',2.6,'DisplayName','a_{z,true}');
    g1=plot(t(1:iend), e.az_hat(1:iend)*1e3,'-','Color',RED,'LineWidth',1.4,'DisplayName','\^a_z (closed loop)');
    g2=plot(e.tR, e.azR_hat*1e3,'-','Color',BLU,'LineWidth',1.4,'DisplayName','\^a_z (REF, known a'')');
    ga=yline(S.a_nom*1e3,':','Color',[0.4 0.4 0.4],'LineWidth',1.6,'DisplayName','a_{nom} (init)');
    xlim([0 T]); ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold');
    legend([g0 g1 g2 ga],'Location','northoutside','Orientation','horizontal','FontSize',LFS-1,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % (3) tracking error
    nexttile; hold on;
    d1=plot(t(1:iend), e.dxz(1:iend),'-','Color',RED,'LineWidth',0.8,'DisplayName','\deltax_z (closed loop)');
    d2=plot(e.tR, e.dxzR,'-','Color',BLU,'LineWidth',0.8,'DisplayName','\deltax_z (REF)');
    xlim([0 T]); ylabel('\deltax_z (nm)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([d1 d2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    text(0.985,0.90,sprintf('std 10-18s: %.1f vs %.1f nm',e.trk_std_cl,e.trk_std_ref),'Units','normalized', ...
         'HorizontalAlignment','right','FontSize',LFS,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'E100_closedloop_hold_h100.png'),'Resolution',200); close(f);
    fprintf('  [E100_closedloop_hold_h100]\n');
end


%% ==================== D (restoring-force experiment, hold) ====================
function d = run_D(S, PAR)
    fprintf('--- D: restoring-force experiment (hold @ h_bar=2.0, T=10s, seed 1) ---\n');
    R = S.R; a_nom = S.a_nom; w = S.w; pz = S.pz;
    cfg = S.cfg;
    cfg.trajectory_type = 'positioning';   % pure hold, no descent/osc
    cfg.h_init = 2.0 * R;                   % h_d = 4.5 um constant
    cfg.T_sim  = 10.0;
    % oracle a'(h_d) and a_true(h_d) at the constant hold height
    [~, cperp_d, drv_d] = calc_correction_functions(2.0, true);
    a_true_hd = a_nom / cperp_d;            % [um/pN]
    oracle_ap = -a_true_hd * drv_d.K_h_perp / R;   % [um/pN/um]
    d.oracle_ap = oracle_ap; d.a_true_hd = a_true_hd;

    % ---- D1: a' channel, esti source, three wrong beta-EWMA inits ----
    so = run_pure_simulation(cfg, struct('seed',1,'collect_diag',true));
    t = so.tout; N = numel(t); Ts = S.Ts;
    a_hat = so.diag.a_hat(:,3);  dxh3 = so.diag.delta_x_hat_3(:,3);
    varr  = local_var_channel_esti(a_hat, dxh3, S.a_cov, S.a_cov);   % §2 blind mean-track chain
    inits = [0.5 1.0 2.0];
    ss2 = t >= (cfg.T_sim - 2);            % last 2 s
    d.inits = inits; d.t = t; d.oracle_line = oracle_ap;
    d.ap_arms = zeros(N, numel(inits));
    fprintf('  D1 (a'' channel, esti source, corrections on, NOT frozen):\n');
    fprintf('    oracle a''(h_d) = %.4e ; mean var[k] last 2s = %.4e\n', oracle_ap, mean(varr(ss2)));
    fprintf('    %-6s %-14s %-14s %-14s\n','init','mean(ap last2s)','/init','/truth');
    for j = 1:numel(inits)
        ap = zeros(N,1); ap(1) = inits(j) * oracle_ap;
        for k = 2:N
            if t(k) < S.t_warm
                ap(k) = ap(k-1);                                % warm-up freeze
            else
                ap(k) = S.beta*ap(k-1) + (1-S.beta)*varr(k);
            end
        end
        d.ap_arms(:,j) = ap;
        mlast = mean(ap(ss2));
        d.ap_final(j) = mlast;
        fprintf('    %-6.2f %.4e     %6.2fx        %6.2fx\n', inits(j), mlast, mlast/(inits(j)*oracle_ap), mlast/oracle_ap);
    end

    % ---- D2: control channel, deliberately-wrong a_hat init, y2 restoring ----
    cfgD = cfg; cfgD.aprime_source = 'known'; cfgD.a_hat_init_scale = 0.5;
    soD = temp_run_pure_sim_centered(cfgD, struct('seed',1,'collect_diag',true));
    tD = soD.tout;
    a_hat_D = soD.diag.a_hat(:,3);
    a_hat_D(1) = a_hat_D(2);               % k=1 logs the unscaled first-call return; the
                                           % scaled 0.5x init first shows at k=2 (cosmetic fix)
    a_true_D = soD.a_true_out(:,3);        % ~ a_true_hd (constant hold)
    d.tD = tD; d.a_hat_D = a_hat_D; d.a_true_D = a_true_D;
    % recovery time constant: time to close (1-1/e)=63.2% of the initial 0.5x gap
    a0 = a_hat_D(2); a_inf = mean(a_true_D(tD > tD(end)-2));   % start = actual scaled init
    gap0 = a_inf - a0;
    tau_rec = NaN;
    if abs(gap0) > 0
        idx = find((1:numel(a_hat_D))' >= 2 & (a_hat_D - a0)/gap0 >= (1 - exp(-1)), 1);
        if ~isempty(idx); tau_rec = tD(idx) - tD(2); end
    end
    d.tau_rec = tau_rec; d.a0 = a0; d.a_inf = a_inf;
    fprintf('  D2 (control channel, a_hat init 0.5x, y2 restoring on):\n');
    fprintf('    a_hat init(0.5x)=%.4e -> a_true=%.4e ; 63%%-recovery time = %.3f s\n', a0, a_inf, tau_rec);
    % final level check
    ss2D = tD >= (cfg.T_sim - 2);
    fprintf('    a_hat mean last 2s = %.4e (/truth %.3fx)\n', mean(a_hat_D(ss2D)), mean(a_hat_D(ss2D))/a_inf);

    % ---- one-line verdict ----
    spread_arms = std(d.ap_final) / mean(d.ap_final);
    d.spread_arms = spread_arms;
    fprintf('  VERDICT (one line):\n');
    fprintf('    a'' arms final spread = %.2f%% (all 3 inits -> var[k]=%.2fx truth): the beta-EWMA LP\n', ...
            100*spread_arms, mean(d.ap_final)/oracle_ap);
    fprintf('    pulls to var[k], but var[k] re-reads the controller''s KNOWN a'' (circular, esti kappa~0),\n');
    fprintf('    NOT an independent measurement. a_hat_z instead has a TRUE independent y2 (mean=a_true)\n');
    fprintf('    -> genuine restoring force, 63%%-recovery %.2f s. So the a'' channel has no INDEPENDENT\n', tau_rec);
    fprintf('    restoring force; its shadow "convergence" vanishes when a'' feeds back (C5 diverges).\n\n');
    fig_d_recovery(d, S, oracle_ap, a_true_hd);
end

function varr = local_var_channel_esti(ah, dh3, a_var, a_cov)
%LOCAL_VAR_CHANNEL_ESTI  esti §2 blind mean-track chain (V-batch implementation).
    N = numel(ah); varr = zeros(N,1);
    abar = ah(1); s2ar = 0; dbar = dh3(1); s2dh = 0;
    for k = 2:N
        abar = (1-a_var)*abar + a_var*ah(k); ar = ah(k)-abar;
        s2ar = (1-a_cov)*s2ar + a_cov*ar^2;
        dbar = (1-a_var)*dbar + a_var*dh3(k); dr = dh3(k)-dbar;
        s2dh = (1-a_cov)*s2dh + a_cov*dr^2;
        if s2dh > 0, varr(k) = sqrt(s2ar/s2dh); end
    end
end

function fig_d_recovery(d, S, oracle_ap, a_true_hd)
    GRN=[0 0.6 0]; RED=[0.8 0 0]; BLU=[0 0.2 0.8]; ORG=[0.9 0.45 0]; PUR=[0.5 0 0.6];
    FS=15; LFS=11; LW=1.8;
    cols = {RED, ORG, PUR};
    f=figure('Position',[80 80 1100 820],'Color','w','Visible','off');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    % top: three arms a'(t) + oracle
    nexttile; hold on; hh=[];
    for j=1:numel(d.inits)
        hh(j)=plot(d.t, d.ap_arms(:,j)*1e3,'-','Color',cols{j},'LineWidth',1.8, ...
             'DisplayName',sprintf('\\^a''_{hd}, init=%.1f\\times oracle',d.inits(j))); %#ok<AGROW>
    end
    ho=yline(oracle_ap*1e3,'--','Color',GRN,'LineWidth',2.4,'DisplayName','oracle a''(h_d)');
    xlim([0 d.t(end)]); ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    legend([hh ho],'Location','northoutside','Orientation','horizontal','FontSize',LFS-1,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    % bottom: a_hat(t) wrong init recovering
    nexttile; hold on;
    g1=plot(d.tD, d.a_true_D*1e3,'-','Color',GRN,'LineWidth',2.6,'DisplayName','a_{z,true}(h_d)');
    g2=plot(d.tD, d.a_hat_D*1e3,'-','Color',BLU,'LineWidth',1.5,'DisplayName','\^a_z (init 0.5\times, y_2 restoring)');
    if ~isnan(d.tau_rec); xline(d.tau_rec,'--','Color',[0.4 0.4 0.4],'LineWidth',1.6,'HandleVisibility','off'); end
    xlim([0 d.tD(end)]); ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    legend([g1 g2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    text(0.985,0.10,sprintf('63%%-recovery = %.2f s',d.tau_rec),'Units','normalized','HorizontalAlignment','right', ...
         'FontSize',LFS,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'D1_recovery_test.png'),'Resolution',200); close(f);
    fprintf('  [D1_recovery_test]\n');
end


%% ==================== shared extraction / helpers ====================
function [t, h_d, h_tru, h_m, dh_true, dh_m, a_tru, a_det_oracle, a_xm, a_hat, dxh3, oap] = extract(so, S)
    t = so.tout; w = S.w; pz = S.pz; R = S.R; a_nom = S.a_nom; N = numel(t);
    h_d   = so.p_d_out*w - pz;
    h_tru = so.p_true_out*w - pz;
    h_m   = so.p_m_out*w - pz;
    dh_true = h_d - h_tru;
    dh_m    = so.diag.delta_x_m(:,3);          % measured, d-delayed tracking error
    a_tru = so.a_true_out(:,3);
    a_det_oracle = so.diag.a_x_det(:,3);       % a_nom/c_perp(h_d)
    a_xm  = so.diag.a_xm(:,3);
    a_hat = so.diag.a_hat(:,3);
    dxh3  = so.diag.delta_x_hat_3(:,3);
    oap = zeros(N,1);
    for k=1:N
        [~,cp,drv]=calc_correction_functions(max(h_d(k)/R,1.001),true);
        oap(k) = -(a_nom/cp)*drv.K_h_perp/R;
    end
end

function adet = build_adet(h_d, apser, a_nom)
%BUILD_ADET  maintained reference a_det[k+1]=a_det[k]+apser[k]*(h_d[k+1]-h_d[k]).
    N = numel(h_d); adet = zeros(N,1); adet(1) = a_nom;
    for k=1:N-1
        adet(k+1) = adet(k) + apser(k)*(h_d(k+1)-h_d(k));
    end
end

function ap_raw = centered_ewma(s, dh_m, a_cov)
%CENTERED_EWMA  fast EWMA of sig2_s / sig2_dh, ap_raw = sqrt(ratio). No mean stage.
    N = numel(s); ap_raw = zeros(N,1);
    s2s = 0; s2dh = 0;
    for k=1:N
        s2s  = (1-a_cov)*s2s  + a_cov*s(k)^2;
        s2dh = (1-a_cov)*s2dh + a_cov*dh_m(k)^2;
        if s2dh > 0; ap_raw(k) = sqrt(s2s/s2dh); end
    end
end

function [ap_smooth, ap_raw, tdiv] = centered_online(a_sig, dh_den, h_d, S, oap, dcount)
%CENTERED_ONLINE  full centered channel: maintained a_det (self-consistent with
%   the slow a'_hdm EWMA), s = a_sig - a_det[k-dcount], fast EWMA ratio, then
%   slow beta-EWMA with warm-up freeze. Mirrors the online closed-loop recursion.
%   Divergence guard: if the self-consistent reference a_det leaves [-BIG,BIG],
%   stop and return the divergence time tdiv (NaN if it stays bounded).
    N = numel(a_sig); ap_raw = nan(N,1); ap_smooth = nan(N,1);
    a_cov = S.a_cov; beta = S.beta; Ts = S.Ts; t_warm = S.t_warm;
    adet = S.a_nom; adet_hist = S.a_nom*ones(dcount+1,1);   % ring for delayed ref
    s2s = 0; s2dh = 0;
    ap_hat = oap(1); tdiv = NaN;
    BIG = 100;                                             % [um/pN] a_det divergence bound (~6800x a_nom)
    for k=1:N
        tk = (k-1)*Ts;
        % maintain reference with current slope estimate
        if k>1; adet = adet + ap_hat*(h_d(k)-h_d(k-1)); end
        if ~isfinite(adet) || abs(adet) > BIG
            tdiv = tk; return;                            % reference ran away
        end
        adet_hist = [adet_hist(2:end); adet];
        adet_del = adet_hist(1);                          % a_det[k-dcount]
        s = a_sig(k) - adet_del;
        s2s  = (1-a_cov)*s2s  + a_cov*s^2;
        s2dh = (1-a_cov)*s2dh + a_cov*dh_den(k)^2;
        if s2dh>0; ap_raw(k) = sqrt(s2s/s2dh); end
        if tk >= t_warm
            ap_hat = beta*ap_hat + (1-beta)*ap_raw(k);
        end
        ap_smooth(k) = ap_hat;
    end
end

function ap = slow_beta(ap_raw, oap, t, S)
%SLOW_BETA  slow beta-EWMA of ap_raw with warm-up freeze (C4/C5 shadow smoother).
    N = numel(ap_raw); ap = zeros(N,1); ap(1) = oap(1);
    for k=2:N
        if t(k) < S.t_warm
            ap(k) = ap(k-1);
        else
            ap(k) = S.beta*ap(k-1) + (1-S.beta)*ap_raw(k);
        end
    end
end

function v = std_osc(x, t, S)
    osc = (t>=S.t2+1)&(t<=S.t3); v = std(x(osc));
end
function v = mean_osc(x, t, S)
    osc = (t>=S.t2+1)&(t<=S.t3); v = mean(x(osc));
end

function [ratio_osc, corr_osc, ratio_full, corr_full] = metrics(ap, oap, t, S)
    osc = (t>=S.t2+1)&(t<=S.t3) & isfinite(ap);
    full = (t>=S.t1) & isfinite(ap);
    if nnz(osc) < 3
        ratio_osc = NaN; corr_osc = NaN; ratio_full = NaN; corr_full = NaN; return;
    end
    ratio_osc = mean(ap(osc)./oap(osc));
    cc = corrcoef(ap(osc), oap(osc)); corr_osc = cc(1,2);
    ratio_full = mean(ap(full)./oap(full));
    cc = corrcoef(ap(full), oap(full)); corr_full = cc(1,2);
end


%% ==================== figures (repo style) ====================
function fig_overlay(t, oap, ap, S, note, ylab, name)
    GRN=[0 0.6 0]; RED=[0.8 0 0]; FS=17; LFS=13; LW=2.0;
    f=figure('Position',[80 80 1100 480],'Color','w','Visible','off'); hold on;
    h1=plot(t,oap*1e3,'-','Color',GRN,'LineWidth',2.8,'DisplayName','a''_{hd,true}=a''(h_d(t))');
    h2=plot(t,ap*1e3,'-','Color',RED,'LineWidth',1.6,'DisplayName',ylab);
    xlim([0 ceil(t(end))]); ylim([0 max(oap*1e3)*2.2]);
    ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    for xb=[S.t1 S.t2 S.t3]; xline(xb,'--','Color',[0.6 0.6 0.6],'LineWidth',1.2,'HandleVisibility','off'); end
    legend([h1 h2],'Location','northoutside','Orientation','horizontal','FontSize',LFS-1,'FontWeight','bold','Box','on');
    text(0.985,0.90,note,'Units','normalized','HorizontalAlignment','right','FontSize',LFS-1,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,[name '.png']),'Resolution',200); close(f);
    fprintf('  [%s]\n', name);
end

function fig_c3_arms(c3, S)
    GRN=[0 0.6 0]; BLU=[0 0.2 0.8]; RED=[0.8 0 0]; FS=17; LFS=12; LW=2.0;
    f=figure('Position',[80 80 820 560],'Color','w','Visible','off'); hold on;
    x = c3.scales;
    mv = mean(c3.meanhdm,2)*1e3; sv = std(c3.meanhdm,0,2)*1e3;
    h1=errorbar(x, mv, sv,'o-','Color',RED,'MarkerFaceColor',RED,'LineWidth',2,'MarkerSize',9,'DisplayName','mean(\^a''_{hdm}) (measured)');
    h2=plot(x, c3.ap_frozen*1e3,'s--','Color',BLU,'LineWidth',2,'MarkerSize',9,'MarkerFaceColor',BLU,'DisplayName','\^a''(frozen)=scale\times oracle (\kappa=0)');
    h3=yline(c3.oracle*1e3,'-','Color',GRN,'LineWidth',2.6,'DisplayName','a''_{true}=oracle (\kappa=1)');
    xlim([0.35 2.15]); xlabel('a'' freeze scale (\times oracle)','FontSize',FS,'FontWeight','bold');
    ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    legend([h1 h2 h3],'Location','northoutside','FontSize',LFS,'FontWeight','bold','Box','on');
    text(0.985,0.06,sprintf('pooled \\kappa_{emp}=%+.2f\\pm%.2f',c3.kappa_pooled,c3.kappa_std),'Units','normalized','HorizontalAlignment','right','FontSize',LFS,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'C3_frozen_arms.png'),'Resolution',200); close(f);
    fprintf('  [C3_frozen_arms]\n');
end

function fig_c4_combo(c4, S)
    GRN=[0 0.6 0]; RED=[0.8 0 0]; BLU=[0 0.2 0.8]; ORG=[0.9 0.45 0]; FS=15; LFS=11; LW=1.8;
    t=c4.t; T=ceil(t(end));
    f=figure('Position',[80 80 1100 1000],'Color','w','Visible','off');
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    nexttile; hold on;
    h1=plot(t,c4.oap*1e3,'-','Color',GRN,'LineWidth',2.8,'DisplayName','a''_{hd,true}');
    h2=plot(t,c4.ap_meas*1e3,'-','Color',RED,'LineWidth',1.8,'DisplayName','\^a''_{hd} (a_{xm}-a_{det}, centered)');
    h3=plot(t,c4.ap_esti*1e3,'-','Color',ORG,'LineWidth',1.4,'DisplayName','\^a''_{hd} (a_{hat}-a_{det}, centered)');
    xlim([0 T]); ylim([0 max([c4.oap;c4.ap_meas;c4.ap_esti])*1e3*1.15]);
    ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    legend([h1 h2 h3],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    nexttile; hold on;
    g1=plot(t,c4.a_tru*1e3,'-','Color',GRN,'LineWidth',2.8,'DisplayName','a_{z,true}');
    g2=plot(t,c4.a_hat*1e3,'-','Color',RED,'LineWidth',1.4,'DisplayName','\^a_z');
    xlim([0 T]); ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold');
    legend([g1 g2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    nexttile; hold on;
    plot(t,c4.dxz,'-','Color',BLU,'LineWidth',1.1);
    yl=max(abs(c4.dxz))*1.1; ylim([-yl yl]); xlim([0 T]);
    for xb=[S.t1 S.t2 S.t3]; xline(xb,'--','Color',[0.4 0.4 0.4],'LineWidth',1.5,'HandleVisibility','off'); end
    ylabel('\deltax_z (nm)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    text(0.985,0.06,sprintf('meas: ratio=%.2f corr=%.2f | esti: ratio=%.2f corr=%.2f',c4.ratio_meas,c4.corr_meas,c4.ratio_esti,c4.corr_esti), ...
         'Units','normalized','HorizontalAlignment','right','FontSize',LFS,'FontWeight','bold','BackgroundColor','w','EdgeColor','k');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'C4_shadow_combined.png'),'Resolution',200); close(f);
    fprintf('  [C4_shadow_combined]\n');
end

function fig_c5_combo(t, oracle_ap, ap_cl, az_true, az_hat, dxz_cl, iend, diverged, t_div, S)
    GRN=[0 0.6 0]; RED=[0.8 0 0]; BLU=[0 0.2 0.8]; FS=15; LFS=11; LW=1.8;
    tp=t(1:iend);
    f=figure('Position',[80 80 1100 1000],'Color','w','Visible','off');
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    nexttile; hold on;
    h1=plot(tp,oracle_ap(1:iend)*1e3,'-','Color',GRN,'LineWidth',2.8,'DisplayName','a''_{hd,true}');
    h2=plot(tp,ap_cl(1:iend)*1e3,'-','Color',RED,'LineWidth',1.8,'DisplayName','\^a''_{hd} (closed-loop, fed back)');
    xlim([0 tp(end)]); ylabel('a'' (nm/pN/\mum)','FontSize',FS,'FontWeight','bold');
    if diverged; xline(t_div,'--','Color',[0.5 0 0.5],'LineWidth',2,'HandleVisibility','off'); end
    legend([h1 h2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    nexttile; hold on;
    g1=plot(tp,az_true(1:iend)*1e3,'-','Color',GRN,'LineWidth',2.8,'DisplayName','a_{z,true}');
    g2=plot(tp,az_hat(1:iend)*1e3,'-','Color',RED,'LineWidth',1.4,'DisplayName','\^a_z (closed-loop)');
    xlim([0 tp(end)]); ylim([0 max(az_true(1:iend))*1e3*1.3]);
    ylabel('a_z (nm/pN)','FontSize',FS,'FontWeight','bold');
    if diverged; xline(t_div,'--','Color',[0.5 0 0.5],'LineWidth',2,'HandleVisibility','off'); end
    legend([g1 g2],'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    nexttile; hold on;
    plot(tp,dxz_cl(1:iend),'-','Color',BLU,'LineWidth',1.1);
    xlim([0 tp(end)]); ylim([-1 1]*max(50,min(400,max(abs(dxz_cl(1:iend)))*1.1)));
    for xb=[S.t1 S.t2]; if xb<=tp(end); xline(xb,'--','Color',[0.4 0.4 0.4],'LineWidth',1.5,'HandleVisibility','off'); end; end
    if diverged
        xline(t_div,'--','Color',[0.5 0 0.5],'LineWidth',2,'HandleVisibility','off');
        text(t_div,0,sprintf('  diverges t=%.2f s',t_div),'Color',[0.5 0 0.5],'FontSize',LFS,'FontWeight','bold','VerticalAlignment','bottom');
    end
    ylabel('\deltax_z (nm)','FontSize',FS,'FontWeight','bold'); xlabel('Time (sec)','FontSize',FS,'FontWeight','bold');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',LW,'Box','on'); grid off;
    exportgraphics(f, fullfile(S.out_dir,'C5_closedloop_combined.png'),'Resolution',200); close(f);
    fprintf('  [C5_closedloop_combined]\n');
end
