% FORK OF test_script/scratch/run_meng_ch4_best.m @ 5d4bd61 (controller config) + test/motion-test
%   test_script/scratch/run_prod_ladder.m @ 8aef954 (the two mainline trajectories, hold +4 s) + run_formC_b.m local_run_once
%   (plant loop) | PURPOSE: does the Meng Ch4 champion estimator carry a gain bias on the mainline 'meng' / 'canon' (deep)
%   trajectories? 10 seeds, z axis, (a_hat - a_true)/a_nom per window | EXPIRES: when the user closes the question
%   | 產線改動不會自動跟上
%   Plant = the mainline scenario (canonical meas noise [0.62 0.57 3.31] nm, same plant functions); controller = the champion
%   at its own operating point (lambda_c 0.4, lambda_f schedule, C_dpmr of the (4.4) loop, C_emp), with sigma_n set to the
%   plant's noise. Output: test_results/meng_ch4_s0/champ_two_traj_<traj>.mat
function out = run_meng_ch4_champion_two_traj(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'meng_ch4_s0');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  kBT = pc.k_B * pc.T;  gN = pc.gamma_N;  R = pc.R;  a_nom = Ts / gN;
    EXTRA = 4.0;   % hold extension used by the mainline ladder

    % ---- scenario (canonical_scenario.m 'deep' on test/motion-test, copied field by field) ----
    cfg = user_config();
    cfg.trajectory_type = 'osc';  cfg.h_init = 50;  cfg.amplitude = 2.5;  cfg.frequency = 1;  cfg.n_cycles = 2;
    cfg.t_hold = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;  cfg.h_min = 1.10 * R;
    cfg.h_bottom = 1.10 * R;  cfg.h_bar_safe = 1.00;
    cfg.ctrl_enable = true;  cfg.thermal_enable = true;  cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.4;  cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    switch traj
        case 'meng'   % run_prod_ladder 'meng' override on top of the canonical config
            cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
            cfg.t_hold = 0.5;  cfg.t_descend_override = 10;  cfg.T_sim = 12.5 + EXTRA;  cfg.h_min = 2.475;
        case 'canon'
            cfg.T_sim = cfg.T_sim + EXTRA;
        otherwise
            error('traj must be meng|canon');
    end
    SIGMA_N = cfg.meas_noise_std(:);
    t1 = cfg.t_hold;  t2 = t1 + cfg.t_descend_override;  t3 = t2 + cfg.n_cycles / cfg.frequency;

    % ---- controller constants: run_meng_ch4_best.m verbatim except sigma_n = plant noise ----
    evalc('cd44 = calc_cdpmr_ch4(0.4, 0.9995, 0.35);');
    cc = build_eq17_constants(struct('lambda_c', 0.4, 'sigma2_n_s', SIGMA_N.^2, 'kBT', kBT, 't_warmup_kf', 0));
    cc.iir_warmup_mode = 'prefill';  cc.force_Q77_zero = true;  cc.h_bar_safe = 1;
    cc.control_law = 'ch4';  cc.lambda_f = [0.9995; 0.9995; 0.999];
    cc.ch4_fdet = false;  cc.ch4_stale_ff = true;  cc.y2_whiten = true;
    cc.lf_schedule = true;  cc.lf_sched_scale = [2; 2; 1];
    cc.C_dpmr_eff = cd44.C_dpmr(1:3);  cc.C_np_eff = cd44.C_n(1:3);
    cc.C_dpmr_eff = cc.C_dpmr_eff .* [1.232; 1.194; 0.938];
    cc.xi_per_axis = (cd44.C_n(1:3) ./ cc.C_dpmr_eff) .* SIGMA_N.^2 / (4 * kBT);

    Tn = round(cfg.T_sim / Ts) + 1;  t = (0:Tn-1)' * Ts;  nS = numel(seeds);
    AH = zeros(Tn, nS);  AT = AH;  HB = AH;  AXM = AH;  LF = AH;  SPA = AH;  hd = zeros(Tn, 1);
    fprintf('[%s] T %.1f s | phases hold %.1f / descent to %.1f / osc to %.1f / hold | trough h_bar %.3f | seeds %s\n', ...
        traj, cfg.T_sim, t1, t2, t3, cfg.h_bottom / R, mat2str(seeds));
    for q = 1:nS
        clear motion_control_law_eq17_core trajectory_generator calc_thermal_force
        rng(seeds(q));
        prm = calc_simulation_params(cfg);  P = prm.Value;
        Pc = struct('ctrl', struct('enable', 1, 'Ts', Ts, 'k_B', pc.k_B, 'T', pc.T, 'gamma', gN, 'sigma2_noise', SIGMA_N.^2), ...
                    'common', struct('R', R, 'p0', P.common.p0, 'gamma_N', gN), ...
                    'wall', struct('w_hat', P.wall.w_hat, 'pz', P.wall.pz, 'enable_wall_effect', 1), ...
                    'traj', struct('amplitude', cfg.amplitude, 'frequency', cfg.frequency));
        p = P.common.p0;  p_m_buffer = repmat(p, 1, cc.d + 1);  pd_for_ctrl = p;
        for k = 1:Tn
            [pd_kp1, del_pd] = trajectory_generator(t(k), P);
            pd_k = pd_for_ctrl;  p_m_delayed = p_m_buffer(:, 1);
            hb = max((dot(p, P.wall.w_hat) - P.wall.pz) / R, 1.001);
            [cpar, cper] = calc_correction_functions(hb);
            a_true = a_nom ./ [cpar; cpar; cper];
            [f_d, ~, dg] = motion_control_law_eq17_core(del_pd, pd_k, p_m_delayed, Pc, cc);
            f_th = calc_thermal_force(p, P);
            p = step_dynamics(p, f_d + f_th, P, Ts);
            p_m_buffer = [p_m_buffer(:, 2:end), p + SIGMA_N .* randn(3, 1)];
            AH(k, q) = dg.a_hat(3) / a_nom;  AT(k, q) = a_true(3) / a_nom;  HB(k, q) = hb;
            AXM(k, q) = dg.a_xm(3) / a_nom;  SPA(k, q) = sqrt(max(dg.P_a(3), 0)) / a_nom;
            if isfield(dg, 'lambda_f_used') && ~isempty(dg.lambda_f_used); LF(k, q) = dg.lambda_f_used(3); else; LF(k, q) = NaN; end
            if q == 1; hd(k) = pd_k(3) / R; end
            pd_for_ctrl = pd_kp1;
        end
        fprintf('   seed %2d done | min w %.4f | a_hat(end)/a_nom %.5f vs true %.5f\n', seeds(q), min(HB(:, q)), AH(end, q), AT(end, q));
    end
    E = AH - AT;  REL = E ./ AT;
    out = struct('traj', traj, 'seeds', seeds, 't', t, 'E', E, 'REL', REL, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, ...
                 'AXM', AXM, 'LF', LF, 'SPA', SPA, 'phases', [t1 t2 t3 cfg.T_sim], 't_hold', t3, 'R', R);

    % ---- console: motion windows first, hold last (feedback 09-07) ----
    W = {'descent',            t > t1 & t <= t2; ...
         'near-wall descent',  t > t1 & t <= t2 & hd < 2; ...
         'osc / bottom dwell', t > t2 & t <= t3; ...
         'hold',               t > t3};
    fprintf('[%s] health: min w %.4f | min a_hat %.5f | NaN %d | lambda_f min %.5f\n', traj, min(HB(:)), min(AH(:)), sum(~isfinite(E(:))), min(LF(:)));
    for i = 1:size(W, 1)
        m = W{i, 2};  if ~any(m); continue; end
        pm = mean(E(m, :), 1);  pr = 100 * mean(REL(m, :), 1);  mE = mean(E, 2);
        [wv, iw] = max(abs(mE(m)));  tw = t(m);  TRq = (hd - HB) * R;  ptr = mean(TRq(m, :), 1);
        fprintf('[%s %-18s] (a_hat-a)/a_nom %+.5f (sd_seed %.5f, SEM %.5f, t %+.1f) | rel %+.2f%% (SEM %.2f) | worst |seed-mean| %.5f at %.2f s | inst sigma_seed %.5f | track err %+.2f nm (SEM %.2f)\n', ...
            traj, W{i, 1}, mean(pm), std(pm), std(pm) / sqrt(nS), mean(pm) / (std(pm) / sqrt(nS)), mean(pr), std(pr) / sqrt(nS), ...
            wv, tw(iw), mean(std(E(m, :), 0, 2)), 1e3 * mean(ptr), 1e3 * std(ptr) / sqrt(nS));
    end
    save(fullfile(od, sprintf('champ_two_traj_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved champ_two_traj_%s.mat\n', traj, traj);
end
