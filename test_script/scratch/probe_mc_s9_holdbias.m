% STATUS: SCRATCH -- 2026-10-14, motion_ctrl S9: is the slow hold drift of a-bar (a_hat held at a_d, a_true drifting below, ~ -0.0014 /s,
%   5/5 seeds) made by the regulation law, or does the S8 estimator drift in any hold? Arm F: same estimator and constants as
%   run_mc_s9_gainreg, NO regulation; fixed trajectory 22.2 R -> h_hold at constant velocity in 1 s, then hold to T_sim.
%   Prints per 1-s window: a_true - a_hat per seed and mean (the quantity the regulation pins at 0 through a_hat = a_d).
function probe_mc_s9_holdbias(h_hold, seeds, T_sim, sig_ws, extra, cfg_extra, dith, save_tag)
    if nargin < 1 || isempty(h_hold); h_hold = 1.105; end
    if nargin < 2 || isempty(seeds); seeds = 1:5; end
    if nargin < 3 || isempty(T_sim); T_sim = 6; end
    if nargin < 4 || isempty(sig_ws); sig_ws = 0.3; end
    if nargin < 5 || isempty(extra); extra = struct(); end
    if nargin < 6 || isempty(cfg_extra); cfg_extra = struct(); end
    if nargin < 7; dith = []; end
    if nargin < 8; save_tag = ''; end   % 10-14: non-empty => save t, a_true - a_hat to test_results/temp_1009_ws/holdbias_<tag>.mat   % 10-14: run_formC_b opts.cmd_dither, struct(amp_R, freq, t_on)   % 10-14: extra config overrides, e.g. struct('thermal_enable', false)   % 10-14: extra ctrl_const overrides, e.g. struct('y2_off', true)   % 10-14: 1e-6 = w_s-hat locked at the plant wall 0
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                'Pf_a_floor', 0.002, 'plaw_seed_cross', true, 'plaw_dc_slope', true, 'plaw_p', 2, 'c_init', 0, 'Pf_c_std', 1e-6, ...
                'delta_init', 0, 'Pf_delta_std', sig_ws, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-5, 5]);
    fn = fieldnames(extra);  for i = 1:numel(fn); cc.(fn{i}) = extra.(fn{i}); end
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', h_hold * R, 'amplitude', 0, 't_hold', 0, ...
                't_descend_override', 1, 'T_sim', T_sim, 'lambda_c', 0.4);
    fc = fieldnames(cfg_extra);  for i = 1:numel(fc); OV.(fc{i}) = cfg_extra.(fc{i}); end
    o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
               'seeds', seeds, 'log_P_full', false, 'cmd_dither', dith, 'allow_below_floor', h_hold < 1.05);
    if ~isempty(dith); fprintf('command dither: %.3f R at %g Hz from %.1f s\n', dith.amp_R, dith.freq, dith.t_on); end
    clear run_formC_b motion_control_law_formC_b;
    evalc('Rr = run_formC_b(o);');
    ns = numel(Rr.runs);  n = numel(Rr.runs{1}.h_bar_d_out);  t = (0:n-1)' * Ts;  [at, ah, hd] = deal(zeros(n, ns));
    for q = 1:ns
        S = Rr.runs{q};  hb = max(S.h_bar_true_out(:), 1.001);  hd(:, q) = S.h_bar_d_out(:);  ah(:, q) = S.a_bar_hat_out(:, 3);
        for i = 1:n; [~, cp] = calc_correction_functions(hb(i)); at(i, q) = 1 / cp; end
    end
    fprintf('[%s] fixed hold at %.3f R (no regulation), sqrt p55[0] %.0e, %d seeds | a_true at h_hold %.4f\n', strjoin([fn; fc]', ','), h_hold, sig_ws, ns, mean(at(t > 1.5, 1)));
    ws = zeros(n, ns);  for q = 1:ns; ws(:, q) = -Rr.runs{q}.ws_hat_out(:, 3); end
    fprintf('w_s-hat at end per seed: %s\n', sprintf('%+.4f ', ws(end, :)));
    H = t >= 2;  dx3 = zeros(ns, 1);  i1 = dx3;  i2 = dx3;  rt = dx3;
    for q = 1:ns
        S = Rr.runs{q};  dx3(q) = mean(S.delta_x_hat_3_out(H, 3)) / R;  i1(q) = mean(S.innov_y1_out(H, 3));  i2(q) = mean(S.innov_y2_out(H, 3));
        p = polyfit(t(H), ah(H, q), 1);  rt(q) = p(1);
    end
    aps = 1 / (h_hold + mean(ws(end, :)))^2;
    fprintf('hold (t >= 2 s) per seed: d a_hat/dt %s /s | mean delta_x_hat_3 %s R | predicted a_hat'' (1-lc) mean(dx3)/Ts %s /s\n', ...
            sprintf('%+.5f ', rt), sprintf('%+.2e ', dx3), sprintf('%+.5f ', aps * 0.6 * dx3 / Ts));
    fprintf('   mean innovation y1 %s | y2 %s\n', sprintf('%+.2e ', i1), sprintf('%+.2e ', i2));
    ih = find(H);  ap0 = 1 / h_hold^2;   % S8 slope with w_s-hat locked at 0 (use only with sig_ws = 1e-6)
    for q = 1:ns
        S = Rr.runs{q};  apb = S.a_prime_out(:, 3) / median(S.a_prime_out(H, 3)) * ap0;  x3 = S.delta_x_hat_3_out(:, 3) / R;
        pr = sum(apb(ih - 1) .* 0.6 .* x3(ih - 1));  tot = ah(ih(end), q) - ah(ih(1) - 1, q);
        fprintf('   seed %d: total d a_hat over hold %+.5f = predict sum %+.5f + update sum %+.5f | var(x3) %.2e R^2\n', ...
                seeds(q), tot, pr, tot - pr, var(x3(ih)));
    end
    for q = 1:ns   % y1 gain update sum over the hold, split into mean(K)*sum(nu) and the K-nu covariance part; sqrt P_aa at hold start / end
        S = Rr.runs{q};  K = S.K_a_y1_out(ih, 3);  nu = S.innov_y1_out(ih, 3);  u = sum(K .* nu);  um = mean(K) * sum(nu);
        Ks = S.K_a_y2_out(ih, 3);  n2 = S.innov_y2_out(ih, 3);
        fprintf('   seed %d: y1 update sum %+.5f (mean K x sum nu %+.5f, K-nu covariance %+.5f) | y2 update sum %+.5f | K_a_y1 %.2e..%.2e | sqrtP_aa/a_nom %.4f..%.4f\n', ...
                seeds(q), u, um, u - um, sum(Ks .* n2), K(1), K(end), S.P_a_out(ih(1), 3) / S.a_nom, S.P_a_out(ih(end), 3) / S.a_nom);
    end
    fprintf('window | a_true - a_hat per seed | mean | h_d\n');
    for w0 = 1:floor(T_sim) - 1
        L = t >= w0 & t < w0 + 1;
        fprintf('%d-%d s | %s | %+.4f | %.4f\n', w0, w0 + 1, sprintf('%+.4f ', mean(at(L, :) - ah(L, :))), mean(mean(at(L, :) - ah(L, :))), mean(mean(hd(L, :))));
    end
    if ~isempty(save_tag); od = fullfile(root, 'test_results', 'temp_1009_ws');  e = at - ah;  save(fullfile(od, sprintf('holdbias_%s.mat', save_tag)), 't', 'e', 'h_hold'); end
end
