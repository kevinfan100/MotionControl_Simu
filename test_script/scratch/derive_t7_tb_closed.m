% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T7: E[T_B] is NOT the Jensen form (T_B is deterministic-dominated, SEM 0.02e-5).
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
% T7: closed form for the anti-coherent term's mean:
%   E[T_B] = E[(a'@true - a'(a_hat)) * dw_true] ~= a'' * C_wd,  C_wd = E[y*dw_true]
%   moment machinery: dw_true fluct = step_c + u[k]  =>  C_wd = C_ws + E[y u] = C_ws
%   so predicted E[T_B](t) = a''(t) * C_ws(t),  a'' = -2 b^2 (1-a)^3  (signed)
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
addpath(genpath(fullfile(WT, 'model')));
od = fullfile(WT, 'test_results', 'loop_mean_bias');
S = load(fullfile(od, 'replay_v2_profile.mat'));
C7 = load(fullfile(od, 'loop_capture_seed7.mat'));
R = load(fullfile(od, 'run_log_seed7.mat'));  r = R.run_log;
ax = 3;  lc = 0.7;  b = 1 - lc;  kk = C7.KK;  t = C7.t(:);
pc = physical_constants();
a_o = r.a_nom / pc.R;  kappa = 4 * pc.k_B * pc.T / pc.R * a_o;  rn = (3.31e-3 / pc.R)^2;
abar = r.a_true_out(kk, ax) / r.a_nom;  bh = r.b_hat_out(kk, ax);
q = kappa * abar;
V = (q * (1 + 2*b^2) + 4*lc*b*q + b^2 * rn) / (1 - lc^2);
Cws = b * (2*q - V);
app = -2 * bh.^2 .* (1 - abar).^3;                 % a'' signed
TB_pred = app .* Cws;
% measured TBm: rebuild from the T6 computation (rerun quickly for all seeds)
TB = [];
for sd = 1:8
    Cq = load(fullfile(od, sprintf('loop_capture_seed%d.mat', sd)));
    Rq = load(fullfile(od, sprintf('run_log_seed%d.mat', sd)));  rq = Rq.run_log;
    kq = Cq.KK;
    ht = rq.h_bar_true_out(:);  dtrs = [0; diff(ht)];  dtrs = dtrs(kq);
    ahq = rq.a_bar_hat_out(kq, ax);  bhq = rq.b_hat_out(kq, ax);
    apH = [0; bhq(1:end-1) .* (1 - ahq(1:end-1)).^2];
    apT = rq.a_prime_true_out(kq, ax) / rq.a_nom;  apTm = [0; apT(1:end-1)];
    TB(:, sd) = (apTm - apH) .* dtrs; %#ok<SAGROW>
end
TBm = mean(TB, 2);
SEG = {'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'hold2', t > 3.7};
fprintf('E[T_B] closed form vs measured (x1e-5):\n');
for g = 1:3
    m = SEG{g,2};
    fprintf('  %-6s pred %+6.2f   meas %+6.2f +- %4.2f\n', SEG{g,1}, 1e5*mean(TB_pred(m)), ...
        1e5*mean(mean(TB(m,:),1)), 1e5*std(mean(TB(m,:),1))/sqrt(8));
end
fprintf('  curve corr(pred, meas smoothed), t>0.5: %+.3f\n', ...
    corr(TB_pred(t>0.5), movmean(TBm(t>0.5), 401)));
fprintf('T7 DONE\n');
