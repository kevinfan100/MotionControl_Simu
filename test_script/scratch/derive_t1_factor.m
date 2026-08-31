% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T1: is the 0.72 slope the first-order vs exact-step gap? NO (slopes identical).
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
od = fullfile(WT, 'test_results', 'loop_mean_bias');
C = load(fullfile(od, 'loop_capture_seed7.mat'));
R = load(fullfile(od, 'run_log_seed7.mat'));  r = R.run_log;
ax = 3;  lam = 0.7;  a_nom = r.a_nom;  kk = C.KK;  n = numel(kk);  t = C.t(:);
ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);
dtrs = [0; diff(ht)];  dtrs = dtrs(kk);            % true step
dwd  = [0; diff(hd)];  dwd  = dwd(kk);
ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
x3 = C.XU(3, :).';  m89 = (C.XU(8, :) + C.XU(9, :)).';
Mhat = dwd + [0; (1-lam)*x3(1:end-1)] + [0; (1-lam)*m89(1:end-1)];
% rebuild G4 (update-level row-4 forcing) exactly as in replay v2
S = load(fullfile(od, 'replay_v2_profile.mat'));
g4 = S.out.G4(:, find([1 2 3 4 5 6 7 8] == 7, 1));    % seed 7 column (seeds were 1:8)
% -- version A (first order): a'*(dtrs - Mhat), a' at posterior a_hat[k-1]
apA = bh .* (1 - ah).^2;
vA = [0; apA(1:end-1)] .* (dtrs - Mhat);
% -- version B (exact-step difference): step from a_hat[k-1] along dtrs vs along Mhat
ahm = [ah(1); ah(1:end-1)];  bhm = [bh(1); bh(1:end-1)];
stepx = @(a, b, M) (1 - (1 - a) ./ (1 + b .* (1 - a) .* M)) - a;
vB = stepx(ahm, bhm, dtrs) - stepx(ahm, bhm, Mhat);
% -- version C: like B but a' evaluated at TRUE a (does the eval point matter?)
at = r.a_true_out(kk, ax) / a_nom;  atm = [at(1); at(1:end-1)];
vC = stepx(atm, bhm, dtrs) - stepx(atm, bhm, Mhat);
m = t > 0.5;
for V = {vA, 'A first-order a''(a_hat)'; vB, 'B exact-step diff (a_hat)'; vC, 'C exact-step diff (a_true)'}.'
    v = V{1};
    fprintf('%-28s slope %.3f   corr %.3f\n', V{2}, v(m) \ g4(m), corr(v(m), g4(m)));
end
fprintf('T1 DONE\n');
