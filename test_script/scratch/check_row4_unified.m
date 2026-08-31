% STATUS: ACTIVE (scratch) | PURPOSE: unified-mechanism check (2026-08-31):
%   G4 ~ a'_hat * (dw_true - M_hat)? Result: slope 0.716 +- 0.005, corr 0.796;
%   gaps-segment means pred -3.59 vs G4 -2.83 (x1e-5, within SEM). Also the
%   gap accounting of the row4 replay (sum reproduces -0.0072).
here = fileparts(mfilename('fullpath'));
WT = fileparts(fileparts(here));
od = fullfile(WT, 'test_results', 'loop_mean_bias');
S = load(fullfile(od, 'replay_v2_profile.mat'));  o = S.out;  t = o.t(:);
lam = 0.7;  alpha = 1 - lam;  ax = 3;
% UNIFIED-MECHANISM CHECK: is the row-4 forcing just  a'_hat x (true displacement - believed displacement)?
SL = zeros(1,8); CC = zeros(1,8);
R4P = zeros(numel(t), 8);
for q = 1:8
    C = load(fullfile(od, sprintf('loop_capture_seed%d.mat', q)));
    R = load(fullfile(od, sprintf('run_log_seed%d.mat', q)));  r = R.run_log;
    kk = C.KK;  n = numel(kk);
    ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);
    dw_true_step = [0; diff(ht)];  dw_true_step = dw_true_step(kk);      % actual step of the particle [R]
    dwd = [0; diff(hd)];  dwd = dwd(kk);
    ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
    aph = bh .* (1 - ah).^2;                                             % filter's a' at its own estimate
    x3 = C.XU(3, :).';  m89 = (C.XU(8, :) + C.XU(9, :)).';
    Mhat = dwd + [0; alpha * x3(1:end-1)] + [0; alpha * m89(1:end-1)];   % believed displacement for predict k-1 -> k
    r4p = aph .* (dw_true_step - Mhat);
    R4P(:, q) = r4p;
    g4 = o.G4(:, q);
    m = t > 0.5;                       % skip first hold (nothing moves)
    SL(q) = r4p(m) \ g4(m);  CC(q) = corr(r4p(m), g4(m));
end
fprintf('[unified row4] per-seed regression G4 ~ a''*(dw_true - M_hat): slope %.3f +- %.3f   corr %.3f +- %.3f\n', ...
        mean(SL), std(SL)/sqrt(8), mean(CC), std(CC)/sqrt(8));
SEG = {'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'hold2', t > 3.7; 'gaps', ~(t < 0.55 | (t > 0.55 & t < 1.45) | (t >= 1.5 & t < 3.5) | t > 3.7)};
for g = 1:4
    m = SEG{g,2};
    fprintf('  %-6s mean (x1e-5): G4 %+6.2f +- %4.2f   pred %+6.2f +- %4.2f\n', SEG{g,1}, ...
        1e5*mean(mean(o.G4(m,:),1)), 1e5*std(mean(o.G4(m,:),1))/sqrt(8), ...
        1e5*mean(mean(R4P(m,:),1)), 1e5*std(mean(R4P(m,:),1))/sqrt(8));
end
% gap accounting for the row4 replay
C7 = load(fullfile(od, 'loop_capture_seed7.mat'));
nsl = size(C7.F,1);  n = numel(t);  i0 = 3;  gb4 = mean(o.G4, 2);  mh = t > 3.7;
segs = {t < 0.55, t > 0.55 & t < 1.45, t >= 1.5 & t < 3.5, t > 3.7, ~(t < 0.55 | (t > 0.55 & t < 1.45) | (t >= 1.5 & t < 3.5) | t > 3.7)};
nm = {'hold1+', 'desc', 'osc', 'hold2', 'gaps'};
tot = 0;
for g = 1:5
    e = zeros(nsl,1);  E = zeros(n,1);
    for i = i0+1:n
        gv = zeros(nsl,1);  gv(4) = gb4(i) * segs{g}(i);
        e = C7.A(:,:,i) * e + gv;  E(i) = e(4);
    end
    fprintf('  row4 contribution from %-7s: %+.4f\n', nm{g}, mean(E(mh)));  tot = tot + mean(E(mh));
end
fprintf('  sum %+.4f (row4-only total was -0.0072)\n', tot);
fprintf('ROW4 DONE\n');
