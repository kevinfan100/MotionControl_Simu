% STATUS: ACTIVE | seed-11 outlier adjudication, part 2 (statistics + arm anatomy)
%   EXPIRES: with audit_seed11_rng_stream.m -- when the seed-11 verdict is on record.
% FORK OF nothing (new diagnostic) | PURPOSE: read the two 8-seed .mat files
%   produced by audit_seed11_rng_stream.m and answer (a) how extreme seed 11 is
%   inside each arm's own spread, (b) whether the two arms rank the seeds the
%   same way (shared draw) and (c) what seed 11's new-arm run does differently.
%   Zero production changes; read-only on test_results/. 產線改動不會自動跟上
%   ORDER: run audit_seed11_rng_stream.m FIRST -- the two .mat files this reads are
%   named by arm tag, so any later driver call with different seeds overwrites them.

clear cd
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

LB = load(fullfile(proj, 'test_results', 'run_formB_ws_t1_y2on.mat'));      outB = LB.out;
LC = load(fullfile(proj, 'test_results', 'run_formC_state_cfgB_y2on.mat')); outC = LC.out;
seeds = outB.seeds;   ns = numel(seeds);
assert(isequal(seeds, outC.seeds), 'seed lists differ');
MB = outB.metrics.rows(:, 1:3);   MC = outC.metrics.rows(:, 1:3);
NAMES = {'desc pk %', 'osc RMS %', 'hold mean %'};
i11 = find(seeds == 11);

fprintf('\n=== how extreme is seed 11 inside each arm''s OWN spread ===\n');
fprintf('%12s | %26s | %26s\n', 'metric', 'formB: z_all  z_LOO  rank', 'formC: z_all  z_LOO  rank');
for m = 1:3
    [zb, zlb, rb] = local_extremeness(MB(:, m), i11);
    [zc, zlc, rc] = local_extremeness(MC(:, m), i11);
    fprintf('%12s | %+8.2f %+8.2f  %2d/%d | %+8.2f %+8.2f  %2d/%d\n', ...
            NAMES{m}, zb, zlb, rb, ns, zc, zlc, rc, ns);
end
fprintf(['(z_all = deviation from the 8-seed mean in 8-seed sd; z_LOO = same with seed 11\n' ...
         ' removed from mean and sd -- the honest outlier statistic; rank 1 = largest |value|)\n']);
fprintf('Grubbs 5%% critical value for n = 8 is 2.13 (two-sided, max |z_all|).\n');

fprintf('\n=== do the two arms rank the seeds the same way? ===\n');
for m = 1:3
    r  = corr(MB(:, m), MC(:, m));
    rs = corr(tiedrank(MB(:, m)), tiedrank(MC(:, m)));
    r_no11 = corr(MB(setdiff(1:ns, i11), m), MC(setdiff(1:ns, i11), m));
    fprintf('%12s: Pearson %+5.2f  Spearman %+5.2f  Pearson w/o seed 11 %+5.2f\n', ...
            NAMES{m}, r, rs, r_no11);
end

% Regression of the new arm on production, seed 11 EXCLUDED, then predict 11.
fprintf('\n=== new-arm value predicted from the production value (fit excludes seed 11) ===\n');
keep = setdiff(1:ns, i11);
for m = 1:3
    X = [ones(numel(keep), 1), MB(keep, m)];
    beta = X \ MC(keep, m);
    res  = MC(keep, m) - X * beta;
    sres = std(res, 1) * sqrt(numel(keep) / max(numel(keep) - 2, 1));
    pred = [1, MB(i11, m)] * beta;
    fprintf('%12s: predicted %+7.2f, observed %+7.2f, residual %+7.2f = %.1f sigma of the fit\n', ...
            NAMES{m}, pred, MC(i11, m), MC(i11, m) - pred, (MC(i11, m) - pred) / max(sres, eps));
end

% ---------------------------------------------------------------------
% What does seed 11 do differently INSIDE the new arm?
% ---------------------------------------------------------------------
fprintf('\n=== new-arm anatomy: state trajectories per seed (z axis) ===\n');
AX = 3;
fprintf('%6s | %9s %9s | %9s %9s | %9s | %9s %9s\n', 'seed', ...
        'da end', 'da min', 'ws end', 'ws range', 'gate frac', 'hold e_a%', 'osc e_a%');
for q = 1:ns
    s = outC.runs{q};
    t = s.tout;
    hold_w = t > 3.8;
    e = 100 * (s.a_hat_out(:, AX) - s.a_true_out(:, AX)) ./ s.a_true_out(:, AX);
    da = s.b_hat_out(:, AX);   ws = s.ws_hat_out(:, AX);
    fprintf('%6d | %+9.4f %+9.4f | %+9.4f %9.4f | %9.3f | %+9.2f %9.2f\n', ...
            seeds(q), da(end), min(da), ws(end), max(ws) - min(ws), ...
            mean(s.gate_out(:, AX)), mean(e(hold_w)), sqrt(mean(e(t > 1.6 & t <= 3.5).^2)));
end

fprintf('\n=== same anatomy for production (formB) ===\n');
fprintf('%6s | %9s %9s | %9s %9s | %9s\n', 'seed', 'b end', 'p end', 'ws end', 'a_hat end', 'hold e_a%');
for q = 1:ns
    s = outB.runs{q};
    t = s.tout;  hold_w = t > 3.8;
    e = 100 * (s.a_hat_out(:, AX) - s.a_true_out(:, AX)) ./ s.a_true_out(:, AX);
    fprintf('%6d | %9.5f %9.5f | %9.5f %9.4f | %+9.2f\n', seeds(q), ...
            s.b_hat_out(end, AX), s.p_hat_out(end, AX), s.ws_hat_out(end, AX), ...
            s.a_hat_out(end, AX), mean(e(hold_w)));
end

% Is the new arm's error simply a monotone amplification of the same physical
% excursion? Compare the true-height and force realisations in the hold window.
fprintf('\n=== physical excursions in the hold window (identical draws, both arms) ===\n');
fprintf('%6s | %22s | %22s\n', 'seed', 'formB: h_bar mean/sd, |Fth_z|', 'formC: h_bar mean/sd, |Fth_z|');
for q = 1:ns
    sB = outB.runs{q};  sC = outC.runs{q};
    hw = sB.tout > 3.8;
    fprintf('%6d | %8.4f %7.4f %6.3f | %8.4f %7.4f %6.3f\n', seeds(q), ...
            mean(sB.h_bar_true_out(hw)), std(sB.h_bar_true_out(hw)), mean(abs(sB.F_th_out(hw, AX))), ...
            mean(sC.h_bar_true_out(hw)), std(sC.h_bar_true_out(hw)), mean(abs(sC.F_th_out(hw, AX))));
end

% ---------------------------------------------------------------------
function [z_all, z_loo, rnk] = local_extremeness(v, i)
%LOCAL_EXTREMENESS  Deviation of element i in full-sample and leave-one-out units.
    z_all = (v(i) - mean(v)) / std(v);
    rest  = v([1:i-1, i+1:end]);
    z_loo = (v(i) - mean(rest)) / std(rest);
    [~, ord] = sort(abs(v - mean(v)), 'descend');
    rnk = find(ord == i);
end
