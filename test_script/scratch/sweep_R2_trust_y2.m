% Does making the filter TRUST the gain readout fix the near-wall bias?
%
% Kevin's question: I claimed the filter "mostly believes the law rather than
% measuring the gain" (y2 carries ~1% of a_hat's update, signal-to-noise
% H2*P*H2'/R2 ~ 3.5e-5). He asked what that has to do with the +30.8%
% left-endpoint integration error found on 08-24. The link is that a Kalman
% filter is SUPPOSED to correct a bad predict with measurement -- so a predict
% error surviving to the end is itself evidence the measurement is not being
% listened to. This sweep tests that directly.
%
% KNOB: ctrl_const.amlpf_var_factor multiplies R2_int (controller:1552).
% formC has no LPF, so it is an empty multiplier sitting at 1 -- i.e. a free
% "how much do you trust y2" dial that needs NO production change.
% Smaller factor -> smaller R2 -> larger K2 -> filter trusts y2 more.
%
% NOTE it is NOT ctrl_const.use_am_lpf. That flag sets the same multiplier to
% 0.089 while the readout is never actually filtered (formC has no a_m_det),
% which is a trap, not an experiment.
%
% PRE-REGISTERED READ:
%   bias falls substantially as the factor drops  -> "does not trust the
%       measurement" is load-bearing; an actual LPF on a_m becomes worth
%       designing.
%   bias flat across two decades                  -> even full trust in y2
%       cannot fix it; the disease has to be treated at the predict/addition
%       side, and the LPF line stays closed.
%
% LIVENESS FIRST (project rule 13, and two dead flags were found on 08-24):
%   R2 must actually move with the factor. Read that BEFORE any bias number.
%   R2 is not exactly proportional -- it is evaluated at a_bar_hat, which
%   differs between arms, and the d*Q44 delay term is not scaled -- so the
%   check is "R2 tracks the factor within a factor of 2", not equality.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

SEEDS = 1:30;  AX = 3;
FACTORS = [0.01, 0.1, 1, 10];      % 1 = production
res = struct('f', {}, 'bias', {}, 'R2', {}, 'K2', {}, 'K1', {}, 'nanfrac', {});

for i = 1:numel(FACTORS)
    f = FACTORS(i);
    clear run_formC_b motion_control_law_formC_b;
    fprintf('\n=== amlpf_var_factor = %g ===\n', f);
    D = run_formC_b(struct('seeds', SEEDS, 'arm', 'best', ...
                           'ctrl_const_override', struct('amlpf_var_factor', f)));
    ns = numel(D.runs);
    t  = D.runs{1}.tout(2:end);
    ih = t >= 3.75;                                  % trough hold
    bias = zeros(1, ns);  R2m = zeros(1, ns);  K2m = zeros(1, ns);  K1m = zeros(1, ns);
    nn = 0;
    for s = 1:ns
        r  = D.runs{s};
        ah = r.a_bar_hat_out(2:end, AX);
        at = r.a_true_out(2:end, AX) / r.a_nom;
        bias(s) = mean((ah(ih) - at(ih)) ./ at(ih));
        R2m(s)  = median(r.R2_out(2:end, AX));
        K2m(s)  = median(abs(r.K_a_y2_out(2:end, AX)));
        K1m(s)  = median(abs(r.K_a_y1_out(2:end, AX)));
        nn = nn + sum(~isfinite(ah));
    end
    res(i) = struct('f', f, 'bias', bias, 'R2', median(R2m), 'K2', median(K2m), ...
                    'K1', median(K1m), 'nanfrac', nn / (ns * numel(t)));
end

iref = find(FACTORS == 1);
fprintf('\n========== LIVENESS (read this first) ==========\n');
fprintf('  factor |  median R2   R2/R2(f=1) | expected | median |K2|  K2/K2(f=1)\n');
for i = 1:numel(res)
    fprintf('  %6.3g | %10.4e  %10.4f | %8.3g | %10.4e  %10.4f\n', ...
            res(i).f, res(i).R2, res(i).R2/res(iref).R2, res(i).f, ...
            res(i).K2, res(i).K2/res(iref).K2);
end
fprintf('  VERDICT: knob is LIVE only if column 3 tracks column 4.\n');

fprintf('\n========== PRIMARY: trough-hold bias on a_hat ==========\n');
fprintf('  factor |    bias      sd      SEM  | paired vs f=1     t   | NaN frac\n');
for i = 1:numel(res)
    b = res(i).bias;  d = b - res(iref).bias;
    if i == iref
        fprintf('  %6.3g | %+7.3f%% %6.3f%% %6.3f%% |    (reference)        | %.1e\n', ...
                res(i).f, 100*mean(b), 100*std(b), 100*std(b)/sqrt(numel(b)), res(i).nanfrac);
    else
        fprintf('  %6.3g | %+7.3f%% %6.3f%% %6.3f%% | %+8.4f%%  %+7.2f | %.1e\n', ...
                res(i).f, 100*mean(b), 100*std(b), 100*std(b)/sqrt(numel(b)), ...
                100*mean(d), mean(d)/(std(d)/sqrt(numel(d))), res(i).nanfrac);
    end
end

fprintf('\n========== y1 vs y2 gain balance ==========\n');
for i = 1:numel(res)
    fprintf('  factor %6.3g : |K2|/|K1| = %.5f\n', res(i).f, res(i).K2 / res(i).K1);
end
save('test_results/am_r22_deep/sweep_R2_trust_y2.mat', 'res', 'FACTORS', 'SEEDS');
fprintf('\nsaved test_results/am_r22_deep/sweep_R2_trust_y2.mat\n');
