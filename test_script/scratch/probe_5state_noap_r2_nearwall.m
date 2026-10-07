% PURPOSE (2026-09-23, user: "check the near-wall R2 30% item (O8) here"): is the booked y2 innovation variance honest
%   at the three fixed heights? O8 (mainline ledger): R2 is evaluated at lambda_c, the loop actually runs at lambda_eff
%   when a_hat != a; Var(a_m)/formula near the wall 1.29-1.35 with the estimated gain, 1.00-1.05 when the law uses the
%   true gain. Here: ratio Var(innov2)/mean(S2) in the last 2 s, arms = law on a_hat (default) vs law on a_true
%   (use_true_gain; the filter still estimates), 5 seeds, cheat init, mirror on (default). If the estimated-gain arm is
%   ~1.3 near the wall and the true-gain arm ~1, O8 reproduces in this estimator.
%   Runs under matlab -batch; prints one line per (height, arm). EXPIRES: with the layer-1 line | 產線改動不會自動跟上
function probe_5state_noap_r2_nearwall(seeds)
    if nargin < 1 || isempty(seeds); seeds = 1:5; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    pc = physical_constants();  R = pc.R;
    fprintf('%-8s %-10s | Var(innov2)/mean(S2) last 2 s | mean innov2 / sqrt(S2) | Var(innov1)/S1\n', 'height', 'law gain');
    for h = [22.2 2.0 1.10]
        for tg = [false true]
            I2 = []; S2 = []; I1 = []; S1 = [];
            for sd = seeds
                cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
                cfg.trajectory_type = 'positioning';  cfg.h_init = h * R;  cfg.h_bottom = h * R;  cfg.amplitude = 0;  cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;
                s = run_pure_simulation(cfg, struct('seed', sd, 'collect_diag', true, 'use_true_gain', tg, 'ctrl_const_override', struct('seed_mode', 'true_h0', 'lf_mode', 'ljung')));
                t = s.tout(:);  m = t >= 1 & s.diag.S_y2(:, 3) > 0;
                I2 = [I2; s.diag.innovation_y2(m, 3)];  S2 = [S2; s.diag.S_y2(m, 3)]; %#ok<AGROW>
                I1 = [I1; s.diag.innovation_y1(m, 3)];  S1 = [S1; s.diag.S_y1(m, 3)]; %#ok<AGROW>
            end
            lab = 'a_hat'; if tg; lab = 'a_true'; end
            fprintf('%-8.2f %-10s | %.3f | %+.3f | %.3f\n', h, lab, var(I2) / mean(S2), mean(I2) / sqrt(mean(S2)), var(I1) / mean(S1));
        end
    end
end
