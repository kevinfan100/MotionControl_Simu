% PURPOSE (2026-09-20): pre-registered acceptance of the no-a' 5-state layer (0920_5state_noaprime.tex S10, items 1-2)
%   1  a_cov invariance: a_cov in {0.025, 0.05, 0.1} on canon deep, seeds 1:3 -> max |delta a_hat|/a_nom (z); expected
%      floating-point level except at G2 gate steps (whitening fingerprint, 08-27 step 4). Compared against the RAW
%      readout arm (y2_whiten = false) where the same sweep must move a_hat (instrument validity).
%   2  y1 honesty: Var(innov1)/mean(S1) per window on canon deep; expected ~1. LOO arms nw_mcorr / corr_input / ma2_aug /
%      fe_km1+upd_before_law off show the mechanism signatures (ratio leaves 1).
%  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function verify_5state_noap_accept()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    seeds = 1:3;
    fprintf('--- 1. a_cov invariance (canon deep, z, max |a_hat(a_cov) - a_hat(0.05)|/a_nom over the run) ---\n');
    for whiten = [true false]
        fprintf('  y2_whiten = %d\n', whiten);
        for sd = seeds
            ref = local_run(0.05, sd, struct('y2_whiten', whiten));
            for ac = [0.025, 0.1]
                s = local_run(ac, sd, struct('y2_whiten', whiten));
                d = abs(s.diag.a_hat(:, 3) - ref.diag.a_hat(:, 3)) / a_nom;
                g = sum(s.diag.gate_active(:, 3) ~= ref.diag.gate_active(:, 3));
                fprintf('    seed %d a_cov %.3f: max diff %.2e (first > 1e-9 at k = %s) | gate-pattern diffs %d\n', ...
                        sd, ac, max(d), mat2str(find(d > 1e-9, 1)), g);
            end
        end
    end
    fprintf('--- 2. y1 honesty Var(innov1)/mean(S1) per window (canon deep, z, seeds 1:3 pooled) ---\n');
    arms = {'ALL ON', struct(); 'nw_mcorr off', struct('nw_mcorr', false); 'corr_input off', struct('corr_input', false); ...
            'ma2_aug off', struct('ma2_aug', false); 'old order (fe_km1+upd_before_law off)', struct('fe_km1', false, 'upd_before_law', false); ...
            'y2_whiten off', struct('y2_whiten', false)};
    wins = {'hold0', [0 0.5]; 'descent', [0.5 1.5]; 'osc', [1.5 3.5]; 'hold-end', [3.5 4.8]};
    fprintf('  %-42s', 'arm');  fprintf('%12s', wins{:, 1});  fprintf('\n');
    for ia = 1:size(arms, 1)
        I = [];  S = [];  T = [];
        for sd = seeds
            s = local_run(0.05, sd, arms{ia, 2});
            I = [I; s.diag.innovation_y1(:, 3)];  S = [S; s.diag.S_y1(:, 3)];  T = [T; s.tout(:)]; %#ok<AGROW>
        end
        fprintf('  %-42s', arms{ia, 1});
        for w = 1:size(wins, 1)
            m = T >= wins{w, 2}(1) & T < wins{w, 2}(2);
            fprintf('%12.3f', var(I(m)) / mean(S(m)));
        end
        fprintf('\n');
    end
end

function s = local_run(a_cov, seed, cc)
    cfg = canonical_scenario(a_cov, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    cc.seed_mode = 'true_h0';
    s = run_pure_simulation(cfg, struct('seed', seed, 'collect_diag', true, 'ctrl_const_override', cc));
end
