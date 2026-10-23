% PURPOSE (2026-09-23): does the y2 gauge-sensitivity correction (y2_mirror, sensitivity in the Jacobian only; the
%   2026-09-22 version also scaled the prediction, which biased the hold and broke the ramp) close the fixed-point
%   over-confidence (1.2-1.5x) without hurting motion? Arms: mirror off / on. Cells: fixed 1.10 R and 22.2 R (10 seeds),
%   meng and canon (5 seeds). Writes a diary to test_results/temp_5state_noap/probe_mirror_<cell>.txt so a long MCP call
%   cannot lose the result. Usage: probe_5state_noap_mirror('fixed') | ('meng') | ('canon') | ('regress')
%   EXPIRES: with the layer-1 line | 產線改動不會自動跟上
function probe_5state_noap_mirror(which)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  R = pc.R;
    diary(fullfile(od, ['probe_mirror_' which '.txt']));  diary on;  c = onCleanup(@() diary('off'));
    switch which
        case 'regress'
            ref = load(fullfile(od, 'canon_best.mat'));
            cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
            s = run_pure_simulation(cfg, struct('seed', 1, 'collect_diag', true, 'ctrl_const_override', struct('seed_mode', 'true_h0', 'lf_mode', 'ljung')));
            fprintf('regression mirror-off vs canon_best seed 1: %.2e\n', max(abs(s.diag.a_hat(:, 3) / a_nom - ref.a_hat(:, 1))));
        case 'fixed'
            for h = [1.10 22.2]
                for mir = [false true]
                    E = []; P = [];
                    for sd = 1:10
                        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
                        cfg.trajectory_type = 'positioning';  cfg.h_init = h * R;  cfg.h_bottom = h * R;  cfg.amplitude = 0;  cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;
                        s = run_pure_simulation(cfg, struct('seed', sd, 'collect_diag', true, 'ctrl_const_override', struct('seed_mode', 'anom', 'lf_mode', 'ljung', 'y2_mirror', mir)));
                        t = s.tout(:);  E(:, sd) = (s.a_true_out(:, 3) - s.diag.a_hat(:, 3)) / a_nom;  P(:, sd) = s.diag.P_a(:, 3) / a_nom^2; %#ok<AGROW>
                    end
                    he = t >= 2;  mid = t >= 1 & t < 1.5;
                    fprintf('fixed %5.2f R mirror=%d | last 1 s: mean %+.4f sd %.4f sqrtP44 %.4f hon %.2f | 1-1.5 s hon %.2f\n', h, mir, mean(E(he, :), 'all'), mean(std(E(he, :), 0, 2)), mean(sqrt(P(he, :)), 'all'), mean(std(E(he, :), 0, 2)) / mean(sqrt(P(he, :)), 'all'), mean(std(E(mid, :), 0, 2)) / mean(sqrt(P(mid, :)), 'all'));
                end
            end
        case {'meng', 'canon'}
            for mir = [false true]
                E = []; A = []; P = [];
                for sd = 1:5
                    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
                    if strcmp(which, 'meng'); cfg.h_init = 15; cfg.h_bottom = 2.5; cfg.amplitude = 0; cfg.frequency = 1; cfg.n_cycles = 1; cfg.t_hold = 0.5; cfg.t_descend_override = 10; cfg.T_sim = 12.5; cfg.h_min = 2.475; end
                    s = run_pure_simulation(cfg, struct('seed', sd, 'collect_diag', true, 'ctrl_const_override', struct('seed_mode', 'true_h0', 'lf_mode', 'ljung', 'y2_mirror', mir)));
                    t = s.tout(:);  E(:, sd) = (s.a_true_out(:, 3) - s.diag.a_hat(:, 3)) / a_nom;  A(:, sd) = s.a_true_out(:, 3) / a_nom;  P(:, sd) = s.diag.P_a(:, 3) / a_nom^2; %#ok<AGROW>
                end
                em = mean(E, 2);  a = mean(A, 2);
                if strcmp(which, 'canon'); mo = t >= 1.5 & t < 3.5; he = t >= 3.8; else; mo = t >= 1.5 & t < 10.5; he = t >= 11.5; end
                fprintf('%-5s mirror=%d | motion rms %.3f max %.3f rel %.3f | hold-end mean %+.4f sd %.4f sqrtP44 %.4f (%.2f)\n', which, mir, rms(em(mo)), max(abs(em(mo))), rms(em(mo) ./ a(mo)), mean(em(he)), mean(std(E(he, :), 0, 2)), mean(sqrt(P(he, :)), 'all'), mean(std(E(he, :), 0, 2)) / mean(sqrt(P(he, :)), 'all'));
            end
    end
end
