% PURPOSE (2026-09-20): the two honesty gaps of the no-a' 5-state battery, each with a rival and its discriminator
%   (derivation-workflow rule 6). Baselines from run_5state_noap_battery.m (test_results/temp_5state_noap/*.mat).
%   GAP 1  hold P honesty 1.5-1.8 (sd_seed / sqrtP44 in the last 1 s of the fixed cells; acceptance band [0.7, 1.4]).
%          [hyp] readout structure beyond the DC colour (u is chi-square, reads the state, a_pd centering)
%          rival  metric artefact: e_a = a_hat - a_true(h_true) carries the instantaneous a_ram (K_h x_ram), not filter error
%          rival  filter structure (rate slot with Q55 = 0, prior P55[0])
%          split  (i) y2_oracle: white Gaussian u with the SAME mean/marginal, IF = 1 -> filter+metric only;
%                 (ii) same battery data, error against a_det(h_bar_d) -> metric only
%   GAP 2  y1 honesty 1.76 in the canon osc window (other windows 0.82-0.97).
%          [hyp] Q_T = 4 kBT a_hat with a_hat wrong by 2-4x inside the swing (the layer's own gain error through Q)
%          rival  the F_dx e_a term of the plant (already excluded: use_true_gain arm gives 2.09, not 1)
%          rival  readout (y2) structure moving the gain slot inside the swing
%          split  (iii) q_gain_true: Q_T at a_true; (iv) y2_oracle on canon; (v) local ratio in 0.1 s bins vs phase
%   Cost ~4 min. EXPIRES: with the layer-1 line | 產線改動不會自動跟上
function probe_5state_noap_honesty()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  R = pc.R;

    % ---- (ii) metric only: fixed cells, error vs a_det(h_bar_d) instead of a_true(h_true) ----
    fprintf('--- GAP 1 (ii): fixed cells, last 1 s, sd_seed / sqrtP44 with e vs a_true(h_true)  |  vs a_det(h_d) ---\n');
    for cn = {'fixed_h22p2', 'fixed_h2p00', 'fixed_h1p10'}
        r = load(fullfile(id, [cn{1} '.mat']));  m = r.t >= r.t(end) - 1;
        [~, cp] = calc_correction_functions(r.hd(1));  a_det = 1 / cp;
        sp = mean(sqrt(r.P_a(m, :)), 'all');
        sd_true = mean(std(r.a_hat(m, :) - r.a_true(m, :), 0, 2));
        sd_det  = mean(std(r.a_hat(m, :) - a_det, 0, 2));
        sd_aram = mean(std(r.a_true(m, :) - a_det, 0, 2));
        fprintf('  %-12s sqrtP44 %.4f | sd vs a_true %.4f (%.2f) | sd vs a_det %.4f (%.2f) | sd of a_ram itself %.4f\n', ...
                cn{1}, sp, sd_true, sd_true / sp, sd_det, sd_det / sp, sd_aram);
    end

    % ---- (i) y2_oracle on the 22.2 R hold: filter + metric only ----
    fprintf('--- GAP 1 (i): fixed 22.2 R, 10 seeds, y2_oracle (white u, IF = 1) ---\n');
    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    cfg.trajectory_type = 'positioning';  cfg.h_init = 22.2 * R;  cfg.h_bottom = 22.2 * R;
    cfg.amplitude = 0;  cfg.t_hold = 0;  cfg.t_descend_override = 0;  cfg.T_sim = 3.0;
    E = [];  Pm = [];
    for sd = 1:10
        s = run_pure_simulation(cfg, struct('seed', sd, 'collect_diag', true, ...
                'ctrl_const_override', struct('seed_mode', 'anom', 'y2_oracle', true)));
        m = s.tout >= 2;  E(:, sd) = (s.diag.a_hat(m, 3) - s.a_true_out(m, 3)) / a_nom;  Pm(:, sd) = s.diag.P_a(m, 3) / a_nom^2; %#ok<AGROW>
    end
    fprintf('  oracle: mean e %+.4f | sd_seed %.4f | sqrtP44 %.4f | honesty %.2f   (baseline real readout 1.53)\n', ...
            mean(E, 'all'), mean(std(E, 0, 2)), mean(sqrt(Pm), 'all'), mean(std(E, 0, 2)) / mean(sqrt(Pm), 'all'));

    % ---- (v) local y1 honesty vs phase from the battery canon data ----
    fprintf('--- GAP 2 (v): canon, y1 honesty in 0.25 s bins (10 seeds pooled) with mean e_a of the bin ---\n');
    r = load(fullfile(id, 'canon.mat'));  edges = 0:0.25:4.75;
    for b = 1:numel(edges) - 1
        m = r.t >= edges(b) & r.t < edges(b + 1);
        fprintf('  %4.2f-%4.2f s  ratio %.2f  e_a %+.3f  a_true %.3f\n', edges(b), edges(b + 1), ...
                var(r.innov1(m, :), 0, 'all') / mean(r.S_y1(m, :), 'all'), mean(r.a_hat(m, :) - r.a_true(m, :), 'all'), mean(r.a_true(m, :), 'all'));
    end

    % ---- (iii)/(iv) canon arms: Q at a_true; oracle y2 ----
    fprintf('--- GAP 2 (iii)/(iv): canon seeds 1:3, y1 honesty per window ---\n');
    arms = {'baseline', struct(); 'q_gain_true', struct('q_gain_true', true); 'y2_oracle', struct('y2_oracle', true); ...
            'q_gain_true + y2_oracle', struct('q_gain_true', true, 'y2_oracle', true)};
    wins = {'hold0', [0 .5]; 'descent', [.5 1.5]; 'osc', [1.5 3.5]; 'hold-end', [3.5 4.8]};
    fprintf('  %-26s', 'arm');  fprintf('%12s', wins{:, 1});  fprintf('   | osc mean e_a\n');
    for ia = 1:size(arms, 1)
        I = [];  S = [];  T = [];  Ea = [];
        for sd = 1:3
            cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
            cc = arms{ia, 2};  cc.seed_mode = 'true_h0';
            s = run_pure_simulation(cfg, struct('seed', sd, 'collect_diag', true, 'ctrl_const_override', cc));
            I = [I; s.diag.innovation_y1(:, 3)];  S = [S; s.diag.S_y1(:, 3)];  T = [T; s.tout(:)]; %#ok<AGROW>
            Ea = [Ea; (s.diag.a_hat(:, 3) - s.a_true_out(:, 3)) / a_nom]; %#ok<AGROW>
        end
        fprintf('  %-26s', arms{ia, 1});
        for w = 1:4; m = T >= wins{w, 2}(1) & T < wins{w, 2}(2); fprintf('%12.3f', var(I(m)) / mean(S(m))); end
        m = T >= 1.5 & T < 3.5;  fprintf('   | %+.3f\n', mean(Ea(m)));
    end
end
