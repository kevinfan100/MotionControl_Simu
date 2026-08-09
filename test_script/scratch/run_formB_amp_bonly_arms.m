function res = run_formB_amp_bonly_arms(opts)
%RUN_FORMB_AMP_BONLY_ARMS  Stage-4 arm ladder for the amplitude-writing probe.
%
%   STATUS: ACTIVE -- experiment runner for
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%   Structural acceptance for the tied arm: verify_formB_amp_tie (must PASS
%   before any number here is read).
%
%   Plant is the REAL plane wall (published two-sphere perpendicular series);
%   the controller knows nothing about it and every seed comes from the
%   far-field method-of-reflections limit. Five arms, identical scenario,
%   identical noise realisation per seed, differing only in how the gain law
%   is written and what prior that writing honestly costs:
%
%     A0  len_prod   production tier-1: (b, p) free, prior 0.0157 / 0.0245
%     A1  len_bonly  length writing, ONLY b free, production prior 0.0157
%     A2  len_wide   length writing, ONLY b free, prior widened to 0.0708
%     A3  len_ws98   as A2 but the law origin declared at 9/8 -- same law as
%                    the tied arm at t = 0, but only J_b in the b column
%     A4  amp        the tied arm: a_bar = 1 - beta/w, J_beta = 1/w^2
%
%   The ladder is a decomposition, not five unrelated arms:
%       A1 -> A2   what the PRIOR WIDTH alone buys
%       A2 -> A3   what the LAW ORIGIN alone buys
%       A3 -> A4   what the JACOBIAN FOLD alone buys
%   Each step changes exactly one thing, so the three effects add.
%
%   Pre-registered landing hypotheses for beta (derivation p.1-2):
%       1.1250  the far-field anchor -- "did not move"
%       1.0833  information-weighted <b_eff> along the commanded trajectory
%       1.0651  minimax over the envelope
%       1.0588  the value the truth demands at the final hold
%
%   Output -> test_results/temp_formB_amp_stage4.mat (gitignored, regenerable)
%
%   Runtime ~4 s per run; 5 arms x 12 seeds ~= 4 min.

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds')
        % House 6-seed convention plus 6 more, deliberately clear of the
        % 1:20 block used by the 2026-08-01 readout-chain arms.
        opts.seeds = [7 11 23 42 101 777 31 53 89 137 211 313];
    end
    if ~isfield(opts, 'save'); opts.save = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(genpath(fullfile(root, 'test_script')));

    B_ANCHOR = 9/8;
    AX_Z     = 3;
    seeds    = opts.seeds(:).';
    n_s      = numel(seeds);

    cfg = local_canonical_cfg();
    pr  = local_amp_priors(cfg);       % [sPbb_amp, floor_a_amp]
    pl  = local_len_priors(cfg);       % [sPbb_len, sPpp_len, floor_a_len]

    fprintf('=== amplitude-writing probe: arm ladder ===\n');
    fprintf('plant = real plane wall; %d seeds; envelope [%.3f, %.3f]\n', ...
            n_s, cfg.h_bottom / 2.25 - 0.1, cfg.h_init / 2.25 + 1);
    fprintf('priors: length b %.4f / p %.4f / floor %.5f | amplitude b %.4f / floor %.5f\n\n', ...
            pl(1), pl(2), pl(3), pr(1), pr(2));

    ARMS = { ...
      'len_prod',  struct('lock_b', false, 'lock_p', false, 'lock_ws', true, ...
                          'Pf_b_std', pl(1), 'Pf_p_std', pl(2), 'Pf_a_floor', pl(3)); ...
      'len_bonly', struct('lock_b', false, 'lock_p', true,  'lock_ws', true, ...
                          'Pf_b_std', pl(1), 'Pf_a_floor', pl(3)); ...
      'len_wide',  struct('lock_b', false, 'lock_p', true,  'lock_ws', true, ...
                          'Pf_b_std', pr(1), 'Pf_a_floor', pl(3)); ...
      'len_ws98',  struct('lock_b', false, 'lock_p', true,  'lock_ws', true, ...
                          'ws_init', B_ANCHOR, ...
                          'Pf_b_std', pr(1), 'Pf_a_floor', pr(2)); ...
      'amp',       struct('law_form_amp', true, 'lock_b', false, 'lock_p', true, ...
                          'lock_ws', true, 'p_init', 1, 'b_init', B_ANCHOR, ...
                          'Pf_b_std', pr(1), 'Pf_a_floor', pr(2)) };

    n_a = size(ARMS, 1);
    res = struct('seeds', seeds, 'cfg', cfg, 'arm_names', {ARMS(:, 1).'}, ...
                 'priors_len', pl, 'priors_amp', pr);
    res.runs = cell(n_a, n_s);
    res.tab  = nan(n_a, n_s, 7);   % b0 b_end sqrtP0 sqrtPend desc osc hold

    t_all = tic;
    for a = 1:n_a
        nm = ARMS{a, 1};
        ov = ARMS{a, 2};
        for q = 1:n_s
            s = run_formB_ws(cfg, struct('seed', seeds(q), ...
                                         'ctrl_const_override', ov));
            m = local_metrics(s, cfg, AX_Z);
            res.runs{a, q} = local_slim(s, AX_Z);
            res.tab(a, q, :) = [s.b_hat_out(1, AX_Z), s.b_hat_out(end, AX_Z), ...
                                s.P_b_out(1, AX_Z),  s.P_b_out(end, AX_Z), ...
                                m.desc, m.osc, m.hold];
        end
        T = squeeze(res.tab(a, :, :));
        fprintf(['%-10s  beta %.4f -> %.4f +- %.4f  (%+.2f sigma) | ' ...
                 'sqrtP %.4f -> %.4f (shrink %4.1f%%) | ' ...
                 'desc %5.2f  osc %5.2f  hold %+6.2f  %%\n'], ...
                nm, mean(T(:, 1)), mean(T(:, 2)), std(T(:, 2)), ...
                mean(T(:, 2) - T(:, 1)) / mean(T(:, 3)), ...
                mean(T(:, 3)), mean(T(:, 4)), ...
                100 * (1 - mean(T(:, 4)) / mean(T(:, 3))), ...
                mean(T(:, 5)), mean(T(:, 6)), mean(T(:, 7)));
    end
    fprintf('\ntotal %.1f s for %d runs\n', toc(t_all), n_a * n_s);

    % ---- the decomposition the ladder was built for ----------------------
    trav = squeeze(mean(res.tab(:, :, 2) - res.tab(:, :, 1), 2));
    fprintf('\nbeta travel decomposition (mean over %d seeds, in units of R):\n', n_s);
    fprintf('  A1 len_bonly (production prior)      %+.5f\n', trav(2));
    fprintf('  + prior width  0.0157 -> 0.0708      %+.5f   (A1 -> A2)\n', trav(3) - trav(2));
    fprintf('  + law origin   1      -> 9/8         %+.5f   (A2 -> A3)\n', trav(4) - trav(3));
    fprintf('  + Jacobian fold J_b   -> J_b + J_ws  %+.5f   (A3 -> A4)\n', trav(5) - trav(4));
    fprintf('  = A4 amp                             %+.5f\n', trav(5));

    if opts.save
        out = fullfile(root, 'test_results', 'temp_formB_amp_stage4.mat');
        save(out, 'res', '-v7.3');
        fprintf('saved %s\n', out);
    end
end

% --------------------------------------------------------------------------
function s2 = local_slim(s, ax)
%LOCAL_SLIM  Keep only what the Stage-5 figures need (the full logs are ~50 MB
%   per run; 60 of them will not fit).
    s2 = struct('t', s.tout(:), ...
                'a_hat',  s.a_hat_out(:, ax),  'a_true', s.a_true_out(:, ax), ...
                'a_xm',   s.a_xm_out(:, ax),   'a_prime', s.a_prime_out(:, ax), ...
                'b_hat',  s.b_hat_out(:, ax),  'P_b',    s.P_b_out(:, ax), ...
                'ws_hat', s.ws_hat_out(:, ax), 'P_a',    s.P_a_out(:, ax), ...
                'h_bar_d', s.h_bar_d_out(:),   'h_bar_true', s.h_bar_true_out(:), ...
                'R', s.R);
end

function m = local_metrics(s, cfg, ax)
    t  = s.tout(:);
    e  = 100 * (s.a_hat_out(:, ax) - s.a_true_out(:, ax)) ./ s.a_true_out(:, ax);
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override;
    t3 = t2 + cfg.n_cycles / cfg.frequency;
    m.desc = max(abs(e(t > t1 & t <= t2)));
    m.osc  = sqrt(mean(e(t > t2 + 0.2 & t <= t3).^2));
    m.hold = mean(e(t > t3 + 0.3));
end

function p = local_amp_priors(cfg)
    [h, c] = local_truth_grid(cfg);
    b_eff = h .* (c - 1) ./ c;
    p = [max(abs(b_eff - 9/8)), max(abs((1 - (9/8) ./ h) - 1 ./ c))];
end

function p = local_len_priors(cfg)
    [h, c, dc] = local_truth_grid(cfg);
    b_eff = (c - 1) .* (h - 1);
    p_eff = -((h - 1) + 9/8) .* dc ./ (c .* (c - 1));
    a_anc = 1 - (1 + (h - 1) / (9/8)).^(-1);
    p = [max(abs(b_eff - 9/8)), max(abs(p_eff - 1)), max(abs(a_anc - 1 ./ c))];
end

function [h, c, dc] = local_truth_grid(cfg)
    pc = physical_constants();
    h  = linspace(cfg.h_bottom / pc.R - 0.1, cfg.h_init / pc.R + 1.0, 20001).';
    c  = zeros(size(h)); dc = zeros(size(h));
    for i = 1:numel(h)
        [~, c(i), dv] = calc_correction_functions(h(i), true);
        dc(i) = dv.dc_perp_dh;
    end
end

function cfg = local_canonical_cfg()
    pc = physical_constants();
    cfg = user_config();
    cfg.trajectory_type = 'osc';
    cfg.h_init    = 50;   cfg.h_bottom = 4.5;  cfg.amplitude = 2.5;
    cfg.frequency = 1;    cfg.n_cycles = 2;
    cfg.t_hold    = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;
    cfg.h_min     = 1.1 * pc.R;
    cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;   cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.h_bar_safe = 1.5;
end
