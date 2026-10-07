% PURPOSE (2026-09-21, user: "simple adaptive lambda_f from speed and height, simple comparison"): the house 10 s ramp
%   (15 -> 2.5 um, 12.5 s) with three arms of the layer-1 estimator: lambda_f = 1 (pure), constant 0.9995, and the
%   Ljung schedule (lf_mode 'ljung': T* = (2 J r^2)^(-1/3), r = |h_dot_d| / (h_d (h_d - 1)) from the command,
%   J = the filter's own relative Fisher rate; clamp lambda_f >= 0.998). 5 seeds. Console: lag peak per height band,
%   hold-end mean / sd / sqrtP44, y1 honesty; figure by plot_5state_noap_lfadapt.m (4 rows incl. lambda_f(t)).
%  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function out = run_5state_noap_lfadapt(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds'); opts.seeds = 1:5; end
    if ~isfield(opts, 'Td');    opts.Td    = 10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  Ts = pc.Ts;
    arms = {'lf1',     struct('lambda_f', 1); ...
            'lf9995',  struct('lambda_f', 0.9995); ...
            'ljung',   struct('lf_mode', 'ljung', 'lf_min', 0.998)};
    out = struct();
    for ia = 1:size(arms, 1)
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
        cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
        cfg.t_hold = 0.5;  cfg.t_descend_override = opts.Td;  cfg.T_sim = 0.5 + opts.Td + 2.0;  cfg.h_min = 2.475;
        cc = arms{ia, 2};  cc.seed_mode = 'true_h0';
        nS = numel(opts.seeds);  res = struct();
        fprintf('\n=== lfadapt %s | Td %d s | %d seeds ===\n', arms{ia, 1}, opts.Td, nS);
        for q = 1:nS
            s = run_pure_simulation(cfg, struct('seed', opts.seeds(q), 'collect_diag', true, 'ctrl_const_override', cc));
            if q == 1
                N = numel(s.tout);  res.t = s.tout(:);  res.hd = s.p_d_out(:, 3) / R;  z = zeros(N, nS);
                res.a_true = z; res.a_hat = z; res.da_hat = z; res.da_true = z; res.P_a = z; res.P_da = z; res.innov1 = z; res.S_y1 = z; res.lf = z; res.trk = z;
            end
            res.a_true(:, q) = s.a_true_out(:, 3) / a_nom;  res.a_hat(:, q) = s.diag.a_hat(:, 3) / a_nom;
            res.da_hat(:, q) = s.diag.delta_a_hat(:, 3) / a_nom;  res.da_true(:, q) = [diff(s.a_true_out(:, 3)); 0] / a_nom;
            res.P_a(:, q) = s.diag.P_a(:, 3) / a_nom^2;  res.P_da(:, q) = s.diag.P_da(:, 3) / a_nom^2;
            res.innov1(:, q) = s.diag.innovation_y1(:, 3);  res.S_y1(:, q) = s.diag.S_y1(:, 3);
            res.lf(:, q) = s.diag.lambda_f_used(:, 3);  res.trk(:, q) = (s.p_d_out(:, 3) - s.p_true_out(:, 3)) * 1e3;
        end
        res.arm = arms{ia, 1};  res.seg = [0.5, 0.5 + opts.Td, 0.5 + opts.Td];  res.seeds = opts.seeds;  res.a_nom = a_nom;
        t = res.t;  e = res.a_true - res.a_hat;  em = mean(e, 2);
        far = t >= res.seg(1) + 1 & t < res.seg(2) & res.hd > 3;  near = t >= res.seg(1) & t < res.seg(2) & res.hd <= 3;  he = t >= t(end) - 1;
        Tstar = Ts ./ (1 - mean(res.lf, 2));
        fprintf('   lag: far band min e_a %+.3f (sd %.3f) | near band min e_a %+.3f (sd %.3f) | ramp-end e_a %+.3f | hold-end mean %+.4f sd %.4f sqrtP44 %.4f (%.2f) | y1 hon ramp %.2f | T* far %.2f s near %.2f s hold %s | trk rms %.1f nm\n', ...
                min(em(far)), mean(std(e(far, :), 0, 2)), min(em(near)), mean(std(e(near, :), 0, 2)), em(find(t >= res.seg(2), 1)), ...
                mean(e(he, :), 'all'), mean(std(e(he, :), 0, 2)), mean(sqrt(res.P_a(he, :)), 'all'), mean(std(e(he, :), 0, 2)) / mean(sqrt(res.P_a(he, :)), 'all'), ...
                var(res.innov1(far | near, :), 0, 'all') / mean(res.S_y1(far | near, :), 'all'), median(Tstar(far)), median(Tstar(near)), mat2str(median(Tstar(he)), 3), rms(res.trk(far | near, :), 'all'));
        save(fullfile(od, sprintf('lfadapt_Td%d_%s.mat', opts.Td, arms{ia, 1})), '-struct', 'res');  out.(arms{ia, 1}) = res;
    end
end
