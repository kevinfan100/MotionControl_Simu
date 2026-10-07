% PURPOSE (2026-09-21, user: "how do we shrink the descent lag on the slow ramp? add lambda_f, test one or two slower cases"):
%   the house slow ramp (15 um -> 2.5 um cosine descent, hold 0.5 s before, 2 s after; run_delta_two_walls 'meng') at
%   T_d in {10, 20, 40} s, layer-1 estimator (5state_noap, cheat init a_hat[0] = a(h_bar_0)) with the gain-block forgetting
%   factor lambda_f in {1 (pure layer 1), 0.9995, 0.998}  (memory T* = Ts/(1-lambda_f) = inf / 1.25 s / 0.31 s).
%   Reported per cell: lag peak e_a (a_true - a_hat, house sign) and its time, best time shift tau, hold-end mean, honesty.
%   Saves test_results/temp_5state_noap/slow_Td<T>_lf<lf>.mat; figures by plot_5state_noap_slow.m.
%  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function out = run_5state_noap_slow(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds   = 1:5; end
    if ~isfield(opts, 'T_d');     opts.T_d     = [10, 20, 40]; end
    if ~isfield(opts, 'lf');      opts.lf      = [1, 0.9995, 0.998]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  Ts = pc.Ts;
    T_HOLD0 = 0.5;  T_AFTER = 2.0;
    out = struct();
    for it = 1:numel(opts.T_d)
        Td = opts.T_d(it);
        for il = 1:numel(opts.lf)
            lf = opts.lf(il);
            cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
            cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
            cfg.t_hold = T_HOLD0;  cfg.t_descend_override = Td;  cfg.T_sim = T_HOLD0 + Td + T_AFTER;  cfg.h_min = 2.475;
            cc = struct('seed_mode', 'true_h0', 'lambda_f', lf);
            tag = sprintf('slow_Td%d_lf%s', Td, strrep(sprintf('%.4f', lf), '.', 'p'));
            nS = numel(opts.seeds);  res = struct();
            fprintf('\n=== %s | ramp %.1f -> %.2f R over %d s (peak %.2f R/s) | lambda_f %.4f (T* %s) | %d seeds ===\n', tag, ...
                    15 / R, 2.5 / R, Td, pi / 2 * (15 - 2.5) / R / Td, lf, local_tstar(lf, Ts), nS);
            for q = 1:nS
                tic; s = run_pure_simulation(cfg, struct('seed', opts.seeds(q), 'collect_diag', true, 'ctrl_const_override', cc)); tr = toc;
                if q == 1
                    N = numel(s.tout);  res.t = s.tout(:);  res.hd = s.p_d_out(:, 3) / R;  z = zeros(N, nS);
                    res.hb = z; res.a_true = z; res.a_hat = z; res.da_hat = z; res.da_true = z; res.P_a = z; res.P_da = z;
                    res.innov1 = z; res.S_y1 = z; res.trk = z;
                end
                res.hb(:, q) = s.p_true_out(:, 3) / R;
                res.a_true(:, q) = s.a_true_out(:, 3) / a_nom;  res.a_hat(:, q) = s.diag.a_hat(:, 3) / a_nom;
                res.da_hat(:, q) = s.diag.delta_a_hat(:, 3) / a_nom;  res.da_true(:, q) = [diff(s.a_true_out(:, 3)); 0] / a_nom;
                res.P_a(:, q) = s.diag.P_a(:, 3) / a_nom^2;  res.P_da(:, q) = s.diag.P_da(:, 3) / a_nom^2;
                res.innov1(:, q) = s.diag.innovation_y1(:, 3);  res.S_y1(:, q) = s.diag.S_y1(:, 3);
                res.trk(:, q) = (s.p_d_out(:, 3) - s.p_true_out(:, 3)) * 1e3;
                fprintf('   seed %2d | %.0f s | a_hat(end) %.4f true %.4f\n', opts.seeds(q), tr, res.a_hat(end, q), res.a_true(end, q));
            end
            res.tag = tag;  res.Td = Td;  res.lf = lf;  res.seg = [T_HOLD0, T_HOLD0 + Td, T_HOLD0 + Td];  res.seeds = opts.seeds;  res.a_nom = a_nom;
            local_report(res, Ts);
            save(fullfile(od, [tag '.mat']), '-struct', 'res');  out.(tag) = res;
        end
    end
end

function s = local_tstar(lf, Ts)
    if lf >= 1; s = 'inf'; else; s = sprintf('%.2f s', Ts / (1 - lf)); end
end

function local_report(res, Ts)
    t = res.t;  e = res.a_true - res.a_hat;  em = mean(e, 2);          % house sign: a_true - a_hat
    ramp = t >= res.seg(1) & t < res.seg(2);  hold_end = t >= t(end) - 1;
    [emin, imin] = min(em .* ramp - 10 * ~ramp);                       % lag error is NEGATIVE in house sign (a_hat above the falling truth)
    rate_sm = movmean(mean(res.da_true, 2), 401);  [rmin, irate] = min(rate_sm);
    at = mean(res.a_true, 2);  ah = mean(res.a_hat, 2);  best = inf;  tau_best = 0;
    for tau = 0:0.02:4
        k = round(tau / Ts);  idx = find(t >= res.seg(1) + 2 & t <= res.seg(2));  idx = idx(idx - k >= 1);
        r = mean((ah(idx) - at(idx - k)).^2);  if r < best; best = r; tau_best = tau; end
    end
    hon_P = mean(std(e(hold_end, :), 0, 2)) / mean(sqrt(res.P_a(hold_end, :)), 'all');
    hon_y1 = var(res.innov1(ramp, :), 0, 'all') / mean(res.S_y1(ramp, :), 'all');
    fprintf('   lag peak e_a %+.3f at t = %.2f s (fastest fall of a at %.2f s, %.2e/step; sqrtP55 there %.2e) | tau %.2f s | hold-end mean %+.4f sd %.4f sqrtP44 %.4f (honesty %.2f) | y1 honesty ramp %.2f | trk rms ramp %.1f nm\n', ...
            emin, t(imin), t(irate), rmin, mean(sqrt(res.P_da(irate, :))), tau_best, mean(e(hold_end, :), 'all'), mean(std(e(hold_end, :), 0, 2)), ...
            mean(sqrt(res.P_a(hold_end, :)), 'all'), hon_P, hon_y1, rms(res.trk(ramp, :), 'all'));
end
