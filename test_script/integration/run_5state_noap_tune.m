% PURPOSE (2026-09-21, user: "keep testing what to adjust; should P init change?"): tuning round of the layer-1.5
%   estimator on the house 10 s ramp (15 -> 2.5 um, cheat init), 5 seeds, arms:
%     ljung        Ljung schedule, far-field slope, clamp 0.998, sqrtP44[0] = 0.5 a_nom          (yesterday's baseline)
%     ljung_p05    same, sqrtP44[0] = 0.05 a_nom (init trusted: the cheat init IS the truth)
%     ljung_c995   same as ljung, clamp lf_min = 0.995 (T* >= 0.125 s)
%     ljung_exact  same as ljung, exact plane log-slope |K_h_perp| (layer-2 knowledge)
%     q55_p05      rate-change budget Q55 = (r a / T*)^2 instead of forgetting, sqrtP44[0] = 0.05
%   Reported: relative error |e|/a on the ramp (rms, max), near/far band rms, start transient max|e| in 0-1.2 s,
%   hold-end relative bias / sd / sqrtP44 / honesty, y1 honesty. Figure: plot_5state_noap_tune.m.
%  STATUS: ACTIVE (moved from scratch 2026-09-22; 0920_5state_noaprime.tex depends on it)
function out = run_5state_noap_tune(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds'); opts.seeds = 1:5; end
    if ~isfield(opts, 'Td');    opts.Td    = 10; end
    if ~isfield(opts, 'arms');  opts.arms  = {'ljung', 'ljung_p05', 'ljung_c995', 'ljung_exact', 'q55_p05'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_5state_noap');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;
    out = struct();
    for ia = 1:numel(opts.arms)
        arm = opts.arms{ia};
        cc = struct('seed_mode', 'true_h0', 'lf_mode', 'ljung', 'lf_min', 0.998);
        switch arm
            case 'ljung'
            case 'ljung_p05';   cc.Pf_a_std_rel = 0.05;
            case 'ljung_c995';  cc.lf_min = 0.995;
            case 'ljung_exact'; cc.lf_slope = 'exact';
            case 'q55_p05';     cc.lf_mode = 'q55';  cc.Pf_a_std_rel = 0.05;
            case 'q55_v2';      cc.lf_mode = 'q55';  cc.Pf_a_std_rel = 0.05;   % corrected budget (r a)^2 / T*
            otherwise; error('unknown arm %s', arm);
        end
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
        cfg.h_init = 15;  cfg.h_bottom = 2.5;  cfg.amplitude = 0;  cfg.frequency = 1;  cfg.n_cycles = 1;
        cfg.t_hold = 0.5;  cfg.t_descend_override = opts.Td;  cfg.T_sim = 0.5 + opts.Td + 2.0;  cfg.h_min = 2.475;
        nS = numel(opts.seeds);  res = struct();
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
        res.arm = arm;  res.seg = [0.5, 0.5 + opts.Td, 0.5 + opts.Td];  res.seeds = opts.seeds;  res.a_nom = a_nom;  res.cc = cc;
        local_report(res);
        save(fullfile(od, sprintf('tune_Td%d_%s.mat', opts.Td, arm)), '-struct', 'res');  out.(arm) = res;
    end
end

function local_report(r)
    t = r.t;  a = mean(r.a_true, 2);  e = mean(r.a_true - r.a_hat, 2);  rel = e ./ a;
    ramp = t >= 1.5 & t < r.seg(2);  far = ramp & r.hd > 3;  near = ramp & r.hd <= 3;  he = t >= t(end) - 1;  st = t <= 1.2;
    [mx, im] = max(abs(rel(ramp)));  tr = t(ramp);
    ea = r.a_true - r.a_hat;
    fprintf('%-12s | rel |e|/a ramp rms %.3f max %.3f (%.1f s) | far %.3f near %.3f | start max|e| %.3f | hold-end rel bias %+.3f  rel sd %.3f  rel sqrtP44 %.3f (hon %.2f) | y1 hon %.2f | trk %.1f nm\n', ...
            r.arm, rms(rel(ramp)), mx, tr(im), rms(rel(far)), rms(rel(near)), max(abs(e(st))), mean(rel(he)), ...
            mean(std(ea(he, :), 0, 2)) / mean(a(he)), mean(sqrt(r.P_a(he, :)), 'all') / mean(a(he)), ...
            mean(std(ea(he, :), 0, 2)) / mean(sqrt(r.P_a(he, :)), 'all'), var(r.innov1(ramp, :), 0, 'all') / mean(r.S_y1(ramp, :), 'all'), rms(r.trk(ramp, :), 'all'));
end
