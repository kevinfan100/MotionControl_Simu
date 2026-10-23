% STATUS: SCRATCH -- 2026-10-08, history review (user: produce the old estimators' results). The OLD controllers as they are in
%   the repo, each with its own defaults, on the Fig. 4.3 trajectory (22.2 -> 1.10 R in 1 s, triangle 1.10 <-> 10 R at 2 Hz),
%   plane wall, lambda_c 0.7 (the value of that time). Variants of run_pure_simulation: '' (default eq17 = 7-state, Meng-like),
%   '6state' (Vpersonal), '5state', '4state'; plus the current noap with only y2 whitening on: 'noap1w' (first order, eps 1e-5),
%   'noap2w' (second order, lambda_f 0.98, Q44 = 0). z axis from ekf_out (column 2 = a_hat_z, checked = diag.a_hat for noap);
%   tracking error = TRUE error z_d - z [um] (same definition for every controller; the measured one could not be aligned
%   from p_d_out / p_m_out for all variants). Alignment p_d(k) vs p_true(k-1) = minimum-rms lag on the noap run (10-08).
%   Cached in test_results/temp_5state_noap/hist_<variant>_s<seed>.mat.
function R = run_history_old(variants, seeds)
    if nargin < 1 || isempty(variants); variants = {'7state', '6state', '5state', '4state'}; end
    if nargin < 2 || isempty(seeds); seeds = 1:3; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    R = struct();
    for iv = 1:numel(variants)
        for s = seeds
            fn = fullfile(id, sprintf('hist_%s_s%d.mat', variants{iv}, s));
            if exist(fn, 'file'); r = load(fn);
            else
                cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 2);  cfg.amplitude = 0.5 * (10 - 1.10) * pc.R;  cfg.lambda_c = 0.7;
                o = struct('seed', s, 'collect_diag', false);
                if startsWith(variants{iv}, 'noap')
                    cfg.eq17_variant = '5state_noap';
                    cc = struct('y2_whiten', true, 'y2_mirror', false, 'ma2_aug', false, 'nw_mcorr', false, 'corr_input', false, ...
                                'lf_mode', 'const', 'seed_mode', 'true_h0');
                    if strcmp(variants{iv}, 'noap1w'); cc.gain_order = 1;  cc.lambda_f = 1;  cc.q44_r2_frac = 1e-5;
                    else;                              cc.gain_order = 2;  cc.lambda_f = 0.98; end
                    o.ctrl_const_override = cc;
                elseif ~strcmp(variants{iv}, '7state'); cfg.eq17_variant = variants{iv}; end
                try
                    S = run_pure_simulation(cfg, o);
                    pd = S.p_d_out(:, 3);
                    r = struct('t', S.tout(:), 'a_true', S.a_true_out(:, 3) / a_nom, 'a_hat', S.ekf_out(:, 2) / a_nom, ...
                               'dxm', [nan; pd(2:end) - S.p_true_out(1:end-1, 3)], 'h', S.p_true_out(:, 3) / pc.R, 'h_d', pd / pc.R, 'ok', true, 'err', '');
                catch ME
                    r = struct('ok', false, 'err', ME.message);
                end
                save(fn, '-struct', 'r');
            end
            R.(['v' variants{iv}])(s) = r;
            if r.ok
                e = r.a_true - r.a_hat;  t = r.t;  tb = [1 1.5 2 2.5];  eb = arrayfun(@(x) mean(e(abs(t - x) < 0.01)), tb);
                fprintf('%-7s s%d | troughs e_a %+.3f | tri mean|e_a| %.3f | dz (true) rms tri %.4f um | min h %.3f | a_hat z at 0.1 s %.3f (true %.3f)\n', ...
                        variants{iv}, s, mean(eb), mean(abs(e(t > 1))), sqrt(mean(r.dxm(t > 1).^2, 'omitnan')), min(r.h), ...
                        interp1(t, r.a_hat, 0.1), interp1(t, r.a_true, 0.1));
            else
                fprintf('%-7s s%d | FAILED: %s\n', variants{iv}, s, r.err);
            end
        end
    end
end
