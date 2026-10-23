% STATUS: SCRATCH -- 2026-10-16 (user): gain regulation with a' difference vs a' state, lowest a_d per wall (plane 0.07, cell 0.50;
%   both h* = 1.078 R), parked at 22.2 R, tau_a 0.1 s (lambda_a = 1 - Ts/0.1), 6 s, lambda_c 0.4, plane init, seeds 1:5.
%   Law z: Delta h_d = (1 - lambda_a)(a_d - a_hat)/s, s = law slope with the SAME gate in both drivers: s takes the estimator's a_hat'
%   only while the low-passed commanded step |den| > THR (den = (1-B) den + B Delta h_d[k-1], B = 2/51), floored at AP_MIN; step capped at VMAX = the 1 s descent speed (22.2 - 1.10) Ts [R/step].
%     diff  : run_pure_simulation, 5state_noap production (eps 1e-5) + ap_ff 'lpf' (beta B, ap_ff_thr = THR in um/step)
%     state : run_formC_b via run_mc_s9_gainreg, ap_slot (prior 0.002, sqrt P_fix 0.4 x (1.1/h_d)^2, fe_cmd_eval) = fig_mc_aps_regulation
%     ws    : 10-19 (user: a' state vs w_s state) same driver, Section 8 estimator (w_s-hat[0] 0, prior 0.3, q 0, p 2) + fe_cmd_eval,
%             same law, gate and step cap; a-hat' logged = the slope the law uses, 1/(h-hat - w_s-hat)^2
%     ws9   : 10-19 same estimator, Section 9 law as written (no gate, no step cap)
%   Per seed: contact (run aborted) or not. Output test_results/temp_1009_ws/apreg_<arm>_<wall>.mat (seed columns, NaN after contact).
function run_mc_apreg(arms, walls, seeds, ad_cell)
    if nargin < 1 || isempty(arms); arms = {'diff', 'state'}; end
    if nargin < 2 || isempty(walls); walls = {'plane', 'cell'}; end
    if nargin < 3 || isempty(seeds); seeds = 1:5; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    T = 6;  TAU = 0.1;  LAM = 1 - pc.Ts / TAU;  B = 2 / 51;  THR = 0.0066;  AP_MIN = 1 / 22.2^2;  VMAX = (22.2 - 1.10) * pc.Ts / 1.0;  AD = struct('plane', 0.07, 'cell', 0.50);  if nargin >= 4 && ~isempty(ad_cell); AD.cell = ad_cell; end
    n = round(T / pc.Ts);
    for ia = 1:numel(arms)
        for iw = 1:numel(walls)
            w = walls{iw};  ad = AD.(w);
            res = struct('arm', arms{ia}, 'wall', w, 'a_d', ad, 'seeds', seeds, 't', (0:n-1)' * pc.Ts, 'contact', false(1, numel(seeds)), ...
                         'B', B, 'THR', THR, 'AP_MIN', AP_MIN, 'LAM', LAM, 'VMAX', VMAX);
            [res.wd, res.h_true, res.a_true, res.a_hat, res.ap_hat, res.dxm] = deal(NaN(n, numel(seeds)));
            for iq = 1:numel(seeds)
                s = seeds(iq);
                try
                    if strcmp(arms{ia}, 'diff'); r = local_diff(w, ad, s, pc, a_nom, T, LAM, B, THR, AP_MIN, VMAX);
                    else; r = local_state(w, ad, s, od, LAM, B, THR, AP_MIN, T, VMAX, arms{ia}); end
                    m = min(n, numel(r.t));  fn = {'wd', 'h_true', 'a_true', 'a_hat', 'ap_hat', 'dxm'};
                    for q = 1:numel(fn); res.(fn{q})(1:m, iq) = r.(fn{q})(1:m); end
                catch ME
                    fprintf('  [%s]\n', ME.message);
                    res.contact(iq) = true;
                end
                st = 'ok'; if res.contact(iq); st = 'CONTACT'; end; fprintf('%-5s %-5s a_d %.2f seed %d: %s\n', arms{ia}, w, ad, s, st);
            end
            tg = '';  if strcmp(w, 'cell') && ad ~= 0.50; tg = sprintf('_ad%03d', round(100 * ad)); end
            save(fullfile(od, sprintf('apreg_%s_%s%s.mat', arms{ia}, w, tg)), '-struct', 'res');
        end
    end
end

function r = local_diff(w, ad, s, pc, a_nom, T, LAM, B, THR, AP_MIN, VMAX)
    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
    ov = struct('trajectory_type', 'positioning', 'h_init', 50, 'h_bottom', 50, 'amplitude', 0, 't_hold', 0, ...
                't_descend_override', 0, 'T_sim', T, 'lambda_c', 0.4);
    fo = fieldnames(ov);  for q = 1:numel(fo); cfg.(fo{q}) = ov.(fo{q}); end
    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', 1e-5, ...
                'ap_ff', 'lpf', 'ap_ff_beta', B, 'ap_ff_thr', THR * pc.R);
    o = struct('seed', s, 'collect_diag', true, 'ctrl_const_override', cc, ...
               'gain_reg', struct('a_d', ad, 'lambda_a', LAM, 'beta', B, 'thr', THR, 'ap_min', AP_MIN, 'v_max', VMAX));
    if strcmp(w, 'cell'); o.plant_cperp = @cell_cperp_delta; end
    clear motion_control_law_eq17_5state_noap;
    S = run_pure_simulation(cfg, o);
    r = struct('t', S.tout(:), 'wd', S.p_d_out(:, 3) / pc.R, 'h_true', S.p_true_out(:, 3) / pc.R, 'a_true', S.a_true_out(:, 3) / a_nom, ...
               'a_hat', S.diag.a_hat(:, 3) / a_nom, 'ap_hat', S.diag.a_prime_hat(:, 3) / a_nom * pc.R, 'dxm', S.diag.delta_x_m(:, 3));
end

function r = local_state(w, ad, s, od, LAM, B, THR, AP_MIN, T, VMAX, arm)
    AP = struct('ap_slot', true, 'fe_cmd_eval', true, 'delta_init', 1 / 22.2^2, 'Pf_delta_std', 0.002, 'plaw_delta_lim', [-1 10], ...
                'plaw_seed_cross', false, 'q77_adapt_P0', 0.4, 'q77_hscale', 1.1);
    if startsWith(arm, 'ws'); AP = struct('fe_cmd_eval', true); end   % 10-19: Section 8 defaults of run_mc_s9_gainreg + fe_cmd_eval
    GX = struct('beta', B, 'thr', THR, 'ap_min', AP_MIN, 'v_max', VMAX);
    if strcmp(arm, 'ws9'); GX = struct(); end   % 10-19: 'ws9' = Section 9 law as written (no slope gate, no step cap)
    tg = sprintf('_apreg_s%d', s);
    evalc('run_mc_s9_gainreg({w}, s, ad, 0.1, T, 0, AP, tg, [], 0, GX);');
    S = load(fullfile(od, sprintf('gainreg_%s_tau100_T%g%s.mat', w, T, tg)));  delete(fullfile(od, sprintf('gainreg_%s_tau100_T%g%s.mat', w, T, tg)));
    assert(abs(1 - S.lambda_a - (1 - LAM)) < 1e-12);
    aph = -S.w_hat;  if startsWith(arm, 'ws'); pc = physical_constants();  aph = S.ap_hat / (pc.Ts / pc.gamma_N); end   % ap_slot: slot 7 holds a'; ws: logged law slope a_prime_out [physical] / a_nom
    r = struct('t', S.t, 'wd', S.wd, 'h_true', S.h_true, 'a_true', S.a_true, 'a_hat', S.a_hat, 'ap_hat', aph, 'dxm', S.dxm);
end
