% STATUS: SCRATCH -- runner for hypotheses_formC_b.md C89 (c known arm, 5 Hz) (Layer-1 session, 2026-10-04/05; moved from the job tmp dir).
%   Writes test_results/temp_plaw_walls/*.mat; the analysis numbers in the ledger row are computed from those files.
% Diagnostic (10-04): same estimator blocks as the c-only f1p2 runs (a slot kept, ma2/nw_mcorr/pred_mean2/law_exact_step off,
% Pf_a_floor 0.002, P_0-free), but the slope is the TRUE a'_true read at the estimator height (ap_known, 'est') and slot 5 locked.
% c drops out entirely: whatever error remains is not from learning c.  ramp_tri, plane + cell, 10 seeds.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test';
addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(fullfile(root, 'test_script', 'scratch'));
B = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_cell_PI_sig010_rho15_rtri_f1.mat'), 'cc');
cc = B.cc;  cc.plaw_cp_slope = false;  cc.q55_adapt = 'off';  cc.lock_b = true;  cc.b_init = 8/9;  cc.Pf_b_std = 0;
for f = {'c_init', 'Pf_c_std', 'p_init', 'Pf_p_std', 'plaw_p_lim', 'plaw_seed_cross', 'q55_adapt_P0'}; if isfield(cc, f{1}); cc = rmfield(cc, f{1}); end; end
OV = struct('frequency', 5, 'n_cycles', 10);
pc = physical_constants();  Ts = pc.Ts;
for w = {'cell', 'plane'}
    o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'ramp_tri', 'verbose', false, ...
               'seeds', 1:10, 'log_P_full', false, 'ap_known', true, 'ap_known_at', 'est');
    if strcmp(w{1}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
    clear run_formC_b motion_control_law_formC_b;
    evalc('Rr = run_formC_b(o);');
    ns = numel(Rr.runs);  n = numel(Rr.runs{1}.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', Rr.runs{1}.h_bar_d_out(:), 'a_true', zeros(n, ns), 'a_hat', zeros(n, ns));
    for q = 1:ns
        S = Rr.runs{q};  hb = max(S.h_bar_true_out(:), 1.001);  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        if strcmp(w{1}, 'cell'); res.a_true(:, q) = 1 ./ cell_cperp_delta(hb);
        else; for i = 1:n; [~, cp] = calc_correction_functions(hb(i)); res.a_true(i, q) = 1 / cp; end; end
    end
    save(fullfile(root, 'test_results', 'temp_plaw_walls', sprintf('aptrue_%s_rtri_f5.mat', w{1})), '-struct', 'res');
    fprintf('saved %s\n', w{1});
end
disp('APTRUE F5 DONE');
