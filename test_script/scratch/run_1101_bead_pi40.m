% STATUS: SCRATCH -- 2026-11-01 (user): complete 40 um/s whole-bead scans with PI regulation (reg_pi, 'cross', tau_i 0.5), seeds 1-3, figures in the Meng style of plot_1030_bead_meng.m for the finished seeds 1, 2.
%   Also mean a-hat - a_33 before the scan, during it and after it, at 40 um/s (PI) and 10 um/s (temp_fillet_opts_kq_v2.mat). Sanity: seeds 1, 2 finish, seed 3 contact at 17.7 s, as run_1101_bead_reg.
%   Outputs temp_fillet_opts_pi40.mat, fig_1101_bead_meng_{3d,time}_pi40_s{1,2}.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  NS = 3;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 10, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'bp', ...
              'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'reg_pi', true, 'reg_pi_cond', 'cross', 'tau_i', 0.5);
rs = cell(1, NS);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor id = 1:NS
    o = base;  o.seeds = id;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{id} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr, 'tsc', r.tsc, 'rs_tend', r.rs_tend);
end
save(fullfile(fo, 'temp_fillet_opts_pi40.mat'), 'rs', 'Rr', 'ad', 'hs', '-v7.3');
S10 = load(fullfile(fo, 'temp_fillet_opts_kq_v2.mat'));
for j = 1:2
    if j == 1; Q = rs; lb = '40 um/s PI'; else; Q = S10.rs(1, :); lb = '10 um/s P'; end
    for id = 1:NS
        r = Q{id};  ok = find(isfinite(r.aN(1:r.kend)));  te = r.rs_tend;  if isnan(te); te = r.t(ok(end)); end
        gb = ok(r.t(ok) >= r.tsc - 0.5 & r.t(ok) < r.tsc);  gs = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= te);  ga = ok(r.t(ok) > te + 1);
        d = r.aN - r.aN_tr;  m = NaN;  if ~isempty(ga); m = mean(d(ga)); end
        fprintf('%-11s seed %d: scan end %.1f s | est - true before %+.4f, during %+.4f, after (1 s later) %+.4f | true mean during %.3f, est mean %.3f | rms est - true during %.4f\n', lb, id, te, mean(d(gb)), mean(d(gs)), m, mean(r.aN_tr(gs)), mean(r.aN(gs)), std(d(gs)));
    end
end
for sd = 1:2
    seed_in = sd;  mat_in = '_pi40';  run(fullfile(here, 'plot_1030_bead_meng.m'));
    for nm = ["3d", "time"]; movefile(fullfile(fo, "fig_1030_bead_meng_" + nm + "_pi40.png"), fullfile(fo, sprintf('fig_1101_bead_meng_%s_pi40_s%d.png', nm, sd))); end
end
system(['open ' fullfile(fo, 'fig_1101_bead_meng_3d_pi40_s1.png') ' ' fullfile(fo, 'fig_1101_bead_meng_time_pi40_s1.png')]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
