% STATUS: SCRATCH -- 2026-11-03 (user): PI with tau_I = 4 tau_P (critical damping of s^2 + s/tau_P + 1/(tau_P tau_I)), tau_P .06 s, tau_I .24 s, against tau_I .5 s. Current flow (scan right after the descent), rho 4 bead, a_d .07, 40 um/s, fixed 36 um raster, seeds 1-3.
%   Pre-registered: mean true error a_d - a_33 and the slow drift of a_33-hat around a_d shorter with .24; no loss of finished runs. Sanity: arm .5 repeats plot_1103_flow.m seed 1. Figure in Preview. Output fig_1103_tauI.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 3;  TI = [0.5 0.24];  AB = char(257);  MU = [char(956) 'm'];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 0], 'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 4, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'ti', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'T', 11 + 1.2 * 27 * 36 / 40 + 1);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  rs = cell(N, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  o.tau_i = TI(IA(q));  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'h', r.h(:), 'aN', r.aN(:), 'aN_tr', r.aN_tr(:));
end
f = figure('Color', 'w', 'Position', [40 40 1800 760], 'Visible', 'off');  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  a1 = nexttile(tl);  hold(a1, 'on');  a2 = nexttile(tl);  hold(a2, 'on');  cc = [0 0 0; 0.85 0 0];  hl = gobjects(2, 1);
for a = 1:2
    fin = 0;  M = [];
    for s = find(IA == a)'
        r = rs{s};  ok = find(isfinite(r.aN(1:r.kend)));  ke = ok(end);  te = r.t(ke);  if isfinite(r.te); te = min(te, r.te); end;  fn1 = isfinite(r.te) && ~r.contact && r.t(ke) >= r.te;  fin = fin + fn1;  m = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= te);
        M = [M; te, mean(ad - r.aN(m)), mean(ad - r.aN_tr(m)), mean(abs(ad - r.aN_tr(m))), mean(r.h(m)) * Rr, 1e3 * (min(r.h(m)) * Rr - Rr), std(movmean(ad - r.aN(m), 806))];
        if ID(s) == 1; k = (1:3:numel(r.t))';  k = k(k <= ke);  hl(a) = plot(a1, r.t(k), movmean(ad - r.aN(k), 81), 'Color', cc(a, :), 'LineWidth', 1.1);  plot(a2, r.t(k), movmean(ad - r.aN_tr(k), 81), 'Color', cc(a, :), 'LineWidth', 1.1); end
    end
    fprintf('tau_I %.2f s: finished %d/%d, ends %s | mean a_d - a33-hat %+.4f, mean a_d - a33 true %+.4f, mean |true| %.4f | gap mean %.3f um (target %.3f) | min gap - contact %s nm | slow wander of e (0.5 s mean) sd %.4f\n', TI(a), fin, NS, mat2str(round(M(:, 1)', 1)), mean(M(:, 2)), mean(M(:, 3)), mean(M(:, 4)), mean(M(:, 5)), hs * Rr, mat2str(round(M(:, 6)', 1)), mean(M(:, 7)));
end
for a = [a1 a2]; yline(a, 0, 'k:', 'HandleVisibility', 'off');  xlim(a, [10 37]);  ylim(a, [-0.03 0.05]);  box(a, 'on');  set(a, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2); end
ylabel(a1, [AB '_d - ' AB '_{33} estimate']);  ylabel(a2, [AB '_d - ' AB '_{33} true']);  xlabel(a2, 'Time (sec)');  legend(a1, hl, {'\tau_I = 0.5 s', '\tau_I = 0.24 s'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
fn = fullfile(fo, 'fig_1103_tauI.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
