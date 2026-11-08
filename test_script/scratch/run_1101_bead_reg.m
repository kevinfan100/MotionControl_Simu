% STATUS: SCRATCH -- 2026-11-01 (user): why the 40 um/s whole-bead raster hits the wall: P regulation offset or normal error. Arms (seeds 1-3, settings of plot_1101_bead40_est.m): base | PI (reg_pi, 'cross', tau_i 0.5) | tau_a 0.25 | true normal for regulation and raster (cheat_w).
%   Pre-registered: offset hypothesis if PI has a_d - a-hat <= 0.01 and more lines; normal hypothesis if the true-normal arm finishes or gains clearly more lines. Sanity: base end times 27.1 / 15.9 / 24.2 s.
%   Console per seed: end, contact, lines, a_d - a-hat, a-hat - a, gap range, whole scan and last 2 s. Figure fig_1101_bead_reg.png (seed 1).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  NS = 3;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 10, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'br', ...
              'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
arm = {'base (P)', 'PI, tau_i 0.5', 'P, tau_a 0.25', 'true normal'};  NA = numel(arm);  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    switch IA(q)
        case 2; o.reg_pi = true;  o.reg_pi_cond = 'cross';  o.tau_i = 0.5;
        case 3; o.tau_a = 0.25;
        case 4; o.cheat_w = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'rs_tend', r.rs_tend);
end
for ia = 1:NA
    for id = 1:NS
        r = rs{ia, id};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)));  ke = ok(end);  g1 = ok(r.t(ok) >= r.tsc + 0.5);  g2 = ok(r.t(ok) >= r.t(ke) - 2);
        fprintf('%-14s seed %d: end %.1f / %.1f s, contact %d, last y %.1f (~%d of %d lines) | scan: a_d-est %+.3f, est-true %+.3f | last 2 s: %+.3f, %+.3f | gap %.2f-%.2f um | mean|a33-a_d| %.3f\n', arm{ia}, id, r.t(ke), r.t(end), r.contact, r.p(ke, 2), max(0, round(r.p(ke, 2) + Xe) + 1), nl, ...
                mean(ad - r.aN(g1)), mean(r.aN(g1) - r.aN_tr(g1)), mean(ad - r.aN(g2)), mean(r.aN(g2) - r.aN_tr(g2)), min(r.h(g1)) * Rr, max(r.h(g1)) * Rr, mean(abs(r.aN_tr(g1) - ad)));
    end
end
f = figure('Color', 'w', 'Position', [20 20 620 * NA 620]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 1};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)));  k = ok(r.t(ok) >= r.tsc - 0.5);  k = k(1:10:end);  t = r.t(k) - r.tsc;
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.1, 'DisplayName', 'true');  plot(t, r.aN(k), 'Color', cE, 'LineWidth', 0.8, 'DisplayName', ['estimate, ' arm{ia}]);  yline(ad, 'k--', 'HandleVisibility', 'off');  ylim([0 0.3]);  ylabel('a_{33} / a_{nom}');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, r.h(k) * Rr, 'Color', cT, 'LineWidth', 1.1);  yline(hs * Rr, 'k--');  yline(Rr, 'k-');  ylim([2 4]);  ylabel('center gap [\mum]');  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2]; xlim(a, [t(1) t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_bead_reg.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
