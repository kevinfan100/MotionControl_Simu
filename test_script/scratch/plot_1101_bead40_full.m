% STATUS: SCRATCH -- 2026-11-01 (user): response of the estimates over a complete 40 um/s whole-bead scan (PI default), seeds 1, 2, one figure per seed: a_33, a_11, slopes a'_33, a'_11, theta_x, theta_y, kappa-hat, gap; from t_scan - 0.5 to the end of the run.
%   Sanity: scan ends 33.3, 33.2 s as run_1101_bead_pi40. Console: per segment (scan, after): mean est - true, rms angle error, a'/true median, fraction outside 0.1-1.9, a' < 0. Figures fig_1101_bead40_full_s{1,2}.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  AB = char(257);  hatc = char(770);  th = char(952);  ka = char(954);  pr = char(8242);  NS = 2;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 10, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'bf', ...
              'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
rs = cell(NS, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor id = 1:NS
    o = base;  o.seeds = id;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{id} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'aT', r.aT, 'aT_tr', r.aT_tr, 'apN', r.apN, 'apT', r.apT, 'kap', r.kap(:), 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
for id = 1:NS
    r = rs{id};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)));  [sN, sT] = local_slopes(r.h(ok));  wh = {ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= r.te), ok(r.t(ok) > r.te + 1)};  lb = {'scan', 'after'};
    fprintf('seed %d: contact %d, scan end %.1f s, run end %.1f s\n', id, r.contact, r.te, r.t(ok(end)));
    for w = 1:2
        g = wh{w};  [~, ig] = ismember(g, ok);  rN = r.apN(g) ./ sN(ig);  rT = r.apT(g) ./ sT(ig);  ex = r.thx(g) - r.th_tr(g, 1);  ey = r.thy(g) - r.th_tr(g, 2);
        fprintf('   %-5s est - true a33 %+.4f, a11 %+.4f | a''33/true %.2f (outside %.2f, <0 %.2f) | a''11/true %.2f (outside %.2f, <0 %.2f) | angle err rms x %.1f, y %.1f deg, mean x %+.1f, y %+.1f | gap %.2f-%.2f um\n', lb{w}, mean(r.aN(g) - r.aN_tr(g)), mean(r.aT(g) - r.aT_tr(g)), median(rN), mean(rN < 0.1 | rN > 1.9), mean(rN < 0), median(rT), mean(rT < 0.1 | rT > 1.9), mean(rT < 0), sqrt(mean(ex.^2)), sqrt(mean(ey.^2)), mean(ex), mean(ey), min(r.h(g)) * Rr, max(r.h(g)) * Rr);
    end
    k = ok(r.t(ok) >= r.tsc - 0.5);  k = k(1:4:end);  t = r.t(k);  [~, ik] = ismember(k, ok);  sNk = sN(ik);  sTk = sT(ik);
    f = figure('Color', 'w', 'Position', [20 20 1950 1250]);  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
    a1 = nexttile(tl);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 0.8);  h3 = yline(ad, 'k--', 'LineWidth', 1.4);  ylim([0 0.3]);  ylabel([AB '_{33}']);
    legend([h1 h2 h3], {'true', sprintf('estimate, seed %d', id), [AB '_d = 0.1']}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl);  hold on;  box on;  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.aT(k), 'Color', cE, 'LineWidth', 0.8);  ylim([0.25 0.65]);  ylabel([AB '_{11}']);
    a3 = nexttile(tl);  hold on;  box on;  plot(t, sNk, 'Color', cT, 'LineWidth', 1.3);  plot(t, r.apN(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-3 4]);  ylabel([AB pr '_{33}']);
    a4 = nexttile(tl);  hold on;  box on;  plot(t, sTk, 'Color', cT, 'LineWidth', 1.3);  plot(t, r.apT(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-1 2]);  ylabel([AB pr '_{11}']);  xlabel('Time (sec)');
    a5 = nexttile(tl);  hold on;  box on;  plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.thy(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-90 90]);  ylabel([th '_y  [deg]']);
    a6 = nexttile(tl);  hold on;  box on;  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.thx(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-90 90]);  ylabel([th '_x  [deg]']);
    a7 = nexttile(tl);  hold on;  box on;  plot(t, r.kap(k), 'Color', cE, 'LineWidth', 0.8);  yline(0, 'k:');  ylim([-1.2 1.2]);  ylabel([ka hatc '  [1/R]']);
    a8 = nexttile(tl);  hold on;  box on;  plot(t, r.h(k) * Rr, 'Color', cT, 'LineWidth', 1.3);  yline(hs * Rr, 'k--', 'LineWidth', 1.2);  yline(Rr, 'k-', 'LineWidth', 1.2);  ylim([2 3.5]);  ylabel('gap  [\mum]');  xlabel('Time (sec)');
    for a = [a1 a2 a3 a4 a5 a6 a7 a8]; xlim(a, [t(1) t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off'); end
    fn = fullfile(fo, sprintf('fig_1101_bead40_full_s%d.png', id));  exportgraphics(f, fn, 'Resolution', 130);  close(f);
end

function [sN, sT] = local_slopes(hb)
    sN = arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
    sT = arrayfun(@(h) (1 / local_cq(max(h, 1.0003) + 1e-4) - 1 / local_cq(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
function c = local_cq(h)
    c = calc_correction_functions(h);
end
