% STATUS: SCRATCH -- 2026-10-28 (user): [variant plot_1028_hemi_xscan_apq: slope Q switched off from t_scan (opts.ap_q_off_from), rest as
%   plot_1028_hemi_xscan] realistic start on the void hemisphere (radius 15 um): the probe starts 15 um above the sphere at polar
%   angle phi0 (default 20 deg) on the +x meridian, so the true wall angle there is theta_y = phi0 while the estimator starts from the plane
%   (theta-hat[0] = 0, prior 5 deg). Hold 0-10 s, regulation to a_d = 0.05 from 10 s (tau_a 2 s, law-derived a' floor), scan from 17 s along
%   the +x meridian (opts.scan_meridian, scan_dir [1 0]) from phi0 to the equator. Document version (form 'local', one-sided gate 2 sqrt P),
%   slopes P_fix^2 I, angle P_fix from t_scan (3 deg at 5 um/s, 4 deg at 40 um/s). Seed 1 drawn, seeds 1-3 in the console.
%   Columns = 5 | 40 um/s; rows: side view (x-z), gains, a'_33 (estimate vs the true slope d a_33 / d h-bar at the true gap), angles.
%   Output test_results/temp_3d_wsonly/fig_1028_hemi_xscan_apq0.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 20;  if exist('ph0_in', 'var'); ph0 = ph0_in; end
FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = (pi / 2 - ph0 * pi / 180) * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_meridian', true, 'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_len', Lscan, 'tag', 'xs', 'ap_q_off_from', tS);
spd = [5 40];  pf = [3 4];  [IS, ID] = ndgrid(1:2, 1:3);  rs = cell(size(IS));
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
parfor q = 1:numel(IS)
    o = base;  o.du_d = spd(IS(q)) * Ts / Rr;  o.T = tS + Lscan / spd(IS(q)) + 0.3;  o.sP_fix_theta_deg = pf(IS(q));  o.seeds = ID(q);
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(h + 1e-4) - 1 / local_cp(h - 1e-4)) / 2e-4, hb);
for is = 1:2
    for id = 1:3
        r = rs{is, id};  Tsc = Lscan / spd(is);  ks = find(r.t >= tS & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  kb = find(r.t < tS);  m = ks(end);
        kh = find(r.t >= tH - 1e-9, 1);  kS = find(r.t >= tS - 1e-9, 1);
        fprintf('%2d um/s seed %d: contact %d | angle error (y) at 0 / 10 / 17 s: %.1f / %.1f / %.1f deg | scan: mean |e theta_y| %.1f, max %.1f | gap %.2f-%.2f um | a''33: at 17 s %.2f (true %.2f), scan mean %.2f (true %.2f)\n', ...
                spd(is), id, r.contact, r.th_tr(1, 2) - r.thy(1), r.th_tr(kh, 2) - r.thy(kh), r.th_tr(kS, 2) - r.thy(kS), mean(abs(r.th_tr(ks, 2) - r.thy(ks))), ...
                max(abs(r.th_tr(ks, 2) - r.thy(ks))), min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, r.apN(kS), slope(r.h(kS)), mean(r.apN(ks)), mean(slope(r.h(ks(1:20:end)))));
    end
end
f = figure('Color', 'w', 'Position', [30 30 2000 1700]);  tl = tiledlayout(5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for is = 1:2
    r = rs{is, 1};  Tsc = Lscan / spd(is);  k = find(r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  t = r.t(k);
    ax = nexttile(tl, is, [2 1]);  hold on;  box on;
    tg = linspace(0, pi / 2, 300);  fill([rb * sin(tg) 0], [rb * cos(tg) 0], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');  plot(r.p(k, 1), r.p(k, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
    kS = find(t >= tS, 1);  plot(r.p(1, 1), r.p(1, 3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');
    plot(r.p(kS, 1), r.p(kS, 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim([-2 24]);  ylim([0 24]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'northeast');
    title(sprintf('start %d° from the top, %d um/s, P_{fix} %d deg, slope Q off in the scan, seed 1', ph0, spd(is), pf(is)));  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
    axg = nexttile(tl, 4 + is);  hold on;  box on;
    h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.3);  ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axp = nexttile(tl, 6 + is);  hold on;  box on;  kd = k(1:10:end);
    h1 = plot(r.t(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.apN(k), 'Color', cE, 'LineWidth', 1.3);  ylim([0 2.5]);
    ylabel(sprintf('%s''_{33}', AB));  legend([h1 h2], {sprintf('d%s_{33}/dh at the true gap', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axa = nexttile(tl, 8 + is);  hold on;  box on;
    h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.3);  ylim([-20 100]);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [axg axp axa]
        xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');
        set(a, 'XLim', [0 t(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, 'fig_1028_hemi_xscan_apq0.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
