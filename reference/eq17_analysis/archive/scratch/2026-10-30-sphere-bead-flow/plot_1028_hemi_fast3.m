% STATUS: SCRATCH -- 2026-10-28 (user): [variant plot_1028_hemi_fast3: flow = hold 10 s, regulation tau_a 2 s, scan from 17 s; arms = slope sqrt P_fix as is |
%   x 1/4 | law (1/9)(1 - a-hat)^2 (opts.ap_pfix_law); speed from spd_in] what the regulation phase can do for a'_33. Void hemisphere (radius 15 um), start 10 deg off the top
%   on the +x meridian, 15 um above the sphere, theta-hat[0] = 0, angle prior 10 deg; hold 0-10 s, regulation to a_d = 0.05 (tau_a 2 s,
%   law-derived a' floor) 10-17 s, meridian scan at 5 um/s (angle P_fix 3 deg) to the equator. Document version, seeds 1-3 (seed 1 drawn).
%   Columns (regulation arms): A as is | B slope sqrt P_fix x 1/4 | C = B + normal dither 0.3 um 4 Hz during 10-17 s (opts.wdither_win).
%   Rows: side view (dashed = start straight down the TRUE normal to the sphere), gains, a'_33 vs the true slope, angles.
%   Output test_results/temp_3d_wsonly/fig_1028_hemi_reg_ap_v%d.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  spd = 5;  if exist('spd_in', 'var'); spd = spd_in; end;  pfa = 3;  if spd > 10; pfa = 4; end
FS = 13;  hatc = char(770);  AB = char(257);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = (pi / 2 - ph0 * pi / 180) * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_meridian', true, 'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_len', Lscan, 'du_d', spd * Ts / Rr, 'T', tS + Lscan / spd + 0.3, ...
              'sP_fix_theta_deg', pfa, 'tag', 'ra');
arm = {'A: slope P_{fix} as is', 'B: slope sqrt P_{fix} x 1/4', 'L: slope sqrt P_{fix} = (1/9)(1 - a-hat)^2'};  na = 3;
[IA, ID] = ndgrid(1:na, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    o.T = 17 + Lscan / spd + 0.5;
    if IA(q) == 2; o.ap_pfix_scale = 0.25; end
    if IA(q) == 3; o.ap_pfix_law = true; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:na
    for id = 1:3
        r = rs{ia, id};  tS = min(r.tsc, 17);  kS = find(r.t >= tS - 1e-9, 1);  kr = find(r.t >= tH + 2 & r.t < tS & (1:numel(r.t))' <= r.kend);  ks = find(r.t >= tS & r.t <= tS + Lscan / spd & (1:numel(r.t))' <= r.kend);
        fprintf('%-46s seed %d: scan starts %.2f s | contact %d | a''33 err: regulation 12-17 s rms %.2f, at 17 s %+.2f, scan rms %.2f | rms a33 err scan %.3f | angle err at 17 s %.1f deg, scan mean %.1f | gap %.2f-%.2f um\n', ...
                arm{ia}, id, tS, r.contact, sqrt(mean((r.apN(kr) - slope(r.h(kr))).^2)), r.apN(kS) - slope(r.h(kS)), sqrt(mean((r.apN(ks(1:5:end)) - slope(r.h(ks(1:5:end)))).^2)), ...
                sqrt(mean((r.aN(ks) - r.aN_tr(ks)).^2)), r.th_tr(kS, 2) - r.thy(kS), mean(abs(r.th_tr(ks, 2) - r.thy(ks))), min(r.h(ks)) * Rr, max(r.h(ks)) * Rr);
    end
end
f = figure('Color', 'w', 'Position', [20 20 2300 1650]);  tl = tiledlayout(5, na, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:na
    r = rs{ia, 1};  tS = min(r.tsc, 17);  k = (1:r.kend)';  ks_ = find(r.t >= tS);  k = k(r.t(k) <= tS + Lscan / spd);  t = r.t(k);
    ax = nexttile(tl, ia, [2 1]);  hold on;  box on;
    tg = linspace(0, pi / 2, 300);  fill([rb * sin(tg) 0], [rb * cos(tg) 0], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
    p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
    plot(r.p(k, 1), r.p(k, 3), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'p');
    plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');
    kS = find(t >= tS, 1);  plot(r.p(kS, 1), r.p(kS, 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim([-2 24]);  ylim([0 32]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  if ia == 1; legend('Location', 'northeast'); end
    title(sprintf('%s, %d um/s, seed 1', arm{ia}, spd));  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
    axg = nexttile(tl, 2 * na + ia);  hold on;  box on;
    h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.2);  ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axp = nexttile(tl, 3 * na + ia);  hold on;  box on;  kd = k(1:10:end);
    h1 = plot(r.t(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.apN(k), 'Color', cE, 'LineWidth', 1.2);  ylim([0 2.5]);
    ylabel(sprintf('%s''_{33}', AB));  legend([h1 h2], {sprintf('d%s_{33}/dh at the true gap', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axa = nexttile(tl, 4 * na + ia);  hold on;  box on;
    h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.2);  ylim([-10 100]);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [axg axp axa]
        xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');
        set(a, 'XLim', [0 t(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, sprintf('fig_1028_hemi_fast3_v%d.png', spd));  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
