% STATUS: SCRATCH -- 2026-10-28 (user): one full turn around a whole sphere (radius 15 um) in the void (opts.hemi_void: nearest surface always the
%   sphere). Start 10 deg off the top on the +x side, 15 um above, theta-hat[0] = 0, angle prior 10 deg; hold 0-10 s, regulation tau_a 2 s to
%   a_d = 0.05, from 17 s scan the great circle in the x-z plane (step along y-hat x w-hat-est, opts.scan_plane_n) for one full turn
%   2 pi (15 + h*) um. Document version, angle P_fix 3 deg (5 um/s) / 4 deg (40 um/s). Arms: B slope sqrt P_fix x 1/4 | L law (1/9)(1 - a-hat)^2.
%   Speed from spd_in (default 40). Rows: side view x-z (full circle), gains, a'_33 vs the true slope, angle theta_y (true unwrapped) and theta_x.
%   Output test_results/temp_3d_wsonly/fig_1028_sphere_circle_v<speed>.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  spd = 40;  if exist('spd_in', 'var'); spd = spd_in; end;  pfa = 3;  if spd > 10; pfa = 4; end;  if exist('pfa_in', 'var'); pfa = pfa_in; end
FS = 13;  hatc = char(770);  AB = char(257);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = 2 * pi * (rb + hs * Rr);  Tsc = Lscan / spd;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'scan_len', Lscan, 'du_d', spd * Ts / Rr, 'T', tS + Tsc + 0.3, ...
              'sP_fix_theta_deg', pfa, 'tag', 'sc');
arm = {'B: slope sqrt P_{fix} x 1/4', 'L: slope sqrt P_{fix} = (1/9)(1 - a-hat)^2'};  na = 2;
[IA, ID] = ndgrid(1:na, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 1; o.ap_pfix_scale = 0.25; else; o.ap_pfix_law = true; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
uw = @(r, k) unwrap(r.th_tr(k, 2) * pi / 180) * 180 / pi;
for ia = 1:na
    for id = 1:3
        r = rs{ia, id};  ks = find(r.t >= tS & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  ty = uw(r, (1:r.kend)');  ty = ty(ks);
        ey = ty - r.thy(ks);  ks5 = ks(1:5:end);
        fprintf('%-46s seed %d: contact %d | scanned %.0f deg of the circle | angle err theta_y mean %.1f max %.1f, theta_x max %.1f | a''33 scan rms %.2f | a33 scan rms %.3f | gap %.2f-%.2f um\n', ...
                arm{ia}, id, r.contact, (r.t(ks(end)) - tS) / Tsc * 360, mean(abs(ey)), max(abs(ey)), max(abs(r.thx(ks) - r.th_tr(ks, 1))), ...
                rms(r.apN(ks5) - slope(r.h(ks5))), rms(r.aN(ks) - r.aN_tr(ks)), min(r.h(ks)) * Rr, max(r.h(ks)) * Rr);
    end
end
f = figure('Color', 'w', 'Position', [20 20 1800 1700]);  tl = tiledlayout(5, na, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:na
    r = rs{ia, 1};  k = (1:r.kend)';  t = r.t(k);
    ax = nexttile(tl, ia, [2 1]);  hold on;  box on;
    tg = linspace(0, 2 * pi, 400);  fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 3, 'DisplayName', 'wall');
    plot(r.p(k, 1), r.p(k, 3), 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'p');
    plot(r.p(1, 1), r.p(1, 3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');
    kS = find(t >= tS, 1);  plot(r.p(kS, 1), r.p(kS, 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim([-22 22]);  ylim([-22 32]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  if ia == 1; legend('Location', 'northeast'); end
    title(sprintf('%s, %d um/s, angle P_{fix} %g deg, seed 1', arm{ia}, spd, pfa));  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
    axg = nexttile(tl, 2 * na + ia);  hold on;  box on;
    h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.2);  ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axp = nexttile(tl, 3 * na + ia);  hold on;  box on;  kd = k(1:10:end);
    h1 = plot(r.t(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.apN(k), 'Color', cE, 'LineWidth', 1.2);  ylim([0 2]);
    ylabel(sprintf('%s''_{33}', AB));  legend([h1 h2], {sprintf('d%s_{33}/dh at the true gap', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axa = nexttile(tl, 4 * na + ia);  hold on;  box on;
    h1 = plot(t, uw(r, k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.2);  ylim([-30 400]);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
    legend([h1 h2 h3], {sprintf('%s_y (unwrapped), %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [axg axp axa]
        xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');
        set(a, 'XLim', [0 t(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, sprintf('fig_1028_sphere_circle_v%d_pf%g.png', spd, pfa));  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
