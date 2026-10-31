% STATUS: SCRATCH -- 2026-10-28 (user): three figures for the whole-sphere scan (radius 15 um, void), settled flow: start 10 deg off the top,
%   theta-hat[0] = 0, angle prior 10 deg, hold 0-10 s, regulation tau_a 2 s to a_d = 0.05, great-circle scan in the x-z plane from 17 s;
%   slope P_fix law (L); document version. Seeds 1-3 (console), the best completing seed drawn.
%   (1) fig_1028_story_10um_turn: 10 um/s, one full turn (about 11 s), angle P_fix 3 deg.
%   (2) fig_1028_story_40um_quarter: 40 um/s, a quarter turn, angle P_fix 4 deg.
%   (3) fig_1028_story_40um_accumulate: 40 um/s, full turn, angle P_fix 4 deg -- what accumulates: angle lag, the outward step per
%       step du sin(lag) and its running sum, against the gap; quarter mark and the point where the probe leaves the sphere (gap > 5 um).
%   x axis of the scan panels = position around the circle phi (deg from the top, atan2(x, z) unwrapped).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;
FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];  cO = [0.85 0.4 0];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'tag', 'st');
cs = [10 3 1; 40 4 0.25; 40 4 1];   % speed um/s, angle P_fix deg, fraction of the turn
[IC, ID] = ndgrid(1:3, 1:3);  rs = cell(size(IC));
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
parfor q = 1:numel(IC)
    c = cs(IC(q), :);  o = base;  o.du_d = c(1) * Ts / Rr;  o.sP_fix_theta_deg = c(2);  o.scan_len = c(3) * Lturn;  o.T = tS + c(3) * Lturn / c(1) + 0.2;  o.seeds = ID(q);
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
S = cell(size(rs));
for q = 1:numel(rs)
    r = rs{q};  c = cs(IC(q), :);  ks = find(r.t >= tS & r.t <= tS + c(3) * Lturn / c(1) & (1:numel(r.t))' <= r.kend);
    phi = unwrap(atan2(r.p(ks, 1), r.p(ks, 3))) * 180 / pi;  ty = unwrap(r.th_tr(1:r.kend, 2) * pi / 180) * 180 / pi;  ty = ty(ks);
    lag = ty - r.thy(ks);  gap = r.h(ks) * Rr;  off = find(gap > 5 | isnan(r.aN(ks)), 1);  if isempty(off); kon = numel(ks); else; kon = off - 1; end
    S{q} = struct('ks', ks, 'phi', phi, 'lag', lag, 'gap', gap, 'kon', kon, 'done', kon / numel(ks), 'ds', c(1) * Ts);
    fprintf('%2d um/s, P_fix %d deg, %.2f turn, seed %d: on the sphere for %.2f of the scan | mean |angle err| %.1f deg (on the sphere) | gap %.2f-%.2f um\n', ...
            c(1), c(2), c(3), ID(q), S{q}.done, mean(abs(lag(1:kon))), min(gap(1:kon)), max(gap(1:kon)));
end
% ---------------- (1) and (2): one scan per figure ----------------
names = {'fig_1028_story_10um_turn', 'fig_1028_story_40um_quarter'};
for ic = 1:2
    done = cellfun(@(s) s.done, S(ic, :));  err = cellfun(@(s) mean(abs(s.lag(1:s.kon))), S(ic, :));  sc = err + 100 * (1 - done);  [~, id] = min(sc);
    r = rs{ic, id};  s = S{ic, id};  c = cs(ic, :);  k = (1:r.kend)';  ks = s.ks;
    f = figure('Color', 'w', 'Position', [30 30 1900 1250]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1, [3 1]);  hold on;  box on;
    tg = linspace(0, 2 * pi, 400);  fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', cGr, 'LineWidth', 3, 'DisplayName', 'wall (sphere, radius 15 \mum)');
    plot(r.p(k, 1), r.p(k, 3), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'probe path p');
    plot(r.p(1, 1), r.p(1, 3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'start (10° off the top, 15 \mum up)');
    plot(r.p(ks(1), 1), r.p(ks(1), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'scan starts (17 s)');
    plot(r.p(ks(end), 1), r.p(ks(end), 3), 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'DisplayName', 'scan ends');
    axis equal;  xlim([-24 24]);  ylim([-24 32]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'southoutside');
    title(sprintf('%d \\mum/s, %s, seed %d', c(1), local_turn(c(3)), ID(ic, id)));  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
    a1 = nexttile(tl, 2);  hold on;  box on;  plot(s.phi, s.gap, 'Color', cE, 'LineWidth', 1.4);  yline(hs * Rr, 'k--', 'LineWidth', 1.4);
    ylabel('gap  [\mum]');  ylim([2 max(3.2, max(s.gap) + 0.2)]);
    text(a1, 0.02, 0.88, sprintf('target h* = %.2f \\mum (dashed); actual %.2f - %.2f \\mum', hs * Rr, min(s.gap), max(s.gap)), 'Units', 'normalized', 'FontSize', FS - 1, 'FontWeight', 'bold');
    a2 = nexttile(tl, 4);  hold on;  box on;  ty = s.lag + r.thy(ks);
    h1 = plot(s.phi, ty, 'Color', cT, 'LineWidth', 2.4);  h2 = plot(s.phi, r.thy(ks), 'Color', cE, 'LineWidth', 1.3);  ylabel(sprintf('%s_y  [\\circ]', th));
    legend([h1 h2], {sprintf('true %s_y', th), sprintf('%s%s_y', th, hatc)}, 'Location', 'northwest');
    text(a2, 0.98, 0.1, sprintf('mean |error| %.1f°, max %.1f°', mean(abs(s.lag)), max(abs(s.lag))), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 1, 'FontWeight', 'bold');
    a3 = nexttile(tl, 6);  hold on;  box on;
    h1 = plot(s.phi, r.aN_tr(ks), 'Color', cT, 'LineWidth', 2.4);  plot(s.phi, r.aT_tr(ks), 'Color', cT, 'LineWidth', 2.4);
    h2 = plot(s.phi, r.aN(ks), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(s.phi, r.aT(ks), 'Color', cG, 'LineWidth', 1.3);  ylim([0 1]);  ylabel('gain');
    xlabel('position around the circle  \phi  [\circ from the top]');
    legend([h1 h2 h3], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    text(a3, 0.98, 0.9, sprintf('rms error %s%s_{33} %.3f, %s%s_{11} %.3f', AB, hatc, rms(r.aN(ks) - r.aN_tr(ks)), AB, hatc, rms(r.aT(ks) - r.aT_tr(ks))), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 1, 'FontWeight', 'bold');
    for a = [a1 a2 a3]; set(a, 'XLim', [s.phi(1) s.phi(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    fn = fullfile(fo, [names{ic} '.png']);  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);
end
% ---------------- (3) what accumulates at 40 um/s, full turn ----------------
f = figure('Color', 'w', 'Position', [30 30 1900 1250]);  tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
cl = {cE, cO, cG};  ax = gobjects(1, 3);  for row = 1:3; ax(row) = nexttile(tl);  hold(ax(row), 'on');  box(ax(row), 'on'); end
for id = 1:3
    r = rs{3, id};  s = S{3, id};  ks = s.ks;  kk = 1:s.kon;  if isempty(kk); continue; end
    out = cumsum(s.ds * sind(s.lag(kk)));                                   % outward step du sin(lag) summed over the scan [um]
    plot(ax(1), s.phi(kk), s.lag(kk), 'Color', cl{id}, 'LineWidth', 1.3, 'DisplayName', sprintf('seed %d', id));
    plot(ax(2), s.phi(kk), out, 'Color', cl{id}, 'LineWidth', 1.3, 'HandleVisibility', 'off');
    plot(ax(3), s.phi(kk), s.gap(kk), 'Color', cl{id}, 'LineWidth', 1.3, 'HandleVisibility', 'off');
    if s.kon < numel(ks)
        for row = 1:3; xline(ax(row), s.phi(s.kon), ':', 'Color', cl{id}, 'LineWidth', 2, 'HandleVisibility', 'off'); end
    end
end
q0 = ph0 + 90;
for row = 1:3; xline(ax(row), q0, 'k--', 'LineWidth', 1.6, 'HandleVisibility', 'off'); end
ylabel(ax(1), sprintf('angle lag  %s_y - %s%s_y  [\\circ]', th, th, hatc));  legend(ax(1), 'Location', 'northwest');
text(ax(1), q0 + 2, 0.92 * max(ylim(ax(1))), 'quarter turn', 'FontSize', FS, 'FontWeight', 'bold');
ylabel(ax(2), sprintf('\\Sigma du sin(lag)  [\\mum]'));
text(ax(2), 0.01, 0.85, 'outward drift: every step goes du sin(lag) off the sphere -- this is the sum that keeps growing', 'Units', 'normalized', 'FontSize', FS - 1, 'FontWeight', 'bold');
ylabel(ax(3), 'gap  [\mum]');  yline(ax(3), hs * Rr, 'k--', 'LineWidth', 1.3, 'HandleVisibility', 'off');  yline(ax(3), 5, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(ax(3), 0.01, 0.85, 'dotted vertical = probe leaves the sphere (gap > 5 \mum) or the estimate diverges', 'Units', 'normalized', 'FontSize', FS - 1, 'FontWeight', 'bold');
xlabel(ax(3), 'position around the circle  \phi  [\circ from the top]');
for row = 1:3; set(ax(row), 'XLim', [ph0 ph0 + 360], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax(row).Toolbar, 'Visible', 'off'); end
fn = fullfile(fo, 'fig_1028_story_40um_accumulate.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function s = local_turn(fr)
    if fr == 1; s = 'one full turn'; else; s = sprintf('%.2g turn', fr); end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
