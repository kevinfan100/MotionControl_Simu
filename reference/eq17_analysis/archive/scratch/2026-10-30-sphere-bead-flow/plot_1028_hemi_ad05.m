% STATUS: SCRATCH -- 2026-10-28 (user): hemisphere (radius 15 um), document version (form 'local', doc_timing + theta_out, one-sided gate
%   2 sqrt P), angle Q P_fix 2 deg from t_scan, a_d = 0.05 (instead of 0.1), fast scan 40 um/s along (u-hat + v-hat)/sqrt 2, seeds 1-3.
%   Settings otherwise as plot_1028_hemi_mekf_speed.m. One figure (best seed): side view, gains, angles (no motion error).
%   Console: the largest per-step jumps of theta-hat and a-hat with the gate state, gap and true angle at that step.
%   Speed from spd_in if set in the workspace (default 40). Output test_results/temp_3d_wsonly/fig_1028_hemi_ad05_v<speed>.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  n = numel(seeds);  ad = 0.05;  spd = 40;  if exist('spd_in', 'var'); spd = spd_in; end;  FS = 15;  hatc = char(770);  AB = char(257);  th = char(952);
tH = 10;  tS = 17;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];  rb = 15;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = pi / 2 * (rb + hs * Rr);
fprintf('a_d = %g -> h* = %.3f R = %.2f um\n', ad, hs, hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'sP_fix_theta_deg', 2, 'wall', 'hemi', 'hemi_r', rb, 'scan_dir', [1 1], 'scan_len', Lscan, ...
              'du_d', spd * Ts / Rr, 'T', tS + Lscan / spd + 0.5);
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
rs = cell(1, n);
parfor i = 1:n; o = base;  o.seeds = seeds(i);  o.tag = 'a5';  R = run_t3_xyz_fixed(o);  rs{i} = R.runs{1}; end
sc = NaN(1, n);
for i = 1:n
    r = rs{i};  kk = local_scan_steps(r, tS);  e = [r.thy(kk) - r.th_tr(kk, 2), r.thx(kk) - r.th_tr(kk, 1)];  sc(i) = mean(sum(abs(e), 2));
    fprintf('seed %d: contact %d | scan %.2f s | mean |angle err| %.1f deg, max y %.1f x %.1f | gate open %.0f%% | min gap %.2f um\n', seeds(i), r.contact, ...
            r.t(kk(end)) - tS, sc(i), max(abs(e(:, 1))), max(abs(e(:, 2))), 100 * mean(r.gate(kk)), min(r.h(kk)) * Rr);
    j = [0; abs(diff(r.thy(kk))) + abs(diff(r.thx(kk)))];  [~, ord] = sort(j, 'descend');
    for q = ord(1:4)'
        m = kk(q);  fprintf('   t-t_scan %.4f: d theta-hat (y, x) %+.2f %+.2f | d a-hat (11, 33) %+.3f %+.3f | d true angle %+.2f %+.2f | gate %d -> %d | a-hat_11 - a-hat_33 %.3f | h %.2f um\n', ...
                            r.t(m) - tS, r.thy(m) - r.thy(m - 1), r.thx(m) - r.thx(m - 1), r.aT(m) - r.aT(m - 1), r.aN(m) - r.aN(m - 1), ...
                            r.th_tr(m, 2) - r.th_tr(m - 1, 2), r.th_tr(m, 1) - r.th_tr(m - 1, 1), r.gate(m - 1), r.gate(m), r.aT(m) - r.aN(m), r.h(m) * Rr); end
end
[~, ib] = min(sc);  r = rs{ib};  k = local_scan_steps(r, tS);  t = r.t(k) - tS;
f = figure('Color', 'w', 'Position', [30 30 1100 1500]);  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl, 1, [2 1]);  hold on;  box on;
sdir = [1; 1] / sqrt(2);  sp = r.p(1:k(end), 1:2) * sdir;  sg = linspace(-25, 30, 600);  zg = sqrt(max(rb^2 - sg.^2, 0));
fill([sg fliplr(sg)], [zg, -5 + 0 * sg], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(sg, zg, 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');  plot(sp, r.p(1:k(end), 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
plot(sp(k(1)), r.p(k(1), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
axis equal;  xlim([-3 22]);  ylim([0 22]);  xlabel('distance along the scan  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'northeast');
axg = nexttile(tl, 3);  hold on;  box on;
h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
ylim([0 1.1]);  ylabel('gain');
legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
axa = nexttile(tl, 4);  hold on;  box on;
h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.4);
ylabel(sprintf('%s  [\\circ]', th));  xlabel('time since t_{scan}  (sec)');  ylim([-60 80]);
legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
title(ax, sprintf('document version, %d um/s, a_d = %g, seed %d', spd, ad, seeds(ib)));
for a = [ax axg axa]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
set([axg axa], 'XLim', [0 t(end)]);
fn = fullfile(fo, sprintf('fig_1028_hemi_ad05_v%d.png', spd));  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function kk = local_scan_steps(r, tS)                            % scan steps until the nearest surface becomes the plane
    kk = find(r.t >= tS & (1:numel(r.t))' <= r.kend);
    e = find(abs(r.th_tr(kk, 1)) < 1e-9 & abs(r.th_tr(kk, 2)) < 1e-9 & (1:numel(kk))' > 10, 1);  if ~isempty(e); kk = kk(1:e - 1); end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
