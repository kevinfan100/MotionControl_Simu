% STATUS: SCRATCH -- 2026-10-28 (user): raster over half of the hemisphere (radius 15 um, void), settled start (10 deg off the top on +x,
%   15 um up, theta-hat[0] = 0, angle prior 10 deg, hold 0-10 s, regulation tau_a 2 s to a_d = 0.05), from 17 s: lines along u (measurement
%   x, direction n x w-hat with n = +-y so y is kept), turn at x = +-X_end, step dy along v (y) between lines (opts.raster).
%   40 um/s, angle P_fix 4 deg, slope L, seeds 1-3 (console), seed 1 drawn. Afternoon layout:
%   (1) fig_1028_raster_scan: top view x-y (sphere outline, path) and side view x-z; gains; angles vs time since t_scan.
%   (2) fig_1028_raster_extra: left before the scan (gains, angles), right tracking error (true local) over the whole run.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;  spd = 40;  pf = 4;  if exist('spd_in', 'var'); spd = spd_in; end;  if exist('pf_in', 'var'); pf = pf_in; end
Xe = 12;  dy = 2;  nl = 7;  kp = exist('kap_in', 'var') && kap_in;  sd = 1;  if exist('seed_in', 'var'); sd = seed_in; end;  FS = 15;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
Tmax = tS + (nl * 2 * 17.4 * asin(Xe / 17.4) + (nl - 1) * dy) / spd * 1.3 + 0.5;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'raster', [Xe dy nl], 'scan_len', 1e9, 'ap_pfix_law', true, ...
              'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', pf, 'T', Tmax, 'tag', 'ra');
if kp; base.kappa_state = true; end
rs = cell(1, 3);
if isempty(gcp('nocreate')); parpool('Processes', 3); end
parfor id = 1:3; o = base;  o.seeds = id;  R = run_t3_xyz_fixed(o);  rs{id} = R.runs{1}; end
wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
for id = 1:3
    r = rs{id};  te = r.rs_tend;  if isnan(te); te = r.t(r.kend); end
    ks = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);  ew = acosd(min(1, sum(wn(r.thx(ks), r.thy(ks)) .* wn(r.th_tr(ks, 1), r.th_tr(ks, 2)), 2)));
    off = find(r.h(ks) * Rr > 5 | isnan(r.aN(ks)), 1);
    fprintf('seed %d: raster done %d (end %.2f s, %.2f s of scan) | first off the sphere / NaN at %s s | w-hat error mean %.1f max %.1f deg | gap %.2f-%.2f um | rms a33 err %.3f\n', ...
            id, ~isnan(r.rs_tend), te, te - tS, num2str(r.t(ks(off))), mean(ew, 'omitnan'), max(ew), min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, rms(r.aN(ks) - r.aN_tr(ks)));
end
r = rs{sd};  te = r.rs_tend;  if isnan(te); te = r.t(r.kend); end
K = find(r.t <= te & (1:numel(r.t))' <= r.kend);  k = find(r.t(K) >= tS);  t = r.t(K(k)) - tS;
% ---------------- (1) the scan ----------------
f = figure('Color', 'w', 'Position', [30 30 1500 1500]);  tl = tiledlayout(4 + kp, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(tl, 1, [2 1]);  hold on;  box on;  tg = linspace(0, 2 * pi, 400);
fill(rb * cos(tg), rb * sin(tg), [0.9 0.9 0.9], 'EdgeColor', cGr, 'LineWidth', 3, 'DisplayName', 'wall (from above)');
plot(r.p(K(k), 1), r.p(K(k), 2), 'Color', cE, 'LineWidth', 1.4, 'DisplayName', 'p (scan)');
plot(r.p(K(k(1)), 1), r.p(K(k(1)), 2), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
axis equal;  xlim([-18 18]);  ylim([-18 18]);  xlabel('x (u)  [\mum]');  ylabel('y (v)  [\mum]');  legend('Location', 'southwest');  title('top view');
a2 = nexttile(tl, 2, [2 1]);  hold on;  box on;  tg = linspace(-pi / 2, pi / 2, 300);
fill([rb * sin(tg) 0], [rb * cos(tg) 0], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
plot(r.p(K, 1), r.p(K, 3), 'Color', cE, 'LineWidth', 1.4, 'DisplayName', 'p');
plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');  plot(r.p(K(k(1)), 1), r.p(K(k(1)), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
axis equal;  xlim([-18 18]);  ylim([0 34]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'northwest');  title('side view');
axg = nexttile(tl, 5, [1 2]);  hold on;  box on;
h1 = plot(t, r.aN_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.aN(K(k)), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.aT(K(k)), 'Color', cG, 'LineWidth', 1.3);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
ylim([0 1.1]);  ylabel('gain');
legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
axa = nexttile(tl, 7, [1 2]);  hold on;  box on;
h1 = plot(t, r.th_tr(K(k), 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(K(k), 1), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.thy(K(k)), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.thx(K(k)), 'Color', cG, 'LineWidth', 1.3);
ylabel(sprintf('%s  [\\circ]', th));  xlabel('time since t_{scan}  (sec)');
legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
title(a1, sprintf('top view: raster, %d um/s, angle P_{fix} %g deg%s, seed %d', spd, pf, repmat(', + \kappa', 1, kp), sd));
axs = [a1 a2 axg axa];
if kp
    akp = nexttile(tl, 9, [1 2]);  hold on;  box on;  kv = r.kap(:);  kv(end + 1:numel(r.t)) = NaN;
    h1 = yline(Rr / (rb + 2.37), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, kv(K(k)), 'Color', cE, 'LineWidth', 1.3);  ylim([-0.1 0.3]);
    ylabel('\kappa  [1/R]');  xlabel('time since t_{scan}  (sec)');  xlabel(axa, '');
    legend([h1 h2], {'\kappa (sphere)', '\kappa-hat'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  axs = [axs akp];
end
for a = axs; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
set(axs(3:end), 'XLim', [0 t(end)]);
fn1 = fullfile(fo, sprintf('fig_1028_raster_v%d%s_scan.png', spd, repmat('_kappa', 1, kp)));  exportgraphics(f, fn1, 'Resolution', 130);  close(f);
% ---------------- (2) before the scan + tracking error ----------------
f = figure('Color', 'w', 'Position', [30 30 2100 1300]);  tl = tiledlayout(6, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
kb = find(r.t(K) < tS);  tb = r.t(K(kb));
axg = nexttile(tl, 1, [3 1]);  hold on;  box on;
h1 = plot(tb, r.aN_tr(K(kb)), 'Color', cT, 'LineWidth', 2.2);  plot(tb, r.aT_tr(K(kb)), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(tb, r.aN(K(kb)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(tb, r.aT(K(kb)), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
ylim([0 1.1]);  ylabel('gain');  title(axg, sprintf('before the scan: hold 0-%d s, regulation to a_d %d-%d s', tH, tH, tS));
legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
axa = nexttile(tl, 7, [3 1]);  hold on;  box on;
h1 = plot(tb, r.th_tr(K(kb), 2), 'Color', cT, 'LineWidth', 2.2);  plot(tb, r.th_tr(K(kb), 1), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(tb, r.thy(K(kb)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(tb, r.thx(K(kb)), 'Color', cG, 'LineWidth', 1.4);
ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
for a = [axg axa]; xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  set(a, 'XLim', [0 tS]); end
dw = zeros(numel(K), 3);
for j = 1:numel(K)
    a = r.th_tr(K(j), :) * pi / 180;  Vt = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))] * [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
    dw(j, :) = (Vt' * (r.pd(K(j), :) - r.p(K(j), :))')';
end
nm = {'u', 'v', 'w'};  axd = gobjects(1, 3);
for row = 1:3
    axd(row) = nexttile(tl, 2 + 4 * (row - 1), [2 1]);  hold on;  box on;  plot(r.t(K), dw(:, row), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.2 0.2]);
    ylabel(sprintf('%s%s  [%sm]', DL, nm{row}, mu));  set(axd(row), 'XLim', [0 te]);
    xline(axd(row), tH, '--', 'Color', cGr, 'LineWidth', 1.3);  xline(axd(row), tS, '-.', 'Color', cGr, 'LineWidth', 1.3);
end
xlabel(axd(3), 'Time (sec)');  title(axd(1), sprintf('tracking error (true local), raster %d um/s', spd));
for a = [axg axa axd]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
fn2 = fullfile(fo, sprintf('fig_1028_raster_v%d%s_extra.png', spd, repmat('_kappa', 1, kp)));  exportgraphics(f, fn2, 'Resolution', 130);  close(f);
system(['open ' fn1 ' ' fn2]);
