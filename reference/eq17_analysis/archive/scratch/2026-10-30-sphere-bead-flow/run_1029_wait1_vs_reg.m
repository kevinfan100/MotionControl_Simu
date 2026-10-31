% STATUS: SCRATCH -- 2026-10-29 (user): (a) result of the chosen flow "descent 1 s, wait 1 s, then regulation + scan together";
%   (b) why the earlier flow (regulation 10-17 s, then scan) did not lose a'-hat_33 in the scan. Void sphere radius 15 um, start 10 deg off
%   the top 15 um up, hold 10 s, full turn at 40 um/s, P regulation tau_a 2 s, kappa on, angle P_fix 2 deg from the scan start. Arms:
%   (1) regulation 10-17 s, scan, slope P_fix law L | (2) same, report P_fix | (3) descent 1 s + wait 1 s, report P_fix |
%   (4) as (3), report -> L at the scan. Seeds 1-5. Figures: fig_1029_wait1_result.png (arm 3, afternoon layout: side view, gains, angles);
%   fig_1029_wait1_vs_reg_ap.png (a'-hat_33 / a'_33 and a_33 of the four arms, seed 1, vs time since the scan start).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'w1');
arm = {'(1) regulation 10-17 s, L', '(2) regulation 10-17 s, report P_{fix}', '(3) descent 1 s + wait 1 s, report P_{fix}', '(4) as (3), report \rightarrow L at the scan'};
tSa = [17 17 12 12];  NA = 4;  NS = 5;
[IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  tS = tSa(IA(q));  o.T = tS + Lturn / spd + 0.2;
    if IA(q) <= 2; o.t_scan = tS;  o.q_theta_from = tS; else; o.descent_T = 1;  o.descent_v = 15 - hs * Rr;  o.post_wait = [1 1]; end
    if IA(q) == 1; o.ap_pfix_law = true; end
    if IA(q) == 4; o.ap_pfix_law = true;  o.ap_pfix_law_from_scan = true; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    tS = tSa(ia);  M = zeros(NS, 6);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  k0 = kk(abs(r.t(kk) - tS) < Ts / 2);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Lturn / spd);  kq = ks(1:10:end);
        qr = r.apN(kq) ./ slope(r.h(kq));  kb = kk(r.t(kk) >= tS - 0.5 & r.t(kk) < tS);  kb = kb(1:10:end);
        M(id, :) = [median(r.apN(kb) ./ slope(r.h(kb))), r.aN(k0) - r.aN_tr(k0), median(qr), mean(qr < 0), mean(abs(r.aN_tr(ks) - ad)), max(r.h(ks)) * Rr];
    end
    fprintf('%-44s| last 0.5 s before scan: a''-hat/true %s, a-hat - a at scan start %s | scan: a''-hat/true median %s, <0 frac %s | mean|a33 - a_d| %s (mean %.3f) | gap max %s um\n', ...
            strrep(strrep(arm{ia}, '\rightarrow', '->'), '_{fix}', '_fix'), mat2str(round(M(:, 1)', 2)), mat2str(round(M(:, 2)', 3)), mat2str(round(M(:, 3)', 2)), ...
            mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 3)), mean(M(:, 5)), mat2str(round(M(:, 6)', 2)));
end
% (a) arm 3 result, afternoon layout, seeds 1 and 2 side by side
f = figure('Color', 'w', 'Position', [20 20 1800 1500]);  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:2
    r = rs{3, c};  tS = tSa(3);  K = find(r.t <= tS + Lturn / spd & (1:numel(r.t))' <= r.kend);  k = find(r.t(K) >= tS);  t = r.t(K(k)) - tS;
    ty = unwrap(r.th_tr(K, 2) * pi / 180) * 180 / pi;
    ax = nexttile(tl, c, [2 1]);  hold on;  box on;  tg = linspace(0, 2 * pi, 400);
    fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
    p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
    plot(r.p(K, 1), r.p(K, 3), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'p');
    plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');  plot(r.p(K(k(1)), 1), r.p(K(k(1)), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim([-21 21]);  ylim([-21 31]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  if c == 1; legend('Location', 'southwest'); end
    title(sprintf('descent 1 s, wait 1 s, then regulation + scan, seed %d', c));
    axg = nexttile(tl, 4 + c);  hold on;  box on;
    h1 = plot(t, r.aN_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(K(k)), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.aT(K(k)), 'Color', cG, 'LineWidth', 1.3);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 0.7]);  ylabel('gain');
    if c == 1; legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    axa = nexttile(tl, 6 + c);  hold on;  box on;
    h1 = plot(t, ty(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(K(k), 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(K(k)), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.thx(K(k)), 'Color', cG, 'LineWidth', 1.3);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('time since t_{scan}  (sec)');
    if c == 1; legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    for a = [ax axg axa]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    set([axg axa], 'XLim', [0 t(end)]);
end
fn = fullfile(fo, 'fig_1029_wait1_result.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);
% (b) a'-hat ratio and a_33 of the four arms, seed 1, from 2 s before the scan
f = figure('Color', 'w', 'Position', [20 20 2800 900]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 1};  tS = tSa(ia);  k = find(r.t >= tS - 2 & r.t <= tS + Lturn / spd & (1:numel(r.t))' <= r.kend);  kd = k(1:10:end);  t = r.t(k) - tS;
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-1.5 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  title(sprintf('%s, seed 1', arm{ia}));
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);  h3 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 0.3]);  ylabel(sprintf('%s_{33}', AB));  xlabel('time since t_{scan}  (sec)');
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-2 Lturn / spd]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_wait1_vs_reg_ap.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
