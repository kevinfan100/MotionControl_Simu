% STATUS: SCRATCH -- 2026-10-29 (user): fast full turn (40 um/s) of the void sphere (radius 15 um), curvature state on, angle P_fix 2 deg,
%   three arms side by side: (1) earlier today -- hold 10 s, regulation (P) 10-17 s, scan from 17 s; (2) hold 10 s, constant-speed descent
%   10-15 s, scan + P regulation from 15 s; (3) as (2) with PI regulation (tau_i 2 s). Seeds 1-3 in the console, seed drawn = the one with
%   the smallest worst-case |a_33 - a_d| over the three arms. Afternoon layout per column: side view, gains, angles vs time since the scan
%   start. Output test_results/temp_3d_wsonly/fig_1029_fast_compare.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  vd = (15 - hs * Rr) / 5;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'fc');
arm = {'regulation 10-17 s, then scan (P)', 'descent, then scan + P', 'descent, then scan + PI'};  tSa = [17 15 15];
[IA, ID] = ndgrid(1:3, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', 9); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  tS = tSa(IA(q));  o.q_theta_from = tS;  o.T = tS + Lturn / spd + 0.2;
    if IA(q) == 1; o.t_scan = tS; else; o.descent_v = vd;  o.descent_T = 5; end
    if IA(q) == 3; o.reg_pi = true;  o.tau_i = 2; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
worst = zeros(1, 3);
for ia = 1:3
    for id = 1:3
        r = rs{ia, id};  tS = tSa(ia);  ks = find(r.t >= tS & r.t <= tS + Lturn / spd & (1:numel(r.t))' <= r.kend);  k2 = ks(ks >= ks(1) + round(0.25 * numel(ks)));
        worst(id) = max(worst(id), mean(abs(r.aN_tr(k2) - ad)));
        fprintf('%-36s seed %d: contact %d | gap min %.2f mean %.2f max %.2f um | after 1/4: a33 - a_d %+.3f, a-hat - a %+.3f\n', arm{ia}, id, r.contact, ...
                min(r.h(ks)) * Rr, mean(r.h(ks)) * Rr, max(r.h(ks)) * Rr, mean(r.aN_tr(k2)) - ad, mean(r.aN(k2) - r.aN_tr(k2)));
    end
end
[~, sd] = min(worst);  if exist('seed_in', 'var'); sd = seed_in; end;  fprintf('seed %d drawn\n', sd);
f = figure('Color', 'w', 'Position', [20 20 2500 1500]);  tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:3
    r = rs{ia, sd};  tS = tSa(ia);  K = find(r.t <= tS + Lturn / spd & (1:numel(r.t))' <= r.kend);  k = find(r.t(K) >= tS);  t = r.t(K(k)) - tS;
    ty = unwrap(r.th_tr(K, 2) * pi / 180) * 180 / pi;
    ax = nexttile(tl, ia, [2 1]);  hold on;  box on;  tg = linspace(0, 2 * pi, 400);
    fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
    p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
    plot(r.p(K, 1), r.p(K, 3), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'p');
    plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');  plot(r.p(K(k(1)), 1), r.p(K(k(1)), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim([-21 21]);  ylim([-21 31]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  if ia == 1; legend('Location', 'southwest'); end
    title(sprintf('%s, seed %d', arm{ia}, sd));
    axg = nexttile(tl, 6 + ia);  hold on;  box on;
    h1 = plot(t, r.aN_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(K(k)), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.aT(K(k)), 'Color', cG, 'LineWidth', 1.3);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 0.7]);  ylabel('gain');
    if ia == 1; legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    axa = nexttile(tl, 9 + ia);  hold on;  box on;
    h1 = plot(t, ty(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(K(k), 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(K(k)), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.thx(K(k)), 'Color', cG, 'LineWidth', 1.3);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('time since t_{scan}  (sec)');
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    for a = [ax axg axa]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    set([axg axa], 'XLim', [0 t(end)]);
end
fn = fullfile(fo, 'fig_1029_fast_compare.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
