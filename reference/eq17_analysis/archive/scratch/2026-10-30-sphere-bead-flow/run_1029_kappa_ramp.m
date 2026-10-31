% STATUS: SCRATCH -- 2026-10-29 (user): fix the scan-start jump of the flow "descent 1 s, wait 1 s, then regulation + scan" (void sphere radius
%   15 um, 40 um/s full turn, kappa on, angle P_fix 2 deg, report slope P_fix, tau_a 0.5 s). Arms: (1) as is | (2) kappa-hat held at 0 with
%   P at the prior until t_scan (opts.kappa_from_scan) | (3) scan speed ramps up over 0.5 s (opts.scan_ramp) | (4) both. Seeds 1-5.
%   Figures: fig_1029_kappa_ramp.png (a_33 / a-hat_33 / a_d, kappa-hat (rb + gap), a'-hat ratio, seed 1, per arm);
%   fig_1029_kappa_ramp_result.png (arm 4, afternoon layout: side view, gains, angles, seeds 1 and 2).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  tS = 12;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, ...
              'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [1 1], 'tag', 'kr');
arm = {'as is', '\kappa-hat from t_{scan}', 'speed ramp 0.5 s', '\kappa-hat from t_{scan} + ramp'};  Tr = [0 0 0.5 0.5];
NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  ia = IA(q);  o.T = tS + Lturn / spd + Tr(ia) / 2 + 0.2;
    if ia == 2 || ia == 4; o.kappa_from_scan = true; end
    if Tr(ia) > 0; o.scan_ramp = Tr(ia); end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    tE = tS + Lturn / spd + Tr(ia) / 2;  M = NaN(NS, 7);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tE);  k5 = ks(r.t(ks) < tS + 0.5);  k0 = ks(1);  kq = ks(1:10:end);  qr = r.apN(kq) ./ slope(r.h(kq));
        M(id, :) = [r.contact, max(r.aN_tr(k5)), min(r.aN_tr(k5)), mean(abs(r.aN_tr(ks) - ad)), mean(abs(r.aN(ks) - ad)), mean(qr < 0), r.kap(k0) * (rb + r.h(k0) * Rr) / Rr];
    end
    fprintf('%-28s| contact %d | kappa-hat x (rb+gap) at t_scan %s | first 0.5 s a33 max %s min %s | mean|a33 - a_d| %s (%.3f) | mean|a-hat - a_d| %.3f | a''<0 frac %s\n', ...
            strrep(strrep(arm{ia}, '\kappa', 'kappa'), '_{scan}', '_scan'), sum(M(:, 1)), mat2str(round(M(:, 7)', 2)), mat2str(round(M(:, 2)', 3)), mat2str(round(M(:, 3)', 3)), ...
            mat2str(round(M(:, 4)', 3)), mean(M(:, 4)), mean(M(:, 5)), mat2str(round(M(:, 6)', 2)));
end
f = figure('Color', 'w', 'Position', [20 20 620 * NA 1250]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    tE = tS + Lturn / spd + Tr(ia) / 2;  r = rs{ia, 1};  k = find(r.t >= tS - 1 & r.t <= tE & (1:numel(r.t))' <= r.kend);  kd = k(1:10:end);  t = r.t(k) - tS;
    a1 = nexttile(tl, ia);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);
    h3 = yline(ad, 'k--', 'LineWidth', 1.5);  ylim([0 0.2]);  ylabel(sprintf('%s_{33}', AB));  title(sprintf('%s, seed 1', arm{ia}));
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, reshape(r.kap(k), [], 1) .* (rb + reshape(r.h(k), [], 1) * Rr) / Rr, 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-3 3]);  ylabel(['\kappa' hatc ' (r_b + gap)']);
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-1.5 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2 a3]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 tE - tS]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_kappa_ramp.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);
f = figure('Color', 'w', 'Position', [20 20 1800 1500]);  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ia = 4;  tE = tS + Lturn / spd + Tr(ia) / 2;
for c = 1:2
    r = rs{ia, c};  K = find(r.t <= tE & (1:numel(r.t))' <= r.kend);  k = find(r.t(K) >= tS);  t = r.t(K(k)) - tS;  ty = unwrap(r.th_tr(K, 2) * pi / 180) * 180 / pi;
    ax = nexttile(tl, c, [2 1]);  hold on;  box on;  tg = linspace(0, 2 * pi, 400);
    fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
    p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
    plot(r.p(K, 1), r.p(K, 3), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'p');
    plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');  plot(r.p(K(k(1)), 1), r.p(K(k(1)), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim([-21 21]);  ylim([-21 31]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  if c == 1; legend('Location', 'southwest'); end
    title(sprintf('\\kappa%s from t_{scan} + ramp 0.5 s, \\tau_a 0.5 s, seed %d', hatc, c));
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
fn = fullfile(fo, 'fig_1029_kappa_ramp_result.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
