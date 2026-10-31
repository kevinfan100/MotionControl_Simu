% STATUS: SCRATCH -- 2026-10-28 (user): [scan along the meridian: step = (e - z) projected on the estimated tangent plane, opts.scan_meridian] hemisphere (radius 15 um) standing in the void (no plane around it: nearest surface always the sphere,
%   opts.hemi_void), regulation with the law-derived slope floor a'_33 >= (8/9)(1 - a-hat_33)^2 (opts.ap_floor_law; b_true in [1, 9/8])
%   instead of 1/22.2^2. Document version (form 'local', doc_timing + theta_out, one-sided gate 2 sqrt P), a_d = 0.05, diagonal scan
%   (u-hat + v-hat)/sqrt 2 from the top to the equator. Seed 1; columns = angle P_fix values. Speed from spd_in (default 5 um/s), P_fix list from pfs_in (default [2 4]).
%   Rows: side view, gains, angles (no motion error). Output test_results/temp_3d_wsonly/fig_1028_hemi_mer_v<speed>_pf<list>.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  spd = 5;  if exist('spd_in', 'var'); spd = spd_in; end;  pfs = [2 4];  if exist('pfs_in', 'var'); pfs = pfs_in; end;  nc = numel(pfs);  seed = 1;
FS = 15;  hatc = char(770);  AB = char(257);  th = char(952);  tH = 10;  tS = 17;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];  rb = 15;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = pi / 2 * (rb + hs * Rr);  Tsc = Lscan / spd;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_meridian', true, ...
              'scan_dir', [1 1], 'scan_len', Lscan, 'du_d', spd * Ts / Rr, 'T', tS + Tsc + 0.5, 'seeds', seed);
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
rs = cell(1, nc);
parfor i = 1:nc; o = base;  o.sP_fix_theta_deg = pfs(i);  o.tag = 'vd';  R = run_t3_xyz_fixed(o);  rs{i} = R.runs{1}; end
f = figure('Color', 'w', 'Position', [30 30 900 * nc 1500]);  tl = tiledlayout(4, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:nc
    r = rs{i};  k = find(r.t >= tS & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  t = r.t(k) - tS;
    E = [r.thy(k) - r.th_tr(k, 2), r.thx(k) - r.th_tr(k, 1)];
    fprintf('P_fix %g deg: contact %d | mean |angle err| %.1f deg, max y %.1f x %.1f | gap %.2f-%.2f um | rms a_33 err %.3f\n', pfs(i), r.contact, ...
            mean(sum(abs(E), 2)), max(abs(E(:, 1))), max(abs(E(:, 2))), min(r.h(k)) * Rr, max(r.h(k)) * Rr, sqrt(mean((r.aN(k) - r.aN_tr(k)).^2)));
    ax = nexttile(tl, i, [2 1]);  hold on;  box on;  kp = 1:k(end);
    sp = r.p(kp, 1:2) * [1; 1] / sqrt(2);  sg = linspace(-rb, rb, 600);  zg = sqrt(rb^2 - sg.^2);
    fill(sg, zg, [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(sg, zg, 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');  plot(sp, r.p(kp, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
    plot(sp(k(1)), r.p(k(1), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim([-3 22]);  ylim([0 22]);  xlabel('distance along the scan  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'northeast');
    title(sprintf('%d um/s, a_d = %g, P_{fix} %g deg, seed %d', spd, ad, pfs(i), seed));
    axg = nexttile(tl, 2 * nc + i);  hold on;  box on;
    h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');
    axa = nexttile(tl, 3 * nc + i);  hold on;  box on;
    h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.4);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('time since t_{scan}  (sec)');  ylim([-90 100]);
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [ax axg axa]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    set([axg axa], 'XLim', [0 t(end)]);
end
fn = fullfile(fo, sprintf('fig_1028_hemi_mer_v%d_pf%s.png', spd, strrep(num2str(pfs), '  ', '_')));  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
