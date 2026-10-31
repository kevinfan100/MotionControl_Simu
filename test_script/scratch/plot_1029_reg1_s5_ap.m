% STATUS: SCRATCH -- 2026-10-29 (user): a'-hat_33 of the run in fig_1029_cur_scan_reg1_s5.png (current flow: hold 10 s at 15 um, descent
%   1 s, regulation from the stop, scan from 12 s at 40 um/s with ramp 0.5 s, kappa-hat from t_scan, tau_a 0.5 s, angle P_fix 2 deg,
%   report slope P_fix; seed 5, threefry). Top: a'_33 (true, at the true gap, red) and a'-hat_33 (blue), log axis, 0 s to the end;
%   bottom: a'-hat_33 / a'_33. Dashed = end of the hold, dotted = end of the descent, dash-dot = scan start.
%   Output test_results/temp_3d_wsonly/fig_1029_cur_ap_reg1_s5.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  Tr = 0.5;  tH = 10;  Tdn = 1;  if exist('td_in', 'var'); Tdn = td_in; end;  pwa = [0 1];  if exist('pw_in', 'var'); pwa = pw_in; end;  tS = tH + Tdn + pwa(2);  sd5 = 5;  if exist('seed_in', 'var'); sd5 = seed_in; end;  FS = 15;  hatc = char(770);  AB = char(257);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  Te = tS + Lturn / spd + Tr / 2;
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
           'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 0.5, 'a_d', ad, ...
           'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
           'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, ...
           'sP_fix_theta_deg', 2, 'descent_T', Tdn, 'descent_v', (15 - hs * Rr) / Tdn, 'post_wait', pwa, 'T', Te + 0.2, 'seeds', sd5, 'tag', 'ap5');
if exist('extra_in', 'var'); fe = fieldnames(extra_in); for i = 1:numel(fe); o.(fe{i}) = extra_in.(fe{i}); end; end
sfx = '';  if exist('sfx_in', 'var'); sfx = sfx_in; end
R = run_t3_xyz_fixed(o);  r = R.runs{1};
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
k = find(r.t <= Te & (1:numel(r.t))' <= r.kend);  kd = k(1:5:end);  sl = slope(r.h(kd));
ks = kd(r.t(kd) >= tS);  q = r.apN(ks) ./ slope(r.h(ks));  kb = kd(r.t(kd) >= tS - 0.5 & r.t(kd) < tS);
fprintf('seed %d:', sd5);  fprintf(' a''-hat/true last 0.5 s before the scan: median %.2f | in the scan: median %.2f, 10-90 %% %.2f to %.2f, < 0 %.0f %%\n', ...
        median(r.apN(kb) ./ slope(r.h(kb))), median(q), prctile(q, 10), prctile(q, 90), 100 * mean(q < 0));
f = figure('Color', 'w', 'Position', [30 30 1700 1000]);  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(tl);  hold on;  box on;
h1 = plot(r.t(kd), sl, 'Color', cT, 'LineWidth', 2.4);  h2 = plot(r.t(k), max(r.apN(k), 1e-3), 'Color', cE, 'LineWidth', 1.1);
set(a1, 'YScale', 'log');  ylim([1e-2 3]);  ylabel(sprintf('%s''_{33}', AB));  ttl2 = '';  if exist('ttl_in', 'var'); ttl2 = ttl_in; end;  title(sprintf('seed %d, descent %g s, wait %g s %s', sd5, Tdn, pwa(2), ttl2));
legend([h1 h2], {sprintf('%s''_{33} (true, at the true gap)', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
a2 = nexttile(tl);  hold on;  box on;
plot(r.t(kd), r.apN(kd) ./ sl, 'Color', cE, 'LineWidth', 1.2);  yline(1, 'r-', 'LineWidth', 1.6);
ylim([-1 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  xlabel('Time (sec)');
for a = [a1 a2]
    xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xline(a, tH + Tdn, ':', 'Color', cGr, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [0 Te]);
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, ['fig_1029_cur_ap_reg1_s5' sfx '.png']);  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
