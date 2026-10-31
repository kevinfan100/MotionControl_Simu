% STATUS: SCRATCH -- 2026-10-28 (user): cos theta-hat_x in the theta_y step near theta_x = -90 deg. Meridian scan along +y (scan_dir [0 1]) to the
%   equator, where the wall normal is +y (theta_x -> -90 deg, gimbal lock of V = R_y R_x). Void hemisphere, a' floor, a_d 0.05, 40 um/s,
%   P_fix 4 deg, seed 2. Columns: with cos | without cos (opts.no_cos). Rows: theta-hat vs truth; w-hat error = angle between the estimated
%   and the true normal (does not depend on the angle convention); gap. Output test_results/temp_3d_wsonly/fig_1028_hemi_nocos_y.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  FS = 15;  hatc = char(770);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = pi / 2 * (rb + hs * Rr);  spd = 40;  Tsc = Lscan / spd;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_meridian', true, 'scan_dir', [0 1], 'scan_len', Lscan, 'du_d', spd * Ts / Rr, 'T', tS + Tsc + 0.5, 'sP_fix_theta_deg', 4, 'seeds', 2, 'tag', 'ny');
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
rs = cell(1, 2);
parfor ic = 1:2; o = base;  o.no_cos = ic == 2;  R = run_t3_xyz_fixed(o);  rs{ic} = R.runs{1}; end
wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];  arm = {'with cos', 'without cos'};
f = figure('Color', 'w', 'Position', [40 40 1900 1300]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for ic = 1:2
    r = rs{ic};  k = find(r.t >= tS & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  t = r.t(k) - tS;
    ew = acosd(min(1, sum(wn(r.thx(k), r.thy(k)) .* wn(r.th_tr(k, 1), r.th_tr(k, 2)), 2)));
    ax1 = nexttile(tl, ic);  hold on;  box on;
    h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.4);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.4);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.4);
    ylim([-200 200]);  ylabel(sprintf('%s  [\\circ]', th));  title(sprintf('%d um/s, P_{fix} 4 deg, scan along +y, %s, seed 2', spd, arm{ic}));
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northwest');
    ax2 = nexttile(tl, 2 + ic);  hold on;  box on;  plot(t, ew, 'Color', cE, 'LineWidth', 1.4);  ylim([0 20]);  ylabel(sprintf('w%s error  [\\circ]', hatc));
    ax3 = nexttile(tl, 4 + ic);  hold on;  box on;  plot(t, r.h(k) * Rr, 'Color', cE, 'LineWidth', 1.4);  yline(hs * Rr, 'k--', 'LineWidth', 1.3);
    ylim([2 3]);  ylabel('gap  [\mum]');  xlabel('time since t_{scan}  (sec)');
    for a = [ax1 ax2 ax3]; set(a, 'XLim', [0 t(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    fprintf('%-11s: mean w-hat error %.1f deg, max %.1f | max |theta-hat_y step| %.1f deg | gap %.2f-%.2f um\n', arm{ic}, mean(ew), max(ew), max(abs(diff(r.thy(k)))), min(r.h(k)) * Rr, max(r.h(k)) * Rr);
end
fn = fullfile(fo, 'fig_1028_hemi_nocos_y.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
