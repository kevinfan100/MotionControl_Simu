% STATUS: SCRATCH -- 2026-10-29 (user): replace the 10-s still hold at 15 um by 3 s with motion along w-hat -- does normal motion help the
%   angle? Flow after the hold: descent 1 s, regulation 1 s, then scan (post_wait [0 1]); void sphere radius 15 um, 40 um/s full turn, ramp
%   0.5 s, kappa-hat from t_scan, tau_a 0.5 s, angle P_fix 2 deg, report slope P_fix, threefry seeds 1-10. Arms: (1) hold 10 s still |
%   (2) hold 3 s still | (3) 3 s with w-dither 3 um 8 Hz (ramp 1 s) | (4) 3 s with w-dither 5 um 4 Hz (ramp 1 s) | (5) 3 s with the 10-21 horizontal circle dither 3 um 8 Hz (ramp 1 s, in u-hat, v-hat). Console: angle error at
%   the end of the hold and at t_scan (mean |.| and max over seeds), mean |a_33 - a_d| over the turn. Figure fig_1029_hold_wdither.png:
%   angles and w (gap) vs time, seed 1, until t_scan.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  Tr = 0.5;  FS = 14;  hatc = char(770);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  Tsc = Lturn / spd + Tr / 2;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'wd');
arm = {'hold 10 s still', 'hold 3 s still', '3 s, w-dither 3 um 8 Hz', '3 s, w-dither 5 um 4 Hz', '3 s, u-v circle 3 um 8 Hz'};  tH = [10 3 3 3 3];  WD = {[], [], [3 8 1], [5 4 1], []};
NA = numel(arm);  NS = 10;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  ia = IA(q);  o.hold_until = tH(ia);  o.T = tH(ia) + 2 + Tsc + 0.2;
    if ~isempty(WD{ia}); o.wdither = WD{ia};  o.wdither_win = [0 tH(ia)]; end
    if ia == 5; o.dither = [3 8 1];  o.dither_win = [0 tH(ia)]; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN_tr', r.aN_tr, 'h', r.h, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
for ia = 1:NA
    tS = tH(ia) + 2;  M = NaN(NS, 5);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;  e = @(k) max(abs([r.thx(k) - r.th_tr(k, 1), r.thy(k) - ty(k)]));
        kh = kk(abs(r.t(kk) - tH(ia)) < Ts / 2);  k0 = kk(abs(r.t(kk) - tS) < Ts / 2);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tsc);
        M(id, :) = [r.contact, e(kh), e(k0), mean(abs(r.aN_tr(ks) - ad)), max(r.h(ks)) * Rr];
    end
    fprintf('%-26s| contact %d | max angle err at the end of the hold: mean %.1f, worst %.1f deg | at t_scan: mean %.1f, worst %.1f deg | mean|a33 - a_d| %.3f (worst %.3f) | gap max %.2f um\n', ...
            arm{ia}, sum(M(:, 1)), mean(M(:, 2)), max(M(:, 2)), mean(M(:, 3)), max(M(:, 3)), mean(M(:, 4)), max(M(:, 4)), max(M(:, 5)));
end
f = figure('Color', 'w', 'Position', [20 20 560 * NA 900]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 1};  tS = tH(ia) + 2;  k = find(r.t <= tS & (1:numel(r.t))' <= r.kend);  ty = unwrap(r.th_tr(k, 2) * pi / 180) * 180 / pi;
    a1 = nexttile(tl, ia);  hold on;  box on;
    h1 = plot(r.t(k), ty, 'Color', cT, 'LineWidth', 2.2);  plot(r.t(k), r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(r.t(k), r.thy(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(r.t(k), r.thx(k), 'Color', cG, 'LineWidth', 1.2);
    ylim([-15 20]);  ylabel(sprintf('%s  [\\circ]', th));  title(sprintf('%s, seed 1', arm{ia}));
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(r.t(k), r.h(k) * Rr, 'Color', cE, 'LineWidth', 1.2);  yline(hs * Rr, 'k:', 'LineWidth', 1.4);
    ylim([0 22]);  ylabel('gap  [\mum]');  xlabel('Time (sec)');
    for a = [a1 a2]; xline(a, tH(ia), '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  xline(a, tH(ia) + 1, ':', 'Color', cGr, 'LineWidth', 1.5, 'HandleVisibility', 'off');  xlim(a, [0 tS]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_hold_wdither.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
