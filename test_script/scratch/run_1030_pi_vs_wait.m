% STATUS: SCRATCH -- 2026-10-30 (user): descent 5 s + wait 1 s + P (reference) vs no wait + PI with a faster integral (gate: integrate after
%   e = a_d - a-hat_33 first crosses 0): tau_i 1 s (zeta 0.71) and tau_i 0.5 s (zeta 0.5); tau_a 0.5 s. Void sphere radius 15 um, 40 um/s
%   full turn, kappa from t_scan, ramp 0.5 s, angle P_fix 2 deg, report slope P_fix, threefry seeds 1-10. Console: a_33 vs a_d over the turn
%   and the angle error at t_scan and over the first 1 s of the scan. Figure fig_1030_pi_vs_wait.png: seed 5, top a_33 per arm, bottom
%   angle error theta-hat - theta (x and y) per arm, both vs time since t_scan.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  Td = 5;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'a_d', ad, 'tau_a', 0.5, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', 0.5, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', Td, 'descent_v', (15 - hs * Rr) / Td, 'tag', 'pw');
nm = {'wait 1 s + P (reference)', 'no wait + PI, \tau_i 1 s', 'no wait + PI, \tau_i 0.5 s'};  PW = [1 0 0];  TI = [NaN 1 0.5];
NL = numel(nm);  NS = 10;  [IL, ID] = ndgrid(1:NL, 1:NS);  rs = cell(size(IL));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IL)
    L = IL(q);  o = base;  o.seeds = ID(q);  o.post_wait = [0 PW(L)];  tS = 10 + Td + PW(L);  o.T = tS + Lturn / spd + 0.5;
    if L >= 2; o.reg_pi = true;  o.tau_i = TI(L);  o.reg_pi_cond = 'cross'; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'tsc', r.tsc, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
for L = 1:NL
    M = NaN(NS, 5);
    for id = 1:NS
        r = rs{L, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc & r.t(kk) <= r.tsc + Lturn / spd + 0.25);  k0 = ks(1);  k1 = ks(r.t(ks) < r.tsc + 1);
        ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;  ex = r.thx - r.th_tr(:, 1);  ey = r.thy - ty;
        M(id, :) = [mean(abs(r.aN_tr(ks) - ad)), mean(r.aN_tr(ks) - ad), max(abs([ex(k0) ey(k0)])), mean(max(abs([ex(k1) ey(k1)]), [], 2)), r.contact];
    end
    fprintf('%-28s| contact %d | mean|a33 - a_d| %.3f | mean(a33 - a_d) %+.3f | angle error at t_scan %s deg (mean %.1f) | mean angle error first 1 s of scan %.1f deg\n', ...
            strrep(nm{L}, '\tau', 'tau'), sum(M(:, 5)), mean(M(:, 1)), mean(M(:, 2)), mat2str(round(M(:, 3)', 1)), mean(M(:, 3)), mean(M(:, 4)));
end
f = figure('Color', 'w', 'Position', [20 20 2400 1150]);  tl = tiledlayout(2, NL, 'TileSpacing', 'compact', 'Padding', 'compact');
for L = 1:NL
    r = rs{L, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= r.tsc - 3 & r.t(kk) <= r.tsc + Lturn / spd + 0.25);  t = r.t(k) - r.tsc;
    ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;
    a1 = nexttile(tl, L);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.8);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1);  h3 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 0.3]);  ylabel(sprintf('%s_{33}', AB));  title(sprintf('%s, seed 5', nm{L}));
    if L == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    a2 = nexttile(tl, NL + L);  hold on;  box on;  h1 = plot(t, r.thy(k) - ty(k), 'Color', cE, 'LineWidth', 1.2);  h2 = plot(t, r.thx(k) - r.th_tr(k, 1), 'Color', cG, 'LineWidth', 1.2);
    yline(0, 'k-', 'HandleVisibility', 'off');  ylim([-10 10]);  ylabel(sprintf('%s%s - %s  [\\circ]', th, hatc, th));  xlabel('time since t_{scan}  (sec)');
    if L == 1; legend([h1 h2], {sprintf('%s%s_y - %s_y', th, hatc, th), sprintf('%s%s_x - %s_x', th, hatc, th)}, 'Location', 'northeast'); end
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-3 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1030_pi_vs_wait.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
