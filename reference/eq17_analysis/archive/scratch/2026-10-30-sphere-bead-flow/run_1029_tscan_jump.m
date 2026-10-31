% STATUS: SCRATCH -- 2026-10-29 (user): what knocks a'-hat_33 (and a_33) at t_scan in the flow "descent 1 s, wait 1 s, then regulation + scan"
%   (void sphere radius 15 um, 40 um/s, tau_a 2 s, kappa on, angle P_fix 2 deg from t_scan, report slope P_fix). At t_scan three things switch
%   on: regulation, angle P_fix, scan. Arms: (1) no scan | (2) no scan, no angle P_fix | (3) no scan, no regulation (angle P_fix on) |
%   (4) scan | (5) scan, no angle P_fix | (6) plane scan | (7) plane scan, kappa off. Seeds 1-5. Console: a'-hat / true in the 0.2 s before
%   and after t_scan, and over the scan; a_33 max in the first 0.3 s. Figure fig_1029_tscan_jump.png: a_33 and a'-hat ratio, seed 1.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  tS = 12;  Tsc = Lturn / spd;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, ...
              'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [1 1], 'T', tS + Tsc + 0.2, 'tag', 'tj');
arm = {'no scan', 'no scan, no angle P_{fix}', 'no scan, no regulation', 'scan', 'scan, no angle P_{fix}', 'plane scan', 'plane scan, \kappa off'};
NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  ia = IA(q);
    if ia <= 3; o.scan_len = 0; end
    if ia == 2 || ia == 5; o.q_theta_off = true; end
    if ia == 3; o.reg_off = true; end
    if ia >= 6; o = rmfield(o, {'wall', 'hemi_r', 'hemi_void', 'hemi_start_deg', 'scan_plane_n'}); end
    if ia == 7; o.kappa_state = false; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'apN', r.apN, 'h', r.h, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    M = NaN(NS, 6);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  q = @(k) median(r.apN(k(1:5:end)) ./ slope(r.h(k(1:5:end))), 'omitnan');
        kb = kk(r.t(kk) >= tS - 0.2 & r.t(kk) < tS);  ka = kk(r.t(kk) >= tS & r.t(kk) < tS + 0.2);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tsc);  k3 = kk(r.t(kk) >= tS & r.t(kk) < tS + 0.3);
        ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;  k0 = kk(abs(r.t(kk) - tS) < Ts / 2);
        M(id, :) = [r.contact, q(kb), q(ka), q(ks), max(r.aN_tr(k3)), r.thy(k0) - ty(k0)];
    end
    fprintf('%-28s| contact %d | a''-hat/true 0.2 s before %s | 0.2 s after %s | whole window %s | a33 max first 0.3 s %s | theta_y err at t_scan %s deg\n', ...
            strrep(strrep(arm{ia}, '_{fix}', '_fix'), '\kappa', 'kappa'), sum(M(:, 1)), mat2str(round(M(:, 2)', 2)), mat2str(round(M(:, 3)', 2)), mat2str(round(M(:, 4)', 2)), ...
            mat2str(round(M(:, 5)', 3)), mat2str(round(M(:, 6)', 1)));
end
f = figure('Color', 'w', 'Position', [20 20 520 * NA 900]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 1};  k = find(r.t >= tS - 1 & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  kd = k(1:10:end);  t = r.t(k) - tS;
    a1 = nexttile(tl, ia);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);
    h3 = yline(ad, 'k--', 'LineWidth', 1.5);  ylim([0 0.2]);  ylabel(sprintf('%s_{33}', AB));  title(sprintf('%s, seed 1', arm{ia}));
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-1.5 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tsc]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_tscan_jump.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
