% STATUS: SCRATCH -- 2026-10-29 (user): after the 1-s constant-speed descent, wait before the regulation / scan (opts.post_wait = [reg scan]
%   delay after the stop). Void sphere radius 15 um, start 10 deg off the top 15 um up, hold 10 s, descent 1 s (15 um -> h*), full turn at
%   40 um/s + P regulation (tau_a 2 s), kappa on, angle P_fix 2 deg (from the scan start), report slope P_fix throughout.
%   Arms: [0 0] direct | [1 1] wait 1 s, then both | [0 1] regulation 1 s, then scan | [1 3] wait 1 s, regulation 2 s, then scan. Seeds 1-5.
%   Figure columns = arms, rows: gap | a_33 (true red, estimate blue, a_d dashed) | a'-hat_33 / a'_33, seed 1, from 10 s.
%   Output test_results/temp_3d_wsonly/fig_1029_desc1_wait.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  PW = [0 0; 1 1; 0 1; 1 3];  NA = size(PW, 1);  NS = 5;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, ...
              'descent_T', 1, 'descent_v', 15 - hs * Rr, 'tag', 'd1w');
arm = {'direct', 'wait 1 s, then regulation + scan', 'regulation 1 s, then scan', 'wait 1 s, regulation 2 s, then scan'};
[IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  o.post_wait = PW(IA(q), :);  o.T = 11 + PW(IA(q), 2) + Lturn / spd + 0.2;
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    tS = 11 + PW(ia, 2);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  k0 = kk(abs(r.t(kk) - tS) < Ts / 2);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Lturn / spd);
        k1 = ks(r.t(ks) < tS + 1);  k2 = ks(ks >= ks(1) + round(0.25 * numel(ks)));  kq = ks(1:10:end);  qr = r.apN(kq) ./ slope(r.h(kq));
        fprintf('%-38s seed %d: kend %d/%d contact %d | at scan start: gap %.2f um, a33 %.3f, a-hat %.3f, a''-hat/true %.2f | first 1 s: gap %.2f-%.2f um | after 1/4: gap %.2f-%.2f, a33 - a_d %+.3f | a''-ratio in scan median %.2f, <0 frac %.2f | floor %d, mean|a33 - a_d| whole scan %.3f\n', ...
                arm{ia}, id, r.kend, numel(r.t), r.contact, r.h(k0) * Rr, r.aN_tr(k0), r.aN(k0), r.apN(k0) / slope(r.h(k0)), ...
                min(r.h(k1)) * Rr, max(r.h(k1)) * Rr, min(r.h(k2)) * Rr, max(r.h(k2)) * Rr, mean(r.aN_tr(k2)) - ad, median(qr, 'omitnan'), mean(qr < 0), r.n_floor, mean(abs(r.aN_tr(ks) - ad)));
    end
end
f = figure('Color', 'w', 'Position', [20 20 2800 1300]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    tS = 11 + PW(ia, 2);  r = rs{ia, 1};  k = find(r.t >= 10 & (1:numel(r.t))' <= r.kend);  kd = k(1:10:end);
    ax1 = nexttile(tl, ia);  hold on;  box on;  plot(r.t(k), r.h(k) * Rr, 'Color', cE, 'LineWidth', 1.4);  yline(hs * Rr, 'k--', 'LineWidth', 1.5);
    ylim([0 8]);  ylabel('gap  [\mum]');  title(sprintf('%s, seed 1', arm{ia}));
    ax2 = nexttile(tl, NA + ia);  hold on;  box on;  h1 = plot(r.t(k), r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(r.t(k), r.aN(k), 'Color', cE, 'LineWidth', 1.3);
    h3 = yline(ad, 'k--', 'LineWidth', 1.5);  ylim([0 0.3]);  ylabel(sprintf('%s_{33}', AB));
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    ax3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  plot(r.t(kd), r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-1 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  xlabel('Time (sec)');
    for a = [ax1 ax2 ax3]; xline(a, 11, ':', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [10 r.t(k(end))]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_desc1_wait.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
