% STATUS: SCRATCH -- 2026-10-29 (user): why the earlier flow (regulation 10-17 s, then scan) did not show the scan-start jump of the
%   flow "descent 1 s, wait 1 s, then regulation + scan". Same sphere / 40 um/s / kappa / angle P_fix 2 deg / tau_a 2 s. Arms:
%   (1) regulation 10-17 s, L | (2) regulation 10-17 s, report P_fix | (3) descent 1 s + wait 1 s, report P_fix. Seeds 1-5.
%   Console at t_scan: time near the wall before the scan (gap < 3 um), theta_y error, kappa-hat vs 1/(rb + gap), a-hat - a, a'-hat / true;
%   first 0.3 s of the scan: a_33 max and min. Figure fig_1029_why_reg_ok.png: gap, theta_y error, kappa-hat of seed 1 vs time since t_scan.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];  cs = lines(3);
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  Tsc = Lturn / spd;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'wy');
arm = {'(1) regulation 10-17 s, L', '(2) regulation 10-17 s, report P_{fix}', '(3) descent 1 s + wait 1 s'};  tSa = [17 17 12];  NA = 3;  NS = 5;
[IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  tS = tSa(IA(q));  o.T = tS + Tsc + 0.2;
    if IA(q) <= 2; o.t_scan = tS;  o.q_theta_from = tS; else; o.descent_T = 1;  o.descent_v = 15 - hs * Rr;  o.post_wait = [1 1]; end
    if IA(q) == 1; o.ap_pfix_law = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'aN', r.aN, 'aN_tr', r.aN_tr, 'apN', r.apN, 'h', r.h, 'thy', r.thy, 'th_tr', r.th_tr, 'kap', r.kap);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    tS = tSa(ia);  M = NaN(NS, 8);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  k0 = kk(abs(r.t(kk) - tS) < Ts / 2);  k3 = kk(r.t(kk) >= tS & r.t(kk) < tS + 0.3);
        ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;  kn = kk(r.t(kk) < tS & r.h(kk) * Rr < 3);
        M(id, :) = [numel(kn) * Ts, r.thy(k0) - ty(k0), r.kap(k0) * (rb + r.h(k0) * Rr) / Rr, r.aN(k0) - r.aN_tr(k0), r.apN(k0) / slope(r.h(k0)), max(r.aN_tr(k3)), min(r.aN_tr(k3)), r.h(k0) * Rr];
    end
    fprintf(['%-40s| near wall before scan %s s | at t_scan: theta_y err %s deg, kappa-hat x (rb + gap) %s (true 1), a-hat - a %s, a''-hat/true %s, gap %s um\n' ...
             '%-40s| first 0.3 s of the scan: a33 max %s, min %s\n'], strrep(arm{ia}, '_{fix}', '_fix'), mat2str(round(M(:, 1)', 1)), mat2str(round(M(:, 2)', 1)), ...
            mat2str(round(M(:, 3)', 2)), mat2str(round(M(:, 4)', 3)), mat2str(round(M(:, 5)', 2)), mat2str(round(M(:, 8)', 2)), '', mat2str(round(M(:, 6)', 3)), mat2str(round(M(:, 7)', 3)));
end
f = figure('Color', 'w', 'Position', [20 20 1500 1150]);  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, 1);
for j = 1:4; ax(j) = nexttile(tl);  hold on;  box on; end
for ia = 1:NA
    r = rs{ia, 1};  tS = tSa(ia);  k = find(r.t >= tS - 2 & r.t <= tS + 1 & (1:numel(r.t))' <= r.kend);  t = r.t(k) - tS;  ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;
    nm = strrep(arm{ia}, '_{fix}', '_{fix}');
    plot(ax(1), t, r.h(k) * Rr, 'Color', cs(ia, :), 'LineWidth', 1.6, 'DisplayName', nm);
    plot(ax(2), t, r.aN_tr(k), 'Color', cs(ia, :), 'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot(ax(3), t, r.thy(k) - ty(k), 'Color', cs(ia, :), 'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot(ax(4), t, reshape(r.kap(k), [], 1) .* (rb + reshape(r.h(k), [], 1) * Rr) / Rr, 'Color', cs(ia, :), 'LineWidth', 1.6, 'HandleVisibility', 'off');
end
yline(ax(2), ad, 'k--', 'LineWidth', 1.4, 'HandleVisibility', 'off');  yline(ax(3), 0, 'k-', 'HandleVisibility', 'off');  yline(ax(4), 1, 'r-', 'LineWidth', 1.6, 'HandleVisibility', 'off');
ylabel(ax(1), 'gap  [\mum]');  ylim(ax(1), [2 3.5]);  ylabel(ax(2), [char(257) '_{33}']);  ylim(ax(2), [0 0.2]);
ylabel(ax(3), ['\theta' char(770) '_y - \theta_y  [\circ]']);  ylim(ax(3), [-12 12]);  ylabel(ax(4), ['\kappa' char(770) ' (r_b + gap)']);  ylim(ax(4), [-1 2]);
xlabel(ax(4), 'time since t_{scan}  (sec)');  legend(ax(1), 'Location', 'northoutside', 'Orientation', 'horizontal');
for j = 1:4; xline(ax(j), 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(ax(j), [-2 1]);  set(ax(j), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax(j).Toolbar, 'Visible', 'off'); end
fn = fullfile(fo, 'fig_1029_why_reg_ok.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
