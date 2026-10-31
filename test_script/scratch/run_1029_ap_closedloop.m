% STATUS: SCRATCH -- 2026-10-29 (user): why a'-hat_33 goes negative in the scan of the current flow (hold 10 s, descent 1 s, regulation 1 s,
%   scan from 12 s, tau_a 0.5 s, ramp 0.5 s, kappa from t_scan, angle P_fix 2 deg, report slope P_fix, threefry). Hypothesis: closed-loop
%   identification bias (regulation-driven normal motion + unmodelled curvature gap drift). Arms, 3 s of scan each: (A) sphere 40 um/s |
%   (B) plane 40 um/s | (C) sphere 10 um/s (curvature drift per second / 16) | (D) sphere 40 um/s, regulation off from t_scan |
%   (E) sphere 40 um/s, true normal for regulation and scan (cheat_w, reference). Seeds 1-10. Console: a'-hat / true in the scan (median,
%   fraction < 0), mean |a33 - a_d|. Figure fig_1029_ap_closedloop.png: a'-hat ratio, seed 5, per arm.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  Tr = 0.5;  tS = 12;  Tw = 3;  FS = 14;  hatc = char(770);  AB = char(257);  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tw, 'tag', 'cl');
arm = {'(A) sphere 40 um/s', '(B) plane 40 um/s', '(C) sphere 10 um/s', '(D) sphere, no regulation in the scan', '(E) sphere, true normal (ref.)'};
NA = numel(arm);  NS = 10;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    switch IA(q)
        case 2; o = rmfield(o, {'wall', 'hemi_r', 'hemi_void', 'hemi_start_deg', 'scan_plane_n'});  o.theta_y = 10;
        case 3; o.du_d = 10 * Ts / Rr;
        case 4; o.reg_off_from = tS;
        case 5; o.cheat_w = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN_tr', r.aN_tr, 'apN', r.apN, 'h', r.h);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    M = NaN(NS, 5);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  kq = ks(1:10:end);  qr = r.apN(kq) ./ slope(r.h(kq));
        kb = kk(r.t(kk) >= tS - 0.5 & r.t(kk) < tS);  kb = kb(1:10:end);
        M(id, :) = [r.contact, median(r.apN(kb) ./ slope(r.h(kb))), median(qr), mean(qr < 0), mean(abs(r.aN_tr(ks) - ad))];
    end
    fprintf('%-40s| contact %d | a''-hat/true 0.5 s before scan %s | in the scan: median %s, <0 frac %s (mean %.2f) | mean|a33 - a_d| %.3f\n', arm{ia}, sum(M(:, 1)), ...
            mat2str(round(median(M(:, 2)), 2)), mat2str(round(M(:, 3)', 2)), mat2str(round(M(:, 4)', 2)), mean(M(:, 4)), mean(M(:, 5)));
end
f = figure('Color', 'w', 'Position', [20 20 560 * NA 500]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  k = find(r.t >= tS - 1 & (1:numel(r.t))' <= r.kend);  kd = k(1:5:end);  a = nexttile(tl);  hold on;  box on;
    plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    xline(0, '-.', 'Color', cGr, 'LineWidth', 1.4);  ylim([-2 3]);  xlim([-1 Tw]);  xlabel('time since t_{scan}  (sec)');
    ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  title(sprintf('%s, seed 5', arm{ia}));  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1029_ap_closedloop.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
