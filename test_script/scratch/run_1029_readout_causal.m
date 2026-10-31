% STATUS: SCRATCH -- 2026-10-29 (user): is the a_33 readout bias the root of the a'-hat_33 failure in the 40 um/s scan? Current flow
%   (hold 10 s, descent 1 s, regulation 1 s, scan from 12 s, ramp 0.5 s, kappa from t_scan, tau_a 0.5 s, angle P_fix 2 deg, report slope
%   P_fix, threefry), sphere radius 15 um, 3 s of scan. Arms (reference arms swap the gain readout from t_scan, opts.ym_cheat):
%   (A) as is | (B) readout = true elements, no noise | (C) readout = true elements + white noise with the model R (own random stream).
%   Seeds 1-10. Console: a'-hat/true in the scan (median, fraction < 0), mean |a33 - a_d|, mean (a-hat - a). Lag check on arm A:
%   mean(readout - trailing mean of the true a_33 over W steps) for W = 1, 10, 40, 160, 400. Figure fig_1029_readout_causal.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  Tr = 0.5;  tS = 12;  Tw = 3;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tw, 'tag', 'rc');
arm = {'(A) as is', '(B) true readout, no noise', '(C) true readout + model noise'};  NA = 3;  NS = 10;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.ym_cheat = 'true';  o.ym_cheat_from = tS; end
    if IA(q) == 3; o.ym_cheat = 'true_noise';  o.ym_cheat_from = tS; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'apN', r.apN, 'h', r.h, 'ym', r.ym);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    M = NaN(NS, 5);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  kq = ks(1:10:end);  qr = r.apN(kq) ./ slope(r.h(kq));
        M(id, :) = [r.contact, median(qr), mean(qr < 0), mean(abs(r.aN_tr(ks) - ad)), mean(r.aN(ks) - r.aN_tr(ks))];
    end
    fprintf('%-34s| contact %d | a''-hat/true median %s | <0 frac %s (mean %.2f) | mean|a33 - a_d| %.3f | mean(a-hat - a) %+.4f\n', arm{ia}, sum(M(:, 1)), ...
            mat2str(round(M(:, 2)', 2)), mat2str(round(M(:, 3)', 2)), mean(M(:, 3)), mean(M(:, 4)), mean(M(:, 5)));
end
Wl = [1 10 40 160 400];  Lg = zeros(NS, numel(Wl));
for id = 1:NS
    r = rs{1, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);
    for w = 1:numel(Wl); tm = movmean(r.aN_tr(kk), [Wl(w) - 1 0]);  Lg(id, w) = mean(r.ym(ks, 2) - tm(ks)); end
end
fprintf('lag check (arm A): mean(readout - trailing mean of true a over W steps), W = %s: %s\n', mat2str(Wl), mat2str(round(mean(Lg, 1), 4)));
f = figure('Color', 'w', 'Position', [20 20 650 * NA 900]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  k = find(r.t >= tS - 1 & (1:numel(r.t))' <= r.kend);  kd = k(1:5:end);  t = r.t(k) - tS;
    a1 = nexttile(tl, ia);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);
    h3 = yline(ad, 'k--', 'LineWidth', 1.5);  ylim([0 0.15]);  ylabel(sprintf('%s_{33}', AB));  title(sprintf('%s, seed 5', arm{ia}));
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-2 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_readout_causal.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
