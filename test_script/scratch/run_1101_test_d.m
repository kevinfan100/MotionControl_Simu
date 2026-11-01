% STATUS: SUPERSEDED-BY run_1101_baseline_v2.m (option keep_diag_from removed: the kept rotation is now the default, rot_diag_legacy gives the old rule) -- 2026-11-01 (user): plan step 2, candidate D: the frame rotation takes a_uw^2 / gap off a-hat_33 each step (exact matrix rotation; 1029 tex keeps first order). opts.keep_diag_from keeps a-hat_11, a-hat_33 through the rotation from t_scan.
%   40 um/s, 10 seeds, paired (first version: 5 seeds, true-normal pair, outside .47 -> .35). Arms: sphere | keep both | keep a33 only | keep a11 only | plane moving | keep both. Pre-registered: D confirmed if sphere keep both has median outside <= 0.1, honesty <= 2, a' < 0 ~ 0, contact + diverging not above base; the single arm reaching this carries it. Sanity: first 5 seeds of the base arms repeat the 5-seed run.
%   Console, scan segment (t_scan + 0.5 .. + 3): outside, a' < 0, honesty, max|ratio - 1|, a'/true, |a33 - a_d|, mean rot, sum rot. Figure fig_1101_test_d.png (all seeds thin, median thick).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  FS = 14;  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', 15, 'tag', 'td');
arm = {'sphere', 'sphere, keep both', 'sphere, keep a33', 'sphere, keep a11', 'plane moving', 'plane moving, keep both'};  NA = numel(arm);  NS = 10;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) <= 4; o.theta_y = 0;  o.th0_deg = [0 0];  o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.scan_plane_n = [0 1 0];  o.kappa_state = true;  o.kappa_from_scan = true; end
    if any(IA(q) == [2 3 4 6]); o.keep_diag_from = tS; end
    if IA(q) == 3; o.keep_diag_which = 'a33'; end
    if IA(q) == 4; o.keep_diag_which = 'a11'; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'ap', r.apN, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'Pap', r.Pap(:), 'rot', r.rot(:, 2), 'contact', r.contact);
end
tg = (-1:0.02:3)';  Rt = NaN(numel(tg), NS, NA);
for ia = 1:NA
    M = NaN(NS, 8);  bad = 0;  nc = 0;
    for id = 1:NS
        r = rs{ia, id};  b = find(~isfinite(r.ap(1:r.kend)) | ~isfinite(r.aN(1:r.kend)), 1);  kl = r.kend;  nc = nc + double(r.contact);  if ~isempty(b); kl = b - round(0.5 / Ts);  bad = bad + 1; end
        kk = (1:kl)';  k = kk(r.t(kk) >= tS - 1);  [tu, iu] = unique(r.t(k));  rat = movmean(r.ap(k) ./ local_slope(r.h(k)), 32);  Rt(:, id, ia) = interp1(tu - tS, rat(iu), tg, 'linear', NaN);
        g = kk(r.t(kk) >= tS + 0.5 & r.t(kk) < tS + 3);  if numel(g) < 50; continue; end
        tr = local_slope(r.h(g));  rt = r.ap(g) ./ tr;  e = r.ap(g) - tr;
        M(id, :) = [mean(rt < 0.1 | rt > 1.9), mean(r.ap(g) < 0), mean(e.^2 ./ r.Pap(g)), max(abs(rt - 1)), median(rt), mean(abs(r.aN_tr(g) - ad)), mean(r.rot(g)), sum(r.rot(g))];
    end
    fprintf('%-26s diverging %d, contact %d | outside %.2f %s | a''<0 %.2f %s | honesty %.2f | max|ratio-1| %.1f | a''/true %+.2f %s | |a33-a_d| %.3f | rot per step %+.2e, sum %s\n', arm{ia}, bad, nc, median(M(:, 1), 'omitnan'), mat2str(round(M(:, 1)', 2)), median(M(:, 2), 'omitnan'), mat2str(round(M(:, 2)', 2)), ...
            median(M(:, 3), 'omitnan'), max(M(:, 4)), median(M(:, 5), 'omitnan'), mat2str(round(M(:, 5)', 2)), median(M(:, 6), 'omitnan'), median(M(:, 7), 'omitnan'), mat2str(round(M(:, 8)', 3)));
end
f = figure('Color', 'w', 'Position', [20 20 440 * NA 460]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    a = nexttile(tl, ia);  hold on;  box on;  plot(tg, Rt(:, :, ia), 'Color', [0.7 0.78 1], 'LineWidth', 0.7, 'HandleVisibility', 'off');  plot(tg, median(Rt(:, :, ia), 2, 'omitnan'), 'Color', cE, 'LineWidth', 2, 'DisplayName', arm{ia});
    yline(1, 'r-', 'LineWidth', 1.4, 'HandleVisibility', 'off');  xline(0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim([-3 4]);  xlim([-1 3]);  ylabel('estimate / true slope');  xlabel('time since t_{scan}  (sec)');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1101_test_d.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function s = local_slope(hb)
    s = arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
