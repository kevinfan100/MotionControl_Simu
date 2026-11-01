% STATUS: SCRATCH -- 2026-10-31 (user): which part of the angle block corrupts a'-hat_33 at 40 um/s (mainline-3d open item 1). Plane theta_y 10, current
%   flow, kappa off in every arm, one change at a time from the clean arm C (angle known, frozen, no P on the off-diagonal slots).
%   Arms: C clean (must reproduce 10-31: fraction 0.00, mean|a33 - a_d| 0.007) | 1 angle starts true, estimated as usual | 2 frame frozen 2 deg off, off-diagonal P
%   alive (real angle error, no jitter; first version with P = 0 was inconsistent: 10/10 NaN) | 3 frame frozen and true, off-diagonal P alive (covariance path, no real error)
%   | 4 arm 1 + no_ang_gain (drops the term a' M_u e_off / gap already in F_e) | 5 arm 3 + fe_cut angain from t_scan (all off-diagonal -> gain rows).
%   Pre-registered: fraction a'-hat < 0 <= 0.05 clean, >= 0.20 path present. Sanity before reading: theta-hat 10.00 in C, 3, 5, 12.00 in 2. Fraction over non-NaN steps; NaN runs counted.
%   Console: fraction a'-hat < 0 (median, mean, per seed), median a'-hat / true, mean|a33 - a_d|, end theta_y. Figure fig_1031_ap_angle_split.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cGr = [0.5 0.5 0.5];  cE = [0 0.25 0.85];  cT = [0.85 0 0];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 1e-3, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'as', 'freeze_angle', true, 'q_theta_off', true);
arm = {'C: angle known, frozen', '1: starts true, estimated', '2: frozen 2 deg off, P alive', '3: frozen true, P alive', '4: arm 1, no a'' M_u e_{off}', '5: arm 3, cut off-diag to gain'};
NA = numel(arm);  NS = 10;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    switch IA(q)
        case {2, 5}; o = rmfield(o, {'freeze_angle', 'q_theta_off'});  o.sP_theta_deg = 10;  if IA(q) == 5; o.no_ang_gain = true; end
        case 3; o = rmfield(o, 'q_theta_off');  o.sP_theta_deg = 10;  o.th0_deg = [0 12];
        case 4; o = rmfield(o, 'q_theta_off');  o.sP_theta_deg = 10;
        case 6; o = rmfield(o, 'q_theta_off');  o.sP_theta_deg = 10;  o.fe_cut = 'angain';  o.fe_cut_from = tS;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'apN', r.apN, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thy', r.thy);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    M = NaN(NS, 6);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);  bad = ~isfinite(r.thy(r.kend)) || r.kend < numel(r.t);
        M(id, 1) = double(r.contact);  M(id, 6) = double(bad);  ks = ks(isfinite(r.apN(ks)));
        if numel(ks) < 2 || bad; continue; end
        M(id, 2:5) = [mean(r.apN(ks) < 0), median(r.apN(ks) ./ slope(r.h(ks))), mean(abs(r.aN_tr(ks) - ad)), r.thy(ks(end))];
    end
    fprintf('%-32s | contact %d, NaN/short runs %d | a''-hat<0 median %.2f mean %.2f %s | a''-hat/true median %.2f | mean|a33-a_d| %.3f | theta_y end %.2f\n', arm{ia}, sum(M(:, 1)), sum(M(:, 6)), ...
            median(M(:, 2), 'omitnan'), mean(M(:, 2), 'omitnan'), mat2str(round(M(:, 2)', 2)), median(M(:, 3), 'omitnan'), median(M(:, 4), 'omitnan'), mean(M(:, 5), 'omitnan'));
end
f = figure('Color', 'w', 'Position', [20 20 500 * NA 800]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  kd = k(1:5:end);  t = r.t(kd) - tS;
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.aN_tr(kd), 'Color', cT, 'LineWidth', 1.3, 'DisplayName', 'a_{33} / a_{nom}');
    plot(t, r.aN(kd), 'Color', cE, 'LineWidth', 1.0, 'DisplayName', arm{ia});  yline(ad, 'k--', 'HandleVisibility', 'off');  ylim([0 0.15]);  ylabel('a_{33} / a_{nom}');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.4);
    ylim([-2 3]);  ylabel('estimate / true slope');  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1031_ap_angle_split.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
