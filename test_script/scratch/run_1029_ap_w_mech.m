% STATUS: SCRATCH -- 2026-10-29 (user): why the normal-position channel (y1 w) pushes a'-hat_33 down at 40 um/s. Logged per step: the force
%   step in the local coordinate vF (a_o f, R), K(15,3), the normal-position innovation nu(3), and corr(a'_33, normal position) from P^f.
%   Current flow (hold 10 s, descent 1 s, regulation 1 s, scan from 12 s, ramp 0.5 s, kappa from t_scan, tau_a 0.5 s, angle P_fix 2 deg,
%   report slope P_fix, threefry), sphere radius 15 um, 3 s of scan, 40 and 10 um/s, seeds 1-10. Console: rms normal / tangential force
%   step, rms |corr|, rms |K(15,3)|, mean and rms of nu_w (nm) before and in the scan. Figure fig_1029_ap_w_mech.png (seed 5, both speeds).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  Tr = 0.5;  tS = 12;  Tw = 3;  FS = 13;  hatc = char(770);  AB = char(257);  cGr = [0.5 0.5 0.5];  cE = [0 0.25 0.85];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tw, 'tag', 'wm');
SP = [40 10];  NS = 10;  [IS, ID] = ndgrid(1:2, 1:NS);  rs = cell(size(IS));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IS)
    o = base;  o.seeds = ID(q);  o.du_d = SP(IS(q)) * Ts / Rr;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'apN', r.apN, 'h', r.h, 'dap', r.dap, 'vF', r.vF, 'k15w', r.k15w, 'nuw', r.nuw, 'c15w', r.c15w, 'aN', r.aN, 'aN_tr', r.aN_tr);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);  nmR = 1e3 * Rr;
for is = 1:2
    M = zeros(NS, 9);
    for id = 1:NS
        r = rs{is, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);  kb = kk(r.t(kk) >= tS - 0.9 & r.t(kk) < tS);  rm = @(x) sqrt(mean(x.^2));
        M(id, :) = [rm(r.vF(kb, 3)), rm(r.vF(ks, 3)), rm(r.vF(ks, 1)), rm(r.c15w(kb)), rm(r.c15w(ks)), rm(r.k15w(kb)), rm(r.k15w(ks)), nmR * mean(r.nuw(ks)), nmR * rm(r.nuw(ks) - mean(r.nuw(ks)))];
    end
    fprintf('%d um/s | rms normal force step before %.2e, scan %.2e | rms tangential force step scan %.2e | rms corr(a'', w) before %.3f, scan %.3f | rms K(15,w) before %.2f, scan %.2f | nu_w mean %+.2f nm, sd %.2f nm\n', ...
            SP(is), mean(M, 1));
end
f = figure('Color', 'w', 'Position', [20 20 1900 1300]);  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for is = 1:2
    r = rs{is, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  kd = k(1:5:end);  t = r.t(k) - tS;  ks0 = k(r.t(k) >= tS);
    a1 = nexttile(tl, is);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.4);
    plot(r.t(ks0) - tS, 1.3 + cumsum(r.dap(ks0, 3)) / slope(hs), 'Color', [0.93 0.69 0.13], 'LineWidth', 1.6);  ylim([-2 3]);
    ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  title(sprintf('%d um/s, seed 5 (yellow: 1.3 + cumulative y1-w update)', SP(is)));
    a2 = nexttile(tl, 2 + is);  hold on;  box on;  plot(t, r.vF(k, 3), 'Color', cE, 'LineWidth', 0.8);  plot(t, r.vF(k, 1), 'Color', [0 0.55 0.2], 'LineWidth', 0.8);
    ylabel('force step  a_o f  [R]');  legend({'normal (w)', 'tangential (u)'}, 'Location', 'northwest');  ylim([-0.06 0.06]);
    a3 = nexttile(tl, 4 + is);  hold on;  box on;  plot(t, r.c15w(k), 'Color', cE, 'LineWidth', 1.2);  yline(0, 'k-');  ylim([-1 1]);  ylabel('corr(a''_{33}, normal pos.)');
    a4 = nexttile(tl, 6 + is);  hold on;  box on;  plot(t, nmR * r.nuw(k), 'Color', [0.7 0.78 1], 'LineWidth', 0.5);  plot(t, nmR * movmean(r.nuw(k), 80), 'Color', cE, 'LineWidth', 1.6);
    yline(0, 'k-');  ylim([-15 15]);  ylabel('normal innovation  [nm]');  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2 a3 a4]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_ap_w_mech.png');  exportgraphics(f, fn, 'Resolution', 115);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
