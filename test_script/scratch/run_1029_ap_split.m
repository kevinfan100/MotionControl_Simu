% STATUS: SCRATCH -- 2026-10-29 (user): which measurement component moves a'-hat_33 during the 40 um/s scan. Non-invasive: the runner logs
%   L.dap = K(15,:) .* nu' (the update of a'-hat_33 by each of the 7 innovations: y1 u, v, w in the local coordinate of V-hat[k]; gain
%   readouts (u u + v v)/2, w w, u w, v w). Current flow (hold 10 s, descent 1 s, regulation 1 s, scan from 12 s at 40 um/s, ramp 0.5 s,
%   kappa from t_scan, tau_a 0.5 s, angle P_fix 2 deg, report slope P_fix, threefry), sphere radius 15 um, 3 s of scan; also 10 um/s.
%   Console: summed update per channel over the scan (mean and sd over 10 seeds, in units of the true slope) and the sum of |update|.
%   Figure fig_1029_ap_split.png (seed 5): a'-hat / true and the cumulative update per channel from t_scan, 40 | 10 um/s.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  Tr = 0.5;  tS = 12;  Tw = 3;  FS = 14;  hatc = char(770);  AB = char(257);  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tw, 'tag', 'sp');
SP = [40 10];  NS = 10;  [IS, ID] = ndgrid(1:2, 1:NS);  rs = cell(size(IS));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IS)
    o = base;  o.seeds = ID(q);  o.du_d = SP(IS(q)) * Ts / Rr;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'apN', r.apN, 'h', r.h, 'dap', r.dap);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
nm = {'y1 u', 'y1 v', 'y1 w', 'ym (uu+vv)/2', 'ym ww', 'ym uw', 'ym vw'};  ap0 = slope(hs);
for is = 1:2
    S = zeros(NS, 7);  A = S;
    for id = 1:NS
        r = rs{is, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  S(id, :) = sum(r.dap(ks, :), 1) / ap0;  A(id, :) = sum(abs(r.dap(ks, :)), 1) / ap0;
    end
    fprintf('%d um/s: summed update of a''-hat over the scan / true slope (mean +- sd over seeds):\n', SP(is));
    for c = 1:7; fprintf('   %-14s %+6.2f +- %5.2f   (sum |.| %6.2f)\n', nm{c}, mean(S(:, c)), std(S(:, c)), mean(A(:, c))); end
end
f = figure('Color', 'w', 'Position', [20 20 1900 1000]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  cs = lines(7);
for is = 1:2
    r = rs{is, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  kd = k(1:5:end);  t = r.t(k) - tS;
    a1 = nexttile(tl, is);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', [0 0.25 0.85], 'LineWidth', 1.4);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-2 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  title(sprintf('%d um/s, seed 5', SP(is)));
    a2 = nexttile(tl, 2 + is);  hold on;  box on;  ks0 = k(r.t(k) >= tS);  t0 = r.t(ks0) - tS;
    for c = 1:7; plot(t0, cumsum(r.dap(ks0, c)) / ap0, 'Color', cs(c, :), 'LineWidth', 1.6, 'DisplayName', nm{c}); end
    yline(0, 'k-', 'HandleVisibility', 'off');  ylim([-3 2]);  ylabel('cumulative update / true slope');  xlabel('time since t_{scan}  (sec)');  if is == 1; legend('Location', 'southwest'); end
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_ap_split.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
