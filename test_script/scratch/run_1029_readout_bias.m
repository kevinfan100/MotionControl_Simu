% STATUS: SCRATCH -- 2026-10-29 (user): where does the +0.012..0.020 bias of the a_33 readout in the 40 um/s scan come from? The readout
%   is y_ww = ((w-hat' dp_r)^2 - C_n S_n,ww) / (C_T kappa_T), dp_r = high-passed measured tracking error. Hypothesis: a deterministic
%   (slow) normal component d of dp_r adds d^2 / (C_T kappa_T). Split w-hat' dp_r into slow = moving mean over Wm steps and fast = rest,
%   predicted bias = mean(slow^2) / (C_T kappa_T) - same in the 1 s before the scan (the moving mean also keeps some noise), compare with
%   the measured readout bias mean(y_ww - a_33 true) (scan - before). Current flow (regulation 1 s, scan from 12 s, ramp 0.5 s, kappa from
%   t_scan, tau_a 0.5 s), 3 s of scan. Arms: sphere 40 | sphere 10 | plane (theta_y 10) 40 um/s. Seeds 1-5.
%   Figure fig_1029_readout_bias.png (seed 5): rows = w-hat' dp_r [nm] raw + slow | readout bias running mean vs predicted d^2 term |
%   slow normal component vs the tangential components of dp_r (slow), columns = arms.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  Tr = 0.5;  tS = 12;  Tw = 3;  Wm = 40;  FS = 14;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tw, 'tag', 'rb');
arm = {'sphere 40 um/s', 'sphere 10 um/s', 'plane 40 um/s'};  NA = 3;  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o = rmfield(o, {'wall', 'hemi_r', 'hemi_void', 'hemi_start_deg', 'scan_plane_n'});  o.theta_y = 10; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'ym', r.ym, 'aN_tr', r.aN_tr, 'vdpr', r.vdpr, 'CTkap', r.CTkap, 'Cn_Sn', r.Cn_Sn);
end
for ia = 1:NA
    M = zeros(NS, 4);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  sl = movmean(r.vdpr(kk, 3), Wm);
        ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);  kb = kk(r.t(kk) >= tS - 0.9 & r.t(kk) < tS);
        meas = mean(r.ym(ks, 2) - r.aN_tr(ks)) - mean(r.ym(kb, 2) - r.aN_tr(kb));
        pred = (mean(sl(ks).^2) - mean(sl(kb).^2)) / r.CTkap;
        tot = (mean(r.vdpr(ks, 3).^2) - mean(r.vdpr(kb, 3).^2)) / r.CTkap - (mean(r.aN_tr(ks)) - mean(r.aN_tr(kb)));
        M(id, :) = [meas, pred, tot, 1e3 * Rr * sqrt(mean(sl(ks).^2))];
    end
    fprintf('%-15s| readout bias (scan - before) %s | predicted from the slow normal part %s | total (w dp_r)^2 excess minus true change %s | rms slow part %s nm\n', ...
            arm{ia}, mat2str(round(M(:, 1)', 4)), mat2str(round(M(:, 2)', 4)), mat2str(round(M(:, 3)', 4)), mat2str(round(M(:, 4)', 2)));
end
f = figure('Color', 'w', 'Position', [20 20 700 * NA 1150]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  t = r.t(k) - tS;  sl = movmean(r.vdpr(kk, :), Wm);  nmf = 1e3 * Rr;
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, nmf * r.vdpr(k, 3), 'Color', [0.7 0.78 1], 'LineWidth', 0.5, 'DisplayName', 'w-hat'' dp_r');
    plot(t, nmf * sl(k, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', sprintf('slow part (%d-step mean)', Wm));  yline(0, 'k-', 'HandleVisibility', 'off');
    ylim([-15 15]);  ylabel('normal  [nm]');  title(sprintf('%s, seed 5', arm{ia}));  if ia == 1; legend('Location', 'northwest'); end
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  e = r.ym(kk, 2) - r.aN_tr(kk);
    plot(t, movmean(e(k - kk(1) + 1), 400), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'readout bias (0.25 s mean)');
    plot(t, movmean(sl(k, 3).^2, 400) / r.CTkap, 'Color', cT, 'LineWidth', 1.8, 'DisplayName', 'slow^2 / (C_T \kappa_T)');  yline(0, 'k-', 'HandleVisibility', 'off');
    ylim([-0.01 0.04]);  ylabel('a_{33} readout - true');  if ia == 1; legend('Location', 'northwest'); end
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;
    plot(t, nmf * sl(k, 1), 'Color', cG, 'LineWidth', 1.4, 'DisplayName', 'u (slow)');  plot(t, nmf * sl(k, 2), 'Color', [0.6 0.3 0.7], 'LineWidth', 1.4, 'DisplayName', 'v (slow)');
    plot(t, nmf * sl(k, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'w (slow)');  yline(0, 'k-', 'HandleVisibility', 'off');
    ylim([-15 15]);  ylabel('slow part of dp_r  [nm]');  xlabel('time since t_{scan}  (sec)');  if ia == 1; legend('Location', 'northwest'); end
    for a = [a1 a2 a3]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_readout_bias.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
