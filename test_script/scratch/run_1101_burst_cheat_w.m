% STATUS: SCRATCH -- 2026-11-01 (user): reference arm: are the y1-w bursts triggered by real gap motion from the angle error. Sphere, current flow, 40 um/s, 5 seeds. Arms: as is | opts.cheat_w (regulation and scan along the
%   true normal, V-hat still estimated and moving). Pre-registered: bursts gone if every y1-w fall <= 0.7 slope and |z| <= 3.9 (frozen-plane limit). Sanity (wiring): cheat arm must show clearly smaller |a33 - a_d| and rms true a33 - a_d.
%   Console: fall, z, corr, a' < 0, a'/true, |a33 - a_d|, rms true a33 - a_d, V-hat turn rate, mean (true - est) a33 in the burst. Figure fig_1101_burst_cheat_w.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  ap0 = (1 / local_cp(hs + 1e-4) - 1 / local_cp(hs - 1e-4)) / 2e-4;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'hemi_start_deg', 10, 'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', 0.5, 'scan_len', 1e6, ...
              'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tw, 'tag', 'cw');
arm = {'sphere, as is', 'sphere, true normal for scan and regulation'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.cheat_w = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'K', r.k15w(:), 'nu', r.nuw(:), 'thx', r.thx, 'thy', r.thy, 'aN', r.aN, 'aN_tr', r.aN_tr, 'ap', r.apN, 'h', r.h);
end
W = round(0.2 / Ts);  rm = @(x) sqrt(mean(x.^2));  slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    M = NaN(NS, 10);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  y = r.K .* r.nu;  ks = kk(r.t(kk) >= tS);  ks = ks(isfinite(y(ks)));  tot = movsum(y(ks), [0 W - 1], 'Endpoints', 'discard');  [mn, i0] = min(tot);  kb = ks(i0):ks(i0) + W - 1;
        kw = ks(r.t(ks) >= tS + 0.5);  rate = sqrt(mean(diff(r.thx(kw)).^2 + diff(r.thy(kw)).^2)) / Ts;
        M(id, :) = [mn / ap0, sum(y(kb)) / sqrt(sum(y(kb).^2)), sum(r.K(kb) .* r.nu(kb)) / sqrt(sum(r.K(kb).^2) * sum(r.nu(kb).^2)), mean(r.ap(kw) < 0), median(r.ap(kw) ./ slope(r.h(kw))), mean(abs(r.aN_tr(kw) - ad)), rm(r.aN_tr(kw) - ad), rate, mean(r.aN_tr(kb) - r.aN(kb)), double(r.kend < numel(r.t))];
    end
    fprintf('%s\n   fall/slope %s | z %s | corr %s\n   a''<0 %s | a''/true %s | |a33-a_d| %s | rms true a33 - a_d %s\n   turn rate deg/s %s | (true-est) a33 in burst %s | contact/short %d\n', arm{ia}, mat2str(round(M(:, 1)', 2)), mat2str(round(M(:, 2)', 1)), mat2str(round(M(:, 3)', 2)), ...
            mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 2)), mat2str(round(M(:, 6)', 3)), mat2str(round(M(:, 7)', 3)), mat2str(round(M(:, 8)', 0)), mat2str(round(M(:, 9)', 3)), sum(M(:, 10)));
end
f = figure('Color', 'w', 'Position', [20 20 800 * NA 900]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  kd = k(1:5:end);  t = r.t(kd) - tS;  ks0 = k(r.t(k) >= tS);
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.aN_tr(kd), 'Color', cT, 'LineWidth', 1.2, 'DisplayName', 'a_{33} / a_{nom}');  plot(t, r.aN(kd), 'Color', cE, 'LineWidth', 1, 'DisplayName', arm{ia});  yline(ad, 'k--', 'HandleVisibility', 'off');  ylim([0 0.15]);  ylabel('a_{33} / a_{nom}');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, r.ap(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.4);  ylim([-3 3]);  ylabel('estimate / true slope');
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  plot(r.t(ks0) - tS, cumsum(r.K(ks0) .* r.nu(ks0)) / ap0, 'Color', [0.93 0.69 0.13], 'LineWidth', 1.5);  yline(0, 'k:');  ylim([-4 4]);  ylabel('cumulative y_1-w update / slope');  xlabel('time since t_{scan}  (sec)');
    for a = [a1 a2 a3]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_burst_cheat_w.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
