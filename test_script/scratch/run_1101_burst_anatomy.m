% STATUS: SCRATCH -- 2026-11-01 (user): anatomy of the y1-w burst of a'-hat_33. Fastest 0.2 s fall of the y1-w update in t_scan .. +3 s, vs the 0.5 s before it. Identity: sum K nu = N rms(K) rms(nu) corr(K, nu)
%   (K = K(15,w), nu = nu_w). Pre-registered: the factor with the largest burst / before ratio names the route. z = sum K nu / sqrt(sum (K nu)^2) replaces the estimated chance level.
%   Current flow, 40 um/s, 5 seeds. Arms: plane V-hat frozen | plane V-hat moving (kappa off) | sphere as is. Diverging seeds: analysis stops 0.5 s before the first non-finite value.
%   Also ratios of rms F_w, rms F_u, |P(a', a_33)|, V-hat turn rate, and min a-hat_33, min true a_33, min eig of A-hat in the burst. Figure fig_1101_burst_anatomy.png (seed 5, +-0.5 s).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 13;  cE = [0 0.25 0.85];  cT = [0.85 0 0];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  ap0 = (1 / local_cp(hs + 1e-4) - 1 / local_cp(hs - 1e-4)) / 2e-4;
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'ba');
arm = {'plane, V-hat frozen', 'plane, V-hat moving', 'sphere, as is'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 1; o.freeze_angle = true; end
    if IA(q) == 3
        o.theta_y = 0;  o.th0_deg = [0 0];  o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.scan_plane_n = [0 1 0];  o.kappa_state = true;  o.kappa_from_scan = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'K', r.k15w(:), 'nu', r.nuw(:), 'Fu', r.vF(:, 1), 'Fw', r.vF(:, 3), 'P', r.B15(:, 8), 'thx', r.thx, 'thy', r.thy, 'aN', r.aN, 'aT', r.aT, 'aN_tr', r.aN_tr, 'me', r.mineig, 'ap', r.apN, 'kap', r.kap(:));
end
W = round(0.2 / Ts);  Wb = round(0.5 / Ts);  rm = @(x) sqrt(mean(x.^2));  B = cell(NA, NS);
for ia = 1:NA
    M = NaN(NS, 16);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  y = r.K .* r.nu;  bad = find(~isfinite(y(1:r.kend)) | ~isfinite(r.thy(1:r.kend)) | ~isfinite(r.aN(1:r.kend)), 1);
        kl = r.kend;  if ~isempty(bad); kl = bad - Wb; end
        ks = kk(r.t(kk) >= tS & kk <= kl);  if numel(ks) < 2 * W + Wb; continue; end
        tot = movsum(y(ks), [0 W - 1], 'Endpoints', 'discard');  [mn, i0] = min(tot);  kb = ks(i0):ks(i0) + W - 1;  kp = max(kb(1) - Wb, 1):kb(1) - 1;  B{ia, id} = kb(1);
        rate = @(g) sqrt(mean((diff(r.thx(g)).^2 + diff(r.thy(g)).^2))) / Ts;  cc = @(g) sum((r.K(g) - 0) .* r.nu(g)) / sqrt(sum(r.K(g).^2) * sum(r.nu(g).^2));
        M(id, :) = [mn / ap0, sum(y(kb)) / sqrt(sum(y(kb).^2)), rm(r.K(kb)) / rm(r.K(kp)), rm(r.nu(kb)) / rm(r.nu(kp)), cc(kb), cc(kp), rm(r.Fw(kb)) / rm(r.Fw(kp)), rm(r.Fu(kb)) / rm(r.Fu(kp)), ...
                    mean(abs(r.P(kb))) / mean(abs(r.P(kp))), rate(kb), rate(kp), min(r.aN(kb)), min(r.aN_tr(kb)), min(r.me(kb)), r.t(kb(1)) - tS, numel(ks) * Ts];
    end
    fprintf('%s\n   fall/slope %s | z %s | t %s\n   ratio rms K %s | rms nu %s | corr(K,nu) burst %s, before %s\n   ratio rms F_w %s | rms F_u %s | |P(a'',a33)| %s\n   turn rate deg/s burst %s, before %s\n   min a-hat33 %s | min true a33 %s | min eig A-hat %s\n', ...
            arm{ia}, mat2str(round(M(:, 1)', 2)), mat2str(round(M(:, 2)', 1)), mat2str(round(M(:, 15)', 2)), mat2str(round(M(:, 3)', 2)), mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 2)), mat2str(round(M(:, 6)', 2)), ...
            mat2str(round(M(:, 7)', 2)), mat2str(round(M(:, 8)', 2)), mat2str(round(M(:, 9)', 2)), mat2str(round(M(:, 10)', 1)), mat2str(round(M(:, 11)', 1)), mat2str(round(M(:, 12)', 3)), mat2str(round(M(:, 13)', 3)), mat2str(round(M(:, 14)', 3)));
end
f = figure('Color', 'w', 'Position', [20 20 620 * NA 1300]);  tl = tiledlayout(5, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  k0 = B{ia, 5};  k = (k0 - 800):min(k0 + 1200, r.kend);  t = (r.t(k) - r.t(k0));  nm = 1e3 * Rr;
    a1 = nexttile(tl, ia);  plot(t, cumsum(r.K(k) .* r.nu(k)) / ap0, 'Color', cE, 'LineWidth', 1.4, 'DisplayName', arm{ia});  ylabel('cum. K \nu / slope');  legend('Location', 'northoutside');
    a2 = nexttile(tl, NA + ia);  plot(t, r.K(k), 'Color', cE, 'LineWidth', 0.8);  ylabel('K(a'', w)');
    a3 = nexttile(tl, 2 * NA + ia);  plot(t, nm * r.nu(k), 'Color', [0.7 0.78 1], 'LineWidth', 0.5);  hold on;  plot(t, nm * movmean(r.nu(k), 40), 'Color', cE, 'LineWidth', 1.3);  ylim([-20 20]);  ylabel('\nu_w [nm]');
    a4 = nexttile(tl, 3 * NA + ia);  plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.1);  hold on;  plot(t, r.aN(k), 'Color', cE, 'LineWidth', 0.9);  yline(ad, 'k--');  ylim([0 0.15]);  ylabel('a_{33} / a_{nom}');
    a5 = nexttile(tl, 4 * NA + ia);  plot(t, r.Fw(k), 'Color', cE, 'LineWidth', 0.5);  ylim([-0.3 0.3]);  ylabel('F_w  [R]');  xlabel('time from burst start  (sec)');
    for a = [a1 a2 a3 a4 a5]; box(a, 'on');  xline(a, 0, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');  xline(a, W * Ts, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');  xlim(a, [t(1) t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_burst_anatomy.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
