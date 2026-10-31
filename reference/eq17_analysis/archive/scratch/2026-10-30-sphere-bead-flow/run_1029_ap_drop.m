% STATUS: SCRATCH -- 2026-10-29 (user): why a'-hat_33 drops to 0.1-0.3 x true right at the scan start. Current flow: descent 1 s, wait 1 s,
%   regulation + scan (void sphere radius 15 um, 40 um/s, ramp 0.5 s, kappa-hat from t_scan, tau_a 0.5 s, angle P_fix 2 deg, report slope P_fix).
%   Arms: (A) current | (B) angle P_fix from t_scan + 0.3 s | (C) no angle P_fix | (D) regulation 1 s before the scan | (E) slope Q = 0 from
%   t_scan | (F) no scan (regulation + angle P_fix only). Seeds 1-5. Console: a'-hat / true median in 0.2 s before, 0-0.3 s, 0.3-0.8 s after
%   t_scan; a_33 min in the first 1 s; mean |a_33 - a_d| over the turn. Figure fig_1029_ap_drop.png: a'-hat ratio and a_33, seed 1.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  tS = 12;  Tr = 0.5;  Tsc = Lturn / spd + Tr / 2;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [1 1], 'T', tS + Tsc + 0.2, 'tag', 'ad');
arm = {'(A) current', '(B) angle P_{fix} from t_{scan} + 0.3 s', '(C) no angle P_{fix}', '(D) regulation 1 s earlier', '(E) slope Q = 0 from t_{scan}', '(F) no scan'};
NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    switch IA(q)
        case 2; o.q_theta_delay = 0.3;
        case 3; o.q_theta_off = true;
        case 4; o.post_wait = [0 1];
        case 5; o.ap_q_off_from = tS;
        case 6; o.scan_len = 0;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'apN', r.apN, 'h', r.h);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    M = NaN(NS, 6);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  q = @(t0, t1) median(r.apN(kk(r.t(kk) >= t0 & r.t(kk) < t1)) ./ slope(r.h(kk(r.t(kk) >= t0 & r.t(kk) < t1))), 'omitnan');
        k1 = kk(r.t(kk) >= tS & r.t(kk) < tS + 1);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tsc);
        M(id, :) = [r.contact, q(tS - 0.2, tS), q(tS, tS + 0.3), q(tS + 0.3, tS + 0.8), min(r.aN_tr(k1)), mean(abs(r.aN_tr(ks) - ad))];
    end
    fprintf('%-40s| contact %d | a''-hat/true: 0.2 s before %s | 0-0.3 s %s | 0.3-0.8 s %s | a33 min first 1 s %s | mean|a33 - a_d| %.3f\n', ...
            strrep(strrep(arm{ia}, '_{fix}', '_fix'), '_{scan}', '_scan'), sum(M(:, 1)), mat2str(round(M(:, 2)', 2)), mat2str(round(M(:, 3)', 2)), mat2str(round(M(:, 4)', 2)), ...
            mat2str(round(M(:, 5)', 3)), mean(M(:, 6)));
end
f = figure('Color', 'w', 'Position', [20 20 560 * NA 900]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 1};  k = find(r.t >= tS - 1 & r.t <= tS + 1.5 & (1:numel(r.t))' <= r.kend);  kd = k(1:5:end);  t = r.t(k) - tS;
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
    ylim([-1 2.5]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  title(sprintf('%s, seed 1', arm{ia}));
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);
    h3 = yline(ad, 'k--', 'LineWidth', 1.5);  ylim([0 0.15]);  ylabel(sprintf('%s_{33}', AB));  xlabel('time since t_{scan}  (sec)');
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 1.5]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_ap_drop.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
