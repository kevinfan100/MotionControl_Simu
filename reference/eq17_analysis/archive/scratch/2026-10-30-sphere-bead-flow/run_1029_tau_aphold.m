% STATUS: SCRATCH -- 2026-10-29 (user): flow "descent 1 s, wait 1 s, then regulation + scan" (40 um/s full turn of the void sphere radius 15 um,
%   kappa on, angle P_fix 2 deg, report slope P_fix). (a) fixes: tau_a 2 | 1 | 0.5 s, and tau_a 2 | 0.5 s with the regulation slope frozen at
%   a'-hat_33 of the scan start (opts.reg_ap_hold). (b) where the a'-hat_33 swing in the scan comes from: no scan (regulation only) | scan on a
%   plane | sphere scan with the true normal for regulation and scan (cheat_w, reference arm). Seeds 1-5.
%   Figures: fig_1029_tau_aphold.png (a), fig_1029_ap_swing_diag.png (b): per arm a-hat_33 / a_33 / a_d and a'-hat_33 / a'_33, seed 1.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  tS = 12;  Tsc = Lturn / spd;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, ...
              'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [1 1], 'T', tS + Tsc + 0.2, 'tag', 'ta');
arm = {'\tau_a 2 s', '\tau_a 1 s', '\tau_a 0.5 s', '\tau_a 2 s, a''-hat frozen', '\tau_a 0.5 s, a''-hat frozen', ...
       'no scan (regulation only)', 'scan on a plane', 'sphere, true normal (reference)'};
NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  ia = IA(q);
    switch ia
        case 2; o.tau_a = 1;
        case 3; o.tau_a = 0.5;
        case 4; o.reg_ap_hold = true;
        case 5; o.tau_a = 0.5;  o.reg_ap_hold = true;
        case 6; o.scan_len = 0;
        case 7; o = rmfield(o, {'wall', 'hemi_r', 'hemi_void', 'hemi_start_deg', 'scan_plane_n'});
        case 8; o.cheat_w = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'apN', r.apN, 'h', r.h);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:NA
    M = NaN(NS, 6);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tsc);  kq = ks(1:10:end);  qr = r.apN(kq) ./ slope(r.h(kq));
        if isempty(ks); continue; end
        M(id, :) = [r.contact, mean(abs(r.aN(ks) - ad)), mean(abs(r.aN_tr(ks) - ad)), median(qr, 'omitnan'), prctile(qr, 90) - prctile(qr, 10), mean(qr < 0)];
    end
    fprintf('%-34s| contact %d | mean|a-hat - a_d| %s (mean %.3f) | mean|a33 - a_d| %s (mean %.3f) | a''-hat/true median %s, 10-90 %% width %s, <0 frac %s\n', ...
            strrep(arm{ia}, '\tau', 'tau'), sum(M(:, 1), 'omitnan'), mat2str(round(M(:, 2)', 3)), mean(M(:, 2), 'omitnan'), mat2str(round(M(:, 3)', 3)), mean(M(:, 3), 'omitnan'), ...
            mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 2)), mat2str(round(M(:, 6)', 2)));
end
for fig = 1:2
    if fig == 1; L = 1:5;  nm = 'fig_1029_tau_aphold.png'; else; L = [1 6 7 8];  nm = 'fig_1029_ap_swing_diag.png'; end
    f = figure('Color', 'w', 'Position', [20 20 650 * numel(L) 900]);  tl = tiledlayout(2, numel(L), 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:numel(L)
        ia = L(j);  r = rs{ia, 1};  k = find(r.t >= tS - 1 & r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  kd = k(1:10:end);  t = r.t(k) - tS;
        a1 = nexttile(tl, j);  hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.2);
        h3 = yline(ad, 'k--', 'LineWidth', 1.5);  ylim([0 0.2]);  ylabel(sprintf('%s_{33}', AB));  title(sprintf('%s, seed 1', arm{ia}));
        if j == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
        a2 = nexttile(tl, numel(L) + j);  hold on;  box on;  plot(r.t(kd) - tS, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3);  yline(1, 'r-', 'LineWidth', 1.6);
        ylim([-1.5 3]);  ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  xlabel('time since t_{scan}  (sec)');
        for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tsc]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    end
    fn = fullfile(fo, nm);  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
