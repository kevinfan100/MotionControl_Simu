% STATUS: SCRATCH -- 2026-10-29 (user): fast full turn (40 um/s, + kappa, angle P_fix 2 deg) of the void sphere, descent then scan + P
%   regulation; arms: descent 5 s, tau_a 2 s (as fig_1029_desc_scan) | descent 10 s, tau_a 2 s | descent 10 s, tau_a 1 s |
%   descent 10 s, tau_a 0.5 s. Seeds 1-3 (console); seed 1 drawn: a_33 (true red), a-hat_33 (blue), a_d vs time since the scan start, and
%   the gap. Output test_results/temp_3d_wsonly/fig_1029_desc10.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
arms = {'descent 5 s, \tau_a 2 s', 5, 2; 'descent 10 s, \tau_a 2 s', 10, 2; 'descent 10 s, \tau_a 1 s', 10, 1; 'descent 10 s, \tau_a 0.5 s', 10, 0.5};  na = size(arms, 1);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'd10');
[IA, ID] = ndgrid(1:na, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', 12); end
parfor q = 1:numel(IA)
    a = arms(IA(q), :);  o = base;  o.seeds = ID(q);  Td = a{2};  tS = 10 + Td;  o.descent_T = Td;  o.descent_v = (15 - hs * Rr) / Td;  o.tau_a = a{3};
    o.q_theta_from = tS;  o.T = tS + Lturn / spd + 0.2;  R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
f = figure('Color', 'w', 'Position', [20 20 2500 900]);  tl = tiledlayout(2, na, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:na
    tS = 10 + arms{ia, 2};
    for id = 1:3
        r = rs{ia, id};  ks = find(r.t >= tS & r.t <= tS + Lturn / spd & (1:numel(r.t))' <= r.kend);  k2 = ks(ks >= ks(1) + round(0.25 * numel(ks)));  kS = ks(1);
        fprintf('%-28s seed %d: contact %d | at the scan start a33 %.3f a-hat %.3f gap %.2f | gap min %.2f mean %.2f max %.2f um | after 1/4: a33 - a_d %+.3f, rms(a33 - a_d) %.3f\n', ...
                strrep(arms{ia, 1}, '\tau', 'tau'), id, r.contact, r.aN_tr(kS), r.aN(kS), r.h(kS) * Rr, min(r.h(ks)) * Rr, mean(r.h(ks)) * Rr, max(r.h(ks)) * Rr, ...
                mean(r.aN_tr(k2)) - ad, rms(r.aN_tr(k2) - ad));
    end
    r = rs{ia, 1};  ks = find(r.t >= tS & r.t <= tS + Lturn / spd & (1:numel(r.t))' <= r.kend);  t = r.t(ks) - tS;
    ax = nexttile(tl, ia);  hold on;  box on;
    h1 = plot(t, r.aN_tr(ks), 'Color', cT, 'LineWidth', 2);  h2 = plot(t, r.aN(ks), 'Color', cE, 'LineWidth', 1.2);  h3 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 0.35]);  ylabel(sprintf('%s_{33}', AB));  title(sprintf('%s, seed 1', arms{ia, 1}));
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    a2 = nexttile(tl, na + ia);  hold on;  box on;  plot(t, r.h(ks) * Rr, 'Color', cE, 'LineWidth', 1.2);  yline(hs * Rr, 'k--', 'LineWidth', 1.5);  yline(Rr, 'r-', 'LineWidth', 1.5);
    ylim([2.1 3.4]);  ylabel('gap  [\mum]');  xlabel('time since t_{scan}  (sec)');
    for a = [ax a2]; set(a, 'XLim', [0 t(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_desc10.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
