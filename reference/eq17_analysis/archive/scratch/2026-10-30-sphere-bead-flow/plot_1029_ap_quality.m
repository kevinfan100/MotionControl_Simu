% STATUS: SCRATCH -- 2026-10-29 (user): quality of a'-hat_33 in the three approaches of the fast full turn (40 um/s, + kappa, angle P_fix 2),
%   void sphere radius 15 um: (1) regulation 10-17 s then scan, slope P_fix law L | (2) descent 10 s then scan, L | (3) descent 10 s with the
%   1019-report slope P_fix, L from the scan. Seeds 1-3 (console: ratio a'-hat / true slope, mean and spread, in the last 2 s of the
%   approach and in the scan); seed 1 drawn: a'_33 (true red) and a'-hat_33 (blue) from 10 s to the end, log axis.
%   Output test_results/temp_3d_wsonly/fig_1029_ap_quality.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'aq');
arm = {'(1) regulation 10-17 s, L', '(2) descent 10 s, L', '(3) descent 10 s, report P_{fix} \rightarrow L'};  tSa = [17 20 20];
[IA, ID] = ndgrid(1:3, 1:3);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', 9); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  tS = tSa(IA(q));  o.q_theta_from = tS;  o.T = tS + Lturn / spd + 0.2;
    if IA(q) == 1; o.t_scan = tS; else; o.descent_T = 10;  o.descent_v = (15 - hs * Rr) / 10; end
    if IA(q) == 3; o.ap_pfix_law_from_scan = true; end
    R = run_t3_xyz_fixed(o);  rs{q} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
for ia = 1:3
    tS = tSa(ia);  Ra = zeros(3, 4);
    for id = 1:3
        r = rs{ia, id};  kk = (1:r.kend)';  ka = kk(r.t(kk) >= tS - 2 & r.t(kk) < tS);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Lturn / spd & isfinite(r.apN(kk)));
        ka = ka(1:10:end);  ks = ks(1:10:end);  qa = r.apN(ka) ./ slope(r.h(ka));  qs = r.apN(ks) ./ slope(r.h(ks));
        Ra(id, :) = [median(qa), prctile(qa, 90) - prctile(qa, 10), median(qs), prctile(qs, 90) - prctile(qs, 10)];
    end
    fprintf('%-42s: a''-hat/true in the last 2 s before the scan: median %s (10-90%% spread %s) | during the scan: median %s (spread %s)\n', ...
            strrep(strrep(arm{ia}, '\rightarrow', '->'), '_{fix}', '_fix'), mat2str(round(Ra(:, 1)', 2)), mat2str(round(Ra(:, 2)', 2)), mat2str(round(Ra(:, 3)', 2)), mat2str(round(Ra(:, 4)', 2)));
end
f = figure('Color', 'w', 'Position', [20 20 2400 700]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:3
    r = rs{ia, 1};  tS = tSa(ia);  k = find(r.t >= 10 & r.t <= tS + Lturn / spd & (1:numel(r.t))' <= r.kend);  kd = k(1:10:end);
    ax = nexttile(tl);  hold on;  box on;
    h1 = plot(r.t(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.4);  h2 = plot(r.t(k), max(r.apN(k), 1e-4), 'Color', cE, 'LineWidth', 1.1);
    xline(tS, '-.', 'Color', cGr, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    set(ax, 'YScale', 'log');  ylim([1e-3 5]);  xlim([10 tS + Lturn / spd]);  xlabel('Time (sec)');  ylabel(sprintf('%s''_{33}', AB));  title(sprintf('%s, seed 1', arm{ia}));
    if ia == 1; legend([h1 h2], {sprintf('%s''_{33} (true, at the true gap)', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'southeast'); end
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1029_ap_quality.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
