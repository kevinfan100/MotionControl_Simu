% STATUS: SCRATCH -- 2026-11-02 (user): the fair form of gate off before the scan. No gate, denominator max(g-hat, .05) (.05 below the true .083; g-hat starts at 0, so plain gate off blows up at t = 0, run_1102_gate_off.m arms B, D, not a test of the descent). Flat wall, 0-12 s, a_d .07, 20 seeds. E true (0,0) | F true (8,-8), against arms A, C (gate on).
%   Pre-registered: no worse than the gate at 10-12 s means the gate is not needed; rms at 11 s lower means it was holding the descent back. Cached in temp_gate_floor.mat. MATLAB window 22. Output fig_1102_gate_floor.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 20;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'go', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 12);
ARM = [0 0 0; 8 -8 0];  [IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  cf = fullfile(fo, 'temp_gate_floor.mat');
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = ID(q);  o.theta_x = ARM(IA(q), 1);  o.theta_y = ARM(IA(q), 2);  if ARM(IA(q), 3); o.gap_gate = 2;  o.gate_one = true; else; o.den_mode = 'both';  o.den_floor = 0.05; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'g', r.dgap(:), 'gate', r.gate(:), 'h', r.h(:), 'contact', r.contact);
    end
    save(cf, 'rs', 'IA', 'ID', '-v7.3');
end
tq = [10 10.5 10.75 11 11.5 12];  nm = {'E true 0, no gate, floor', 'F true (8,-8), no gate, floor'};  f = figure(22);  clf(f);  set(f, 'Color', 'w', 'Position', [50 60 1800 860], 'Visible', 'on', 'Name', 'gate off before the scan, 20 seeds');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'rowmajor');
for a = 1:2
    id = find(IA == a);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false));  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false));  Tx = ARM(a, 1);  Ty = ARM(a, 2);  ct = sum(cellfun(@(r) r.contact, rs(id)));  gm = min(cellfun(@(r) min(r.g(r.t >= 10 & r.t < 11)), rs(id)));  nf = sum(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)) < numel(rs{id(1)}.t) - 2);
    fprintf('%s: data to %.2f s, runs ended early %d, contact %d, min g-hat in descent %.3f\n', nm{a}, t(end), nf, ct, gm);
    for j = 1:numel(tq); i = find(t >= tq(j), 1);  if isempty(i); continue; end;  ex = X(i, :) - Tx;  ey = Y(i, :) - Ty;  fprintf('  t %5.2f s: mean err x %+.2f y %+.2f (SEM %.2f %.2f), rms %.2f\n', t(i), mean(ex), mean(ey), std(ex) / sqrt(NS), std(ey) / sqrt(NS), sqrt(mean([ex ey].^2))); end
    k = find(t >= 8);  k = k(1:4:end);  ax = nexttile(tl);  hold(ax, 'on');  plot(ax, t(k), X(k, :) - Tx, 'Color', [0.75 0.8 0.92], 'LineWidth', 0.4);  plot(ax, t(k), Y(k, :) - Ty, 'Color', [0.92 0.8 0.75], 'LineWidth', 0.4);  h1 = plot(ax, t(k), mean(X(k, :) - Tx, 2), 'Color', cE, 'LineWidth', 2);  h2 = plot(ax, t(k), mean(Y(k, :) - Ty, 2), 'Color', cT, 'LineWidth', 2);  yline(ax, 0, 'k-');  xline(ax, 10, 'k:', 'HandleVisibility', 'off');  xline(ax, 11, 'k:', 'HandleVisibility', 'off');  ylim(ax, [-15 15]);  xlim(ax, [8 12]);  ylabel(ax, ['error of ' th ' [deg]']);  title(ax, nm{a});  box(ax, 'on');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  if a > 0; xlabel(ax, 'Time (sec)'); end;  if a == 1; legend(ax, [h1 h2], {[th '_x mean'], [th '_y mean']}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
end
exportgraphics(f, fullfile(fo, 'fig_1102_gate_floor.png'), 'Resolution', 120);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
