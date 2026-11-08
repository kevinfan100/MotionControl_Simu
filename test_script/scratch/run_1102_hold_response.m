% STATUS: SCRATCH -- 2026-11-02 (user): is the angle estimate in the 10 s hold a wrong response or the honest wandering of a weakly informed estimate. Flat wall, hold at 15 um, estimate starts at 0. A true (0, 0) | B true (8, -8) deg, 20 seeds each, same seeds, Q as in the scans.
%   Pre-registered: right response = mean of theta-hat moves to the truth within 2 SEM and rms error / sqrt P near 1 at 2, 5, 10 s; wrong response = mean stays away from the truth or ratio well above 1. Sanity: increment sd near sqrt(P/k). Cached in temp_hold_response.mat. MATLAB window 16. Output fig_1102_hold_response.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  NS = 20;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);
TH = [0 0; 8 -8];  [IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  cf = fullfile(fo, 'temp_hold_response.mat');
base = struct('h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', 0.07, 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'hr', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'T', 10);
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = ID(q);  o.theta_x = TH(IA(q), 1);  o.theta_y = TH(IA(q), 2);  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'dth', r.dth, 'kend', r.kend);
    end
    save(cf, 'rs', 'IA', 'ID', '-v7.3');
end
n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs));  t = rs{1}.t(1:n);  tq = [1 2 5 10];  if isempty(gcp('nocreate')); end
f = figure(16);  clf(f);  set(f, 'Color', 'w', 'Position', [50 60 1800 860], 'Visible', 'on', 'Name', 'angle response in the hold, 20 seeds');  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  tt = {'x', 'y'};
for a = 1:2
    id = find(IA == a);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false));  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false));  Px = cell2mat(cellfun(@(r) r.sP(1:n, 1), rs(id)', 'UniformOutput', false));  Py = cell2mat(cellfun(@(r) r.sP(1:n, 2), rs(id)', 'UniformOutput', false));
    for c = 1:2
        if c == 1; D = X; P = Px; else; D = Y; P = Py; end;  tru = TH(a, c);  k = (1:8:n)';
        for j = 1:numel(tq); i = find(t >= tq(j), 1);  if isempty(i); i = n; end;  e = D(i, :) - tru;  fprintf('true (%g, %g) theta_%s t %2d s: mean %+.2f (truth %+g), SEM %.2f, sd %.2f, rms err %.2f, sqrt P %.2f, ratio %.2f\n', TH(a, 1), TH(a, 2), tt{c}, tq(j), mean(D(i, :)), tru, std(D(i, :)) / sqrt(NS), std(D(i, :)), rms(e), mean(P(i, :)), rms(e) / mean(P(i, :))); end
        ax = nexttile(tl);  hold(ax, 'on');  plot(ax, t(k), D(k, :), 'Color', [0.75 0.8 0.92], 'LineWidth', 0.4);  h2 = plot(ax, t(k), mean(D(k, :), 2), 'Color', cE, 'LineWidth', 2);  h1 = plot(ax, t(k), tru * ones(size(k)), 'Color', cT, 'LineWidth', 1.6);  h3 = plot(ax, t(k), mean(D(k, :), 2) + mean(P(k, :), 2), 'k--');  plot(ax, t(k), mean(D(k, :), 2) - mean(P(k, :), 2), 'k--');  ylabel(ax, [th '_' tt{c} '  [deg]']);  xlim(ax, [0 10]);  ylim(ax, [-20 20]);  box(ax, 'on');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  if a == 2; xlabel(ax, 'Time (sec)'); end
        if a == 1 && c == 1; legend(ax, [h1 h2 h3], {'True', 'Mean of 20 seeds', '\pm\surdP'}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    end
end
exportgraphics(f, fullfile(fo, 'fig_1102_hold_response.png'), 'Resolution', 120);
