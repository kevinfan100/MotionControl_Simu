% STATUS: SCRATCH -- 2026-11-02 (user): plateau or noise. Circle 3 um 8 Hz at a fixed point, flat wall true (8,-8), seeds 1-5, 40 s. Noise from the data falls as 1 / sqrt(time): rms 1.5 deg at 10 s gives about .75 at 40 s, and each seed moves. A bias stays near 1.5 deg with each seed at the same value. Figure in Preview. Output fig_1102_circle_long.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  Tx = 8;  Ty = -8;  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 50, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'cl', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 40, 'dither', [3 8 2], 'dither_win', [0 50]);
rs = cell(NS, 1);  if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:NS
    o = base;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth);
end
n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs));  t = rs{1}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs', 'UniformOutput', false)) - Tx;  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs', 'UniformOutput', false)) - Ty;  P = sqrt(mean(cell2mat(cellfun(@(r) (r.sP(1:n, 1).^2 + r.sP(1:n, 2).^2) / 2, rs', 'UniformOutput', false)), 2));  er = sqrt(mean((X.^2 + Y.^2) / 2, 2));
for x = [5 10 20 30 40]; i = find(t >= x - 0.002, 1);  if isempty(i); i = n; end;  fprintf('t %2d s: rms %.2f, sqrt P %.2f, ratio %.2f | err x %s y %s\n', x, er(i), P(i), er(i) / P(i), mat2str(round(X(i, :), 1)), mat2str(round(Y(i, :), 1))); end
f = figure('Color', 'w', 'Position', [40 40 1600 860], 'Visible', 'off');  tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  k = (1:8:n)';
a = nexttile(tl);  plot(a, t(k), X(k, :), 'LineWidth', 0.8);  yline(a, 0, 'Color', cT, 'LineWidth', 1.4);  ylim(a, [-5 5]);  ylabel(a, ['error of ' th '_x  [deg]']);
b = nexttile(tl);  plot(b, t(k), Y(k, :), 'LineWidth', 0.8);  yline(b, 0, 'Color', cT, 'LineWidth', 1.4);  ylim(b, [-5 5]);  ylabel(b, ['error of ' th '_y  [deg]']);
c = nexttile(tl);  hold(c, 'on');  h1 = plot(c, t(k), movmean(er(k), 25), 'Color', cE, 'LineWidth', 1.5);  h2 = plot(c, t(k), P(k), 'k--', 'LineWidth', 1.1);  i10 = find(t >= 10, 1);  h3 = plot(c, t(k), er(i10) * sqrt(10 ./ max(t(k), 10)), 'Color', cT, 'LineWidth', 1.2);  ylim(c, [0 4]);  ylabel(c, 'rms error  [deg]');  xlabel(c, 'Time (sec)');  legend(c, [h1 h2 h3], {'rms, 5 seeds', '\surdP', '1/\surdt from 10 s'}, 'Location', 'northeast');
for q = [a b c]; xlim(q, [2 40]);  box(q, 'on');  set(q, 'FontSize', 13, 'FontWeight', 'bold', 'LineWidth', 1.2); end
fn = fullfile(fo, 'fig_1102_circle_long.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
