% STATUS: SCRATCH -- 2026-11-02 (user): does vertical motion help the angle. Fixed mean height 15 um, flat wall true (8,-8), seeds 1-5, 0-10 s. Arms: none | horizontal circle 3 um 8 Hz | vertical oscillation along w-hat 3 um 8 Hz (opts.wdither), 2 s ramp. [derivation] symmetric matrix: a_uw is read from F_u -> w and from F_w -> u alike, a single-direction oscillation carries half the force power of the circle per axis pair. Pre-registered: vertical rms at 10 s well below no motion; about as the circle if the F_w path works; near no motion = the F_w path is broken. Sanity: no motion 4 deg, circle 1.5 deg. Figure in Preview. Output fig_1102_vertical.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  Tx = 8;  Ty = -8;  cE = [0 0.25 0.85];  th = char(952);  cT = [0.85 0 0];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 50, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 've', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10);
[IA, ID] = ndgrid(1:3, 1:NS);  N = numel(IA);  rs = cell(N, 1);  nm = {'no motion', 'horizontal circle 3 \mum', 'vertical 3 \mum'};
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o.dither = [3 8 2];  o.dither_win = [0 50]; end;  if IA(q) == 3; o.wdither = [3 8 2];  o.wdither_win = [0 50]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'h', r.h(:), 'vF', r.vF, 'aN', r.aN(:), 'aN_tr', r.aN_tr(:), 'aT', r.aT(:), 'aT_tr', r.aT_tr(:));
end
f = figure('Color', 'w', 'Position', [40 40 1800 820], 'Visible', 'off');  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
for a = 1:3
    id = find(IA == a);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false)) - Tx;  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false)) - Ty;  P = sqrt(mean(cell2mat(cellfun(@(r) (r.sP(1:n, 1).^2 + r.sP(1:n, 2).^2) / 2, rs(id)', 'UniformOutput', false)), 2));  er = sqrt(mean((X.^2 + Y.^2) / 2, 2));  w = t >= 4;  r1 = rs{id(1)};
    fprintf('%s: data to %.2f s, rms F_u %.4f F_v %.4f F_w %.4f, h %.2f..%.2f um | ', nm{a}, t(end), rms(r1.vF(w, 1)), rms(r1.vF(w, 2)), rms(r1.vF(w, 3)), min(r1.h(w)) * Rr, max(r1.h(w)) * Rr);  for x = [2 4 6 8 10]; i = find(t >= x - 0.002, 1);  if isempty(i); i = n; end;  fprintf('t %2d rms %.2f sqrtP %.2f | ', x, er(i), P(i)); end;  ea = cellfun(@(r) mean(r.aT(r.t >= 8) - r.aT_tr(r.t >= 8)), rs(id));  en = cellfun(@(r) mean(r.aN(r.t >= 8) - r.aN_tr(r.t >= 8)), rs(id));  fprintf('a11 err %+.4f a33 err %+.4f\n', mean(ea), mean(en));
    k = (1:3:n)';  b = nexttile(tl);  hold(b, 'on');  plot(b, t(k), [X(k, :), Y(k, :)], 'LineWidth', 0.6);  yline(b, 0, 'Color', cT, 'LineWidth', 1.4);  ylim(b, [-12 12]);  ylabel(b, ['error of ' th '_x, ' th '_y  [deg]']);  title(b, nm{a});
    c = nexttile(tl);  hold(c, 'on');  h1 = plot(c, t(k), movmean(er(k), 11), 'Color', cE, 'LineWidth', 1.5);  h2 = plot(c, t(k), P(k), 'k--', 'LineWidth', 1.1);  ylim(c, [0 10]);  ylabel(c, 'rms error  [deg]');  xlabel(c, 'Time (sec)');  if a == 1; legend(c, [h1 h2], {'rms, 5 seeds', '\surdP'}, 'Location', 'northeast'); end
    for q = [b c]; xlim(q, [0 10]);  box(q, 'on');  set(q, 'FontSize', 13, 'FontWeight', 'bold', 'LineWidth', 1.2); end
end
fn = fullfile(fo, 'fig_1102_vertical.png');  exportgraphics(f, fn, 'Resolution', 110);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
