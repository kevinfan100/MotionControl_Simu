% STATUS: SCRATCH -- 2026-11-03 (user): 1 s against 5 s descent, seeds 1-5, flat wall true (8,-8), 1 s at the bottom, no scan. Time axis from the bottom (descent end). Rows: rms angle error of w-hat, mean a_33 estimate minus true. Sanity: seed 1 as plot_1103_descent_1v5.m. Figure in Preview. Output fig_1103_descent_1v5_seeds.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  Tx = 8;  Ty = -8;  TD = [1 5];  NS = 5;  FS = 15;  AB = char(257);  hc = char(770);  cc = [0 0 0; 0.85 0 0];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'd5s', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  rs = cell(N, 1);  if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  o.descent_T = TD(IA(q));  o.descent_v = (15 - hs * Rr) / TD(IA(q));  o.T = 10 + TD(IA(q)) + 1;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'aN', r.aN(:), 'aN_tr', r.aN_tr(:));
end
f = figure('Color', 'w', 'Position', [40 40 1500 900], 'Visible', 'off');  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  a1 = nexttile(tl);  hold(a1, 'on');  a2 = nexttile(tl);  hold(a2, 'on');  hl = gobjects(2, 1);
for j = 1:2
    id = find(IA == j);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n) - (10 + TD(j));  E = zeros(n, NS);  D = zeros(n, NS);
    for s = 1:NS; r = rs{id(s)};  E(:, s) = acosd(min(sum(local_w(r.thx(1:n), r.thy(1:n)) .* local_w(Tx * ones(n, 1), Ty * ones(n, 1)), 2), 1)).^2;  D(:, s) = r.aN(1:n) - r.aN_tr(1:n); end
    er = sqrt(mean(E, 2));  dm = mean(D, 2);  fprintf('descent %g s:', TD(j));  for x = [-TD(j) -TD(j) / 2 0 0.5 1]; i = find(t >= x - 1e-6, 1);  if isempty(i); i = n; end;  fprintf(' | t %+.1f s err %.2f, a33 est-true %+.3f', x, er(i), dm(i)); end;  fprintf('\n');
    k = (1:3:n)';  hl(j) = plot(a1, t(k), movmean(er(k), 11), 'Color', cc(j, :), 'LineWidth', 1.5);  plot(a2, t(k), movmean(dm(k), 11), 'Color', cc(j, :), 'LineWidth', 1.5);
end
yline(a2, 0, 'k:', 'HandleVisibility', 'off');  ylabel(a1, ['angle error of w' hc ', rms 5 seeds  [deg]']);  ylabel(a2, [AB '_{33} estimate - true, mean 5 seeds']);  xlabel(a2, 'time from the bottom  [s]');  ylim(a1, [0 8]);  ylim(a2, [-0.05 0.12]);
for a = [a1 a2]; xlim(a, [-7 1]);  box(a, 'on');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end;  legend(a1, hl, {'descent 1 s', 'descent 5 s'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
fn = fullfile(fo, 'fig_1103_descent_1v5_seeds.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open -a Preview "' fn '"']);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
