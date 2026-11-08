% STATUS: SCRATCH -- 2026-11-03 (user): is the gate needed when the gains start at the true values. Flat wall true (8,-8), seeds 1-5, 0-12 s (hold 10 s, descent 1 s, 1 s at the bottom). Arms: A gate on, seed start (default) | B no gate, seed start | C no gate, gains start at the true values (opts.gain0 numeric, reference arm). Pre-registered: C without blow-up and no worse than A means the gate only guards the start-up of the gains. Sanity: A as run_1102_gate_off.m arm C. Figure in Preview. Output fig_1102_gate_true_init.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  Tx = 8;  Ty = -8;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  [cp0, cn0] = calc_correction_functions(15 / Rr);  gt0 = [1 / cp0, 1 / cn0];
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'gi', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 12);
fprintf('true gains at 15 um: a_11 %.4f a_33 %.4f; seed start: a_11 %.4f a_33 %.4f\n', gt0, 1 - (9/16) / (15 / Rr), 1 - (9/8) / (15 / Rr));
[IA, ID] = ndgrid(1:3, 1:NS);  N = numel(IA);  rs = cell(N, 1);  nm = {'A gate on, seed start', 'B no gate, seed start', 'C no gate, true start'};
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if IA(q) == 1; o.gap_gate = 2;  o.gate_one = true; end;  if IA(q) == 3; o.gain0 = gt0; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'g', r.dgap(:), 'sP', r.sPth);
end
f = figure('Color', 'w', 'Position', [40 40 1700 560], 'Visible', 'off');  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for a = 1:3
    id = find(IA == a);  last = cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id));  ok = id(last >= numel(rs{id(1)}.t) - 2);  gmin = cellfun(@(r) min(r.g(r.t > 0.05 & isfinite(r.g))), rs(id));  g05 = cellfun(@(r) r.g(find(r.t >= 0.05, 1)), rs(id));
    fprintf('%s: blew up %d/%d | g-hat at 0.05 s %s, min %s', nm{a}, numel(id) - numel(ok), numel(id), mat2str(round(g05', 3)), mat2str(round(gmin', 3)));
    if isempty(ok); fprintf('\n'); continue; end;  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(ok)));  t = rs{ok(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(ok)', 'UniformOutput', false)) - Tx;  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(ok)', 'UniformOutput', false)) - Ty;  er = sqrt(mean((X.^2 + Y.^2) / 2, 2));
    for x = [2 5 10 11 12]; i = find(t >= x - 0.002, 1);  if isempty(i); i = n; end;  fprintf(' | t %d rms %.2f', x, er(i)); end;  fprintf('\n');
    b = nexttile(tl);  hold(b, 'on');  k = (1:3:n)';  plot(b, t(k), [X(k, :), Y(k, :)], 'LineWidth', 0.6);  yline(b, 0, 'r', 'LineWidth', 1.4);  ylim(b, [-12 12]);  xlim(b, [0 12]);  title(b, nm{a});  xlabel(b, 'Time (sec)');  ylabel(b, 'error of \theta_x, \theta_y  [deg]');  box(b, 'on');  set(b, 'FontSize', 13, 'FontWeight', 'bold', 'LineWidth', 1.2);
end
fn = fullfile(fo, 'fig_1102_gate_true_init.png');  exportgraphics(f, fn, 'Resolution', 110);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
