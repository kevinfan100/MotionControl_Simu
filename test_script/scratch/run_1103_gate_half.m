% STATUS: SCRATCH -- 2026-11-03 (user): (1) gate only at the start: gate on, then always open from 0.5, 1, 2 s (opts.gate_off_from, denominator max(g-hat, .05)), against gate on throughout; flat wall true (8,-8), seeds 1-5, 0-12 s. (2) scan right after the descent (post_wait 0) against 1 s at the bottom, flat wall true (8,-8), 3 lines of 36 um, seeds 1-5, current settings (kappa-hat and angle Q from the scan start).
%   Pre-registered: (1) no blow-up and same rms as the gate means the gate is needed only for the first fraction of a second. (2) angle error in the first 2 s of the scan and min gap; the immediate scan starts with about 3 deg instead of 1.3. Figures in Preview. Outputs fig_1103_gate_half.png, fig_1103_scan_now.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  Tx = 8;  Ty = -8;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'gh', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 12);
GO = [inf 0.5 1 2];  NG = numel(GO);  [IA, ID] = ndgrid(1:NG + 2, 1:NS);  N = numel(IA);  rs = cell(N, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  a = IA(q);
    if a <= NG; if isfinite(GO(a)); o.gate_off_from = GO(a);  o.den_mode = 'both';  o.den_floor = 0.05; end
    else; o.raster_local = [36 1 3 0.05];  o.T = 12 + 1.2 * 3 * 36 / 40 + 1;  if a == NG + 2; o.post_wait = [0 0]; end; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'g', r.dgap(:), 'h', r.h(:), 'tsc', r.tsc, 'te', r.rs_tend, 'contact', r.contact);
end
f = figure('Color', 'w', 'Position', [40 40 1900 520], 'Visible', 'off');  tl = tiledlayout(1, NG, 'TileSpacing', 'compact', 'Padding', 'compact');
for a = 1:NG
    id = find(IA == a);  last = cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id));  ok = id(last >= numel(rs{id(1)}.t) - 2);  tt = sprintf('gate open from %g s', GO(a));  if ~isfinite(GO(a)); tt = 'gate throughout'; end
    fprintf('%-22s blew up %d/%d, g-hat min after the gate %s', tt, NS - numel(ok), NS, mat2str(round(cellfun(@(r) min(r.g(r.t >= min(GO(a), 12) & isfinite(r.g))), rs(id))', 3)));
    if isempty(ok); fprintf('\n'); continue; end;  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(ok)));  t = rs{ok(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(ok)', 'UniformOutput', false)) - Tx;  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(ok)', 'UniformOutput', false)) - Ty;  er = sqrt(mean((X.^2 + Y.^2) / 2, 2));
    for x = [2 5 10 11 12]; i = find(t >= x - 0.002, 1);  if isempty(i); i = n; end;  fprintf(' | t %d rms %.2f', x, er(i)); end;  fprintf('\n');
    b = nexttile(tl);  hold(b, 'on');  k = (1:3:n)';  plot(b, t(k), [X(k, :), Y(k, :)], 'LineWidth', 0.6);  yline(b, 0, 'r', 'LineWidth', 1.4);  ylim(b, [-12 12]);  xlim(b, [0 12]);  title(b, sprintf('%s, blew up %d/%d', tt, NS - numel(ok), NS));  xlabel(b, 'Time (sec)');  ylabel(b, 'error of \theta_x, \theta_y  [deg]');  box(b, 'on');  set(b, 'FontSize', 12, 'FontWeight', 'bold', 'LineWidth', 1.2);
end
fn = fullfile(fo, 'fig_1103_gate_half.png');  exportgraphics(f, fn, 'Resolution', 110);  close(f);
nm = {'1 s at the bottom (now)', 'scan right after the descent'};  f2 = figure('Color', 'w', 'Position', [60 60 1500 560], 'Visible', 'off');  a2 = axes(f2);  hold(a2, 'on');  cc = [0 0 0; 0.85 0 0];  hl = gobjects(2, 1);
for j = 1:2
    id = find(IA == NG + j);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  E = zeros(n, NS);  for s = 1:NS; r = rs{id(s)};  E(:, s) = acosd(min(sum(local_w(r.thx(1:n), r.thy(1:n)) .* local_w(Tx * ones(n, 1), Ty * ones(n, 1)), 2), 1)); end;  er = sqrt(mean(movmean(E.^2, 81, 1), 2));  tsc = rs{id(1)}.tsc;  fin = cellfun(@(r) isfinite(r.te) && ~r.contact, rs(id));  mg = cellfun(@(r) 1e3 * (min(r.h(r.t >= r.tsc & isfinite(r.h))) * Rr - Rr), rs(id));
    w = @(a0, b0) rms(er(t >= a0 & t < b0));  fprintf('%-30s scan starts %.2f s, finished %d/%d | err at scan start %.2f, first 1 s %.2f, 1-2 s %.2f, 2-3 s %.2f deg | min gap - contact %s nm\n', nm{j}, tsc, sum(fin), NS, er(find(t >= tsc, 1)), w(tsc, tsc + 1), w(tsc + 1, tsc + 2), w(tsc + 2, tsc + 3), mat2str(round(mg', 1)));
    k = (1:3:n)';  hl(j) = plot(a2, t(k) - tsc, er(k), 'Color', cc(j, :), 'LineWidth', 1.3);
end
xline(a2, 0, 'k:', 'HandleVisibility', 'off');  xlim(a2, [-2 4]);  ylim(a2, [0 8]);  xlabel(a2, 'time from the scan start  [s]');  ylabel(a2, ['angle error of w' char(770) '  [deg]']);  box(a2, 'on');  set(a2, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2);  legend(a2, hl, nm, 'Location', 'northoutside', 'Orientation', 'horizontal');
fn2 = fullfile(fo, 'fig_1103_scan_now.png');  exportgraphics(f2, fn2, 'Resolution', 120);  close(f2);  system(['open -a Preview "' fn '" "' fn2 '"']);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
