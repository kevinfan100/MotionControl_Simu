% STATUS: SCRATCH -- 2026-11-02 (user): is the angle locked (true a_uw stays off while the filter does not correct it). Circle 3 um, flat wall true (8,-8), seeds 1-3, t 0-10 s. Arms: A as is | B at 5 s angle P reset to (5 deg)^2 (opts.p_off_kick). [hypothesis] P collapsed in the first seconds, the gain is too small to correct. Rival: the residual goes into another state. Split: in B the error falls within 1-2 s to well below the plateau means P lock; no change means absorbed elsewhere.
%   Sanity: A repeats 1.5 deg. Figure in Preview. Output fig_1102_p_kick.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 3;  FS = 13;  Tx = 8;  Ty = -8;  th = char(952);  cT = [0.85 0 0];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'pk', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10, 'dither', [3 8 2], 'dither_win', [0 10]);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  rs = cell(N, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o.p_off_kick = [5 5]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'off_tr', r.off_tr);
end
f = figure('Color', 'w', 'Position', [40 40 1650 800], 'Visible', 'off');  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');  tt = {'A as is', 'B angle P reset to 5 deg at 5 s'};
for a = 1:2
    id = find(IA == a);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false));  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false));  k = (1:2:n)';
    fprintf('%s:', tt{a});  for x = [4.9 5.5 6 7 8 10]; i = find(t >= x - 0.002, 1);  if isempty(i); i = n; end;  fprintf(' | t %.1f rms %.2f (x %s, y %s)', x, sqrt(mean([(X(i, :) - Tx) (Y(i, :) - Ty)].^2)), mat2str(round(X(i, :), 1)), mat2str(round(Y(i, :), 1))); end;  fprintf('\n');
    b = nexttile(tl);  hold(b, 'on');  plot(b, t(k), X(k, :), 'LineWidth', 0.9);  yline(b, Tx, 'Color', cT, 'LineWidth', 1.5);  ylim(b, [0 16]);  ylabel(b, [th '_x  [deg]']);  title(b, tt{a});
    c = nexttile(tl);  hold(c, 'on');  plot(c, t(k), Y(k, :), 'LineWidth', 0.9);  yline(c, Ty, 'Color', cT, 'LineWidth', 1.5);  ylim(c, [-16 0]);  ylabel(c, [th '_y  [deg]']);  xlabel(c, 'Time (sec)');
    for q = [b c]; xlim(q, [3 10]);  xline(q, 5, 'k:', 'HandleVisibility', 'off');  box(q, 'on');  set(q, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2); end
end
fn = fullfile(fo, 'fig_1102_p_kick.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
