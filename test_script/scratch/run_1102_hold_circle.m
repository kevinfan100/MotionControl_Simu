% STATUS: SCRATCH -- 2026-11-02 (user): angle convergence at a fixed point (15 um) with horizontal motion, current code (curvature coupling off before the scan). Flat wall true (8,-8), estimate starts at 0, t 0-10 s, seeds 1-5. Arms: no motion | circle 1 um | 3 um, 8 Hz, 2 s ramp, in the tangent plane of V-hat.
%   Pre-registered: rms at 10 s falls with the circle (3 seeds gave 1.69 against 4.11 deg, check_1102_p_budget2.m). Sanity: no motion about 4 deg. Figure in Preview, no MATLAB window. Output fig_1102_hold_circle.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  FS = 13;  Tx = 8;  Ty = -8;  AMP = [0 1 3];  NA = 3;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'hc', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10);
[IA, ID] = ndgrid(1:NA, 1:NS);  N = numel(IA);  rs = cell(N, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if AMP(IA(q)) > 0; o.dither = [AMP(IA(q)) 8 2];  o.dither_win = [0 10]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'h', r.h(:));
end
f = figure('Color', 'w', 'Position', [40 40 1900 1050], 'Visible', 'off');  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
for a = 1:NA
    id = find(IA == a);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false));  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false));  er = sqrt(mean(((X - Tx).^2 + (Y - Ty).^2) / 2, 2));  sp = sqrt(mean(cell2mat(cellfun(@(r) (r.sP(1:n, 1).^2 + r.sP(1:n, 2).^2) / 2, rs(id)', 'UniformOutput', false)), 2));  k = (1:3:n)';
    fprintf('circle %g um: data to %.2f s | ', AMP(a), t(end));  for x = [2 4 6 8 10]; i = find(t >= x - 0.002, 1);  if isempty(i); i = n; end;  fprintf('t %2d: rms %.2f, sqrt P %.2f, mean err %+.2f %+.2f | ', x, er(i), sp(i), mean(X(i, :)) - Tx, mean(Y(i, :)) - Ty); end;  fprintf('\n');
    b = nexttile(tl);  hold(b, 'on');  plot(b, t(k), X(k, :), 'LineWidth', 0.8);  yline(b, Tx, 'Color', cT, 'LineWidth', 1.5);  ylim(b, [-5 22]);  ylabel(b, [th '_x  [deg]']);  title(b, sprintf('circle %g \\mum', AMP(a)));
    c = nexttile(tl);  hold(c, 'on');  plot(c, t(k), Y(k, :), 'LineWidth', 0.8);  yline(c, Ty, 'Color', cT, 'LineWidth', 1.5);  ylim(c, [-22 5]);  ylabel(c, [th '_y  [deg]']);
    d = nexttile(tl);  hold(d, 'on');  h1 = plot(d, t(k), movmean(er(k), 11), 'Color', cE, 'LineWidth', 1.5);  h2 = plot(d, t(k), sp(k), 'k--', 'LineWidth', 1.1);  ylim(d, [0 12]);  ylabel(d, 'rms error  [deg]');  xlabel(d, 'Time (sec)');  if a == 1; legend(d, [h1 h2], {'rms, 5 seeds', '\surdP'}, 'Location', 'northeast'); end
    for q = [b c d]; xlim(q, [0 10]);  box(q, 'on');  set(q, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  end
end
fn = fullfile(fo, 'fig_1102_hold_circle.png');  exportgraphics(f, fn, 'Resolution', 110);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
