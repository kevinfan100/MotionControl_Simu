% STATUS: SCRATCH -- 2026-11-02 (user): flat wall true (8,-8) deg, t 0-11 s (hold 10 s, descent 1 s), estimate starts at 0, 20 seeds. Arms: no motion | dither circle 1 um | 3 um, 8 Hz, 2 s ramp, tangent plane of V-hat, 0-10 s (10-21 flow). Why does horizontal motion help.
%   [hypothesis] the off-diagonal elements are read through the force along u, v (row w of G_a: F_u a_uw + F_v a_vw), so the angle information grows with the sum of F_u^2 + F_v^2; without motion the force is only the feedback on thermal noise. Rival: motion helps through something else (gain gap, geometry). Split: rms error x sqrt(sum F_t^2) the same in the three arms if the force sets it. Sanity: arm none repeats 4 deg at 10 s. Cached in temp_hold_dither.mat. MATLAB window 24. Output fig_1102_hold_dither.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 20;  FS = 13;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Tx = 8;  Ty = -8;  AMP = [0 1 3];  NA = 3;
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'hd', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 11);
[IA, ID] = ndgrid(1:NA, 1:NS);  N = numel(IA);  cf = fullfile(fo, 'temp_hold_dither.mat');
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = ID(q);  if AMP(IA(q)) > 0; o.dither = [AMP(IA(q)) 8 2];  o.dither_win = [0 10]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'g', r.dgap(:), 'vF', r.vF, 'gate', r.gate(:), 'h', r.h(:), 'off', r.off_k1, 'off_tr', r.off_tr, 'dth', r.dth);
    end
    save(fullfile(fo, 'temp_hold_dither.mat'), 'rs', 'IA', 'ID', 'AMP', '-v7.3');
end
tq = [1 2 4 7 10 11];  cc = [0 0 0; 0 0.45 0.85; 0.85 0 0];  f = figure(24);  clf(f);  set(f, 'Color', 'w', 'Position', [50 60 1850 860], 'Visible', 'on', 'Name', 'horizontal motion in the hold, 20 seeds');  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(6, 1);  for i = 1:6; ax(i) = nexttile(tl);  hold(ax(i), 'on'); end;  hl = gobjects(NA, 1);
for a = 1:NA
    id = find(IA == a);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false)) - Tx;  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false)) - Ty;  sp = sqrt(mean(cell2mat(cellfun(@(r) r.sP(1:n, 1), rs(id)', 'UniformOutput', false)).^2 / 2 + cell2mat(cellfun(@(r) r.sP(1:n, 2), rs(id)', 'UniformOutput', false)).^2 / 2, 2));  Fu2 = mean(cell2mat(cellfun(@(r) r.vF(1:n, 1).^2 + r.vF(1:n, 2).^2, rs(id)', 'UniformOutput', false)), 2);  Fw2 = mean(cell2mat(cellfun(@(r) r.vF(1:n, 3).^2, rs(id)', 'UniformOutput', false)), 2);  G = mean(cell2mat(cellfun(@(r) r.g(1:n), rs(id)', 'UniformOutput', false)), 2);  er = sqrt(mean((X.^2 + Y.^2) / 2, 2));  CF = cumsum(Fu2);
    fprintf('dither %g um: data to %.2f s, mean h at 10 s %.2f um\n', AMP(a), t(end), mean(cellfun(@(r) r.h(find(r.t >= 9.9, 1)), rs(id))) * Rr);
    for j = 1:numel(tq); i = find(t >= tq(j), 1);  if isempty(i); i = n; end;  fprintf('  t %4.1f s: mean err x %+.2f y %+.2f, rms err %.2f, sqrt P %.2f, ratio %.2f, rms F_t step %.3e, sum F_t^2 %.3e, err x sqrt(sum) %.3e, mean g-hat %.3f\n', t(i), mean(X(i, :)), mean(Y(i, :)), er(i), sp(i), er(i) / sp(i), sqrt(mean(Fu2(1:i))), CF(i), er(i) * sqrt(CF(i)), G(i)); end
    k = (1:6:n)';  hl(a) = plot(ax(1), t(k), movmean(er(k), 9), 'Color', cc(a, :), 'LineWidth', 1.5);  plot(ax(2), t(k), sp(k), 'Color', cc(a, :), 'LineWidth', 1.5);  plot(ax(3), t(k), CF(k), 'Color', cc(a, :), 'LineWidth', 1.5);  plot(ax(4), t(k), mean(X(k, :), 2), 'Color', cc(a, :), 'LineWidth', 1.2);  plot(ax(5), t(k), mean(Y(k, :), 2), 'Color', cc(a, :), 'LineWidth', 1.2);  plot(ax(6), t(k), G(k), 'Color', cc(a, :), 'LineWidth', 1.3);
end
L = {'rms error [deg]', '\surdP [deg]', 'sum F_u^2 + F_v^2', 'mean error \theta_x [deg]', 'mean error \theta_y [deg]', 'mean a_{11} - a_{33}'};  for i = 1:6; ylabel(ax(i), L{i});  xlim(ax(i), [0 11]);  box(ax(i), 'on');  set(ax(i), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  xline(ax(i), 10, 'k:', 'HandleVisibility', 'off');  if i > 3; xlabel(ax(i), 'Time (sec)'); end; end
set(ax(1), 'YScale', 'log');  set(ax(2), 'YScale', 'log');  set(ax(3), 'YScale', 'log');  ylim(ax(1), [0.2 15]);  ylim(ax(2), [0.2 30]);  ylim(ax(4), [-9 4]);  ylim(ax(5), [-4 9]);  ylim(ax(6), [0 0.25]);  legend(ax(1), hl, {'no motion', 'circle 1 \mum', 'circle 3 \mum'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
exportgraphics(f, fullfile(fo, 'fig_1102_hold_dither.png'), 'Resolution', 120);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
