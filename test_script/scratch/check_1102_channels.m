% STATUS: SCRATCH -- 2026-11-02 (user): where the angle update comes from. Flat wall true (8,-8), seeds 1-5, 0-10 s, no motion and circle 3 um. Arms: both channels | off-diagonal updated by the position readings y1 only (k_off_cut 'ym') | by the gain readout y4 only (k_off_cut 'y1'). Log: per-step update of a_uw, a_vw from each channel, gain size, summed update in degrees, rms error, sqrt P. Rival readings: motion helps through y1 or through y4; sqrt P too small because both channels are counted as independent. If each channel alone is honest (ratio near 1) and both together is not, the two channels share information. Figure in Preview. Output fig_1102_channels.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  Tx = 8;  Ty = -8;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', Tx, 'theta_y', Ty, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 50, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'ch', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10);
MO = [0 3];  CUT = {'', 'ym', 'y1'};  cn = {'both', 'y1 only', 'y4 only'};  [IM, IC, ID] = ndgrid(1:2, 1:3, 1:NS);  N = numel(IM);  rs = cell(N, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if MO(IM(q)) > 0; o.dither = [3 8 2];  o.dither_win = [0 50]; end;  if IC(q) > 1; o.k_off_cut = CUT{IC(q)}; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'd1', r.dof1, 'd4', r.dof4, 'kof', r.kof, 'g', r.dgap(:));
end
f = figure('Color', 'w', 'Position', [40 40 1600 760], 'Visible', 'off');  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'rowmajor');
for m = 1:2
    for c = 1:3
        id = find(IM == m & IC == c);  n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs(id)));  t = rs{id(1)}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false)) - Tx;  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false)) - Ty;  P = sqrt(mean(cell2mat(cellfun(@(r) (r.sP(1:n, 1).^2 + r.sP(1:n, 2).^2) / 2, rs(id)', 'UniformOutput', false)), 2));  er = sqrt(mean((X.^2 + Y.^2) / 2, 2));  w = t >= 4;  D1 = 0;  D4 = 0;  Kk = [0 0];  for s = id'; r = rs{s}; D1 = D1 + mean(r.d1(w, :).^2, 'all');  D4 = D4 + mean(r.d4(w, :).^2, 'all');  Kk = Kk + mean(r.kof(w, :), 1); end;  Kk = Kk / NS;
        fprintf('circle %g, %-8s: data to %.1f s | rms %.2f, sqrt P %.2f, ratio %.2f at 10 s (at 5 s %.2f, %.2f) | update rms y1 %.2e, y4 %.2e | K y1 %.2e, y4 %.2e\n', MO(m), cn{c}, t(end), er(end), P(end), er(end) / P(end), er(find(t >= 5, 1)), P(find(t >= 5, 1)), sqrt(D1 / NS), sqrt(D4 / NS), Kk(1), Kk(2));
        a = nexttile(tl);  hold(a, 'on');  k = (1:4:n)';  h1 = plot(a, t(k), movmean(er(k), 11), 'Color', [0 0.25 0.85], 'LineWidth', 1.5);  h2 = plot(a, t(k), P(k), 'k--', 'LineWidth', 1.1);  ylim(a, [0 10]);  xlim(a, [0 10]);  title(a, sprintf('circle %g \\mum, %s', MO(m), cn{c}));  box(a, 'on');  set(a, 'FontSize', 13, 'FontWeight', 'bold', 'LineWidth', 1.2);  if m == 2; xlabel(a, 'Time (sec)'); end;  if c == 1; ylabel(a, 'rms error  [deg]'); end;  if m == 1 && c == 1; legend(a, [h1 h2], {'rms, 5 seeds', '\surdP'}, 'Location', 'northeast'); end
    end
end
fn = fullfile(fo, 'fig_1102_channels.png');  exportgraphics(f, fn, 'Resolution', 110);  close(f);  system(['open -a Preview "' fn '"']);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
