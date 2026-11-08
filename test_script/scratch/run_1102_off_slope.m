% STATUS: SCRATCH -- 2026-11-02 (user): where the angle becomes over-confident, 0-12 s. [hypothesis] P of the off-diagonal slots has no term for the growing gap g-hat: a fixed angle error e gives an off-diagonal error g e, so sqrt(P_off) should grow with g, with Fe(12,12) = 1 the angle sqrt P = sqrt(P_off) / g shrinks with no information. Rival: the near wall really gives the information and the over-confidence comes elsewhere. Split: opts.off_slope (Fe(12:13,12:13) = 1 + (a'_11 - a'_33) / g M_w) on against off, seeds 1-20, same seeds, ~line 698.
%   Pre-registered: rms error / sqrt P at 12 s from about 3 to about 1 with off_slope, and rms error not worse. Sanity: arm off repeats 1.5 deg, .47 deg at seeds 1-5. Cached in temp_off_slope.mat. MATLAB window 17. Output fig_1102_off_slope.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 20;  FS = 13;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  cf = fullfile(fo, 'temp_off_slope.mat');
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 4, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'descent_T', 1, 'post_wait', [0 1], 'tag', 'os', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'T', 12);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = ID(q);  o.off_slope = IA(q) == 2;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'g', r.dgap(:), 'gt', r.aT_tr(:) - r.aN_tr(:));
    end
    save(cf, 'rs', 'IA', 'ID', '-v7.3');
end
n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs));  t = rs{1}.t(1:n);  tq = [10 10.25 10.5 10.75 11 11.25 11.5 12];  nm = {'off (default)', 'off_slope on'};  f = figure(17);  clf(f);  set(f, 'Color', 'w', 'Position', [60 80 1700 800], 'Visible', 'on', 'Name', 'angle honesty 0-12 s, 20 seeds');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for a = 1:2
    id = find(IA == a);  E = [cell2mat(cellfun(@(r) r.thx(1:n), rs(id)', 'UniformOutput', false)), cell2mat(cellfun(@(r) r.thy(1:n), rs(id)', 'UniformOutput', false))];  P = [cell2mat(cellfun(@(r) r.sP(1:n, 1), rs(id)', 'UniformOutput', false)), cell2mat(cellfun(@(r) r.sP(1:n, 2), rs(id)', 'UniformOutput', false))];  er = sqrt(mean(E.^2, 2));  sp = sqrt(mean(P.^2, 2));
    fprintf('%s\n', nm{a});
    for j = 1:numel(tq); i = find(t >= tq(j), 1);  if isempty(i); i = n; end;  fprintf('  t %5.2f s: rms err %.2f deg, sqrt P %.2f deg, ratio %.2f\n', t(i), er(i), sp(i), er(i) / sp(i)); end
    k = find(t >= 8);  k = k(1:4:end);  ax = nexttile(tl);  hold(ax, 'on');  h1 = plot(ax, t(k), movmean(er(k), 25), 'Color', cT, 'LineWidth', 1.6);  h2 = plot(ax, t(k), sp(k), 'Color', cE, 'LineWidth', 1.6);  xline(ax, 10, 'k:', 'HandleVisibility', 'off');  xline(ax, 11, 'k:', 'HandleVisibility', 'off');  set(ax, 'YScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  xlim(ax, [8 12]);  ylim(ax, [0.2 12]);  box(ax, 'on');  xlabel(ax, 'Time (sec)');  ylabel(ax, 'angle  [deg]');  title(ax, nm{a}, 'Interpreter', 'none');  legend(ax, [h1 h2], {'rms error, 20 seeds', '\surdP'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
exportgraphics(f, fullfile(fo, 'fig_1102_off_slope.png'), 'Resolution', 120);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
