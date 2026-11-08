% STATUS: SCRATCH -- 2026-11-02 (user): current response before the scan, t 0-12 s, current code (curvature coupling off before the scan), same options as the bead scans, seeds 1-5, true angle 0. theta-hat_x, theta-hat_y per seed | angle error of w-hat rms and sqrt P | a_11 - a_33. MATLAB window 28. Sanity: rms about 4 deg at 10 s, about 1.5 deg at 12 s. Output fig_1102_pre12.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 5;  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  AB = char(257);  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 4, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'descent_T', 1, 'post_wait', [0 1], 'tag', 'p12', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'T', 12);
rs = cell(NS, 1);  if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:NS
    o = base;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'thx', r.thx(:), 'thy', r.thy(:), 'sP', r.sPth, 'g', r.dgap(:), 'gt', r.aT_tr(:) - r.aN_tr(:));
end
n = min(cellfun(@(r) find(isfinite(r.thx), 1, 'last'), rs));  t = rs{1}.t(1:n);  X = cell2mat(cellfun(@(r) r.thx(1:n), rs', 'UniformOutput', false));  Y = cell2mat(cellfun(@(r) r.thy(1:n), rs', 'UniformOutput', false));  S = sqrt(mean(cell2mat(cellfun(@(r) r.sP(1:n, 1).^2 + r.sP(1:n, 2).^2, rs', 'UniformOutput', false)), 2));  W = zeros(n, NS);  for q = 1:NS; W(:, q) = acosd(min(cos(X(:, q) * pi / 180) .* cos(Y(:, q) * pi / 180), 1)); end;  er = sqrt(mean(W.^2, 2));
for x = [2 5 10 11 12]; i = find(t >= x - 0.001, 1);  if isempty(i); i = n; end;  fprintf('t %4.1f s: theta_x %s, theta_y %s deg | w-hat error rms %.2f, sqrt P (2 axes) %.2f deg\n', x, mat2str(round(X(i, :), 1)), mat2str(round(Y(i, :), 1)), er(i), S(i)); end
if any(findall(0, 'Type', 'figure') == 28); close(28); end
f = figure(28);  set(f, 'Color', 'w', 'Position', [50 50 1800 900], 'Visible', 'on', 'Name', 'response before the scan, 5 seeds');  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  k = (1:3:n)';
a = nexttile(tl);  hold(a, 'on');  plot(a, t(k), X(k, :), 'LineWidth', 0.8);  yline(a, 0, 'Color', cT, 'LineWidth', 1.4);  ylabel(a, [th '_x  [deg]']);  ylim(a, [-15 15]);
b = nexttile(tl);  hold(b, 'on');  plot(b, t(k), Y(k, :), 'LineWidth', 0.8);  yline(b, 0, 'Color', cT, 'LineWidth', 1.4);  ylabel(b, [th '_y  [deg]']);  ylim(b, [-15 15]);
c = nexttile(tl);  hold(c, 'on');  h1 = plot(c, t(k), movmean(er(k), 11), 'Color', cE, 'LineWidth', 1.5);  h2 = plot(c, t(k), S(k), 'k--', 'LineWidth', 1.1);  ylim(c, [0 15]);  ylabel(c, ['angle error of w' char(770) '  [deg]']);  xlabel(c, 'Time (sec)');  legend(c, [h1 h2], {'rms, 5 seeds', '\surdP'}, 'Location', 'northeast');
d = nexttile(tl);  hold(d, 'on');  G = mean(cell2mat(cellfun(@(r) r.g(1:n), rs', 'UniformOutput', false)), 2);  h3 = plot(d, t(k), rs{1}.gt(k), 'Color', cT, 'LineWidth', 1.5);  h4 = plot(d, t(k), G(k), 'Color', cE, 'LineWidth', 1);  ylim(d, [0 0.45]);  ylabel(d, [AB '_{11} - ' AB '_{33}']);  xlabel(d, 'Time (sec)');  legend(d, [h3 h4], {'True', 'Estimate, mean'}, 'Location', 'northwest');
for q = [a b c d]; xlim(q, [0 12]);  xline(q, 10, 'k:', 'HandleVisibility', 'off');  xline(q, 11, 'k:', 'HandleVisibility', 'off');  box(q, 'on');  set(q, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(q.Toolbar, 'Visible', 'off'); end
exportgraphics(f, fullfile(fo, 'fig_1102_pre12.png'), 'Resolution', 120);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
