% STATUS: SCRATCH -- 2026-11-04 (user): control force variance at a fixed point against the closed-loop derivation sigma_F^2 = (1 - lc)/(1 + lc) 4 k_B T / (a_nom a). Flat wall, fixed point at h-bar 10 and 1.1, seed 1, 10 s, statistics over 3-10 s. Arms: with measurement noise | w-axis noise lowered to .6 nm (as u, v; noise 0 makes R singular). Force rotated to the local frame of V-hat[k] (F_u, F_v, F_w). Theory per axis with the true gain (a_11 for u, v; a_33 for w). Figure in Preview. Output fig_1104_force_var.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);  params = calc_simulation_params(cfg);  P = params.Value;  Ts = P.common.Ts;  a_nom = Ts / P.common.gamma_N;  a_o = a_nom / P.common.R;  kT = P.ctrl.k_B * P.ctrl.T;  lc = 0.4;  sn = cfg.meas_noise_std(:)';   % runner default lambda_c
fprintf('k_B T %.4e (P units), a_nom %.4e um/pN, lambda_c %.2f, meas noise std %s um\n', kT, a_nom, lc, mat2str(sn, 3));
H = [10 1.1];  NZ = [1 0];  [IH, IN] = ndgrid(1:2, 1:2);  N = numel(IH);  rs = cell(N, 1);
base = struct('theta_x', 0, 'theta_y', 0, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 50, 'ap_q', true, 'ap_floor_law', true, 'tag', 'fv', 'T', 10, 'seeds', 1);
if isempty(gcp('nocreate')); parpool('Processes', 4); end
parfor q = 1:N
    o = base;  o.h0 = H(IH(q));  o.lambda_c = lc;  if NZ(IN(q)) == 0; o.noise_um = [sn(1) sn(2) 0.0006]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'fa', r.fa, 'thx', r.thx(:), 'thy', r.thy(:), 'aT', r.aT_tr(:), 'aN', r.aN_tr(:));
end
fd2 = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');  f = figure('Color', 'w', 'Position', [40 40 1700 800], 'Visible', 'off');  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');  cE = [0 0.25 0.85];  cT = [0.85 0 0];
for q = 1:N
    r = rs{q};  n = min(numel(r.t), size(r.fa, 1));  F = NaN(n, 3);  fd = r.fa(1:n, :) / a_o;
    for i = 2:n; c0 = r.thx(i - 1) * pi / 180;  s0 = r.thy(i - 1) * pi / 180;  V = [cos(s0) 0 sin(s0); 0 1 0; -sin(s0) 0 cos(s0)] * [1 0 0; 0 cos(c0) -sin(c0); 0 sin(c0) cos(c0)];  F(i, :) = (V' * fd(i, :)')'; end
    m = r.t(1:n) >= 3 & all(isfinite(F), 2);  aa = [mean(r.aT(m)) mean(r.aT(m)) mean(r.aN(m))];  snq = sn;  if NZ(IN(q)) == 0; snq(3) = 0.0006; end
    thT = (1 - lc) / (1 + lc) * 4 * kT * a_nom .* aa;  thN = 2 * (1 - lc)^2 / (1 + lc) * snq.^2;  th = sqrt((thT + thN) ./ (a_nom * aa).^2);  me = std(F(m, :));
    fprintf('h-bar %4.1f, w noise %.2f nm: measured sd F_u %.3f F_v %.3f F_w %.3f pN | two-term formula %.3f %.3f %.3f pN | ratio %.2f %.2f %.2f | thermal / noise part of F_w %.2f / %.2f pN^2\n', H(IH(q)), 1e3 * snq(3), me, th, me ./ th, thT(3) / (a_nom * aa(3))^2, thN(3) / (a_nom * aa(3))^2);
    if NZ(IN(q)) == 1
        a = nexttile(tl);  k = find(m);  k = k(1:3:end);  h1 = plot(a, r.t(k), F(k, 3), 'Color', cE, 'LineWidth', 0.5);  hold(a, 'on');  h2 = yline(a, th(3), '--', 'Color', cT, 'LineWidth', 1.6);  yline(a, -th(3), '--', 'Color', cT, 'LineWidth', 1.6);  ylim(a, [-12 12]);  xlim(a, [3 10]);  ylabel(a, 'F_w  [pN]');  xlabel(a, 'Time (sec)');
        lg = legend(a, [h1 h2], {'simulation', '\pm\sigma_F, derivation'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  title(lg, sprintf('h = %.1f R', H(IH(q))));
        b = nexttile(tl);  histogram(b, F(m, 3), 80, 'Normalization', 'pdf', 'FaceColor', [0.6 0.7 0.9], 'EdgeColor', 'none');  hold(b, 'on');  xx = linspace(-12, 12, 400);  plot(b, xx, exp(-xx.^2 / (2 * th(3)^2)) / (th(3) * sqrt(2 * pi)), 'Color', cT, 'LineWidth', 1.8);  xlim(b, [-12 12]);  xlabel(b, 'F_w  [pN]');  ylabel(b, 'pdf');
        for c = [a b]; box(c, 'on');  set(c, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2); end
    end
    if NZ(IN(q)) == 1 && IH(q) == 2; Fk = F(m, :);  tk = r.t(m);  thk = th; end
end
g = figure('Color', 'w', 'Position', [40 40 1900 780], 'Visible', 'off');  tg = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');  L = {'F_u', 'F_v', 'F_w'};  YL = [6 6 12];
for c = 1:3
    a = nexttile(tg);  k = 1:3:numel(tk);  h1 = plot(a, tk(k), Fk(k, c), 'Color', cE, 'LineWidth', 0.5);  hold(a, 'on');  h2 = yline(a, thk(c), '--', 'Color', cT, 'LineWidth', 1.6);  yline(a, -thk(c), '--', 'Color', cT, 'LineWidth', 1.6);  ylim(a, [-YL(c) YL(c)]);  xlim(a, [3 10]);  ylabel(a, [L{c} '  [pN]']);  xlabel(a, 'Time (sec)');
    if c == 1; lg = legend(a, [h1 h2], {'simulation', '\pm\sigma_F, derivation'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  title(lg, 'h = 1.1 R'); end
    b = nexttile(tg);  histogram(b, Fk(:, c), 80, 'Normalization', 'pdf', 'FaceColor', [0.6 0.7 0.9], 'EdgeColor', 'none');  hold(b, 'on');  xx = linspace(-YL(c), YL(c), 400);  plot(b, xx, exp(-xx.^2 / (2 * thk(c)^2)) / (thk(c) * sqrt(2 * pi)), 'Color', cT, 'LineWidth', 1.8);  xlim(b, [-YL(c) YL(c)]);  xlabel(b, [L{c} '  [pN]']);  ylabel(b, 'pdf');
    for e = [a b]; box(e, 'on');  set(e, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2); end
end
exportgraphics(g, fullfile(fd2, 'fig_1104_force_var_uvw.png'), 'Resolution', 130);  close(g);
fn = fullfile(fd2, 'fig_1104_force_var.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);
