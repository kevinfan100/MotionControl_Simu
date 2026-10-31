% STATUS: SCRATCH -- 2026-10-28 (user): Meng-style 3D view of the raster over half of the void hemisphere (radius 15 um), settled flow
%   + curvature state (opts.kappa_state), 5 um/s, angle P_fix 1 deg, seed 1. Left: sphere (transparent), probe path during the scan,
%   estimated normals w-hat (every 5 um of path). Right: wall reconstructed from the run -- point = p - h-hat R w-hat, h-hat from
%   a-hat_33 through the plane law (inverse of 1/c_perp), coloured by z; true sphere as a wire mesh. Console: rms distance of the
%   reconstructed points from the true sphere. Output test_results/temp_3d_wsonly/fig_1028_raster_3d.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  rb = 15;  tS = 17;  Ts = 1 / 1612;  FS = 14;  hatc = char(770);
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
           'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', 0.05, ...
           't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
           'scan_dir', [1 0], 'hemi_start_deg', 10, 'ap_pfix_law', true, 'raster', [12 2 7], 'scan_len', 1e9, 'kappa_state', true, ...
           'du_d', 5 * Ts / Rr, 'sP_fix_theta_deg', 1, 'T', tS + 40, 'seeds', 1, 'tag', 'r3');
if isempty(gcp('nocreate')); parpool('Processes', 2); end
rc = cell(1, 1);  parfor i = 1:1; R = run_t3_xyz_fixed(o);  rc{i} = R.runs{1}; end;  r = rc{1};
te = r.rs_tend;  if isnan(te); te = r.t(r.kend); end
k = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);  P = r.p(k, :);
W = [sind(r.thy(k)) .* cosd(r.thx(k)), -sind(r.thx(k)), cosd(r.thy(k)) .* cosd(r.thx(k))];
hg = linspace(1.001, 30, 4000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);   % plane law a_33(h) (monotone) -> invert by interpolation
hh = interp1(ag, hg, min(max(r.aN(k), ag(1)), ag(end)));                    % h-hat [R] from a-hat_33
S = P - (hh * Rr) .* W;                                                    % reconstructed wall points
dS = vecnorm(S, 2, 2) - rb;  fprintf('reconstructed wall: rms distance from the sphere %.3f um, max %.3f um (%d points)\n', rms(dS), max(abs(dS)), numel(dS));
f = figure('Color', 'w', 'Position', [30 30 2100 950]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
[TH, PH] = meshgrid(linspace(0, 2 * pi, 90), linspace(0, pi / 2, 40));  XS = rb * sin(PH) .* cos(TH);  YS = rb * sin(PH) .* sin(TH);  ZS = rb * cos(PH);
ax1 = nexttile(tl);  hold on;
surf(XS, YS, ZS, 'FaceColor', [0.82 0.84 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'wall (sphere)');
plot3(P(:, 1), P(:, 2), P(:, 3), 'Color', [0 0.25 0.85], 'LineWidth', 1.6, 'DisplayName', 'probe path p');
s = [0; cumsum(vecnorm(diff(P), 2, 2))];  iq = arrayfun(@(v) find(s >= v, 1), 0:5:s(end));
quiver3(P(iq, 1), P(iq, 2), P(iq, 3), 2.2 * W(iq, 1), 2.2 * W(iq, 2), 2.2 * W(iq, 3), 0, 'Color', [0.85 0.1 0.1], 'LineWidth', 1.2, 'MaxHeadSize', 0.3, 'DisplayName', ['estimated normal w' hatc]);
axis equal;  view(-35, 30);  xlim([-17 17]);  ylim([-17 17]);  zlim([0 22]);  grid off;  box on;
xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  legend('Location', 'northoutside', 'Orientation', 'horizontal');  light('Position', [-20 -30 40], 'Style', 'infinite');  lighting gouraud;
title('raster 5 \mum/s with \kappa, seed 1: path and estimated normals');
ax2 = nexttile(tl);  hold on;
mesh(XS, YS, ZS, 'EdgeColor', [0.7 0.7 0.7], 'FaceColor', 'none', 'DisplayName', 'true sphere');
scatter3(S(1:20:end, 1), S(1:20:end, 2), S(1:20:end, 3), 8, S(1:20:end, 3), 'filled', 'DisplayName', 'reconstructed wall');
colormap(ax2, parula);  cb = colorbar(ax2);  cb.Label.String = 'z  [\mum]';
axis equal;  view(-35, 30);  xlim([-17 17]);  ylim([-17 17]);  zlim([0 22]);  grid off;  box on;
xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
title(sprintf('wall reconstructed from p, %s_{33} and w%s', ['a' hatc], hatc));
for a = [ax1 ax2]; set(a, 'FontSize', FS, 'FontWeight', 'bold');  set(a.Toolbar, 'Visible', 'off'); end
fn = fullfile(fo, 'fig_1028_raster_3d.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
