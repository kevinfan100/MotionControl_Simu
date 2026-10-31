% STATUS: SCRATCH -- 2026-10-29 (user): Meng glass bead, simplified (two-wall superposition of the plane law, wall 'bead2', bead radius 5 um
%   on the glass). The concave junction bead / glass needs a ~110 deg jump of the normal for the probe centre (the two offset surfaces
%   meet at a crease), so this run scans the cap only: raster inside a disc of radius Rd = 6 um (top view, opts.raster_disc), lines
%   along x, steps 1 um along y (12 lines), after a transfer along -y to the first line. Flow: start 10 deg off the bead top, 15 um
%   above it, theta-hat[0] = 0, prior 10 deg; hold 0-10 s; constant-speed descent 10-15 s (no regulation); from 15 s scan + regulation.
%   Arms: 5 um/s (angle P_fix 2) | 5 um/s + kappa (1) | 40 um/s + kappa (2). Seed 1.
%   Figure (Meng style, 3D): left bead + glass, path and estimated normals; right wall reconstructed from p - h-hat R w-hat (h-hat from
%   a-hat_33 through the plane law), per arm. Output fig_1029_bead_cap_3d.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  rbd = 5;  tH = 10;  Td = 5;  tS = tH + Td;  Ts = 1 / 1612;  FS = 13;  hatc = char(770);  Rd = 6;  dy = 1;
hs = fzero(@(h) 1 / local_cp(h) - 0.05, [1.001 30]);  vd = (15 - hs * Rr) / Td;
arms = struct('name', {'5 um/s', '5 um/s + \kappa', '40 um/s + \kappa'}, 'spd', {5, 5, 40}, 'pf', {2, 1, 2}, 'kp', {false, true, true});
rs = cell(1, 3);
if isempty(gcp('nocreate')); parpool('Processes', 3); end
parfor ia = 1:3
    a = arms(ia);  Tsc = (2 * Rd * 12 * 1.25 + 2 * Rd) / a.spd + 1;
    o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', 0.05, ...
               'ap_q', true, 'q_theta_from', tS, 'wall', 'bead2', 'bead_r', rbd, 'bead_top_deg', 10, 'ap_floor_law', true, 'scan_dir', [1 0], ...
               'ap_pfix_law', true, 'raster_disc', [Rd dy], 'scan_len', 1e9, 'descent_v', vd, 'descent_T', Td, 'du_d', a.spd * Ts / Rr, ...
               'sP_fix_theta_deg', a.pf, 'T', tS + Tsc, 'seeds', 1, 'tag', 'bc');
    if a.kp; o.kappa_state = true; end
    R = run_t3_xyz_fixed(o);  rs{ia} = R.runs{1};
end
hg = linspace(1.001, 30, 4000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);
f = figure('Color', 'w', 'Position', [20 20 2300 1300]);  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
[TH, PH] = meshgrid(linspace(0, 2 * pi, 80), linspace(0, pi, 50));  XS = rbd * sin(PH) .* cos(TH);  YS = rbd * sin(PH) .* sin(TH);  ZS = rbd + rbd * cos(PH);
for ia = 1:3
    r = rs{ia};  te = r.rs_tend;  if isnan(te); te = r.t(r.kend); end
    k = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);  P = r.p(k, :);
    W = [sind(r.thy(k)) .* cosd(r.thx(k)), -sind(r.thx(k)), cosd(r.thy(k)) .* cosd(r.thx(k))];
    hh = interp1(ag, hg, min(max(r.aN(k), ag(1)), ag(end)));  S = P - (hh * Rr) .* W;
    dS = vecnorm(S - [0 0 rbd], 2, 2) - rbd;  ok = isfinite(dS);
    kv = NaN;  if isfield(r, 'kap') && numel(r.kap) >= k(end); kv = r.kap(k(end)); end
    fprintf('%-16s: contact %d | raster done %d (scan %.1f s) | gap %.2f-%.2f um | reconstructed wall: rms %.3f max %.3f um | kappa-hat end %.3f (true %.3f)\n', ...
            strrep(arms(ia).name, '\kappa', 'kappa'), r.contact, ~isnan(r.rs_tend), te - tS, min(r.h(k)) * Rr, max(r.h(k)) * Rr, rms(dS(ok)), max(abs(dS(ok))), kv, Rr / (rbd + hs * Rr));
    ax1 = nexttile(tl, ia);  hold on;
    surf(XS, YS, ZS, 'FaceColor', [0.82 0.84 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'bead');
    [GX, GY] = meshgrid(linspace(-10, 10, 2));  surf(GX, GY, 0 * GX, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.4, 'DisplayName', 'glass');
    plot3(P(:, 1), P(:, 2), P(:, 3), 'Color', [0 0.25 0.85], 'LineWidth', 1.3, 'DisplayName', 'probe path p');
    s = [0; cumsum(vecnorm(diff(P), 2, 2))];  iq = arrayfun(@(v) find(s >= v, 1), 0:3:s(end));
    quiver3(P(iq, 1), P(iq, 2), P(iq, 3), 1.5 * W(iq, 1), 1.5 * W(iq, 2), 1.5 * W(iq, 3), 0, 'Color', [0.85 0.1 0.1], 'LineWidth', 1, 'MaxHeadSize', 0.3, 'DisplayName', ['w' hatc]);
    axis equal;  view(-35, 30);  xlim([-10 10]);  ylim([-10 10]);  zlim([0 16]);  box on;  light('Position', [-20 -30 40], 'Style', 'infinite');  lighting gouraud;
    xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  title(arms(ia).name);  if ia == 1; legend('Location', 'northoutside', 'Orientation', 'horizontal'); end
    ax2 = nexttile(tl, 3 + ia);  hold on;
    mesh(XS, YS, ZS, 'EdgeColor', [0.75 0.75 0.75], 'FaceColor', 'none', 'DisplayName', 'true bead');
    scatter3(S(1:15:end, 1), S(1:15:end, 2), S(1:15:end, 3), 6, S(1:15:end, 3), 'filled', 'DisplayName', 'reconstructed wall');
    colormap(ax2, parula);  caxis(ax2, [5 10]);  axis equal;  view(-35, 30);  xlim([-10 10]);  ylim([-10 10]);  zlim([0 16]);  box on;
    xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  if ia == 1; legend('Location', 'northoutside', 'Orientation', 'horizontal'); end
    if ia == 3; cb = colorbar(ax2);  cb.Label.String = 'z  [\mum]'; end
    for a = [ax1 ax2]; set(a, 'FontSize', FS, 'FontWeight', 'bold');  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_bead_cap_3d.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
