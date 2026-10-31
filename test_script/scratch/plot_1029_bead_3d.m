% STATUS: SCRATCH -- 2026-10-29 (user): 3D view of what the whole-bead raster has scanned so far (temp_fillet_opts.mat), drawing only the
%   bead (r 5 um) and the glass, no fillet. Two runs: (A) 5 um/s seed 2 (rho 6) and (B) rho 10 seed 1. Left: probe-centre path p coloured by
%   height; right: wall points reconstructed from the estimates only, p - h-hat R w-hat (h-hat from a-hat_33 = 1 / c_perp(h-hat), w-hat from
%   theta-hat), every 40th step of the scan. Output test_results/temp_3d_wsonly/fig_1029_bead_3d.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
addpath(genpath(fullfile(root, 'model')));
S = load(fullfile(fo, 'temp_fillet_opts.mat'));  rs = S.rs;  Rr = S.Rr;  FS = 14;
pick = [1 2; 2 1];  ttl = {'5 um/s, seed 2 (16 of 27 lines)', 'fillet 10 um in the plant, seed 1 (21 of 27 lines)'};
hg = linspace(1.0005, 30, 4000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);   % a_33(h) table for the inversion
[Xs, Ys, Zs] = sphere(60);  rbd = 5;
f = figure('Color', 'w', 'Position', [20 20 1900 1500]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:2
    r = rs{pick(i, 1), pick(i, 2)};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc & isfinite(r.aN(kk)));  kd = ks(1:40:end);
    for c = 1:2
        ax = nexttile(tl, 2 * (i - 1) + c);  hold on;  box on;
        surf(rbd * Xs, rbd * Ys, rbd * Zs + rbd, 'FaceColor', [0.75 0.75 0.75], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        patch([-16 16 16 -16], [-16 -16 16 16], [0 0 0 0], [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        if c == 1
            scatter3(r.p(kd, 1), r.p(kd, 2), r.p(kd, 3), 6, r.p(kd, 3), 'filled');  title(sprintf('%s: probe centre p', ttl{i}));
        else
            ah = min(max(r.aN(kd), ag(1)), ag(end));  hh = interp1(ag, hg, ah);  tx = r.thx(kd) * pi / 180;  ty = r.thy(kd) * pi / 180;
            wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  wp = r.p(kd, :) - (hh * Rr) .* wh;
            scatter3(wp(:, 1), wp(:, 2), wp(:, 3), 6, wp(:, 3), 'filled');  title(sprintf('%s: wall from the estimates', ttl{i}));
        end
        colormap(ax, parula);  caxis(ax, [0 12]);  cb = colorbar(ax);  cb.Label.String = 'z  [\mum]';
        axis equal;  xlim([-16 16]);  ylim([-16 16]);  zlim([-1 14]);  view(-35, 30);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(ax.Toolbar, 'Visible', 'off');
    end
end
fn = fullfile(fo, 'fig_1029_bead_3d.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
