% STATUS: SCRATCH -- 2026-11-02 (user): overall angle response, before and after the scan start, current settings (off_slope off), fixed 36 um, rho 4, a_d .07, 40 um/s, seed 1 of temp_square_why.mat. Rows theta_x, theta_y, angle between w-hat and w. MATLAB window 19. Sanity: scan ends 37.1 s. Output fig_1102_angle_overall.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  th = char(952);  load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');  r = rs{find([A.a] == 3 & [A.seed] == 1)};
ok = find(isfinite(r.aN(1:r.kend)));  K = {ok(r.t(ok) < r.tsc), ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te)};  st = [1 3];  YL = {[-8 8], [-90 90]; [-8 8], [-115 115]};  e = acosd(min(sum(local_w(r.thx(ok), r.thy(ok)) .* local_w(r.th_tr(ok, 1), r.th_tr(ok, 2)), 2), 1));  E = NaN(numel(r.t), 1);  E(ok) = e;
for c = 1:2; fprintf('%d: t %.1f-%.1f s, angle w-hat to w median %.2f, 95%% %.2f deg, value at end %.2f deg\n', c, r.t(K{c}(1)), r.t(K{c}(end)), median(E(K{c})), prctile(E(K{c}), 95), E(K{c}(end))); end
if any(findall(0, 'Type', 'figure') == 19); close(19); end
f = figure(19);  set(f, 'Color', 'w', 'Position', [40 40 1850 1000], 'Visible', 'on', 'Name', 'angle response before and after the scan start, seed 1');  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
for c = 1:2
    k = K{c}(1:st(c):end);  t = r.t(k);  t = t(:);
    for q = 1:2
        a = nexttile(tl);  hold(a, 'on');  h1 = plot(a, t, r.th_tr(k, q), 'Color', cT, 'LineWidth', 1.3);  if q == 1; y = r.thx(k); else; y = r.thy(k); end;  h2 = plot(a, t, y, 'Color', cE, 'LineWidth', 0.8);  ylim(a, YL{q, c});  ylabel(a, [th '_' char('x' + q - 1) '  [deg]']);  if q == 1; legend(a, [h1 h2], {'True', 'Estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end;  local_s(a, t, FS, c);
    end
    a = nexttile(tl);  plot(a, t, E(k), 'Color', cE, 'LineWidth', 0.7);  ylim(a, [0 35]);  ylabel(a, ['angle error of w' char(770) '  [deg]']);  xlabel(a, 'Time (sec)');  local_s(a, t, FS, c);
end
exportgraphics(f, fullfile(fo, 'fig_1102_angle_overall.png'), 'Resolution', 120);
function local_s(a, t, fs, c)
    box(a, 'on');  xlim(a, [t(1) t(end)]);  set(a, 'FontSize', fs, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');  if c == 1; xline(a, 10, 'k:', 'HandleVisibility', 'off');  xline(a, 11, 'k:', 'HandleVisibility', 'off'); end
end
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
