% STATUS: SCRATCH -- 2026-11-02 (user): how to crop the scan figure, compare Meng Fig. 52 (right panel has no floor). temp_meng_traj.mat (run_1102_meng_traj.m, rho 4, a_d .1, seed 1), no new simulation. A full path | B window 24 x 16 um, v steps removed | C B above 3.4 um, blue probe and red wall points. Odd lines alone still show the mesh, so it is not out-and-back.
%   v-step segments found from the motion along v-hat. Sanity: 26 v steps of 1 um (27 lines). Output fig_1102_crop_options.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
FS = 13;  MU = [char(956) 'm'];  BL = [0 0 1];  RD = [0.9 0 0];  VW = [-37.5 22];
S = load(fullfile(fo, 'temp_meng_traj.mat'));  r = S.rs{2, 1};  Rr = S.Rr;  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);  hh = interp1(ag, hg, min(max(r.aN(ok), ag(1)), ag(end)));
tx = r.thx(ok) * pi / 180;  ty = r.thy(ok) * pi / 180;  wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  wp = NaN(numel(r.t), 3);  wp(ok, :) = movmean(r.p(ok, :) - (hh(:) * Rr) .* wh, 41, 1);
pp = r.p(ks, :);  ww = wp(ks, :);  fprintf('ours: x %.1f..%.1f, y %.1f..%.1f, z %.2f..%.2f um\n', min(pp(:, 1)), max(pp(:, 1)), min(pp(:, 2)), max(pp(:, 2)), min(pp(:, 3)), max(pp(:, 3)));
f = figure('Color', 'w', 'Position', [20 20 2100 640]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
tx = r.thx(ks) * pi / 180;  ty = r.thy(ks) * pi / 180;  vh = [sin(ty) .* sin(tx), cos(tx), cos(ty) .* sin(tx)];  dv = [0; sum(diff(pp, 1, 1) .* vh(2:end, :), 2)];  sc = movmean(dv, 21);  isv = sc > 0.012;  [~, nv] = bwlabel(isv);  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  e = find(diff([0; isv; 0]) == 1);  x = find(diff([0; isv; 0]) == -1);  ln = arrayfun(@(i) sum(dv(e(i):x(i) - 1)), 1:numel(e));
fprintf('v steps found %d, length median %.2f um\n', nv, median(ln));
ln_id = cumsum(diff([0; stepv]) == 1) + 1;  odd = mod(ln_id, 2) == 1;  win = abs(pp(:, 1)) <= 12 & abs(pp(:, 2)) <= 8 & ~stepv;
fprintf('lines %d, x travel of the first lines: %s um\n', max(ln_id), mat2str(round(arrayfun(@(i) pp(find(ln_id == i & ~stepv, 1, 'last'), 1) - pp(find(ln_id == i & ~stepv, 1, 'first'), 1), 1:6), 1)));
po = pp;  po(~(win & odd), :) = NaN;  pe = pp;  pe(~(win & ~odd), :) = NaN;
a = nexttile(tl);  hold on;  plot3(pp(:, 1), pp(:, 2), pp(:, 3), 'Color', BL, 'LineWidth', 0.8);  title('A  full path');  local_axes(a, VW, FS, MU, [-18 26]);
pb = pp;  pb(~win, :) = NaN;
a = nexttile(tl);  hold on;  plot3(pb(:, 1), pb(:, 2), pb(:, 3), 'Color', BL, 'LineWidth', 1.1);  title('B  window 24 x 16 \mum, v steps removed');  local_axes(a, VW, FS, MU, [-13 13]);
on = win & pp(:, 3) > 3.4;  pc = pp;  wc = ww;  pc(~on, :) = NaN;  wc(~on, :) = NaN;
a = nexttile(tl);  hold on;  plot3(pc(:, 1), pc(:, 2), pc(:, 3), 'Color', BL, 'LineWidth', 1.1);  plot3(wc(:, 1), wc(:, 2), wc(:, 3), 'Color', RD, 'LineWidth', 1.3);  title('C  B above z = 3.4 \mum: probe (blue), wall (red)');  local_axes(a, VW, FS, MU, [-13 13]);
exportgraphics(f, fullfile(fo, 'fig_1102_crop_options.png'), 'Resolution', 130);  close(f);

function local_axes(a, vw, fs, mu, xl)
    axis(a, 'equal');  xlim(a, xl);  ylim(a, [-14 14]);  zlim(a, [0 14]);  view(a, vw(1), vw(2));  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' mu ')']);  ylabel(a, ['Y (' mu ')']);  zlabel(a, ['Z (' mu ')']);  set(a, 'FontSize', fs, 'LineWidth', 1.1);  set(a.Toolbar, 'Visible', 'off');
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
