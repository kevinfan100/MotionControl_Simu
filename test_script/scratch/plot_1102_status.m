% STATUS: SCRATCH -- 2026-11-02 (user): compact picture of the current state. Fixed 36 um, rho 4, a_d .07, 40 um/s, seeds 1-5 of temp_square_why.mat, current settings. Top: a_11 - a_33 true and estimated (seed 1). Bottom: angle error of w-hat, rms of 5 seeds over 50 ms. Lines at descent, bottom, scan start. Console: at 10 s the error of a_11, a_33 and of the off-diagonal element implied by the angle (g x angle), temp_hold_angle12.mat, to compare the information in the three elements. MATLAB window 20. Output fig_1102_status.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  AB = char(257);  load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');  id = find([A.a] == 3);  n = min(cellfun(@(r) find(isfinite(r.aN(1:r.kend)), 1, 'last'), rs(id)));  n = min(n, find(rs{id(1)}.t <= rs{id(1)}.te, 1, 'last'));
E2 = zeros(n, numel(id));  for i = 1:numel(id); r = rs{id(i)};  e = acosd(min(sum(local_w(r.thx(1:n), r.thy(1:n)) .* local_w(r.th_tr(1:n, 1), r.th_tr(1:n, 2)), 2), 1));  E2(:, i) = e.^2; end;  er = sqrt(mean(movmean(E2, 81, 1), 2));
r = rs{id(1)};  t = r.t(1:n);  t = t(:);  k = (1:4:n)';  r.aT_tr = NaN(n, 1);  r.aT_tr(k) = arrayfun(@(h) 1 / local_cpar(h), r.h(k));  if any(findall(0, 'Type', 'figure') == 20); close(20); end
f = figure(20);  set(f, 'Color', 'w', 'Position', [60 60 1750 820], 'Visible', 'on', 'Name', 'current state');  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a = nexttile(tl);  hold(a, 'on');  h1 = plot(a, t(k), r.aT_tr(k) - r.aN_tr(k), 'Color', cT, 'LineWidth', 1.4);  h2 = plot(a, t(k), r.aT(k) - r.aN(k), 'Color', cE, 'LineWidth', 0.7);  ylim(a, [-0.1 0.6]);  ylabel(a, [AB '_{11} - ' AB '_{33}']);  legend(a, [h1 h2], {'True', 'Estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
b = nexttile(tl);  hold(b, 'on');  plot(b, t(k), er(k), 'Color', cE, 'LineWidth', 1.1);  ylim(b, [0 16]);  ylabel(b, ['angle error of w' char(770) '  [deg]']);  xlabel(b, 'Time (sec)');
for q = [a b]; xlim(q, [0 t(end)]);  for x = [10 11 12]; xline(q, x, 'k:', 'HandleVisibility', 'off'); end;  box(q, 'on');  set(q, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(q.Toolbar, 'Visible', 'off'); end
exportgraphics(f, fullfile(fo, 'fig_1102_status.png'), 'Resolution', 120);
S = load(fullfile(fo, 'temp_hold_angle12.mat'), 'rs');  e11 = [];  e33 = [];  eo = [];
for q = 1:numel(S.rs); h = S.rs{q};  m = h.t >= 9 & h.t < 10;  e11 = [e11; h.aT(m) - h.aT_tr(m)];  e33 = [e33; h.aN(m) - h.aN_tr(m)];  eo = [eo; (h.aT_tr(m) - h.aN_tr(m)) .* sind(h.thx(m)); (h.aT_tr(m) - h.aN_tr(m)) .* sind(h.thy(m))]; end
fprintf('9-10 s, rms over 5 seeds: a_11 error %.4f, a_33 error %.4f, off-diagonal error (g x angle) %.4f\n', rms(e11), rms(e33), rms(eo));
for x = [5 10 11.5 15 25 35]; i = k(find(t(k) >= x, 1));  fprintf('t %.1f s: angle error rms %.2f deg, true gap of gains %.3f\n', x, er(i), r.aT_tr(i) - r.aN_tr(i)); end
function c = local_cpar(h)
    [c, ~] = calc_correction_functions(h);
end
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
