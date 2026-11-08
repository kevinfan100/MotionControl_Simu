% STATUS: SCRATCH -- 2026-11-02 (user): recover the wall from the scan: p - R (1 + h*) w-hat, w-hat from the estimated angles, h* = h-bar at a_d minus 1 (constant), each scan line drawn as a line (41-sample moving mean, v steps removed) in the radial profile over the bead, fillet and glass. No legend. temp_rho4_report.mat (plot_1102_rho4_report.m: rho 4, a_d .07, seed 1).
%   Console also gives the arm with the real-time estimate of h-bar and the true normal. Sanity: floor points near z = 0, top of the bead near z = 10. Output fig_1102_wall_recover.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.07;  rb = 5;  rho = 4;  MU = [char(956) 'm'];  cE = [0 0.25 0.85];
load(fullfile(fo, 'temp_rho4_report.mat'), 'r');
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  hst = hs - 1;  fprintf('h-bar at a_d %.4f, h* %.4f, R (1 + h*) = %.3f um\n', hs, hst, Rr * (1 + hst));
ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= r.rs_tend);  p = r.p(ks, :);
[Pr, Pz] = local_profile(rb, rho, 40);
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);  hh = interp1(ag, hg, min(max(r.aN(ks), ag(1)), ag(end)));  hh = hh(:);
Wc = local_w(r.thx(ks), r.thy(ks));  Wt = local_w(r.th_tr(ks, 1), r.th_tr(ks, 2));
A = {p - Rr * (1 + hst) * Wc, p - Rr * hh .* Wc, p - Rr * (1 + hst) * Wt};  nm = {'constant h*, estimated w', 'real-time h, estimated w', 'constant h*, true w'};
for q = 1:3
    W = A{q};  rq = hypot(W(:, 1), W(:, 2));  dd = local_dist(rq, W(:, 3), Pr, Pz);  on = hypot(p(:, 1), p(:, 2)) < 12;
    sp = W(:, 3) > 5 & rq < 8;  Am = [2 * W(sp, :), ones(sum(sp), 1)];  cf = Am \ sum(W(sp, :).^2, 2);  c0 = cf(1:3)';  rf = sqrt(cf(4) + c0 * c0');
    fprintf('%s: distance to the true wall on the bead mean %.3f, 95%% %.3f, max %.3f um; floor z mean %+.3f um; fitted sphere centre (%.2f, %.2f, %.2f), radius %.2f um\n', nm{q}, mean(dd(on)), prctile(dd(on), 95), max(dd(on)), mean(W(rq > 12, 3)), c0, rf);
end
W = A{1};  rq = hypot(W(:, 1), W(:, 2));
f = figure('Color', 'w', 'Position', [20 20 1300 820]);  a = axes(f);  hold(a, 'on');  th = linspace(0, pi, 400);
tx = r.thx(ks) * pi / 180;  ty = r.thy(ks) * pi / 180;  vh = [sin(ty(:)) .* sin(tx(:)), cos(tx(:)), cos(ty(:)) .* sin(tx(:))];  dv = [0; sum(diff(p, 1, 1) .* vh(2:end, :), 2)];  stepv = conv(double(movmean(dv, 21) > 0.012), ones(61, 1), 'same') > 0;
Wl = movmean(W, 41, 1);  rl = hypot(Wl(:, 1), Wl(:, 2));  zl = Wl(:, 3);  rl(stepv) = NaN;  zl(stepv) = NaN;  plot(a, rl, zl, 'Color', cE, 'LineWidth', 0.7);
plot(a, rb * sin(th), rb + rb * cos(th), 'Color', [0.35 0.35 0.35], 'LineWidth', 2.2);  plot(a, [0 20], [0 0], 'k', 'LineWidth', 2.2);
ix = Pz < rho - rho * cos(0) + rho & Pr > rb * 0.9 & Pz > 0 & Pz < 4.5;  plot(a, Pr(ix), Pz(ix), 'r:', 'LineWidth', 1.6);
xlim(a, [0 20]);  ylim(a, [-0.5 13]);  axis(a, 'equal');  xlim(a, [0 20]);  ylim(a, [-0.5 13]);  box(a, 'on');  xlabel(a, ['radial position  [' MU ']']);  ylabel(a, ['z  [' MU ']']);  set(a, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
exportgraphics(f, fullfile(fo, 'fig_1102_wall_recover.png'), 'Resolution', 130);  close(f);

function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function d = local_dist(rq, zq, Pr, Pz)
    d = zeros(numel(rq), 1);  for i = 1:numel(rq); d(i) = sqrt(min((Pr - rq(i)).^2 + (Pz - zq(i)).^2)); end
end
function [PR, PZ] = local_profile(rb, rho, Rm)
    Rt = 2 * sqrt(rb * rho);  a_t = atan2(Rt, rho - rb);  b = linspace(0, a_t, 800);  c = linspace(a_t, 0, 600);  PR = [rb * sin(b), Rt - rho * sin(c), linspace(Rt, Rm, 1500)];  PZ = [rb + rb * cos(b), rho - rho * cos(c), zeros(1, 1500)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
