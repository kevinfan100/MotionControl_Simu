% STATUS: SCRATCH -- 2026-11-01 (user): test of the two-factor form: y1-w update of a'-hat_33 ~ P(a', a_33) x (fixed correlation of nu_w with F_w). 0.5 s bins from t_scan - 2 to t_scan + 3,
%   y = sum K(15,w) nu_w, x = mean P(a', a_33) (log L.B15 slot 8). Plane theta_y 10, current flow, kappa off, 5 seeds. Arms: P alive 40 | P alive 10 | C 40 | control without a_uw-hat | V-hat moving (arm 1).
%   Pre-registered: holds if pooled R^2 >= 0.5, intercept within 2 sem, arm slopes one sign and within a factor 2. Sanity: C 40 summed update positive; non-finite bins counted.
%   Console: pooled fit, slope per arm (with sem over bins), hold-only and scan-only pooled slopes. Figure fig_1101_two_factor.png.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  FS = 14;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', 15, 'tag', 'tf', 'freeze_angle', true);
arm = {'P alive, 40', 'P alive, 10', 'C, 40', 'control without a_{uw}, 40', 'V-hat moving, 40'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
pctRunOnAll warning('off', 'all');
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    if IA(q) == 4; o.ctrl_no_off_from = tS; end
    if IA(q) == 5; o = rmfield(o, 'freeze_angle'); end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'dap', r.dap(:, 3), 'P', r.B15(:, 8));
end
ed = (tS - 2):0.5:(tS + 3);  NB = numel(ed) - 1;  X = NaN(NA, NS, NB);  Y = X;
for ia = 1:NA
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';
        for b = 1:NB
            g = kk(r.t(kk) >= ed(b) & r.t(kk) < ed(b + 1));
            if numel(g) >= 800 && all(isfinite(r.dap(g))) && all(isfinite(r.P(g))); X(ia, id, b) = mean(r.P(g));  Y(ia, id, b) = sum(r.dap(g)); end
        end
    end
end
x = X(:);  y = Y(:);  ok = isfinite(x);  [ax, ~, sx] = local_fit(x(ok), y(ok));
fprintf('bins %d of %d | pooled y = %+.3f + %+.1f x, sem %.3f, %.1f | R^2 %.2f\n', sum(ok), numel(ok), ax(1), ax(2), sx(1), sx(2), local_r2(x(ok), y(ok), ax));
for ia = 1:NA
    xi = reshape(X(ia, :, :), [], 1);  yi = reshape(Y(ia, :, :), [], 1);  g = isfinite(xi);  [a, ~, s] = local_fit(xi(g), yi(g));
    fprintf('%-28s bins %3d | intercept %+.3f +- %.3f | slope %+.1f +- %.1f | R^2 %.2f | mean x %+.5f | mean y %+.3f\n', arm{ia}, sum(g), a(1), s(1), a(2), s(2), local_r2(xi(g), yi(g), a), mean(xi(g)), mean(yi(g)));
end
for w = 1:2
    if w == 1; bs = 1:4; lb = 'before t_scan'; else; bs = 5:NB; lb = 'from t_scan'; end
    xw = reshape(X(:, :, bs), [], 1);  yw = reshape(Y(:, :, bs), [], 1);  g = isfinite(xw);  [a, ~, s] = local_fit(xw(g), yw(g));
    fprintf('%-14s pooled: intercept %+.3f +- %.3f, slope %+.1f +- %.1f, R^2 %.2f\n', lb, a(1), s(1), a(2), s(2), local_r2(xw(g), yw(g), a));
end
f = figure('Color', 'w', 'Position', [20 20 900 760]);  a = axes(f);  hold(a, 'on');  box(a, 'on');  cs = lines(NA);
for ia = 1:NA
    xi = reshape(X(ia, :, :), [], 1);  yi = reshape(Y(ia, :, :), [], 1);  plot(a, 1e3 * xi, yi, 'o', 'Color', cs(ia, :), 'MarkerFaceColor', cs(ia, :), 'MarkerSize', 5, 'DisplayName', arm{ia});
end
xr = [min(x(ok)) max(x(ok))];  plot(a, 1e3 * xr, ax(1) + ax(2) * xr, 'k-', 'LineWidth', 1.6, 'DisplayName', 'pooled fit');  xline(a, 0, 'k:', 'HandleVisibility', 'off');  yline(a, 0, 'k:', 'HandleVisibility', 'off');
xlabel(a, 'P(a'', a_{33}) x 10^3, 0.5 s mean');  ylabel(a, 'y_1-w update of a'' over 0.5 s');  legend(a, 'Location', 'northoutside', 'NumColumns', 3);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);
fn = fullfile(fo, 'fig_1101_two_factor.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function [a, r, s] = local_fit(x, y)
    A = [ones(size(x)), x];  a = A \ y;  r = y - A * a;  s = sqrt(diag(inv(A' * A)) * sum(r.^2) / (numel(y) - 2));
end
function v = local_r2(x, y, a)
    r = y - a(1) - a(2) * x;  v = 1 - sum(r.^2) / sum((y - mean(y)).^2);
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
