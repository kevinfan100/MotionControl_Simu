% STATUS: SCRATCH -- 2026-10-29 (user): a'-hat_33 during a 1-s constant-speed descent and the 1-s hold after it (no regulation, no scan),
%   to design a slope P_fix schedule. Arms: R report setting (plane, 22.2 R -> 1.10 R, no hold) with the report slope P_fix 0.4 (1.1/h_d)^2 |
%   C current (void sphere radius 15 um, start 10 deg off the top 15 um up, hold 10 s, 15 um -> h*, theta-hat[0] = 0, + kappa) with the report
%   P_fix | C with law L. Seeds 1-10. Figure: top row seed 1, a'_33 (true, red) and a'-hat_33 (blue), log axis, with the sqrt P_fix used
%   (dashed, report formula / law L); bottom row ratio a'-hat / true, mean and +-1 sd over the seeds. Console: ratio in windows.
%   Output test_results/temp_3d_wsonly/fig_1029_ap_schedule.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  FS = 13;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
common = struct('theta_x', 0, 'theta_y', 0, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
                'gate_one', true, 'doc_timing', true, 'theta_out', true, 'tau_a', 2, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
                'scan_len', 0, 'reg_off', true, 'descent_T', 1, 'q_theta_from', 1e9, 'sP_fix_theta_deg', 2, 'tag', 'as');
arms = {'R: report setting, report P_{fix}', 'C: current, report P_{fix}', 'C: current, law L'};  t0 = [0 10 10];
[IA, ID] = ndgrid(1:3, 1:10);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = common;  o.seeds = ID(q);
    if IA(q) == 1; o.h0 = 22.2;  o.hold_until = 0;  o.descent_v = (22.2 - 1.10) * Rr;
    else; o.h0 = 15 / Rr;  o.hold_until = 10;  o.descent_v = 15 - hs * Rr;  o.wall = 'hemi';  o.hemi_r = 15;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.kappa_state = true; end
    o.ap_pfix_law = IA(q) == 3;  o.T = t0(IA(q)) + 2;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'apN', r.apN, 'aN', r.aN, 'h', r.h, 'hd', r.hd_int);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
f = figure('Color', 'w', 'Position', [20 20 2400 1100]);  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
tg = (-0.2:5 / 1612:2)';  W = {[0 0.5], [0.5 0.9], [0.9 1], [1 1.2], [1.2 2]};
for ia = 1:3
    Q = NaN(numel(tg), 10);
    for id = 1:10
        r = rs{ia, id};  tt = r.t - t0(ia);  ok = (1:numel(tt))' <= r.kend & isfinite(r.apN);  ks = find(ok);  ks = ks(1:5:end);
        Q(:, id) = interp1(tt(ks), r.apN(ks) ./ slope(r.h(ks)), tg);
    end
    s = '';  for w = 1:numel(W); m = tg >= W{w}(1) & tg < W{w}(2);  v = median(Q(m, :), 1, 'omitnan');  s = [s sprintf(' | %.1f-%.1f s: %.2f (sd %.2f)', W{w}(1), W{w}(2), mean(v), std(v))]; end %#ok<AGROW>
    fprintf('%-34s: a''-hat/true (median in the window, mean over seeds)%s\n', strrep(arms{ia}, '_{fix}', '_fix'), s);
    r = rs{ia, 1};  tt = r.t - t0(ia);  k = find(tt >= -0.2 & (1:numel(tt))' <= r.kend);  kd = k(1:5:end);
    hdc = max(r.hd(k), 1.1);  pf = (0.4 * (1.1 ./ hdc).^2);  if ia == 3; pf = (1 / 9) * (1 - min(r.aN(k), 1)).^2; end
    ax = nexttile(tl, ia);  hold on;  box on;
    h1 = plot(tt(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.4);  h2 = plot(tt(k), max(r.apN(k), 1e-4), 'Color', cE, 'LineWidth', 1.1);  h3 = plot(tt(k), pf, 'k--', 'LineWidth', 1.4);
    xline(1, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  set(ax, 'YScale', 'log');  ylim([1e-3 3]);  xlim([-0.2 2]);
    ylabel(sprintf('%s''_{33}', AB));  title(sprintf('%s, seed 1', arms{ia}), 'FontSize', FS);
    legend([h1 h2 h3], {sprintf('%s''_{33} (true)', AB), sprintf('%s%s''_{33}', AB, hatc), '\surd P_{fix} used'}, 'Location', 'southeast');
    a2 = nexttile(tl, 3 + ia);  hold on;  box on;  mu = median(Q, 2, 'omitnan');  lo = prctile(Q, 25, 2);  hi = prctile(Q, 75, 2);  okb = isfinite(lo) & isfinite(hi);
    fill([tg(okb); flipud(tg(okb))], [lo(okb); flipud(hi(okb))], [0.75 0.82 1], 'EdgeColor', 'none', 'DisplayName', '25-75 % of 10 seeds');
    plot(tg, mu, 'Color', cE, 'LineWidth', 2, 'DisplayName', 'median');  yline(1, 'r-', 'LineWidth', 1.6, 'HandleVisibility', 'off');
    xline(1, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim([-0.5 2.5]);  xlim([-0.2 2]);
    ylabel(sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB));  xlabel('time from the descent start  (sec)');  if ia == 1; legend('Location', 'northwest'); end
    for a = [ax a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_ap_schedule.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
