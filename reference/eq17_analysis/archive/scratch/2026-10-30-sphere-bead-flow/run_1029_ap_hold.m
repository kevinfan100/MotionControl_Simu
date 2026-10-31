% STATUS: SCRATCH -- 2026-10-29 (user): the 1-s hold after a 1-s constant-speed descent (no regulation, no scan) -- the gap is constant,
%   so the slope should not be driven as in motion. C current flow (void sphere radius 15 um, start 10 deg off the top 15 um up, hold 10 s,
%   15 um -> h*, theta-hat[0] = 0, + kappa), report slope P_fix in the descent; in the hold: (1) report P_fix kept | (2) slope Q = 0
%   (a' a constant, opts.ap_q_off_from) | (3) law L (opts.ap_pfix_law_from_scan, switch at the stop). Seeds 1-10.
%   Figure: row 1 seed 1 a'_33 (true red, estimate blue, log axis); row 2 a'-hat / true; row 3 e_a33 = a_33 - a-hat_33 (median and 25-75 %
%   over the seeds). Console: windows. Output test_results/temp_3d_wsonly/fig_1029_ap_hold.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  FS = 13;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
common = struct('theta_x', 0, 'theta_y', 0, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
                'gate_one', true, 'doc_timing', true, 'theta_out', true, 'tau_a', 2, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
                'scan_len', 0, 'reg_off', true, 'descent_T', 1, 'q_theta_from', 1e9, 'sP_fix_theta_deg', 2, 'tag', 'as');
arms = {'hold: report P_{fix} kept', 'hold: slope Q = 0', 'hold: law L'};  t0 = [10 10 10];
[IA, ID] = ndgrid(1:3, 1:10);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = common;  o.seeds = ID(q);
    o.h0 = 15 / Rr;  o.hold_until = 10;  o.descent_v = 15 - hs * Rr;  o.wall = 'hemi';  o.hemi_r = 15;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.kappa_state = true;
    if IA(q) == 2; o.ap_q_off_from = 11; end
    if IA(q) == 3; o.ap_pfix_law = true;  o.ap_pfix_law_from_scan = true; end
    o.T = t0(IA(q)) + 2;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'apN', r.apN, 'aN', r.aN, 'aT', r.aN_tr, 'h', r.h, 'hd', r.hd_int, 'tsc', r.tsc);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
f = figure('Color', 'w', 'Position', [20 20 2400 1450]);  tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
tg = (-0.2:5 / 1612:2)';  W = {[0.5 0.9], [0.9 1], [1 1.2], [1.2 2], [1.8 2]};
for ia = 1:3
    Q = NaN(numel(tg), 10);  E = Q;
    for id = 1:10
        r = rs{ia, id};  tt = r.t - t0(ia);  ok = (1:numel(tt))' <= r.kend & isfinite(r.apN);  ks = find(ok);  ks = ks(1:5:end);
        Q(:, id) = interp1(tt(ks), r.apN(ks) ./ slope(r.h(ks)), tg);  E(:, id) = interp1(tt(ks), r.aT(ks) - r.aN(ks), tg);
    end
    s = '';  for w = 1:numel(W); m = tg >= W{w}(1) & tg < W{w}(2);  v = median(Q(m, :), 1, 'omitnan');  e = median(E(m, :), 1, 'omitnan');
        s = [s sprintf(' | %.1f-%.1f s: ratio %.2f (sd %.2f), e_a %+.4f (sd %.4f)', W{w}(1), W{w}(2), mean(v), std(v), mean(e), std(e))]; end %#ok<AGROW>
    fprintf('%-26s%s\n', strrep(arms{ia}, '_{fix}', '_fix'), s);
    r = rs{ia, 1};  tt = r.t - t0(ia);  k = find(tt >= -0.2 & (1:numel(tt))' <= r.kend);  kd = k(1:5:end);
    ax = nexttile(tl, ia);  hold on;  box on;
    h1 = plot(tt(kd), slope(r.h(kd)), 'Color', cT, 'LineWidth', 2.4);  h2 = plot(tt(k), max(r.apN(k), 1e-4), 'Color', cE, 'LineWidth', 1.1);
    xline(1, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  set(ax, 'YScale', 'log');  ylim([1e-2 3]);  xlim([-0.2 2]);
    ylabel(sprintf('%s''_{33}', AB));  title(sprintf('%s, seed 1', arms{ia}), 'FontSize', FS);
    legend([h1 h2], {sprintf('%s''_{33}', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'southeast');
    a2 = nexttile(tl, 3 + ia);  a3 = nexttile(tl, 6 + ia);
    for pr = 1:2
        if pr == 1; a = a2;  M = Q;  yl = [-0.5 2.5];  y0 = 1;  lab = sprintf('%s%s''_{33} / %s''_{33}', AB, hatc, AB);
        else; a = a3;  M = E;  yl = [-0.06 0.06];  y0 = 0;  lab = sprintf('e_{%s33} = %s_{33} - %s%s_{33}', AB, AB, AB, hatc); end
        axes(a);  hold on;  box on;  mu = median(M, 2, 'omitnan');  lo = prctile(M, 25, 2);  hi = prctile(M, 75, 2);  okb = isfinite(lo) & isfinite(hi);
        fill([tg(okb); flipud(tg(okb))], [lo(okb); flipud(hi(okb))], [0.75 0.82 1], 'EdgeColor', 'none', 'DisplayName', '25-75 % of 10 seeds');
        plot(tg, mu, 'Color', cE, 'LineWidth', 2, 'DisplayName', 'median');  yline(y0, 'r-', 'LineWidth', 1.6, 'HandleVisibility', 'off');
        xline(1, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim(yl);  xlim([-0.2 2]);  ylabel(lab);
        if pr == 2; xlabel('time from the descent start  (sec)'); end;  if ia == 1 && pr == 1; legend('Location', 'northwest'); end
    end
    for a = [ax a2 a3]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_ap_hold.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
