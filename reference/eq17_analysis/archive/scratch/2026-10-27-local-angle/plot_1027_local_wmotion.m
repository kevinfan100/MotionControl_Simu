% STATUS: SCRATCH -- 2026-10-27 (user): does motion along w speed up the angle in the document version (1026; run_t3_xyz_fixed form
%   'local', one-sided gate 2 sqrt P)? Wall theta_y = 20 deg, theta-hat[0] = 0, gains at the seeds, prior 10 deg, slope q = 0, 10 s, seeds
%   1-10. Arms: (A) fixed at 15 um; (B) circle 3 um 8 Hz in the tangent plane (u v); (C) oscillation along w-hat 3 um 8 Hz around 15 um
%   (opts.wdither, height 12-18 um); (D) ramp toward the wall along -w-hat 5 um/s for 10 um (15 -> 5 um in 2 s, then hold; Meng Fig. 40 type).
%   Criteria written before the run: time both angle errors stay < 5 deg (per seed), rms at 1/2/5/10 s; contact count.
%   apq = true (set before calling): the three motion arms use the slope Q = P_fix^2 I (opts.ap_q), as the 10-21 flow.
%   Figure: rows rms theta_y error, rms theta_x error (10 seeds); four arms. Output test_results/temp_3d_wsonly/fig_1027_local_wmotion.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);  hatc = char(770);  th = ['θ' hatc];  mu = char(181);
if ~exist('apq', 'var'); apq = false; end                     % 10-27 (user): motion arms with the slope Q = P_fix^2 I (opts.ap_q)
lab = {'fixed 15 µm', 'circle u-v 3 µm 8 Hz', 'w oscillation 3 µm 8 Hz', 'ramp to wall 15 → 5 µm'};
col = {[0.45 0.45 0.45], [0 0.3 0.85], [0.85 0.4 0], [0.5 0.1 0.6]};  na = numel(lab);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
jobs = cell(1, na * n);
parfor q = 1:na * n
    c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'seeds', seeds(i), 'tag', 'wm');
    if c == 2; o.dither = [3 8 1]; elseif c == 3; o.wdither = [3 8 1]; elseif c == 4; o.ramp = [5 10]; end
    if c >= 2 && apq; o.ap_q = true; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
fig = figure('Color', 'w', 'Position', [60 60 1600 800]);  tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile;  hold on;  box on;  a2 = nexttile;  hold on;  box on;  hl = gobjects(1, na);
t = jobs{1}.t;  ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);
for c = 1:na
    rs = jobs((c - 1) * n + (1:n));  EY = NaN(numel(t), n);  EX = EY;  tin = zeros(1, n);  nc = 0;
    for i = 1:n
        r = rs{i};  k = 1:r.kend;  EY(k, i) = 20 - r.thy(k);  EX(k, i) = -r.thx(k);  nc = nc + r.contact;
        last = find(max(abs(EY(k, i)), abs(EX(k, i))) > 5, 1, 'last');
        if isempty(last); tin(i) = 0; elseif last >= numel(k); tin(i) = NaN; else; tin(i) = r.t(last + 1); end
    end
    ey = sqrt(mean(EY.^2, 2, 'omitnan'));  ex = sqrt(mean(EX.^2, 2, 'omitnan'));
    hl(c) = plot(a1, t, ey, 'Color', col{c}, 'LineWidth', 2);  plot(a2, t, ex, 'Color', col{c}, 'LineWidth', 2);
    hmin = min(cellfun(@(r) min(r.h(1:r.kend)), rs)) * Rr;
    fprintf('%-26s | contact %d | min h %.1f um | both < 5 deg from %s s (median %.2f) | rms theta_y 1/2/5/10 s %s | theta_x %s\n', lab{c}, nc, hmin, ...
            mat2str(round(tin, 2)), median(tin, 'omitnan'), mat2str(round(ey(ks)', 1)), mat2str(round(ex(ks)', 1)));
end
ylabel(a1, sprintf('rms(θ_y - %s_y)  [\\circ]', th));  ylabel(a2, sprintf('rms(θ_x - %s_x)  [\\circ]', th));  xlabel(a2, 'Time (sec)');
legend(a1, hl, lab, 'Location', 'northoutside', 'Orientation', 'horizontal');
set([a1 a2], 'XLim', [0 T], 'YLim', [0 15], 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2);
set(a1.Toolbar, 'Visible', 'off');  set(a2.Toolbar, 'Visible', 'off');
exportgraphics(fig, fullfile(fo, sprintf('fig_1027_local_wmotion%s.png', repmat('_apq', 1, apq))), 'Resolution', 150);  close(fig);
system(['open ' fullfile(fo, sprintf('fig_1027_local_wmotion%s.png', repmat('_apq', 1, apq)))]);
