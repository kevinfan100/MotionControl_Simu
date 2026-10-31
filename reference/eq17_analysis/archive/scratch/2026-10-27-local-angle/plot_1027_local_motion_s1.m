% STATUS: SCRATCH -- 2026-10-27 (user): the document version (1026_local_angle_estimator.tex; run_t3_xyz_fixed form 'local', one-sided gate
%   2 sqrt P) at 15 um, wall theta_y = 20 deg, theta-hat[0] = 0, gains at the seeds, prior 10 deg, 10 s: fixed point vs circle 3 um 8 Hz in
%   the estimated tangent plane (amplitude ramped over 1 s; slope q = 0). One seed shown (user: "a good single seed"): over seeds 1-10,
%   smallest mean over 2-10 s of |theta_y err| + |theta_x err| + 100 (|a_par err| + |a_perp err|) in the circle arm; the same seed in both.
%   Columns: fixed | circle. Rows: theta_y error, theta_x error, a_par (estimate, truth at the true height), a_perp.
%   Output test_results/temp_3d_wsonly/fig_1027_local_motion_s1.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);  hatc = char(770);  th = ['θ' hatc];  ah = ['a' hatc];  mu = char(181);
lab = {'fixed point', sprintf('circle 3 %sm, 8 Hz', mu)};
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
jobs = cell(1, 2 * n);
parfor q = 1:2 * n
    c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'seeds', seeds(i), 'tag', 'ms');
    if c == 2; o.dither = [3 8 1]; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
sc = zeros(1, n);
for i = 1:n
    r = jobs{n + i};  k = find(r.t >= 2 & r.t <= 10);  k = k(k <= r.kend);
    sc(i) = mean(abs(20 - r.thy(k)) + abs(r.thx(k)) + 100 * (abs(r.aT_tr(k) - r.aT(k)) + abs(r.aN_tr(k) - r.aN(k))));
end
[~, ib] = min(sc);  sb = seeds(ib);  rs = {jobs{ib}, jobs{n + ib}};
fprintf('circle-arm scores (2-10 s): %s -> seed %d\n', mat2str(round(sc, 2)), sb);
fig = figure('Color', 'w', 'Position', [40 40 2000 1250]);  tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:4
    for c = 1:2
        r = rs{c};  k = 1:r.kend;  t = r.t(k);  a = nexttile;  hold on;  box on;
        switch row
            case 1; yline(0, 'r', 'LineWidth', 2.5);  h = plot(t, 20 - r.thy(k), 'Color', [0 0.25 0.85], 'LineWidth', 1.6);  ylim([-10 25]);
                if c == 1; ylabel(sprintf('θ_y - %s_y  [\\circ]', th)); end
                legend(h, sprintf('%s, seed %d', lab{c}, sb), 'Location', 'northoutside');
            case 2; yline(0, 'r', 'LineWidth', 2.5);  plot(t, -r.thx(k), 'Color', [0 0.5 0.2], 'LineWidth', 1.6);  ylim([-10 25]);
                if c == 1; ylabel(sprintf('θ_x - %s_x  [\\circ]', th)); end
            case 3; h2 = plot(t, r.aT_tr(k), 'r', 'LineWidth', 2);  h1 = plot(t, r.aT(k), 'Color', [0 0.25 0.85], 'LineWidth', 1.2);  ylim([0.85 1.0]);
                if c == 1; ylabel(sprintf('%s_%s', ah, char(8741))); end
                legend([h1 h2], {'estimate', 'true'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
            case 4; plot(t, r.aN_tr(k), 'r', 'LineWidth', 2);  plot(t, r.aN(k), 'Color', [0 0.25 0.85], 'LineWidth', 1.2);  ylim([0.75 0.9]);
                if c == 1; ylabel(sprintf('%s_%s', ah, char(8869))); end;  xlabel('Time (sec)');
        end
        xlim([0 T]);  set(a, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1027_local_motion_s1.png'), 'Resolution', 150);  close(fig);
for c = 1:2
    r = rs{c};  ks = arrayfun(@(s) find(r.t >= s - 1e-9, 1), [0.5 1 2 5 10]);  k = 1:r.kend;
    last = find(max(abs(20 - r.thy(k)), abs(r.thx(k))) > 5, 1, 'last');  tin = r.t(min(last + 1, numel(k)));
    fprintf('%-18s seed %d (0.5/1/2/5/10 s): theta_y err %s | theta_x err %s | a_par err %s | a_perp err %s | both angles < 5 deg from %.2f s\n', ...
            lab{c}, sb, mat2str(round(20 - r.thy(ks)', 1)), mat2str(round(-r.thx(ks)', 1)), mat2str(round(r.aT_tr(ks)' - r.aT(ks)', 3)), ...
            mat2str(round(r.aN_tr(ks)' - r.aN(ks)', 3)), tin);
end
% all seeds, for the record
for c = 1:2
    tins = zeros(1, n);  e10 = zeros(2, n);
    for i = 1:n
        r = jobs{(c - 1) * n + i};  k = 1:r.kend;  last = find(max(abs(20 - r.thy(k)), abs(r.thx(k))) > 5, 1, 'last');
        if isempty(last); tins(i) = 0; elseif last >= numel(k); tins(i) = NaN; else; tins(i) = r.t(last + 1); end
        e10(:, i) = [20 - r.thy(r.kend); -r.thx(r.kend)];
    end
    fprintf('%-18s all seeds: time both angles stay < 5 deg %s s | rms at 10 s theta_y %.1f theta_x %.1f deg\n', lab{c}, mat2str(round(tins, 2)), sqrt(mean(e10(1, :).^2)), sqrt(mean(e10(2, :).^2)));
end
system(['open ' fullfile(fo, 'fig_1027_local_motion_s1.png')]);
