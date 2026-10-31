% STATUS: SCRATCH -- 2026-10-28 (user): document figure fig_1026_gain_angle.png for 1026_local_angle_estimator.tex Section 5: document
%   version (form 'local', doc_timing + theta_out, one-sided gate 2 sqrt P), wall theta_y = 20 deg, theta-hat[0] = 0, gains at the seeds,
%   20 s, seed 1 (parallel run, same noise as fig_1026_fixed). Columns 15 | 5 um; rows: a-hat_11, a-hat_33 (truth red); theta-hat_y,
%   theta-hat_x = theta-hat[0] + sum delta-theta-hat (truth red).
%   Also (console): why the 1025 figure looks faster -- 1025 estimated angle with gains at 1 (as plotted in the 1025 document) and with
%   gains at the seeds, vs the document version, seeds 1-10 at 15 um: time both angle errors stay < 5 deg, rms at 1/2/5/10 s.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
Rr = 2.25;  T = 20;  FS = 15;  hatc = char(770);  th = ['θ' hatc];  ah = ['a' hatc];  mu = char(181);  um = [15 5];
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
% ---- figure ----
rf = cell(1, 2);
parfor ih = 1:2
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', um(ih) / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'form', 'local', ...
               'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'seeds', 1, 'tag', 'ga');
    R = run_t3_xyz_fixed(o);  rf{ih} = R.runs{1};
end
fig = figure('Color', 'w', 'Position', [60 80 1500 720]);  tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:2
    for ih = 1:2
        r = rf{ih};  k = 1:r.kend;  t = r.t(k);  nexttile;  hold on;  box on;
        if row == 1
            yline(r.aT_true(1), 'r', 'LineWidth', 2.5);  yline(r.aN_true(1), 'r', 'LineWidth', 2.5);
            h1 = plot(t, r.aT(k), 'Color', [0 0.25 0.85], 'LineWidth', 1.6);  h2 = plot(t, r.aN(k), 'Color', [0.85 0.4 0], 'LineWidth', 1.6);
            g = [r.aT_true(1) r.aN_true(1)];  ylim([min(g) - 0.08, max(g) + 0.08]);
            legend([h1 h2], {sprintf('%s_{11}', ah), sprintf('%s_{33}   (%d %sm, seed 1)', ah, um(ih), mu)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
            if ih == 1; ylabel('gain'); end
        else
            yline(20, 'r', 'LineWidth', 2.5);  yline(0, 'r', 'LineWidth', 2.5);
            h1 = plot(t, r.thy(k), 'Color', [0 0.25 0.85], 'LineWidth', 1.6);  h2 = plot(t, r.thx(k), 'Color', [0 0.5 0.2], 'LineWidth', 1.6);
            ylim([-10 30]);  legend([h1 h2], {sprintf('%s_y', th), sprintf('%s_x', th)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
            if ih == 1; ylabel('[\circ]'); end;  xlabel('Time (sec)');
        end
        xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(gca().Toolbar, 'Visible', 'off');
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1026_gain_angle.png'), 'Resolution', 150);  close(fig);
% ---- comparison with the 1025 estimated angle ----
seeds = 1:10;  n = numel(seeds);  arms = {'1025 estimated, gains start at 1', '1025 estimated, gains at seeds', 'document version'};  na = 3;
jobs = cell(1, na * n);
parfor q = 1:na * n
    c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'seeds', seeds(i), 'tag', 'cm');
    if c <= 2; o.form = 'estimated'; if c == 2; o.gain0 = 'seed'; end
    else; o.form = 'local';  o.gain0 = 'seed';  o.gap_gate = 2;  o.gate_one = true;  o.doc_timing = true;  o.theta_out = true; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
t = jobs{1}.t;  ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);
for c = 1:na
    rs = jobs((c - 1) * n + (1:n));  tin = NaN(1, n);  EY = zeros(4, n);  EX = EY;
    for i = 1:n
        r = rs{i};  k = 1:r.kend;  e = max(abs(r.thy(k) - 20), abs(r.thx(k)));  last = find(e > 5, 1, 'last');
        if isempty(last); tin(i) = 0; elseif last < numel(k); tin(i) = r.t(last + 1); end
        EY(:, i) = r.thy(ks) - 20;  EX(:, i) = r.thx(ks);
    end
    fprintf('%-34s | both < 5 deg from (per seed) %s s, median %.2f | rms theta_y 1/2/5/10 s %s | theta_x %s\n', arms{c}, mat2str(round(tin, 1)), ...
            median(tin, 'omitnan'), mat2str(round(sqrt(mean(EY.^2, 2))', 1)), mat2str(round(sqrt(mean(EX.^2, 2))', 1)));
end
