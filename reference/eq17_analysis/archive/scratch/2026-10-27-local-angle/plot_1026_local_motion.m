% STATUS: SCRATCH -- 2026-10-26 (user): how much faster does the angle of the local version converge with motion? 20 deg wall, 15 um,
%   theta-hat[0] = 0, gains at the seeds, prior 10 deg, one-sided gate 2 sqrt P, seeds 1-3, 10 s. Arms:
%   (A) fixed point; (B) circle 3 um radius, 8 Hz, in the estimated tangent plane (u-hat, v-hat), amplitude ramped over 1 s
%   (run_t3_xyz_fixed opts.dither, as the 10-21 flow), slope q = 0; (C) as B with the slope q on (P_fix^2 I, as the 10-21 flow).
%   Criteria written before the run: time when |theta_y error| first stays below 5 deg; error at 1 / 2 / 5 / 10 s.
%   Rows: theta_y error, theta_x error, gap h (um); seed 1 dark, 2-3 light. Output test_results/temp_3d_wsonly/fig_1026_local_motion.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:3;  n = numel(seeds);  hatc = char(770);  th = ['θ' hatc];  nm = 'xy';
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];  cK = [0.2 0.2 0.2];  cKl = [0.7 0.7 0.7];
lab = {'fixed point', 'circle 3 µm 8 Hz', 'circle 3 µm 8 Hz, slope q on'};  na = numel(lab);
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
jobs = cell(1, na * n);
parfor q = 1:na * n
    c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'seeds', seeds(i), 'tag', 'mo');
    if c >= 2; o.dither = [3 8 1]; end
    if c == 3; o.ap_q = true; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
runs = reshape(jobs, n, na)';
fig = figure('Color', 'w', 'Position', [40 60 2300 1050]);  tiledlayout(3, na, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    for c = 1:na
        nexttile;  hold on;  box on;
        for i = n:-1:1
            r = runs{c, i};  k = 1:r.kend;  s1 = i == 1;  lw = 1 + 0.8 * s1;
            switch row
                case 1; col = cBl; if s1; col = cB; end;  h = plot(r.t(k), 20 - r.thy(k), 'Color', col, 'LineWidth', lw);
                case 2; col = cGl; if s1; col = cG; end;  h = plot(r.t(k), -r.thx(k), 'Color', col, 'LineWidth', lw);
                case 3; col = cKl; if s1; col = cK; end;  h = plot(r.t(k), r.h(k) * Rr, 'Color', col, 'LineWidth', lw);
            end
            if s1; hs = h; end
        end
        if row < 3; yline(0, 'r', 'LineWidth', 2.5);  ylim([-15 25]); else; ylim([12 18]); end
        uistack(hs, 'top');  xlim([0 T]);
        if c == 1
            switch row
                case 1; ylabel(sprintf('θ_y - %s_y  [\\circ]', th));
                case 2; ylabel(sprintf('θ_x - %s_x  [\\circ]', th));
                case 3; ylabel(sprintf('h  [%sm]', char(181)));
            end
        end
        if row == 1; legend(hs, sprintf('%s,  seed 1 (2-3 light)', lab{c}), 'Location', 'northoutside'); end
        if row == 3; xlabel('Time (sec)'); end
        set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1026_local_motion.png'), 'Resolution', 150);  close(fig);
for c = 1:na
    for i = 1:n
        r = runs{c, i};  k = 1:r.kend;  e = abs(20 - r.thy(k));  ex = abs(r.thx(k));
        last = find(max(e, ex) > 5, 1, 'last');  tin = NaN;  if isempty(last); tin = 0; elseif last < numel(k); tin = r.t(last + 1); end
        ks = arrayfun(@(s) find(r.t >= s - 1e-9, 1), [1 2 5 10]);
        fprintf('%-30s seed %d: both angle errors stay < 5 deg from t = %.2f s | theta_y err %s | theta_x err %s | contact %d\n', ...
                lab{c}, i, tin, mat2str(round(20 - r.thy(ks)', 1)), mat2str(round(-r.thx(ks)', 1)), r.contact);
    end
end
system(['open ' fullfile(fo, 'fig_1026_local_motion.png')]);
