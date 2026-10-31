% STATUS: SCRATCH -- 2026-10-26 (user): first run of 1026_local_angle_estimator.tex (run_t3_xyz_fixed opts.form = 'local') next to
%   the estimated angle of 1025 Section 6 (form 'estimated'). 20 deg wall, fixed point 15 um, seed 1, 5 s, theta-hat[0] = (0, 15) deg
%   (5 deg off, inside the small-angle assumption). Columns: estimated angle (1025), local version with the reflection seeds as initial
%   gains (same as the 1025 arm), local version with the true gains and a small gain prior (reference arm, not a result).
%   Rows: theta_y-hat (red = 20), theta_x-hat (red = 0), a_par-hat - a_perp-hat (red = true). Criterion written before the run:
%   the local version is wired correctly if the reference arm converges like the 1025 arm.
%   10-26 (user): fourth arm = local version with the angle step only when |a_par-hat - a_perp-hat| > 2 sqrt(P_gap) (drawn third).
%   Output test_results/temp_3d_wsonly/fig_1026_local_vs_B.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 5;  FS = 14;  cB = [0 0.25 0.85];  cG = [0 0.5 0.2];  hatc = char(770);
base = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', 1, 'T', T, 'aprime', true, 'th0_deg', [0 15], 'gain0', 'seed');
o = {base, base, base, base};
o{1}.form = 'estimated';  o{1}.tag = 'B';
o{2}.form = 'local';  o{2}.tag = 'L_seed';
o{4}.form = 'local';  o{4}.tag = 'L_gate';  o{4}.gap_gate = 2;
o{3}.form = 'local';  o{3}.tag = 'L_true';  o{3}.gain0 = [0.9156 0.8327];  o{3}.sP_gain = 0.003;
arm = {'estimated angle (1025)', 'local version', 'local version, true gains (reference)', 'local version, angle step if |gap| > 2 sqrt P'};
runs = cell(1, 4);
for c = 1:4; R = run_t3_xyz_fixed(o{c});  runs{c} = R.runs{1}; end
fig = figure('Color', 'w', 'Position', [40 60 2200 900]);
tiledlayout(3, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    for c = [1 2 4 3]
        nexttile;  hold on;  box on;  r = runs{c};  k = 1:r.kend;  t = r.t(k);
        switch row
            case 1; yline(20, 'r', 'LineWidth', 2.5);  plot(t, r.thy(k), 'Color', cB, 'LineWidth', 1.6);  ylim([0 40]);
                if c == 1; ylabel(['θ' hatc '_y  [\circ]']); end;  text(0.03, 0.93, arm{c}, 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');
            case 2; yline(0, 'r', 'LineWidth', 2.5);  plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.6);  ylim([-20 20]);
                if c == 1; ylabel(['θ' hatc '_x  [\circ]']); end
            case 3; yline(r.aT_true(1) - r.aN_true(1), 'r', 'LineWidth', 2.5);  plot(t, r.aT(k) - r.aN(k), 'Color', cB, 'LineWidth', 1.2);
                yline(0, ':', 'Color', [0.3 0.3 0.3]);  ylim([-0.15 0.2]);
                if c == 1; ylabel(sprintf('%s_%s - %s_%s', ['a' hatc], char(8741), ['a' hatc], char(8869))); end;  xlabel('Time (sec)');
        end
        xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1026_local_vs_B.png'), 'Resolution', 150);
close(fig);
for c = 1:4
    r = runs{c};  k = 1:r.kend;  kk = find(r.t(k) >= 4);
    fprintf('%-40s | theta-hat at 5 s (%.2f, %.2f) | |theta_y error| mean 4-5 s %.2f | gap below 0: %.1f %% of steps\n', arm{c}, ...
            r.thx(r.kend), r.thy(r.kend), mean(abs(r.thy(kk) - 20)), 100 * mean(r.aT(k) - r.aN(k) < 0));
    if c == 4; kg = find(r.gate(k), 1); fprintf('    gate open %.1f %% of steps, first at t = %.3f s\n', 100 * mean(r.gate(k)), r.t(kg)); end
end
