% STATUS: SCRATCH -- 2026-10-25 (user, test 2): estimated angle (1025 Section 6, run_t3_xyz_fixed form 'estimated') vs 1015
%   (run_t2_3d_reg fixed_point) on the same seeds: 20 deg wall, 15 um, theta-hat[0] = (0, 15) deg, 10 s, seeds 1-3, slope q on in
%   both (1015 always has it). Same states and dynamics, different process-output form (six elements in x y z vs four local readouts).
%   Criterion written before the run: the paired difference (estimated - 1015) of theta_y, theta_x, a_perp stays well below the spread
%   across seeds (ratio of rms paired difference to seed sd < 0.5 over 2-10 s). Columns 1015 | estimated (shared y), rows theta_y error,
%   theta_x error, a_perp (red = true). Output test_results/temp_3d_wsonly/fig_1025_vs_1015.png | EXPIRES: when test 3 is done
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
seeds = 1:3;  T = 10;  h0 = 15 / 2.25;  th0 = [0 15];  n = numel(seeds);
E = cell(1, n);  F = cell(1, n);
for i = 1:n
    R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', h0, 'seeds', seeds(i), 'T', T, 'form', 'estimated', 'th0_deg', th0, 'ap_q', true, 'tag', 'est'));
    E{i} = R.runs{1};
    R = run_t2_3d_reg(struct('theta_y', 20, 'h0', h0, 'seeds', seeds(i), 'fixed_point', true, 't_scan', 100, 'T_scan', T - 100, ...
                             'th0_deg', th0, 'save', false, 'tag', 'ref'));
    F{i} = R.runs{1};
end
% ---- criterion: paired difference vs seed spread (2-10 s) ----
k = find(E{1}.t >= 2 & E{1}.t <= T);  nm = {'theta_y', 'theta_x', 'a_perp'};
get = @(r, j) [r.thy(k), r.thx(k), r.aN(k)] * ((1:3)' == j);
for j = 1:3
    dE = zeros(numel(k), n);  sE = zeros(numel(k), n);
    for i = 1:n; dE(:, i) = get(E{i}, j) - get(F{i}, j);  sE(:, i) = get(E{i}, j); end
    rp = sqrt(mean(dE(:).^2));  ss = mean(std(sE, 0, 2));
    fprintf('%-8s rms(estimated - 1015) %.4g | seed sd %.4g | ratio %.2f -> %s\n', nm{j}, rp, ss, rp / ss, local_pf(rp / ss < 0.5));
end
% ---- figure ----
FS = 15;  hatc = char(770);  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
ttl = {'1015 (four local readouts)', 'estimated angle (six elements)'};  runs = {F, E};
fig = figure('Color', 'w', 'Position', [80 80 1300 950]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    for c = 1:2
        nexttile;  hold on;  box on;  hs = gobjects(0);
        for i = n:-1:1
            r = runs{c}{i};  kk = 1:r.kend;  s1 = (seeds(i) == 1);
            switch row
                case 1; y = r.thy(kk) - 20;  col = cBl; if s1; col = cB; end
                case 2; y = r.thx(kk);       col = cGl; if s1; col = cG; end
                case 3; y = r.aN(kk);        col = cBl; if s1; col = cB; end
            end
            h = plot(r.t(kk), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
        end
        if row < 3; yline(0, 'r', 'LineWidth', 2.5); else; yline(E{1}.aN_true(1), 'r', 'LineWidth', 2.5); end
        uistack(hs, 'top');  xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        switch row
            case 1
                ylim([-8 8]);  if c == 1; ylabel('\theta_y error  [\circ]'); end
                legend(hs, sprintf('%s_y - %s_y,  seed 1   (%s)', ['θ' hatc], char(952), ttl{c}), 'Location', 'northoutside');
                text(0.97, 0.88, sprintf('15 %sm, %s_y[0] = 15%s', char(181), ['θ' hatc], char(176)), 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
            case 2
                ylim([-8 8]);  if c == 1; ylabel('\theta_x error  [\circ]'); end
                legend(hs, sprintf('%s_x - %s_x', ['θ' hatc], char(952)), 'Location', 'northoutside');
            case 3
                ylim([0.7 1]);  if c == 1; ylabel([char(257) '_' char(8869)]); end
                legend(hs, sprintf('%s_%s  (red = true)', ['a' hatc], char(8869)), 'Location', 'northoutside');  xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1025_vs_1015.png'), 'Resolution', 150);
close(fig);

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
