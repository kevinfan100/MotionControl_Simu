% STATUS: SCRATCH -- 2026-10-25 (user): first run of 1025_computed_estimated_angle.tex Section 6 (theta as a state, theta_m with the cross
%   terms in R) next to Section 5 (computed angle), 20 deg wall, fixed points 15 / 50 um, seeds 1-3, 5 s, A-bar' on (run_t3_xyz_fixed
%   opts.aprime, opts.th_state). One figure per height; columns = computed angle | angle as a state; rows = theta_y error (blue),
%   theta_x error (green), theta-hat minus theta(A-hat) (angle-state arm: the lock), sqrt P of theta. Seed 1 dark, seeds 2-3 light.
%   Console: first use of theta_m, time of divergence (first NaN). Output test_results/temp_3d_wsonly/fig_t3_thstate_{15,50}um.png
%   | EXPIRES: when option (B) replaces Section 6
linerr = @(e) local_break(mod(e + 90, 180) - 90);
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  um = [15 50];  seeds = 1:3;  T = 5;  FS = 15;  hatc = char(770);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];  cK = [0.2 0.2 0.2];  cKl = [0.7 0.7 0.7];
arm = {'computed angle (Section 5)', 'angle as a state (Section 6)'};
for ih = 1:2
    runs = cell(2, numel(seeds));
    for is = 1:numel(seeds)
        o = struct('theta_y', 20, 'h0', um(ih) / Rr, 'seeds', seeds(is), 'T', T, 'aprime', true, 'tag', 'c2');
        R = run_t3_xyz_fixed(o);  runs{1, is} = R.runs{1};
        o.th_state = true;  o.tag = 'c3';  R = run_t3_xyz_fixed(o);  runs{2, is} = R.runs{1};
        r3 = runs{2, is};  k1 = find(r3.use_thm, 1);  kn = find(isnan(r3.thy), 1);
        fprintf('%d um seed %d: theta_m first used t = %.3f s; theta-hat NaN from t = %.3f s\n', um(ih), seeds(is), r3.t(k1), r3.t(max([kn, 1])));
    end
    fig = figure('Color', 'w', 'Position', [80 80 1300 1150]);
    tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:4
        for c = 1:2
            nexttile;  hold on;  box on;  hs = gobjects(0);
            for is = numel(seeds):-1:1
                r = runs{c, is};  s1 = (seeds(is) == 1);  k = 1:numel(r.t);
                switch row
                    case 1; y = linerr(r.thy(k) - 20);          col = cBl; if s1; col = cB; end
                    case 2; y = linerr(r.thx(k));               col = cGl; if s1; col = cG; end
                    case 3; y = linerr(r.thy(k) - r.thA(k, 2)); col = cKl; if s1; col = cK; end
                    case 4
                        if c == 1; y = NaN(size(k)); else; y = r.sPth(k, 2); end
                        col = cBl; if s1; col = cB; end
                end
                h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
            end
            if row < 4; yline(0, 'r', 'LineWidth', 2.5); end
            if ~isempty(hs); uistack(hs, 'top'); end
            xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
            switch row
                case 1
                    ylim([-30 30]);  if c == 1; ylabel('\theta_y error  [\circ]'); end
                    legend(hs, sprintf('%s_y - %s_y,  seed 1   (%s)', ['θ' hatc], 'θ', arm{c}), 'Location', 'northoutside');
                    text(0.97, 0.88, sprintf('%d %sm', um(ih), char(181)), 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
                         'FontSize', FS, 'FontWeight', 'bold');
                case 2
                    ylim([-30 30]);  if c == 1; ylabel('\theta_x error  [\circ]'); end
                    legend(hs, sprintf('%s_x - %s_x', ['θ' hatc], 'θ'), 'Location', 'northoutside');
                case 3
                    ylim([-30 30]);  if c == 1; ylabel('[\circ]'); end
                    legend(hs, sprintf('%s_y - %s_y(%s)', ['θ' hatc], 'θ', ['A' hatc]), 'Location', 'northoutside');
                case 4
                    ylim([0 12]);  if c == 1; ylabel('[\circ]'); end
                    if c == 2; legend(hs, sprintf('%sP of %s_y', char(8730), 'θ'), 'Location', 'northoutside'); end
                    xlabel('Time (sec)');
            end
        end
    end
    exportgraphics(fig, fullfile(fo, sprintf('fig_t3_thstate_%dum.png', um(ih))), 'Resolution', 150);
    close(fig);
end

function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
