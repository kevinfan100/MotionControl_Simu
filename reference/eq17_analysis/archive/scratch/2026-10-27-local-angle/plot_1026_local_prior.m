% STATUS: SCRATCH -- 2026-10-26 (user): the local version (run_t3_xyz_fixed form 'local'), horizontal wall (case 1), 15 um,
%   theta-hat[0] = (0, 0), gains at the seeds, gate 2 sqrt P, 10 s, seeds 1-3. Columns: angle prior sqrt P_theta[0] = 10 deg |
%   0 deg (the user: "prior 10 -> 0"). With a zero angle prior the off-diagonal prior is zero too (tied), so the angle cannot move.
%   Style as plot_1025_fixed.m: rows theta_y error, theta_x error, gains (true red, estimate); seed 1 dark, seeds 2-3 light; no P band.
%   wall = [theta_x theta_y], sp = the two priors (deg) may be set before calling.
%   Output test_results/temp_3d_wsonly/fig_1026_local_prior_wall<tx>_<ty>_sp<a>_<b>.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
if ~exist('wall', 'var'); wall = [0 0]; end
Rr = 2.25;  T = 10;  seeds = 1:3;  n = numel(seeds);  FS = 15;  hatc = char(770);  th = ['θ' hatc];  ah = ['a' hatc];
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];  cO = [0.85 0.4 0];  cOl = [1 0.75 0.55];
if ~exist('sp', 'var'); sp = [10 0]; end                       % 10-26 (user): sp = [10 5] with wall = [0 5] (case 2)
ttl = arrayfun(@(v) sprintf('%sP_{θ}[0] = %g%s', char(8730), v, char(176)), sp, 'UniformOutput', false);
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
runs = cell(2, n);  jobs = cell(1, 2 * n);
parfor q = 1:2 * n
    c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;
    o = struct('theta_x', wall(1), 'theta_y', wall(2), 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'sP_theta_deg', sp(c), 'seeds', seeds(i), 'tag', 'pr');
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
for q = 1:2 * n; runs{1 + floor((q - 1) / n), q - floor((q - 1) / n) * n} = jobs{q}; end
fig = figure('Color', 'w', 'Position', [60 60 1700 1000]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    for c = 1:2
        nexttile;  hold on;  box on;
        for i = n:-1:1
            r = runs{c, i};  k = 1:r.kend;  s1 = (i == 1);  lw = 1 + 0.8 * s1;
            switch row
                case 1; col = cBl; if s1; col = cB; end;  h = plot(r.t(k), wall(2) - r.thy(k), 'Color', col, 'LineWidth', lw);
                case 2; col = cGl; if s1; col = cG; end;  h = plot(r.t(k), wall(1) - r.thx(k), 'Color', col, 'LineWidth', lw);
                case 3
                    cp = cBl;  cn = cOl;  if s1; cp = cB;  cn = cO; end
                    h = plot(r.t(k), r.aT(k), 'Color', cp, 'LineWidth', lw);  h2 = plot(r.t(k), r.aN(k), 'Color', cn, 'LineWidth', lw);
            end
            if s1; hs = h;  if row == 3; hs2 = h2; end; end
        end
        if row < 3
            yline(0, 'r', 'LineWidth', 2.5);  ylim([-15 15]);  uistack(hs, 'top');
        else
            yline(runs{1, 1}.aT_true(1), 'r', 'LineWidth', 2.5);  yline(runs{1, 1}.aN_true(1), 'r', 'LineWidth', 2.5);  ylim([0.75 1.0]);
        end
        switch row
            case 1
                if c == 1; ylabel(sprintf('θ_y - %s_y  [\\circ]', th)); end
                legend(hs, sprintf('%s,  seed 1 (seeds 2-3 light)', ttl{c}), 'Location', 'northoutside');
            case 2
                if c == 1; ylabel(sprintf('θ_x - %s_x  [\\circ]', th)); end
            case 3
                if c == 1; ylabel('gain'); end
                legend([hs hs2], {sprintf('%s_%s', ah, char(8741)), sprintf('%s_%s', ah, char(8869))}, 'Location', 'northoutside', 'Orientation', 'horizontal');
                xlabel('Time (sec)');
        end
        xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    end
end
exportgraphics(fig, fullfile(fo, sprintf('fig_1026_local_prior_wall%d_%d_sp%d_%d.png', wall, sp)), 'Resolution', 150);  close(fig);
for c = 1:2
    for i = 1:n
        r = runs{c, i};
        fprintf('%-18s seed %d: theta-hat at 10 s (%.2f, %.2f) | max |theta-hat| %.2f deg | a_par %.4f a_perp %.4f (true %.4f %.4f)\n', ...
                strrep(ttl{c}, char(8730), 'sqrt'), seeds(i), r.thx(r.kend), r.thy(r.kend), max(abs([r.thx(1:r.kend); r.thy(1:r.kend)])), ...
                r.aT(r.kend), r.aN(r.kend), r.aT_true(1), r.aN_true(1));
    end
end
