% STATUS: SCRATCH -- 2026-10-26 (user): two-sided gate |a_par-hat - a_perp-hat| > 2 sqrt(P_gap) vs one-sided a_par-hat - a_perp-hat >
%   2 sqrt(P_gap) (physics: a_par > a_perp near a wall). Local version, 15 um, wall theta_y = 20 deg, theta-hat[0] = 0, prior 10 deg,
%   gains at the seeds, seeds 1-3, 10 s. Rows: theta_y error, theta_x error; seed 1 dark, 2-3 light. Console: 10 s errors, time the
%   gate last closed, largest angle step at a gate opening. 10-26 later (user): columns = 2 sqrt P | |gap| > 0.01 | |gap| > 0.04.
%   Output test_results/temp_3d_wsonly/fig_1026_gate_abs.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
wall = [0 20];  Rr = 2.25;  T = 10;  seeds = 1:3;  n = numel(seeds);  hatc = char(770);  th = ['θ' hatc];  nm = 'xy';
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
lab = {'gate |gap| > 2√P', 'gate |gap| > 0.01', 'gate |gap| > 0.04'};  na = numel(lab);   % 10-26 (user): fixed |gap| thresholds
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
jobs = cell(1, na * n);
parfor q = 1:na * n
    c = 1 + floor((q - 1) / n);  i = q - (c - 1) * n;
    o = struct('theta_x', wall(1), 'theta_y', wall(2), 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'seeds', seeds(i), 'tag', 'gs');
    if c == 2; o.gap_min = 0.01; elseif c == 3; o.gap_min = 0.04; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
runs = reshape(jobs, n, na)';
fig = figure('Color', 'w', 'Position', [60 60 2300 750]);  tiledlayout(2, na, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:2
    ax = 3 - row;
    for c = 1:na
        nexttile;  hold on;  box on;
        for i = n:-1:1
            r = runs{c, i};  k = 1:r.kend;  s1 = i == 1;
            if ax == 2; col = cBl; if s1; col = cB; end;  e = wall(2) - r.thy(k); else; col = cGl; if s1; col = cG; end;  e = wall(1) - r.thx(k); end
            h = plot(r.t(k), e, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
        end
        yline(0, 'r', 'LineWidth', 2.5);  uistack(hs, 'top');  ylim([-15 25]);  xlim([0 T]);
        if c == 1; ylabel(sprintf('θ_%s - %s_%s  [\\circ]', nm(ax), th, nm(ax))); end
        if row == 1; legend(hs, sprintf('%s,  seed 1 (2-3 light)', lab{c}), 'Location', 'northoutside'); else; xlabel('Time (sec)'); end
        set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1026_gate_abs.png'), 'Resolution', 150);  close(fig);
for c = 1:na
    for i = 1:n
        r = runs{c, i};  k = 1:r.kend;  g = r.gate(k)';  op = find(g(2:end) == 1 & g(1:end-1) == 0) + 1;  lc = find(g == 0, 1, 'last');
        dmax = 0;  if ~isempty(op); dmax = max(max(abs(r.dth(op, :)))); end
        fprintf('%-28s seed %d: err at 10 s theta_y %+.2f theta_x %+.2f | gate last closed %.2f s, closed %d steps | largest step at an opening %.2f deg\n', ...
                lab{c}, i, wall(2) - r.thy(r.kend), -r.thx(r.kend), r.t(lc), sum(g == 0), dmax);
    end
end
system(['open ' fullfile(fo, 'fig_1026_gate_abs.png')]);
