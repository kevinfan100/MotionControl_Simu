% STATUS: SCRATCH -- 2026-10-27 (user): document figures for 1026_local_angle_estimator.tex Section 6, in the style of the 1025 document
%   (fig_1025_fixed_{15,5}um.png and fig_1025_a_quality.png). Wall theta_y = 20 deg, theta-hat[0] = 0, gains at the reflection seeds,
%   prior 10 deg, slope q = 0, 20 s, seeds 1-10, paired: left 1025 estimated angle (form 'estimated'), right 1026 local version (form
%   'local', one-sided gate 2 sqrt P_gap as a simulation setting).
%   10-27 later (user): the document shows the local version only, no gain figure. Figure 1 (fig_1026_fixed.png): columns 15 | 5 um,
%   rows theta_y error, theta_x error (theta-hat - theta), dark = best seed, light = next two (the 1025 runs stay for the console numbers).
%     15 um (user 10-27: "a better response"): the 3 seeds with the smallest mean |angle error| over 2-20 s of the local version;
%     5 um: seeds 1-3. Same seeds in both columns.
%   Figure 4 (fig_1026_angle_update.png): 15 um, best seed, theta-hat, sum L_10 e, sum delta-theta-hat.
%   Outputs reference/eq17_analysis/derivation/figures/
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
Rr = 2.25;  T = 20;  seeds = 1:10;  n = numel(seeds);  um = [15 5];  FS = 15;  hatc = char(770);  th = ['θ' hatc];  ah = ['a' hatc];  mu = char(181);
abar = char(257);  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];  cO = [0.85 0.4 0];
fm = {'estimated', 'local'};  ttl = {sprintf('estimated angle, %s[0] = 0', th), sprintf('local version, %s[0] = 0', th)};
nj = numel(um) * 2 * n;  jobs = cell(1, nj);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
parfor q = 1:nj
    ih = 1 + floor((q - 1) / (2 * n));  rr = q - (ih - 1) * 2 * n;  f = 1 + floor((rr - 1) / n);  i = rr - (f - 1) * n;
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', um(ih) / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', fm{f}, 'seeds', seeds(i), 'tag', 'doc');
    if f == 2; o.gap_gate = 2;  o.gate_one = true;  o.theta_out = true; end   % 10-27 (user): document timing -- angle from a_uw[k+1], a_vw[k+1] / gap[k], no separate L_10
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
get = @(ih, f) jobs((ih - 1) * 2 * n + (f - 1) * n + (1:n));
% ---- seed choice at 15 um (local version) ----
rl = get(1, 2);  sc = zeros(1, n);  sg = zeros(1, n);
for i = 1:n
    r = rl{i};  k = find(r.t >= 2 & r.t <= T);  k = k(k <= r.kend);
    sc(i) = mean(abs(20 - r.thy(k)) + abs(r.thx(k)));
    sg(i) = mean(abs(r.aT_true(k) - r.aT(k)) + abs(r.aN_true(k) - r.aN(k)));
end
[~, oa] = sort(sc);  sel15 = oa(1:3);  [~, og] = sort(sg);  selg = og(1:3);
fprintf('15 um angle scores %s -> seeds %s | gain scores %s -> seeds %s\n', mat2str(round(sc, 2)), mat2str(seeds(sel15)), mat2str(round(sg, 4)), mat2str(seeds(selg)));
sel = {sel15, 1:3};
% ---- figure 1: angle error of the local version, columns 15 | 5 um, style of fig_1025_fixed ----
fig = figure('Color', 'w', 'Position', [60 80 1200 720]);  tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:2
    for ih = 1:2
        nexttile;  hold on;  box on;  rs = get(ih, 2);  ss = sel{ih};
        for j = 3:-1:1
            r = rs{ss(j)};  k = 1:r.kend;  s1 = j == 1;
            if row == 1; y = r.thy(k) - 20; col = cBl; if s1; col = cB; end; else; y = r.thx(k); col = cGl; if s1; col = cG; end; end
            h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
        end
        yline(0, 'r', 'LineWidth', 2.5);  uistack(hs, 'top');  xlim([0 T]);  ylim([-30 30]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(gca().Toolbar, 'Visible', 'off');
        if row == 1
            if ih == 1; ylabel('\theta_y error  [\circ]'); end
            legend(hs, sprintf('%s_y - θ_y,  seed %d   (%d %sm)', th, seeds(ss(1)), um(ih), mu), 'Location', 'northoutside');
        else
            if ih == 1; ylabel('\theta_x error  [\circ]'); end
            legend(hs, sprintf('%s_x - θ_x', th), 'Location', 'northoutside');  xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1026_fixed.png'), 'Resolution', 150);  close(fig);
% ---- figure 4: how the angle is updated (15 um, best seed) ----
rb = rl{sel15(1)};  k = 1:rb.kend;  tk = rb.t(k);  nmx = 'xy';
fig = figure('Color', 'w', 'Position', [60 60 1500 750]);  tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:2
    ax = 3 - row;  nexttile;  hold on;  box on;  tv = [0 20];  thv = rb.thy(k);  if ax == 1; thv = rb.thx(k); end
    yline(tv(ax), 'r', 'LineWidth', 2.5);
    h1 = plot(tk, thv, 'k', 'LineWidth', 2);  h2 = plot(tk, cumsum(rb.dLe(k, ax)), 'Color', cO, 'LineWidth', 1.4);  h3 = plot(tk, cumsum(rb.dth(k, ax)), 'Color', cB, 'LineWidth', 1.8);
    ylim([-10 30]);  xlim([0 T]);  ylabel(sprintf('θ_%s  [\\circ]', nmx(ax)));
    legend([h1 h2 h3], {sprintf('%s_%s  (15 %sm, seed %d)', th, nmx(ax), mu, seeds(sel15(1))), 'Σ L_{10} e', sprintf('Σ δ%s_%s', th, nmx(ax))}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(gca().Toolbar, 'Visible', 'off');
end
xlabel('Time (sec)');
exportgraphics(fig, fullfile(fo, 'fig_1026_angle_update.png'), 'Resolution', 150);  close(fig);
for ih = 1:2
    for f = 1:2
        rs = get(ih, f);  EY = cellfun(@(r) r.thy(r.kend) - 20, rs);  EX = cellfun(@(r) r.thx(r.kend), rs);
        e10 = cellfun(@(r) r.thy(find(r.t >= 10, 1)) - 20, rs);
        fprintf('%2d um %-10s rms at 10 / 20 s: theta_y %.1f / %.1f, theta_x %.1f / %.1f deg\n', um(ih), fm{f}, sqrt(mean(e10.^2)), sqrt(mean(EY.^2)), ...
                sqrt(mean(cellfun(@(r) r.thx(find(r.t >= 10, 1)), rs).^2)), sqrt(mean(EX.^2)));
    end
end
system(['open ' fullfile(fo, 'fig_1026_fixed.png')]);
