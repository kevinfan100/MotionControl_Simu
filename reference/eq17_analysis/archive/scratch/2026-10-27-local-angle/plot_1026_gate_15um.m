% STATUS: SCRATCH -- 2026-10-26 (user): the 10-25 fixed-point test (20 deg wall, 15 um, seeds 1-10, 10 s, theta-hat[0] = 0, prior
%   10 deg; as plot_1025_est_slow_doc.m) for the local version with off-diagonal states and the angle step only when
%   |a_par-hat - a_perp-hat| > 2 sqrt(P_gap) (run_t3_xyz_fixed form 'local', gap_gate 2). Arms:
%   (1) estimated angle of 1025 as on 10-25 (gains start at 1); (2) the same with the gains at the reflection seeds;
%   (3) local version + gate, gains at the reflection seeds (gains at 1 give a zero prior of a_uw, a_vw: the angle never moves).
%   Figure 1: rms over seeds of theta_y - theta-hat_y and theta_x - theta-hat_x. Figure 2 (seed 1, arm 3): a_uw-hat after the
%   update vs the true a_uw in the same local coordinate, the angle step delta-theta-hat_y, theta-hat_y, gate.
%   10-26 later: arm 3 = the code written as the document (theta-hat[k+1] = theta-hat[k] + delta-theta-hat[k] + L e, L = F_e K).
%   Outputs test_results/temp_3d_wsonly/fig_1026_gate_15um_rms.png, fig_1026_gate_15um_auw_s1.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);  FS = 15;  hatc = char(770);
col = {[0.45 0.45 0.45], [0 0.3 0.85], [0.85 0.4 0], [0.6 0.2 0.6]};
arm = {'estimated angle (1025), gains start at 1', 'estimated angle (1025), gains start at seeds', 'local version (document 10-26), gate 2√P'};
if ~exist('th0', 'var'); th0 = [0 0]; end                      % 10-26 (user): th0 = [0 15] -> small-angle test (5 deg off)
sfx = '';  if th0(2) ~= 0; sfx = sprintf('_th0_%d', th0(2)); end
base = struct('theta_y', 20, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', th0);
o = {base, base, base};
o{1}.form = 'estimated';  o{1}.tag = 'g1';
o{2}.form = 'estimated';  o{2}.gain0 = 'seed';  o{2}.tag = 'g2';
o{3}.form = 'local';  o{3}.gain0 = 'seed';  o{3}.gap_gate = 2;  o{3}.tag = 'g3';
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
runs = cell(3, n);
for c = 1:3
    rc = cell(1, n);  oc = o{c};
    parfor i = 1:n
        oi = oc;  oi.seeds = seeds(i);  R = run_t3_xyz_fixed(oi);  rc{i} = R.runs{1};
    end
    runs(c, :) = rc;
end
% ---- figure 1: rms over seeds ----
t = runs{1, 1}.t;  fig = figure('Color', 'w', 'Position', [80 80 1500 560]);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = [nexttile(1), nexttile(2)];  hold(ax(1), 'on');  hold(ax(2), 'on');  box(ax(1), 'on');  box(ax(2), 'on');  hl = gobjects(1, 3);
for c = 1:3
    EY = NaN(numel(t), n);  EX = EY;
    for i = 1:n
        r = runs{c, i};  k = 1:r.kend;  EY(k, i) = 20 - r.thy(k);  EX(k, i) = 0 - r.thx(k);
    end
    ey = sqrt(mean(EY.^2, 2, 'omitnan'));  ex = sqrt(mean(EX.^2, 2, 'omitnan'));
    hl(c) = plot(ax(1), t, ey, 'Color', col{c}, 'LineWidth', 2);  plot(ax(2), t, ex, 'Color', col{c}, 'LineWidth', 2);
    ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);
    nd = sum(cellfun(@(r) any(abs(r.thy(1:r.kend)) > 90), runs(c, :)));
    fprintf('%-46s | rms theta_y err at 1/2/5/10 s %s | rms theta_x %s deg | seeds with |theta-hat_y| > 90: %d\n', arm{c}, ...
            mat2str(round(ey(ks)', 1)), mat2str(round(ex(ks)', 1)), nd);
    if c == 3
        g = cellfun(@(r) find(r.gate(1:r.kend), 1), runs(3, :));  og = cellfun(@(r) mean(r.gate(1:r.kend)), runs(3, :));
        fprintf('    gate: first open at %s s | open fraction %s\n', mat2str(round(t(g)', 3)), mat2str(round(og, 2)));
    end
end
set(ax, 'XLim', [0 T], 'YLim', [0 max(25 * (th0(2) == 0), 10)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
ylabel(ax(1), sprintf('rms(θ_y - θ%s_y)  [\\circ]', hatc));  ylabel(ax(2), sprintf('rms(θ_x - θ%s_x)  [\\circ]', hatc));
xlabel(ax(1), 'Time (sec)');  xlabel(ax(2), 'Time (sec)');
legend(ax(1), hl, arm, 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', 12);
exportgraphics(fig, fullfile(fo, ['fig_1026_gate_15um_rms' sfx '.png']), 'Resolution', 150);  close(fig);
% ---- figure 2: seed 1 of the gate arm ----
r = runs{3, 1};  k = 1:r.kend;  tk = r.t(k);
fig = figure('Color', 'w', 'Position', [80 80 1300 1000]);
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;  hold on;  box on;
yyaxis left;  h2 = plot(tk, r.off_post(k, 1) * 1e3, '-', 'Color', [0 0.3 0.85], 'LineWidth', 0.8);  ylim([-0.6 0.6]);
ylabel(sprintf('%s_{uw} \\times 10^3', ['a' hatc]));  set(gca, 'YColor', [0 0.3 0.85]);
yyaxis right;  h1 = plot(tk, r.off_tr(k, 1) * 1e3, '-', 'Color', 'r', 'LineWidth', 2.5);  ylim([-30 30]);  if th0(2) ~= 0; ylim([-10 10]); end
ylabel('true a_{uw} \times 10^3');  set(gca, 'YColor', 'r');
legend([h1 h2], {'true (local coordinate of the estimate, right axis)', 'estimate after the update (left axis)'}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;  yline(0, 'r', 'LineWidth', 2);
h3 = plot(tk, r.dLe(k, 2), 'Color', [0.85 0.4 0], 'LineWidth', 0.8);  h4 = plot(tk, r.dth(k, 2), 'Color', [0 0.3 0.85], 'LineWidth', 1);
ylabel('[\circ]');  ylim([-0.5 0.5]);
legend([h4 h3], {sprintf('δθ%s_y[k]  (from a%s_{uw}[k])', hatc, hatc), 'L_{10} e  (θ_y row)'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;  yline(20, 'r', 'LineWidth', 2.5);  plot(tk, r.thy(k), 'Color', [0 0.3 0.85], 'LineWidth', 1.6);
ylabel(sprintf('θ%s_y  [\\circ]', hatc));  ylim([-5 35]);  if th0(2) ~= 0; ylim([10 30]); end
nexttile;  hold on;  box on;  plot(tk, r.gate(k), 'k', 'LineWidth', 1);  ylim([-0.2 1.2]);  ylabel('gate open');  xlabel('Time (sec)');
set(findall(fig, 'Type', 'axes'), 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
exportgraphics(fig, fullfile(fo, ['fig_1026_gate_15um_auw_s1' sfx '.png']), 'Resolution', 150);  close(fig);
kz = find(tk >= 0.5 & tk <= 2);
fprintf('seed 1: sum over 10 s of delta-theta-hat_y %.2f deg, of L e (theta_y row) %.2f deg\n', sum(r.dth(k, 2)), sum(r.dLe(k, 2)));
fprintf('seed 1 gate arm: corr(a_uw-hat, true) 0.5-2 s %.2f | true a_uw at 0 / 1 / 5 / 10 s %s\n', corr(r.off_post(kz, 1), r.off_tr(kz, 1)), ...
        mat2str(round(r.off_tr(arrayfun(@(s) find(tk >= s - 1e-9, 1), [0 1 5 10]), 1)', 4)));
