% STATUS: SCRATCH -- 2026-10-27 (user): the four gain estimates of the local version (1026_local_angle_estimator.tex, run_t3_xyz_fixed
%   form 'local'), fixed point 15 um, wall theta_y = 20 deg, theta-hat[0] = 0, gains at the seeds, prior 10 deg, one-sided gate 2 sqrt P,
%   seeds 1-3, 10 s. Rows: a_par, a_perp, a_uw, a_vw. Red = truth in the local coordinate of the estimate at that step (for a_uw, a_vw
%   this is (a_par - a_perp) x the angle error); blue = estimate after the feedback (before the angle step and the change of coordinate).
%   Seed 1 dark, seeds 2-3 light. Output test_results/temp_3d_wsonly/fig_1026_local_gains.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:3;  n = numel(seeds);  hatc = char(770);  ah = ['a' hatc];
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cR = [0.9 0 0];  cRl = [1 0.6 0.6];
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
runs = cell(1, n);
parfor i = 1:n
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'seeds', seeds(i), 'tag', 'lg');
    R = run_t3_xyz_fixed(o);  runs{i} = R.runs{1};
end
nmr = {sprintf('%s_%s', ah, char(8741)), sprintf('%s_%s', ah, char(8869)), sprintf('%s_{uw}', ah), sprintf('%s_{vw}', ah)};
yl = {[0.85 1.0], [0.75 0.9], [-0.03 0.03], [-0.03 0.03]};
fig = figure('Color', 'w', 'Position', [60 40 1500 1300]);  tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:4
    nexttile;  hold on;  box on;
    for i = n:-1:1
        r = runs{i};  k = 1:r.kend;  s1 = i == 1;  t = r.t(k);
        switch row
            case 1; est = r.aT(k);  tru = r.aT_true(k);
            case 2; est = r.aN(k);  tru = r.aN_true(k);
            case 3; est = r.off_post(k, 1);  tru = r.off_tr(k, 1);
            case 4; est = r.off_post(k, 2);  tru = r.off_tr(k, 2);
        end
        cb = cBl;  cr = cRl;  if s1; cb = cB;  cr = cR; end
        h2 = plot(t, tru, 'Color', cr, 'LineWidth', 1 + 1.2 * s1);  h1 = plot(t, est, 'Color', cb, 'LineWidth', 0.8 + 0.6 * s1);
        if s1; hs = [h1 h2]; end
    end
    uistack(hs, 'top');  ylim(yl{row});  xlim([0 T]);  ylabel(nmr{row});
    if row == 1; legend(hs, {'estimate (seed 1; seeds 2-3 light)', 'true'}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    if row == 4; xlabel('Time (sec)'); end
    set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);
end
exportgraphics(fig, fullfile(fo, 'fig_1026_local_gains.png'), 'Resolution', 150);  close(fig);
ks = arrayfun(@(s) find(runs{1}.t >= s - 1e-9, 1), [0.5 1 2 5 10]);
for i = 1:n
    r = runs{i};
    fprintf('seed %d (t = 0.5/1/2/5/10 s): a_par err %s | a_perp err %s | a_uw true %s est %s | a_vw true %s est %s\n', i, ...
            mat2str(round(r.aT_true(ks)' - r.aT(ks)', 4)), mat2str(round(r.aN_true(ks)' - r.aN(ks)', 4)), ...
            mat2str(round(r.off_tr(ks, 1)', 4)), mat2str(round(r.off_post(ks, 1)', 5)), mat2str(round(r.off_tr(ks, 2)', 4)), mat2str(round(r.off_post(ks, 2)', 5)));
end
system(['open ' fullfile(fo, 'fig_1026_local_gains.png')]);
