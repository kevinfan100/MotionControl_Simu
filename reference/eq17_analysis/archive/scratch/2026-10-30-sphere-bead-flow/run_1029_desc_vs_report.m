% STATUS: SCRATCH -- 2026-10-29 (user): why the 1-s descent (fig_1029_desc1s, left) deviates more than the 1019 report Fig. 2.1.
%   Same estimator (form 'local', slope P_fix of the report, a' state), constant-speed descent in 1 s, no regulation, no scan, then held.
%   Arms move from the report setting to ours one change at a time:
%   R  plane wall theta 0, start 22.2 R, descent to 1.10 R at t = 0 (no hold), angle known (theta-hat = 0 correct)
%   H  as R + 10 s hold before the descent
%   S  as H but start 15 um, descent to h* = 2.37 um (1.054 R)
%   C  current: void sphere radius 15 um, start 10 deg off the top 15 um up, theta-hat[0] = 0, + kappa (fig_1029_desc1s)
%   Seeds 1-10. Console: a-hat_33 - a_33 averaged over the first / second half of the descent and over the 0.2 s after it (mean, sd over seeds),
%   and the max |a-hat_33 - a_33| during the descent. Figure: seed 1 of each arm, a_33 and a-hat_33 vs time from the descent start.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
common = struct('theta_x', 0, 'theta_y', 0, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
                'gate_one', true, 'doc_timing', true, 'theta_out', true, 'tau_a', 2, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
                'scan_len', 0, 'reg_off', true, 'descent_T', 1, 'q_theta_from', 1e9, 'sP_fix_theta_deg', 2, 'tag', 'dv');
arms = {'R: report setting (plane, 22.2 R -> 1.10 R, no hold)', 'H: R + 10 s hold', 'S: H, 15 um -> h* = 2.37 um', 'C: current (sphere, angle from 0, + kappa)'};
[IA, ID] = ndgrid(1:4, 1:10);  rs = cell(size(IA));  t0 = [0 10 10 10];
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = common;  o.seeds = ID(q);
    switch IA(q)
        case 1; o.h0 = 22.2;  o.hold_until = 0;  o.descent_v = (22.2 - 1.10) * Rr;
        case 2; o.h0 = 22.2;  o.hold_until = 10;  o.descent_v = (22.2 - 1.10) * Rr;
        case 3; o.h0 = 15 / Rr;  o.hold_until = 10;  o.descent_v = 15 - hs * Rr;
        case 4; o.h0 = 15 / Rr;  o.hold_until = 10;  o.descent_v = 15 - hs * Rr;  o.wall = 'hemi';  o.hemi_r = 15;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.kappa_state = true;
    end
    o.T = t0(IA(q)) + 1.2;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'aN', r.aN, 'aN_tr', r.aN_tr, 'h', r.h, 'contact', r.contact);
end
for ia = 1:4
    M = zeros(10, 4);
    for id = 1:10
        r = rs{ia, id};  tt = r.t - t0(ia);  e = r.aN - r.aN_tr;
        M(id, :) = [mean(e(tt >= 0 & tt < 0.5)), mean(e(tt >= 0.5 & tt < 1)), mean(e(tt >= 1 & tt < 1.2 & (1:numel(tt))' <= r.kend)), max(abs(e(tt >= 0 & tt < 1)))];
    end
    fprintf('%-52s: a-hat - a, descent 1st half %+.3f (sd %.3f), 2nd half %+.3f (sd %.3f), after %+.3f (sd %.3f) | max |err| %.3f (median of seeds)\n', ...
            arms{ia}, mean(M(:, 1)), std(M(:, 1)), mean(M(:, 2)), std(M(:, 2)), mean(M(:, 3)), std(M(:, 3)), median(M(:, 4)));
end
f = figure('Color', 'w', 'Position', [30 30 2300 600]);  tl = tiledlayout(1, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:4
    r = rs{ia, 1};  tt = r.t - t0(ia);  k = find(tt >= -0.05 & (1:numel(tt))' <= r.kend);
    ax = nexttile(tl);  hold on;  box on;  h1 = plot(tt(k), r.aN_tr(k), 'Color', cT, 'LineWidth', 2.4);  h2 = plot(tt(k), r.aN(k), 'Color', cE, 'LineWidth', 1.2);
    ylim([0 1.05]);  xlim([-0.05 1.2]);  xlabel('time from the descent start  (sec)');  ylabel(sprintf('%s_{33}', AB));  title(arms{ia}, 'FontSize', FS - 2);
    if ia == 1; legend([h1 h2], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc)}, 'Location', 'southwest'); end
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1029_desc_vs_report.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
