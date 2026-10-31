% STATUS: SCRATCH -- 2026-10-29 (user): [variant run_1029_desc_hold1s: hold after the descent extended to 1 s; arms R and C only; error
%   a-hat_33 - a_33 after the stop in windows 0-0.2, 0.2-0.5, 0.5-1 s; figure = mean and band (10 seeds) of the error vs time] why the 1-s descent (fig_1029_desc1s, left) deviates more than the 1019 report Fig. 2.1.
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
arms = {'R: report setting (plane, 22.2 R -> 1.10 R, no hold)', 'H: R + 10 s hold', 'S: H, 15 um -> h* = 2.37 um', 'C: current (sphere, angle from 0, + kappa)'};  use = [1 4];
[IA, ID] = ndgrid(use, 1:10);  rs = cell(size(IA));  t0 = [0 10 10 10];
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = common;  o.seeds = ID(q);
    switch IA(q)
        case 1; o.h0 = 22.2;  o.hold_until = 0;  o.descent_v = (22.2 - 1.10) * Rr;
        case 2; o.h0 = 22.2;  o.hold_until = 10;  o.descent_v = (22.2 - 1.10) * Rr;
        case 3; o.h0 = 15 / Rr;  o.hold_until = 10;  o.descent_v = 15 - hs * Rr;
        case 4; o.h0 = 15 / Rr;  o.hold_until = 10;  o.descent_v = 15 - hs * Rr;  o.wall = 'hemi';  o.hemi_r = 15;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.kappa_state = true;
    end
    o.T = t0(IA(q)) + 2;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'aN', r.aN, 'aN_tr', r.aN_tr, 'h', r.h, 'contact', r.contact);
end
f = figure('Color', 'w', 'Position', [30 30 1900 600]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:2
    ia = use(j);  M = zeros(10, 4);  tg = (-0.2:1 / 1612:2)';  E = NaN(numel(tg), 10);
    for id = 1:10
        r = rs{j, id};  tt = r.t - t0(ia);  e = r.aN - r.aN_tr;  ok = (1:numel(tt))' <= r.kend;
        M(id, :) = [mean(e(tt >= 0.5 & tt < 1)), mean(e(tt >= 1 & tt < 1.2 & ok)), mean(e(tt >= 1.2 & tt < 1.5 & ok)), mean(e(tt >= 1.5 & tt < 2 & ok))];
        E(:, id) = interp1(tt(ok), e(ok), tg);
    end
    fprintf('%-52s: a-hat - a: descent 2nd half %+.3f (sd %.3f) | after the stop 0-0.2 s %+.4f (sd %.4f), 0.2-0.5 s %+.4f (sd %.4f), 0.5-1 s %+.4f (sd %.4f)\n', ...
            arms{ia}, mean(M(:, 1)), std(M(:, 1)), mean(M(:, 2)), std(M(:, 2)), mean(M(:, 3)), std(M(:, 3)), mean(M(:, 4)), std(M(:, 4)));
    ax = nexttile(tl);  hold on;  box on;  mu = mean(E, 2, 'omitnan');  sd = std(E, 0, 2, 'omitnan');
    fill([tg; flipud(tg)], [mu - sd; flipud(mu + sd)], [0.75 0.82 1], 'EdgeColor', 'none', 'DisplayName', '\pm 1 sd (10 seeds)');
    plot(tg, mu, 'Color', cE, 'LineWidth', 2, 'DisplayName', 'mean over seeds');  yline(0, 'k--', 'LineWidth', 1.3, 'HandleVisibility', 'off');
    xline(1, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4, 'HandleVisibility', 'off');
    xlim([-0.2 2]);  ylim([-0.1 0.15]);  xlabel('time from the descent start  (sec)');  ylabel(sprintf('%s%s_{33} - %s_{33}', AB, hatc, AB));  title(arms{ia}, 'FontSize', FS - 2);
    if j == 1; legend('Location', 'northwest'); end
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1029_desc_hold1s.png');  exportgraphics(f, fn, 'Resolution', 140);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
