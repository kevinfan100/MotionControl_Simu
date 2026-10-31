% STATUS: SCRATCH -- 2026-10-29 (user): in the 10-s hold at 15 um above the void sphere (start 10 deg off the top) theta-hat_x settles near
%   +10 deg although theta_x = 0. Check whether this is the estimator (axis mix-up / sign) or weak information at 15 um: hold only, 10 s,
%   theta-hat[0] = 0, prior 10 deg, seeds 1-5. Arms: (1) sphere, start 10 deg off the top (as the flow) | (2) plane tilted theta_y 10,
%   theta_x 0 | (3) plane theta_y 0, theta_x 10 | (4) plane theta_y 10 at 5 um | (5) sphere at 5 um. Console: theta-hat - theta (x, y) mean
%   over 8-10 s; gain anisotropy a_11 - a_33. Figure fig_1029_hold_angle_check.png: theta-hat vs time, seed 1, per arm.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  FS = 14;  hatc = char(770);  th = char(952);  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'ap_q', true, 'ap_floor_law', true, ...
              'scan_len', 0, 'reg_off', true, 'T', 10, 'tag', 'hc');
arm = {'sphere, 10 deg off the top, 15 um', 'plane \theta_y 10, 15 um', 'plane \theta_x 10, 15 um', 'plane \theta_y 10, 5 um', 'sphere, 10 deg off the top, 5 um'};
NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    switch IA(q)
        case 1; o.wall = 'hemi';  o.hemi_r = 15;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.kappa_state = true;
        case 2; o.theta_y = 10;
        case 3; o.theta_x = 10;
        case 4; o.theta_y = 10;  o.h0 = 5 / Rr;
        case 5; o.wall = 'hemi';  o.hemi_r = 15;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.kappa_state = true;  o.h0 = 5 / Rr;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr, 'aT_tr', r.aT_tr, 'aN_tr', r.aN_tr);
end
for ia = 1:NA
    E = zeros(NS, 2);
    for id = 1:NS
        r = rs{ia, id};  k = find(r.t >= 8 & (1:numel(r.t))' <= r.kend);  E(id, :) = [mean(r.thx(k) - r.th_tr(k, 1)), mean(r.thy(k) - r.th_tr(k, 2))];
    end
    r = rs{ia, 1};  fprintf('%-36s| theta-hat - theta over 8-10 s: x %s, y %s deg | a_11 - a_33 = %.3f\n', strrep(arm{ia}, '\theta', 'theta'), ...
            mat2str(round(E(:, 1)', 1)), mat2str(round(E(:, 2)', 1)), mean(r.aT_tr(10:end)) - mean(r.aN_tr(10:end)));
end
f = figure('Color', 'w', 'Position', [20 20 520 * NA 520]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 1};  k = find((1:numel(r.t))' <= r.kend);  a = nexttile(tl);  hold on;  box on;
    h1 = plot(r.t(k), r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(r.t(k), r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(r.t(k), r.thy(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(r.t(k), r.thx(k), 'Color', cG, 'LineWidth', 1.2);
    ylim([-15 25]);  xlabel('Time (sec)');  ylabel(sprintf('%s  [\\circ]', th));  title(sprintf('%s, seed 1', arm{ia}));
    if ia == 1; legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1029_hold_angle_check.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);
