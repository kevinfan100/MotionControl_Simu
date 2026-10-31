% STATUS: SCRATCH -- 2026-10-28 (user): the theta_x = -70 deg column of plot_1028_fixed_nocos.m drawn per seed.
%   Setting: plane wall theta_x = -70, theta_y = 20 deg; probe held at h = 5 um (no motion, no regulation, no scan), 5 s;
%   theta-hat[0] = (-70, 17) deg, angle prior 3 deg, gains at the seeds; document version (form 'local', one-sided gate 2 sqrt P).
%   Columns = seeds 1-3; rows: theta-hat_x, theta-hat_y (truth red, with cos blue, without cos orange), w-hat error, gate open (1) / closed (0).
%   Log time axis to show the first milliseconds. Output test_results/temp_3d_wsonly/fig_1028_fixed70.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  FS = 14;  hatc = char(770);  th = char(952);  cA = [0 0.25 0.85];  cB = [0.85 0.4 0];  cT = [0.85 0 0];
rq = cell(2, 3);
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
parfor q = 1:6
    [ic, id] = ind2sub([2 3], q);
    o = struct('theta_x', -70, 'theta_y', 20, 'h0', 5 / Rr, 'T', 5, 'aprime', true, 'th0_deg', [-70 17], 'sP_theta_deg', 3, 'gain0', 'seed', 'form', 'local', ...
               'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'no_cos', ic == 2, 'seeds', id, 'tag', 'f70');
    R = run_t3_xyz_fixed(o);  rq{q} = R.runs{1};
end
wn = @(tx, ty) [sind(ty) .* cosd(tx), -sind(tx), cosd(ty) .* cosd(tx)];
f = figure('Color', 'w', 'Position', [30 30 2100 1500]);  tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for id = 1:3
    ax = gobjects(1, 4);  for row = 1:4; ax(row) = nexttile(tl, 3 * (row - 1) + id);  hold(ax(row), 'on');  box(ax(row), 'on'); end
    yline(ax(1), -70, 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'true');  yline(ax(2), 20, 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'true');
    for ic = 1:2
        r = rq{ic, id};  k = (1:r.kend)';  t = r.t(k);  c = cA;  nm = 'with cos';  if ic == 2; c = cB;  nm = 'without cos'; end
        plot(ax(1), t, r.thx(k), 'Color', c, 'LineWidth', 1.4, 'DisplayName', nm);  plot(ax(2), t, r.thy(k), 'Color', c, 'LineWidth', 1.4, 'DisplayName', nm);
        plot(ax(3), t, acosd(min(1, wn(r.thx(k), r.thy(k)) * wn(-70, 20)')), 'Color', c, 'LineWidth', 1.4);
        stairs(ax(4), t, double(r.gate(k)) + 0.03 * (ic - 1.5), 'Color', c, 'LineWidth', 1.2);
    end
    title(ax(1), sprintf('seed %d', id));  ylabel(ax(1), sprintf('%s%s_x  [\\circ]', th, hatc));  ylabel(ax(2), sprintf('%s%s_y  [\\circ]', th, hatc));
    ylabel(ax(3), sprintf('w%s error  [\\circ]', hatc));  ylabel(ax(4), 'gate open');  xlabel(ax(4), 'Time (sec), log scale');
    ylim(ax(1), [-160 -40]);  ylim(ax(2), [-120 120]);  ylim(ax(3), [0 90]);  ylim(ax(4), [-0.2 1.2]);  set(ax(4), 'YTick', [0 1]);
    if id == 1; legend(ax(1), 'Location', 'southwest'); end
    set(ax, 'XScale', 'log', 'XLim', [1e-3 5], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  for a = ax; set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1028_fixed70.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);
