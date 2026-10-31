% STATUS: SCRATCH -- 2026-10-27 (user): angle in the KF (1026 document) vs angle out of the KF (run_t3_xyz_fixed opts.theta_out: theta-hat
%   kept outside, P has no angle block, angle step = updated a_vw / gap, angle error in F_e written as e_off / gap). Wall theta_y = 20 deg,
%   theta-hat[0] = 0, gains at the seeds, prior 10 deg (off-diagonal prior = gap x 10 deg), one-sided gate 2 sqrt P, 10 s, seeds 1-10
%   (paired), 15 and 5 um. Criteria written before the run: same information -> rms within the seed spread; report the paired difference
%   at 1/2/5/10 s (mean +- SEM), honesty rms / sqrt P, seeds leaving the wall, largest angle step at a gate opening.
%   Figure: columns 15 | 5 um, rows rms theta_y error, rms theta_x error (10 seeds), two arms.
%   Output test_results/temp_3d_wsonly/fig_1027_theta_out.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 10;  seeds = 1:10;  n = numel(seeds);  um = [15 5];  hatc = char(770);  th = ['θ' hatc];  mu = char(181);
arm = {'angle in the KF (document)', 'angle out of the KF'};  col = {[0.85 0.4 0], [0 0.3 0.85]};
nj = 2 * 2 * n;  jobs = cell(1, nj);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
parfor q = 1:nj
    ih = 1 + floor((q - 1) / (2 * n));  rem = q - (ih - 1) * 2 * n;  c = 1 + floor((rem - 1) / n);  i = rem - (c - 1) * n;
    o = struct('theta_x', 0, 'theta_y', 20, 'h0', um(ih) / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'theta_out', c == 2, 'seeds', seeds(i), 'tag', 'to');
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
fig = figure('Color', 'w', 'Position', [60 60 1700 750]);  tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(2, 2);  for a = 1:4; ax(a) = nexttile(a);  hold(ax(a), 'on');  box(ax(a), 'on'); end
for ih = 1:2
    t = jobs{(ih - 1) * 2 * n + 1}.t;  ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 2 5 10]);  E = cell(1, 2);
    for c = 1:2
        rs = jobs((ih - 1) * 2 * n + (c - 1) * n + (1:n));
        EY = NaN(numel(t), n);  EX = EY;  SY = EY;  SX = EY;  jmp = zeros(1, n);
        for i = 1:n
            r = rs{i};  k = 1:r.kend;  EY(k, i) = 20 - r.thy(k);  EX(k, i) = -r.thx(k);  SY(k, i) = r.sPth(k, 2);  SX(k, i) = r.sPth(k, 1);
            g = r.gate(k)';  op = find(g(2:end) == 1 & g(1:end-1) == 0) + 1;  if ~isempty(op); jmp(i) = max(max(abs(r.dth(op, :)))); end
        end
        bad = any(abs(EY) > 60 | abs(EX) > 60, 1);  E{c} = {EY, EX};
        ey = sqrt(mean(EY(:, ~bad).^2, 2, 'omitnan'));  ex = sqrt(mean(EX(:, ~bad).^2, 2, 'omitnan'));
        h(c) = plot(ax(1, ih), t, ey, 'Color', col{c}, 'LineWidth', 2);  plot(ax(2, ih), t, ex, 'Color', col{c}, 'LineWidth', 2);
        fprintf('%2d um %-28s | leave %d | rms theta_y 1/2/5/10 s %s | theta_x %s | honesty y %s x %s | max jump at gate opening %.1f deg (median %.1f)\n', ...
                um(ih), arm{c}, sum(bad), mat2str(round(ey(ks)', 1)), mat2str(round(ex(ks)', 1)), ...
                mat2str(round((ey(ks) ./ mean(SY(ks, ~bad), 2))', 2)), mat2str(round((ex(ks) ./ mean(SX(ks, ~bad), 2))', 2)), max(jmp), median(jmp));
    end
    dy = abs(E{2}{1}(ks, :)) - abs(E{1}{1}(ks, :));  dx = abs(E{2}{2}(ks, :)) - abs(E{1}{2}(ks, :));
    fprintf('%2d um paired |err| out - in: theta_y %s +- %s | theta_x %s +- %s deg (1/2/5/10 s, mean +- SEM)\n', um(ih), ...
            mat2str(round(mean(dy, 2)', 2)), mat2str(round(std(dy, 0, 2)' / sqrt(n), 2)), mat2str(round(mean(dx, 2)', 2)), mat2str(round(std(dx, 0, 2)' / sqrt(n), 2)));
    legend(ax(1, ih), h, cellfun(@(a) sprintf('%d %sm, %s', um(ih), mu, a), arm, 'UniformOutput', false), 'Location', 'northoutside', 'Orientation', 'horizontal');
end
for ih = 1:2
    ylim(ax(1, ih), [0 15]);  ylim(ax(2, ih), [0 15]);  xlabel(ax(2, ih), 'Time (sec)');
end
ylabel(ax(1, 1), sprintf('rms(θ_y - %s_y)  [\\circ]', th));  ylabel(ax(2, 1), sprintf('rms(θ_x - %s_x)  [\\circ]', th));
set(ax(:), 'XLim', [0 T], 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2);
arrayfun(@(a) set(a.Toolbar, 'Visible', 'off'), ax(:));
exportgraphics(fig, fullfile(fo, 'fig_1027_theta_out.png'), 'Resolution', 150);  close(fig);
system(['open ' fullfile(fo, 'fig_1027_theta_out.png')]);
