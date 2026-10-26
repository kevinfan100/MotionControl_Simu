% STATUS: SCRATCH -- 2026-10-24 (user: option B, single seed): flow_wait5 (start 15 um, wait 5 s, regulation, scan from 12 s), computed
%   angle with A-bar'. Columns: without B | with B (command direction theta_c weighted by the angle uncertainty from P).
%   Rows: theta_y error, theta_x error (light = theta-hat from A-hat, dark = direction used for the commands), gap h [um] (red = target),
%   a_N (red = true). Line-angle errors in +-90 deg. Output test_results/temp_3d_wsonly/fig_t3_flow_wait5_B.png | EXPIRES: when B is decided
linerr = @(e) local_break(mod(e + 90, 180) - 90);
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
C = load(fullfile(fo, 'temp_t3_flow_wait5_B.mat'));  runs = {C.rc, C.rB};  Rr = 2.25;  hstar = C.rb.h_star * Rr;
FS = 15;  hatc = char(770);  abar = char(257);  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
ttl = {'computed angle, without B', 'computed angle, with B'};
fig = figure('Color', 'w', 'Position', [80 80 1300 1250]);
tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:4
    for c = 1:2
        nexttile;  hold on;  box on;  r = runs{c};  k = 1:r.kend;  hh = gobjects(0);  lab = {};
        switch row
            case {1, 2}
                q = 3 - row;  tru = [20, 0];  e_hat = linerr(([r.thx(k), r.thy(k)] * [q == 1; q == 2]) - tru(row));
                if c == 2; e_cmd = linerr(r.thc(k, q) - tru(row)); else; e_cmd = e_hat; end
                yline(0, 'r', 'LineWidth', 2.5);
                cl = {cBl, cGl};  cdk = {cB, cG};
                if c == 2; hh(end+1) = plot(r.t(k), e_hat, 'Color', cl{row}, 'LineWidth', 1); lab{end+1} = 'from A-hat'; end
                hh(end+1) = plot(r.t(k), e_cmd, 'Color', cdk{row}, 'LineWidth', 1.6);  lab{end+1} = 'used for commands';
                ylim([-30 30]);  if c == 1; ylabel(sprintf('\\theta_%s error  [\\circ]', char('y' * (row == 1) + 'x' * (row == 2)))); end
            case 3
                yline(hstar, 'r', 'LineWidth', 2.5);  hh = plot(r.t(k), r.h(k) * Rr, 'Color', cB, 'LineWidth', 1.6);  lab = {'gap h'};
                ylim([0 16]);  if c == 1; ylabel(sprintf('h  [%sm]', char(181))); end
            case 4
                plot(r.t(k), r.aN_tr(k), 'r', 'LineWidth', 2.5);  hh = plot(r.t(k), r.aNw(k), 'Color', cB, 'LineWidth', 1.6);
                lab = {sprintf('%s_N%s', abar, hatc)};  ylim([0 1]);  if c == 1; ylabel(sprintf('%s_N', abar)); end;  xlabel('Time (sec)');
        end
        xlim([0 C.o.T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        legend(hh, lab, 'Location', 'northoutside', 'Orientation', 'horizontal');
        if row == 1
            text(0.97, 0.88, sprintf('%s, seed 1', ttl{c}), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_t3_flow_wait5_B.png'), 'Resolution', 150);
close(fig);

function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
