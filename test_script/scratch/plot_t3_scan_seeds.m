% STATUS: SCRATCH -- 2026-10-24: run_t3_scan_seeds (10 seeds), computed vs estimated angle in the wait / regulation / scan flow.
%   Columns computed | estimated; rows theta_y error, theta_x error (line-angle error in +-90 deg), gap minus target h - h* [um]
%   (approach is off the top of the frame). Seed 1 dark, seeds 2-10 light. Console: contact, closest approach, scan statistics.
%   Output test_results/temp_3d_wsonly/fig_t3_scan_seeds.png | EXPIRES: when the scan summary is written
linerr = @(e) local_break(mod(e + 90, 180) - 90);
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_scan_seeds.mat'));  Rr = 2.25;  n = numel(S.seeds);  tsc = S.o.t_scan;
FS = 15;  hatc = char(770);  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
arm = {'computed angle', 'estimated angle'};  runs = {S.rc, S.rb};
hstar = S.rb{1}.h_star * Rr;
fig = figure('Color', 'w', 'Position', [80 80 1300 1000]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    for c = 1:2
        nexttile;  hold on;  box on;  hs = gobjects(0);
        for i = n:-1:1
            r = runs{c}{i};  k = 1:r.kend;  s1 = (S.seeds(i) == 1);
            switch row
                case 1; y = linerr(r.thy(k) - 20); col = cBl; if s1; col = cB; end
                case 2; y = linerr(r.thx(k));      col = cGl; if s1; col = cG; end
                case 3; y = r.h(k) * Rr - hstar;   col = cBl; if s1; col = cB; end
            end
            h = plot(r.t(k), y, 'Color', col, 'LineWidth', 0.8 + 1 * s1);  if s1; hs = h; end
        end
        yline(0, 'r', 'LineWidth', 2.5);  uistack(hs, 'top');  xlim([0 S.o.T]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        switch row
            case 1
                ylim([-30 30]);  if c == 1; ylabel('\theta_y error  [\circ]'); end
                legend(hs, sprintf('%s_y - %s_y,  seed 1   (%s)', ['θ' hatc], char(952), arm{c}), 'Location', 'northoutside');
                text(0.97, 0.88, sprintf('wait 5 s, regulation, scan from %d s; %d seeds', tsc, n), 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
            case 2
                ylim([-30 30]);  if c == 1; ylabel('\theta_x error  [\circ]'); end
                legend(hs, sprintf('%s_x - %s_x', ['θ' hatc], char(952)), 'Location', 'northoutside');
            case 3
                ylim([-1.5 1.5]);  if c == 1; ylabel(sprintf('h - h*  [%sm]', char(181))); end
                legend(hs, sprintf('gap minus target (h* = %.2f %sm)', hstar, char(181)), 'Location', 'northoutside');  xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_t3_scan_seeds.png'), 'Resolution', 150);
close(fig);
% ---- console ----
fprintf('\n arm        | contact | closest h [R] median (min) | scan: |h-h*| rms um median (max) | scan max |h-h*| um median (max) | scan |e_thy| rms deg median (max)\n');
for c = 1:2
    M = NaN(n, 5);
    for i = 1:n
        r = runs{c}{i};  k = 1:r.kend;  ks = k(r.t(k) >= tsc);
        dh = r.h(ks) * Rr - hstar;  ey = mod(r.thy(ks) - 20 + 90, 180) - 90;
        M(i, :) = [r.contact, min(r.h(k)), rms(dh), max(abs(dh)), rms(ey)];
    end
    fprintf(' %-10s |  %d/%d    | %.3f (%.3f)                 | %.3f (%.3f)                     | %.3f (%.3f)                      | %.2f (%.2f)\n', ...
            strtok(arm{c}), sum(M(:,1)), n, median(M(:,2)), min(M(:,2)), median(M(:,3)), max(M(:,3)), median(M(:,4)), max(M(:,4)), median(M(:,5)), max(M(:,5)));
    S.(['M' num2str(c)]) = M;
end
fprintf(' paired (computed - estimated) scan rms |h-h*|: computed worse in %d / %d seeds\n', sum(S.M1(:,3) > S.M2(:,3)), n);

function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
