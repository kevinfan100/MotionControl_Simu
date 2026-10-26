% STATUS: SCRATCH -- 2026-10-26 (user): replicate Meng dissertation Sec. 5.5 / Fig. 40 with the estimated angle (1025 Section 6). Wall
%   rotated 45 deg about x (theta_x = 45, theta_y = 0), start 15 um (a_perp 0.83, Meng lambda_w ~0.82), ramp along -w-hat 5 um/s for
%   11.5 um (2.3 s), then hold; seed 1, 4 s; slope q on (moving). Arms, one change at a time:
%   A theta-hat(0) = 0, Q_theta = 0 | B A + forgetting factor 0.999 on the angles | C A with theta-hat(0) = 45 deg | D B + C.
%   Criteria (written before the run): time theta_x-hat stays within 5 deg of 45; sd of theta_y-hat over 1-2.3 s (Meng ~ +-2 deg).
%   Rows: theta_x-hat (red = 45), theta_y-hat (red = 0), gap h. Output test_results/temp_3d_wsonly/fig_1026_meng_fig40.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  T = 4;  FS = 14;  hatc = char(770);  cB = [0 0.25 0.85];  cG = [0 0.5 0.2];
arm = {'A: \theta(0) = 0', 'B: A + \lambda_f = 0.999', 'C: \theta(0) = 45\circ', 'D: B + C'};
th0 = {[0 0], [0 0], [45 0], [45 0]};  lf = {[], 0.999, [], 0.999};
if isempty(gcp('nocreate')); parpool('Processes', 4); end
runs = cell(1, 4);
parfor c = 1:4
    o = struct('theta_y', 0, 'theta_x', 45, 'h0', 15 / Rr, 'seeds', 1, 'T', T, 'aprime', true, 'form', 'estimated', ...
               'th0_deg', th0{c}, 'ramp', [5 11.5], 'ap_q', true, 'tag', 'm40');
    if ~isempty(lf{c}); o.lf_theta = lf{c}; end
    R = run_t3_xyz_fixed(o);  runs{c} = R.runs{1};
end
fig = figure('Color', 'w', 'Position', [40 60 1900 900]);
tiledlayout(3, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    for c = 1:4
        nexttile;  hold on;  box on;  r = runs{c};  k = 1:r.kend;  t = r.t(k);
        switch row
            case 1; yline(45, 'r', 'LineWidth', 2.5);  plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.6);  ylim([-10 70]);
                if c == 1; ylabel('\theta_x  [\circ]'); end
                text(0.97, 0.12, arm{c}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', 14, 'FontWeight', 'bold');
            case 2; yline(0, 'r', 'LineWidth', 2.5);  plot(t, r.thy(k), 'Color', cB, 'LineWidth', 1.6);  ylim([-20 20]);
                if c == 1; ylabel('\theta_y  [\circ]'); end
            case 3; plot(t, r.h(k) * Rr, 'Color', cB, 'LineWidth', 1.6);  ylim([0 16]);
                if c == 1; ylabel(sprintf('h  [%sm]', char(181))); end;  xlabel('Time (sec)');
        end
        xline(2.3, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2);  xlim([0 T]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    end
    if row == 1
        % legend once
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1026_meng_fig40.png'), 'Resolution', 150);
close(fig);
for c = 1:4
    r = runs{c};  k = 1:r.kend;  ex = abs(r.thx(k) - 45);  last = find(ex > 5, 1, 'last');  tin = 0;
    if ~isempty(last); if last < numel(k); tin = r.t(last + 1); else; tin = NaN; end; end
    kk = find(r.t(k) >= 1 & r.t(k) <= 2.3);
    fprintf('%-26s | contact %d | theta_x within 5 deg of 45 from t = %.2f s | theta_y sd 1-2.3 s %.2f deg, mean %.2f | theta_x sd 1-2.3 s %.2f\n', ...
            strrep(arm{c}, '\', ''), r.contact, tin, std(r.thy(kk)), mean(r.thy(kk)), std(r.thx(kk)));
end
