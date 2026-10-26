% STATUS: SCRATCH -- 2026-10-25 (user, hypothesis 2 on why Meng dissertation Fig. 40 converges fast): estimated angle (1025 Section 6)
%   with Meng's motion (Sec. 5.5: ramp toward the wall at 5 um/s for 11.5 um, then hold) vs the 15 um fixed point, same seeds 1-3,
%   20 deg wall, theta-hat[0] = (0, 15) deg, 5 s, slope q on (moving). Start 15 um (a_perp 0.833, Meng's lambda_w starts at ~0.82),
%   end of ramp 3.5 um. Columns: fixed point | ramp; rows theta_y error, theta_x error, sqrt P of theta_y, gap h [um].
%   Console: sqrt P of theta_y at 0.5 / 1 / 2.3 / 5 s, mean |error| over 2.3-5 s. Output fig_1025_meng_ramp.png | EXPIRES: when Fig. 40 is explained
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  T = 5;  n = numel(seeds);  FS = 15;  hatc = char(770);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
ttl = {'fixed point 15 \mum', 'ramp 5 \mum/s, 11.5 \mum'};  runs = cell(2, n);
for i = 1:n
    o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', T, 'form', 'estimated', 'th0_deg', [0 15], 'ap_q', true, 'tag', 'fix');
    R = run_t3_xyz_fixed(o);  runs{1, i} = R.runs{1};
    o.ramp = [5 11.5];  o.tag = 'ramp';  R = run_t3_xyz_fixed(o);  runs{2, i} = R.runs{1};
end
tt = [0.5 1 2.3 5];
for c = 1:2
    sp = NaN(n, numel(tt));  me = NaN(n, 2);
    for i = 1:n
        r = runs{c, i};  k = arrayfun(@(s) find(r.t >= s - 1e-9, 1), tt);  kk = find(r.t >= 2.3 & r.t <= r.t(r.kend));
        sp(i, :) = r.sPth(k, 2)';  me(i, :) = [mean(abs(r.thy(kk) - 20)), mean(abs(r.thx(kk)))];
    end
    fprintf('%-26s | contact %d | sqrtP theta_y at %s s: %s deg | mean |err| 2.3-5 s: theta_y %s, theta_x %s deg\n', ttl{c}, ...
            sum(cellfun(@(r) r.contact, runs(c, :))), mat2str(tt), mat2str(round(mean(sp, 1), 2)), mat2str(round(me(:, 1)', 2)), mat2str(round(me(:, 2)', 2)));
end
fig = figure('Color', 'w', 'Position', [80 80 1300 1150]);
tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:4
    for c = 1:2
        nexttile;  hold on;  box on;  hs = gobjects(0);
        for i = n:-1:1
            r = runs{c, i};  k = 1:r.kend;  s1 = (seeds(i) == 1);
            switch row
                case 1; y = r.thy(k) - 20;   col = cBl; if s1; col = cB; end
                case 2; y = r.thx(k);        col = cGl; if s1; col = cG; end
                case 3; y = r.sPth(k, 2);    col = cBl; if s1; col = cB; end
                case 4; y = r.h(k) * Rr;     col = cBl; if s1; col = cB; end
            end
            h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);  if s1; hs = h; end
        end
        if row < 3; yline(0, 'r', 'LineWidth', 2.5); end
        uistack(hs, 'top');  xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        switch row
            case 1
                ylim([-10 10]);  if c == 1; ylabel('\theta_y error  [\circ]'); end
                legend(hs, sprintf('%s_y - %s_y,  seed 1   (%s)', ['θ' hatc], char(952), ttl{c}), 'Location', 'northoutside');
            case 2
                ylim([-10 10]);  if c == 1; ylabel('\theta_x error  [\circ]'); end
                legend(hs, sprintf('%s_x - %s_x', ['θ' hatc], char(952)), 'Location', 'northoutside');
            case 3
                ylim([0 10]);  if c == 1; ylabel('[\circ]'); end
                legend(hs, sprintf('%sP of %s_y', char(8730), char(952)), 'Location', 'northoutside');
            case 4
                ylim([0 16]);  if c == 1; ylabel(sprintf('h  [%sm]', char(181))); end
                legend(hs, 'gap h', 'Location', 'northoutside');  xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1025_meng_ramp.png'), 'Resolution', 150);
close(fig);
