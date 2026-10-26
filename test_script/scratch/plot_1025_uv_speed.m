% STATUS: SCRATCH -- 2026-10-25 (user): does a faster u v motion make the angle converge faster? 15 um, 20 deg wall, circle radius 3 um
%   (2 s ramp), frequency 4 / 8 / 16 / 32 Hz (tangential speed ~ 2 pi A f = 75 / 150 / 300 / 600 um/s at full radius), seeds 1-3, 4 s,
%   both forms (estimated: theta-hat[0] = 0). Console: first time |theta_y error| and |theta_x error| stay below 5 deg for the rest
%   of the run, mean |error| over 3-4 s, gap range. Output test_results/temp_3d_wsonly/fig_1025_uv_speed.png | EXPIRES: when read
linerr = @(e) mod(e + 90, 180) - 90;
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  T = 4;  n = numel(seeds);  fr = [4 8 16 32];  forms = {'computed', 'estimated'};
res = cell(2, numel(fr), n);
for f = 1:2
    for j = 1:numel(fr)
        for i = 1:n
            o = struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', T, 'aprime', true, 'dither', [3 fr(j) 2], 'ap_q', true, 'tag', 'uv');
            if f == 2; o.form = 'estimated';  o.th0_deg = [0 0]; end
            R = run_t3_xyz_fixed(o);  res{f, j, i} = R.runs{1};
        end
        ts = NaN(n, 1);  me = NaN(n, 1);  hr = NaN(n, 2);
        for i = 1:n
            r = res{f, j, i};  k = 1:r.kend;  e = max(abs(linerr(r.thy(k) - 20)), abs(linerr(r.thx(k))));
            last = find(e > 5, 1, 'last');  if isempty(last); ts(i) = 0; elseif last < numel(k); ts(i) = r.t(last + 1); end
            kk = find(r.t(k) >= 3);  me(i) = mean(e(kk));  hr(i, :) = [min(r.h(k)), max(r.h(k))] * Rr;
        end
        fprintf('%-9s %2d Hz | contact %d | time both errors stay < 5 deg: %s s | mean max-error 3-4 s %s deg | gap %.1f-%.1f um\n', forms{f}, fr(j), ...
                sum(cellfun(@(r) r.contact, squeeze(res(f, j, :)))), mat2str(round(ts', 2)), mat2str(round(me', 2)), min(hr(:, 1)), max(hr(:, 2)));
    end
end
% figure: rows = forms, columns = frequencies, theta_y error (blue) and theta_x error (green), seed 1 dark
FS = 14;  hatc = char(770);  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
fig = figure('Color', 'w', 'Position', [60 80 1900 750]);
tiledlayout(2, numel(fr), 'TileSpacing', 'compact', 'Padding', 'compact');
for f = 1:2
    for j = 1:numel(fr)
        nexttile;  hold on;  box on;
        for i = n:-1:1
            r = res{f, j, i};  k = 1:r.kend;  s1 = (seeds(i) == 1);
            cy = cBl;  cx = cGl;  if s1; cy = cB;  cx = cG; end
            hy = plot(r.t(k), linerr(r.thy(k) - 20), 'Color', cy, 'LineWidth', 1 + 0.8 * s1);
            hx = plot(r.t(k), linerr(r.thx(k)), 'Color', cx, 'LineWidth', 1 + 0.8 * s1);
            if s1; h1 = hy; h2 = hx; end
        end
        yline(0, 'r', 'LineWidth', 2.5);  uistack([h1 h2], 'top');  xlim([0 T]);  ylim([-30 30]);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
        if j == 1; ylabel(sprintf('%s angle error [%s]', forms{f}, char(176))); end
        if f == 1
            legend([h1 h2], {sprintf('%s_y - %s_y', ['θ' hatc], char(952)), sprintf('%s_x - %s_x', ['θ' hatc], char(952))}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal');
            text(0.97, 0.9, sprintf('circle 3 %sm, %d Hz', char(181), fr(j)), 'Units', 'normalized', 'HorizontalAlignment', 'right', ...
                 'FontSize', 13, 'FontWeight', 'bold');
        else
            xlabel('Time (sec)');
        end
    end
end
exportgraphics(fig, fullfile(fo, 'fig_1025_uv_speed.png'), 'Resolution', 150);
close(fig);
