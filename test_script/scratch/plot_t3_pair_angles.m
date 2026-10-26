% STATUS: SCRATCH -- 2026-10-24 (angles only since 10-24 later; user: comparison in the style of the 10-22/23 slides, ~/Downloads/1022.pptx): run_t3_pair, 20 deg wall,
%   fixed points 50 um and 15 um, 20 s. One figure per height; columns = computed angle | estimated angle; rows = theta_y error (blue),
%   theta_x error (green), A_xz (red = true, blue = estimate; for the estimated angle A-hat_xz = (a_N-hat - a_T-hat) w_x w_z from its
%   states). Seed 1 dark and thick, seeds 2-3 light (1022 style); errors theta-hat - theta [deg] with the red zero line; text
%   "<h> um (a_N/a_T = ...)" inside the top panels; x label Time (sec). Rows share y across columns.
%   Output test_results/temp_3d_wsonly/fig_t3_pair_angles_{50um,15um}.png | EXPIRES: when the comparison is read
linerr = @(e) local_break(mod(e + 90, 180) - 90);   % 10-24: line-angle error in (-90, 90], broken where it crosses +-90
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
if ~exist('tmax', 'var'); tmax = []; end                 % 10-24: tmax = 5 -> first 5 s only (files _first5s)
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_pair.mat'));
Rr = 2.25;  um = [50 15];  FS = 15;
hatc = char(770);  abar = char(257);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];
arm = {'computed angle', 'estimated angle'};
for ih = 1:2
    sel = find(abs(S.HH - um(ih) / Rr) < 1e-6)';  [~, o] = sort(S.SS(sel), 'descend');  sel = sel(o);   % seed 1 drawn last (on top)
    a1 = S.xyz{sel(end)};  ratio = a1.aN_true(1) / a1.aT_true(1);  Axz_true = a1.A_true(1, 3);
    fig = figure('Color', 'w', 'Position', [80 80 1300 720]);
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2                                      % 10-24 user: angles only (A_xz row dropped)
        for c = 1:2
            nexttile;  hold on;  box on;
            hs = gobjects(0);
            for i = sel
                if c == 1; r = S.xyz{i}; else; r = S.ref{i}; end
                k = 1:numel(r.t);  if isfield(r, 'kend'); k = 1:r.kend; end
                s1 = (S.SS(i) == 1);
                if row == 1;     y = linerr(r.thy(k) - S.thy);  col = cBl;  if s1; col = cB; end
                elseif row == 2; y = linerr(r.thx(k));          col = cGl;  if s1; col = cG; end
                else
                    if c == 1; y = r.a(k, 5);
                    else; y = (r.aN(k) - r.aT(k)) .* sind(r.thy(k)) .* cosd(r.thx(k)) .* cosd(r.thy(k)) .* cosd(r.thx(k)); end
                    col = cBl;  if s1; col = cB; end
                end
                h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);
                if s1; hs = h; end
            end
            if row < 3; yline(0, 'r', 'LineWidth', 2.5); else; yline(Axz_true, 'r', 'LineWidth', 2.5); end
            uistack(hs, 'top');  xlim([0 S.T]);  if ~isempty(tmax); xlim([0 tmax]); end
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
            if row == 1
                ylim([-30 30]);  if c == 1; ylabel('\theta_y error  [\circ]'); end
                legend(hs, sprintf('%s_y - %s_y,  seed 1   (%s)', ['θ' hatc], 'θ', arm{c}), 'Location', 'northoutside');
                text(0.97, 0.9, sprintf('%d %sm   (%s_N/%s_T = %.2f)', um(ih), char(181), abar, abar, ratio), 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
            elseif row == 2
                ylim([-30 30]);  if c == 1; ylabel('\theta_x error  [\circ]'); end
                legend(hs, sprintf('%s_x - %s_x,  seed 1', ['θ' hatc], 'θ'), 'Location', 'northoutside');
                xlabel('Time (sec)');
            else
                ylim([-0.06 0.03]);  if c == 1; ylabel([char(256) '_{xz}']); end
                legend(hs, sprintf('%s_{xz},  seed 1   (red = true)', char(194)), 'Location', 'northoutside');
                xlabel('Time (sec)');
            end
        end
    end
    sfx = '';  if ~isempty(tmax); sfx = sprintf('_first%ds', tmax); end
    exportgraphics(fig, fullfile(fo, sprintf('fig_t3_pair_angles_%dum%s.png', um(ih), sfx)), 'Resolution', 150);
    close(fig);
end


function e = local_break(e)
    e = e(:);  j = find(abs(diff(e)) > 90);  e(j + 1) = NaN;
end
