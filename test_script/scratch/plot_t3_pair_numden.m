% STATUS: SCRATCH -- 2026-10-24 (user: the comparison figure with the numerator and the denominator of tan 2theta_y = -2 A_xz / (A_xx - A_zz)
%   shown separately, no theta_x): run_t3_pair, 20 deg wall, 20 s. One figure per height; columns = computed angle | estimated angle;
%   rows = theta_y error, numerator -2 A-hat_xz, denominator A-hat_xx - A-hat_zz (red = true). For the estimated angle A-hat is built from
%   its states: A-hat = a_T-hat I + (a_N-hat - a_T-hat) w-hat w-hat^T. 1022 style: seed 1 dark, seeds 2-3 light, Time (sec);
%   numerator / denominator rows use the same y range at both heights. Output test_results/temp_3d_wsonly/fig_t3_pair_numden_{50um,15um}.png
%   plane = 'yz' (10-24 user): same figure for theta_x, numerator -2 A_yz (true 0), denominator A_yy - A_zz; files _yz_.
%   EXPIRES: when the comparison is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
S = load(fullfile(fo, 'temp_t3_pair.mat'));
if ~exist('plane', 'var'); plane = 'xz'; end            % 'xz': theta_y from (A_xx - A_zz, -2 A_xz); 'yz': theta_x from (A_yy - A_zz, -2 A_yz)
Rr = 2.25;  um = [50 15];  FS = 15;  hatc = char(770);  abar = char(257);  Ahat = char(194);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];
arm = {'computed angle', 'estimated angle'};
for ih = 1:2
    sel = find(abs(S.HH - um(ih) / Rr) < 1e-6)';  [~, o] = sort(S.SS(sel), 'descend');  sel = sel(o);
    a1 = S.xyz{sel(1)};  At = a1.A_true;  ratio = a1.aN_true(1) / a1.aT_true(1);
    if strcmp(plane, 'xz'); tru = {0, -2 * At(1,3), At(1,1) - At(3,3)}; else; tru = {0, -2 * At(2,3), At(2,2) - At(3,3)}; end
    fig = figure('Color', 'w', 'Position', [80 80 1300 1050]);
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:3
        for c = 1:2
            nexttile;  hold on;  box on;  hs = gobjects(0);
            for i = sel
                s1 = (S.SS(i) == 1);  col = cBl;  if s1; col = cB; end
                if c == 1
                    r = S.xyz{i};  k = 1:numel(r.t);  axx = r.a(k,1);  azz = r.a(k,3);  axz = r.a(k,5);  ayy = r.a(k,2);  ayz = r.a(k,6);
                else
                    r = S.ref{i};  k = 1:r.kend;
                    wx = sind(r.thy(k)) .* cosd(r.thx(k));  wz = cosd(r.thy(k)) .* cosd(r.thx(k));  g = r.aN(k) - r.aT(k);
                    wy = -sind(r.thx(k));  axx = r.aT(k) + g .* wx.^2;  azz = r.aT(k) + g .* wz.^2;  axz = g .* wx .* wz;  ayy = r.aT(k) + g .* wy.^2;  ayz = g .* wy .* wz;
                end
                if strcmp(plane, 'xz'); n_ = -2 * axz; d_ = axx - azz; e_ = r.thy(k) - S.thy; else; n_ = -2 * ayz; d_ = ayy - azz; e_ = r.thx(k); end
                if row == 1; y = e_; elseif row == 2; y = n_; else; y = d_; end
                h = plot(r.t(k), y, 'Color', col, 'LineWidth', 1 + 0.8 * s1);
                if s1; hs = h; end
            end
            yline(tru{row}, 'r', 'LineWidth', 2.5);  uistack(hs, 'top');  xlim([0 S.T]);
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
            if row == 1
                ylim([-30 30]);
                if strcmp(plane, 'xz'); ax_ = 'y'; else; ax_ = 'x'; end
                if c == 1; ylabel(sprintf('\\theta_%s error  [\\circ]', ax_)); end
                legend(hs, sprintf('%s_%s - %s_%s,  seed 1   (%s)', ['θ' hatc], ax_, char(952), ax_, arm{c}), 'Location', 'northoutside');
                text(0.97, 0.9, sprintf('%d %sm   (%s_N/%s_T = %.2f)', um(ih), char(181), abar, abar, ratio), 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
            elseif row == 2
                ylim([-0.04 0.12]);  if c == 1; ylabel('numerator'); end
                legend(hs, sprintf('-2%s_{%s},  seed 1   (red = true %.3f)', Ahat, plane, tru{2}), 'Location', 'northoutside');
            else
                ylim([-0.04 0.12]);  if c == 1; ylabel('denominator'); end
                legend(hs, sprintf('%s_{%s%s} - %s_{zz},  seed 1   (red = true %.3f)', Ahat, plane(1), plane(1), Ahat, tru{3}), 'Location', 'northoutside');
                xlabel('Time (sec)');
            end
        end
    end
    sfx = '';  if strcmp(plane, 'yz'); sfx = '_yz'; end
    exportgraphics(fig, fullfile(fo, sprintf('fig_t3_pair_numden%s_%dum.png', sfx, um(ih))), 'Resolution', 150);
    close(fig);
end
