% PURPOSE (2026-10-07, branch test/conly-meng-law): the two-panel reading figure for "did j6 change the c estimate": c_hat(t) of all
%   10 seeds (thin blue) with c_eff(h_d) (red) for the current law (left) and the journal law (right), one wall, one arm; shared
%   y axis. explanation-style.md rule 9 (situation vs truth, what the estimator did). Output test_results/temp_conly_meng/.
function plot_conly_meng_chat_seeds(wall, tag)
    if nargin < 1 || isempty(wall); wall = 'cell'; end
    if nargin < 2; tag = 'pfix015_'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'test_results', 'temp_conly_meng');
    laws = {'eq17', 'j6'};  hat = char(770);
    fig = figure('Visible', 'off', 'Position', [100 100 1100 380]);
    yl = [0, 0];
    for il = 1:2
        S = load(fullfile(id, sprintf('f1p2_%s_PI_%s%s_rtri_f1.mat', wall, tag, laws{il})));
        yl = [min(yl(1), min(S.c_hat(:))), max(yl(2), max([S.c_hat(:); S.c_eff_d(:)]))];
    end
    for il = 1:2
        S = load(fullfile(id, sprintf('f1p2_%s_PI_%s%s_rtri_f1.mat', wall, tag, laws{il})));
        ax = subplot(1, 2, il);
        plot(S.t, S.c_hat, 'Color', [0.3 0.5 1.0 0.55], 'LineWidth', 0.6);  hold on;
        plot(S.t, S.c_eff_d, 'r', 'LineWidth', 1.4);  hold off;  box on;
        ylim(yl + [-0.05, 0.05]);  xlabel('t [s]');  ylabel('c');
        tri = S.t >= 1 & S.t <= 3;  ctri = mean(S.c_hat(tri, :), 1);
        lg = legend(ax, {['c' hat ' (' laws{il} '), 10 seeds'], 'c_{eff}(h_d)'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
        lg.Title.String = sprintf('%s wall, %s | c%s 1-3 s: mean %+.3f, seed sd %.3f', wall, strrep(tag, '_', ' '), hat, mean(ctri), std(ctri));
    end
    fn = fullfile(od, sprintf('chat_seeds_%s%s.png', tag, wall));  exportgraphics(fig, fn, 'Resolution', 150);  close(fig);
    fprintf('figure -> %s\n', fn);
end
