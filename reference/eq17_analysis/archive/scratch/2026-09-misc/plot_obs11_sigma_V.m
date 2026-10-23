% PURPOSE (2026-09-16): the |V| tiles of the [1/1] two-version observability (0916_estb_11_two_versions.tex), drawn the
%   way plot_obs5_sigma_V.m drew the 09-14 comparison: each tile is |V| with the numbers written in (rows = state,
%   columns = directions v_1..v_5), each column labelled with its own sigma_i. One figure per plant (cell / plane);
%   rows = trajectory (canon, Meng), columns = A descent, B descent, A hold, B hold.
%   Data from probe_obs_11_two_versions (seed 7, 500-step windows).
%   Output reference/eq17_analysis/derivation/figures/obs11_sigma_V_{cell,plane}.png | EXPIRES: with the [1/1] line
function plot_obs11_sigma_V(o)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(here);
    if nargin < 1 || isempty(o); o = probe_obs_11_two_versions(7, 500); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TR = {'canon', 'meng'};  WN = {'descent', 'hold'};  VER = {'A', 'B'};
    LABA = {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'c_0', 'd_1'};  LABB = {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'a_w', 'd_1'};
    FS = 13;  LFS = 12;  AXLW = 1.4;
    for pl = {'cell', 'plane'}
        f = figure('Units','inches','Position',[0 0 15 6.6], 'Color','w', 'Visible','off');
        tiledlayout(2, 4, 'TileSpacing','compact', 'Padding','compact');
        for c = 1:2
            for iw = 1:2
                for m = 1:2
                    X = o.(TR{c}).(pl{1}).(WN{iw})(m);  sv = X.sv(:).';  V = X.V;  n = 5;
                    lab = LABA;  if m == 2; lab = LABB; end
                    nexttile; hold on; box on;
                    imagesc(abs(V), [0 1]);  colormap(flipud(gray));
                    for i = 1:n
                        for j = 1:n
                            tc = 'k';  if abs(V(i,j)) > 0.55; tc = 'w'; end
                            text(j, i, sprintf('%.2f', V(i,j)), 'HorizontalAlignment','center', 'FontSize', FS-2, ...
                                 'FontWeight','bold', 'Color', tc);
                        end
                    end
                    set(gca, 'YDir','reverse', 'XTick', 1:n, 'YTick', 1:n, 'YTickLabel', lab, 'FontSize', FS, ...
                             'FontWeight','bold', 'LineWidth', AXLW, 'TickLength', [0 0]);
                    xt = cell(1, n);
                    for j = 1:n; xt{j} = sprintf('v_%d\\newline%s', j, local_num(sv(j))); end
                    set(gca, 'XTickLabel', xt, 'XTickLabelRotation', 0);
                    xlabel('\sigma_i', 'FontSize', FS, 'FontWeight','bold');
                    xlim([0.5 n+0.5]);  ylim([0.5 n+0.5]);
                    h = plot(NaN, NaN, 'w.');
                    legend(h, {sprintf('%s   %s   %s   t_0 = %.2f s', TR{c}, VER{m}, WN{iw}, X.t)}, 'Location','northoutside', ...
                           'Orientation','horizontal', 'FontSize', LFS, 'FontWeight','bold', 'Box','on');
                end
            end
        end
        fn = fullfile(od, sprintf('obs11_sigma_V_%s.png', pl{1}));  exportgraphics(f, fn, 'Resolution', 150);  close(f);
        fprintf('figure: %s\n', fn);
    end
end

function s = local_num(x)
    if x < 1e-6;      s = '0';
    elseif x >= 1e4;  s = sprintf('%.1fe4', x/1e4);
    else;             s = sprintf('%.0f', x); end
end
