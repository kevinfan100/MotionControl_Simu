% PURPOSE (2026-09-14): figure 1 of the comparison reading copy (0915_estb_variants_compare.tex) -- the whitened
%   observability SVD of the three versions at one window per trajectory, drawn the way the console prints it:
%   each tile is |V| with the numbers written in (rows = state, columns = directions), and each column is labelled
%   with its own sigma. Data from probe_obs5_V_two_versions (seed 7, 500-step windows, production chord-seed arm).
%   Output reference/eq17_analysis/derivation/figures/obs5_sigma_V.png | EXPIRES: with the b line | 產線改動不會自動跟上
function plot_obs5_sigma_V(o5)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(here);
    if nargin < 1 || isempty(o5); o5 = probe_obs5_V_two_versions(7, 500); end
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TR = {'canon', 'meng'};  NAME = {'original', 'substituted', '4-state'};
    LAB = {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'a_w', 'b'};
    FS = 13;  LFS = 12;  AXLW = 1.4;
    f = figure('Units','inches','Position',[0 0 12 6.4], 'Color','w', 'Visible','off');
    tiledlayout(2, 3, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        for m = 1:3
            X = o5.(TR{c})(m);  sv = X.sv(:).';  V = X.V;
            n = 5;  lab = LAB;
            if m == 3; n = 4;  lab = LAB([1 2 3 5]); end       % the 4-state has no a_w row and no dead direction
            V = V(1:n, 1:n);  sv = sv(1:n);
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
            legend(h, {sprintf('%s   %s', TR{c}, NAME{m})}, 'Location','northoutside', 'Orientation','horizontal', ...
                   'FontSize', LFS, 'FontWeight','bold', 'Box','on');
        end
    end
    if ~exist(od, 'dir'); mkdir(od); end
    fn = fullfile(od, 'obs5_sigma_V.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end

function s = local_num(x)
    if x < 1e-6;      s = '0';
    elseif x >= 1e4;  s = sprintf('%.1fe4', x/1e4);
    else;             s = sprintf('%.0f', x); end
end
