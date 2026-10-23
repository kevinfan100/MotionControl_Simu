% STATUS: ACTIVE (scratch) | PURPOSE: acceptance figure for the exact law step
%   on the seed-at-truth pair (canon). Left = a'_true arm, right = b_true arm.
%   Blue-light = forward Euler (before), blue = exact step (after), red dashed
%   on the LEFT panel = the pre-registered zero-parameter prediction: cumulative
%   open-loop left-endpoint ratchet on the commanded path. The claim under test:
%   Euler-mean tracks the red curve; exact-mean collapses to ~0.
function plot_seedtruth_exact_compare()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    A = load(fullfile(od,'arms30_seedtruth_pair_canon.mat'));        % euler
    B = load(fullfile(od,'arms30_seedtruth_pair_canon_exact.mat'));  % exact
    % zero-parameter prediction: cumulative -[a(w+dw)-a(w)-a'(w)dw] on commanded path
    hd = A.aptrue.hd; Nn = numel(hd); aa = zeros(Nn,1); pp = zeros(Nn,1); dh = 1e-5;
    for i = 1:Nn
        wq = max(hd(i),1.001); [~,c0] = calc_correction_functions(wq); aa(i) = 1/c0;
        [~,c1] = calc_correction_functions(wq+dh); [~,c2] = calc_correction_functions(max(wq-dh,1.001));
        pp(i) = (1/c1-1/c2)/((wq+dh)-max(wq-dh,1.001));
    end
    pred = [0; cumsum(-((aa(2:end)-aa(1:end-1)) - pp(1:end-1).*diff(hd)))];
    TAG = {'aptrue','btrue'}; NM = {'a''_{true}', 'b_{true}'};
    C_EU = [0.55 0.74 0.96]; C_EX = [0 0.2 0.9]; C_PR = [0.8 0 0];
    FS = 15; AXLW = 1.6;
    f = figure('Units','inches','Position',[0 0 13 4.6],'Color','w','Visible','off');
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for a = 1:2
        dA = A.(TAG{a}); dB = B.(TAG{a}); t = dA.t; nS = size(dA.E,2);
        nexttile; hold on;
        yline(0,'-','Color',[0.55 0.55 0.55],'LineWidth',0.8,'HandleVisibility','off');
        H1 = plot(t, mean(dA.E,2), '-', 'Color', C_EU, 'LineWidth', 2.2);
        H2 = plot(t, mean(dB.E,2), '-', 'Color', C_EX, 'LineWidth', 2.2);
        if a == 1
            H3 = plot(t, pred, '--', 'Color', C_PR, 'LineWidth', 2.0);
            legend([H1 H2 H3], {'Euler (before)','exact step (after)', ...
                   'open-loop ratchet (0-param pred.)'}, ...
                   'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',10,'FontWeight','bold','Box','on');
        else
            legend([H1 H2], {'Euler (before)','exact step (after)'}, ...
                   'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',10,'FontWeight','bold','Box','on');
        end
        title(NM{a}, 'FontSize', FS+2, 'FontWeight','bold');
        if a == 1; ylabel('mean (\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
        xlim([0 5]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
        fprintf('[exact_compare %s] E(end): euler %+.5f -> exact %+.5f (%d seeds)\n', ...
            TAG{a}, mean(dA.E(end,:)), mean(dB.E(end,:)), nS);
    end
    png = fullfile(od, 'seedtruth_exact_compare.png');
    exportgraphics(f, png, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', png);
end
