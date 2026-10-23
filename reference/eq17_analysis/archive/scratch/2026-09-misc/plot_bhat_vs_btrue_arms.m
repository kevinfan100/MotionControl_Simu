% PURPOSE (2026-09-14, user: "why does b_hat not track b_true?"): put the four closed-loop arms' b_hat next to the
%   plant's LOCAL b_true(w) along the same trajectory. b_true is read from the plant curve at the true height,
%   b_true(w) = (-dc_perp/dh / c_perp^2) / (1 - 1/c_perp)^2, i.e. the b that would make the law exact THERE.
%   Console: the range of b_true over the band, the range of b_hat per arm, and the chord seed for reference.
%   Output test_results/apd_acov_meng/bhat_vs_btrue_arms.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = plot_bhat_vs_btrue_arms()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'canon', 'meng'};  ARM = {'orig', 'green', 'orange', '4state'};  NAME = {'original', 'green', 'orange', '4-state'};
    COL = {[0 0.2 0.9], [0 0.6 0.25], [0.9 0.5 0], [0.6 0 0.6]};  LW = [4.0 2.8 1.8 1.0];
    FS = 15;  LFS = 12;  AXLW = 1.8;  nA = numel(ARM);  out = struct();
    f = figure('Units','inches','Position',[0 0 12 4.6], 'Color','w', 'Visible','off');
    tiledlayout(1, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        D = cell(1, nA);
        for a = 1:nA; D{a} = load(fullfile(od, sprintf('amodel_fe_cl_%s_%s.mat', TR{c}, ARM{a}))); end
        t = D{1}.t;  hb = mean(D{1}.HB, 2);  bt = zeros(size(hb));
        for i = 1:numel(hb)
            [~, cp, dd] = calc_correction_functions(max(hb(i), 1.001), true);
            bt(i) = (-dd.dc_perp_dh / cp^2) / (1 - 1/cp)^2;
        end
        fprintf('[%s] b_true(w) along the run: %.4f .. %.4f (near-wall end %.4f) | chord seed %.4f\n', TR{c}, ...
                min(bt), max(bt), bt(end), mean(D{1}.B(1,:)));
        for a = 1:nA
            bb = mean(D{a}.B, 2);
            fprintf('   %-8s b_hat %.4f .. %.4f, end %.4f (moved %+.4f from the seed)\n', NAME{a}, min(bb), max(bb), bb(end), bb(end) - bb(1));
        end
        nexttile(c); hold on; box on;  H = gobjects(1, nA + 1);
        for a = 1:nA; H(a) = plot(t, mean(D{a}.B, 2), '-', 'Color', COL{a}, 'LineWidth', LW(a)); end
        H(nA+1) = plot(t, bt, '--', 'Color', [0.85 0 0], 'LineWidth', 2.4);
        legend(H, [NAME, {'b_{true}(w) at the true height'}], 'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',3);
        if c == 1; ylabel('$\hat{b}$', 'Interpreter','latex', 'FontSize', FS+3); end
        xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 t(end)]); ylim([0.83 0.95]);
        out.(TR{c}) = struct('t', t, 'b_true', bt);
    end
    fn = fullfile(od, 'bhat_vs_btrue_arms.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
