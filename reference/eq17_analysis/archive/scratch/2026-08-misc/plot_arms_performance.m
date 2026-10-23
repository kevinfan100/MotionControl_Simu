% STATUS: ACTIVE (scratch figure) | PURPOSE: what the four (a_pd, a_cov) arms
%   actually estimate on the Meng 10 s ramp -- the simulation result, not the
%   readout mechanism. Seed-averaged, one time axis.
%
%   Rows
%     1  a_h        truth (red) and each arm's a-hat
%     2  a-hat relative error [%]        <- the judgment
%     3  b-hat      truth (red) and each arm's b-hat
function out = plot_arms_performance(ARMS, ax, tlim)

    if nargin < 2 || isempty(ax);   ax = 3; end
    if nargin < 3 || isempty(tlim); tlim = [0.5 10.5]; end

    nA = size(ARMS, 1);
    b_of_w = local_b_true_curve();
    D = cell(1, nA);
    for c = 1:nA
        O = ARMS{c,1};  ns = numel(O.runs);
        r0 = O.runs{1};  t = r0.tout(2:end);
        m = t >= tlim(1) & t <= tlim(2);
        AH = zeros(sum(m), ns); AT = AH; BH = AH; BT = AH;
        for q = 1:ns
            r  = O.runs{q};
            ad = r.a_hat_out(1,ax) / r.a_bar_hat_out(1,ax);
            AH(:,q) = r.a_bar_hat_out([false; m], ax) * ad;
            AT(:,q) = r.a_true_out([false; m], ax);
            BH(:,q) = r.b_hat_out([false; m], ax);
            BT(:,q) = b_of_w(r.h_bar_true_out([false; m], 1));
        end
        D{c} = struct('t', t(m), 'ah', mean(AH,2), 'at', mean(AT,2), ...
                      'bh', mean(BH,2), 'bt', mean(BT,2), ...
                      'e', 100*mean((AH-AT)./AT, 2), 'ns', ns);
    end

    COL = [0 0.2 0.9; 0.10 0.55 0.15; 0.55 0.30 0.75; 0.93 0.50 0.05];
    COL_TRUE = [0.8 0 0];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [10 10 1600 1500], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    A = gobjects(3,1);

    a = nexttile(tl); A(1)=a; hold(a,'on');
    h = gobjects(nA+1,1);
    h(1) = plot(a, D{1}.t, D{1}.at, '-', 'Color', COL_TRUE, 'LineWidth', 3.2);
    for c = 1:nA
        h(c+1) = plot(a, D{c}.t, D{c}.ah, '-', 'Color', COL(c,:), 'LineWidth', 1.8);
    end
    legend(a, h, [{'a_h  true'}, ARMS(:,2).'], 'Location','northoutside', ...
        'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel(a, 'a_h  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold');

    a = nexttile(tl); A(2)=a; hold(a,'on');
    yline(a, 0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
    for c = 1:nA
        plot(a, D{c}.t, D{c}.e, '-', 'Color', COL(c,:), 'LineWidth', 2.0);
    end
    ylabel(a, {'a-hat error','(%)'}, 'FontSize', FS, 'FontWeight', 'bold');
    ylim(a, [-12 12]);

    a = nexttile(tl); A(3)=a; hold(a,'on');
    h = gobjects(nA+1,1);
    h(1) = plot(a, D{1}.t, D{1}.bt, '-', 'Color', COL_TRUE, 'LineWidth', 3.2);
    for c = 1:nA
        h(c+1) = plot(a, D{c}.t, D{c}.bh, '-', 'Color', COL(c,:), 'LineWidth', 1.8);
    end
    legend(a, h, [{'b  true'}, ARMS(:,2).'], 'Location','northoutside', ...
        'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel(a, 'b', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');

    for q = 1:3
        set(A(q),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
        grid(A(q),'off'); xlim(A(q), tlim);
        if q < 3; set(A(q),'XTickLabel',[]); end
    end

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fn = fullfile(root,'test_results','apd_acov_meng','arms_performance.png');
    exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure -> %s  (%d seeds averaged)\n', fn, D{1}.ns);
    out = D;
end

function b_of_w = local_b_true_curve()
    wq = linspace(1.05, 30, 4000);  cp = zeros(size(wq));
    for i = 1:numel(wq); [~, c] = calc_correction_functions(wq(i)); cp(i) = c; end
    a_tr = 1 ./ cp;
    b_tr = gradient(a_tr, wq) ./ (1 - a_tr).^2;
    b_of_w = @(w) interp1(wq, b_tr, w, 'linear', 'extrap');
end
