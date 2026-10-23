function out = check_b_route_into_P44(seeds)
%CHECK_B_ROUTE_INTO_P44  Is the law-shape error ALREADY budgeted in P44 through
%   the b state (P55 -> F_e(4,5) -> P44), before a new Q44 term is derived?
%
%   out = check_b_route_into_P44(1:8);
%
% STATUS: ACTIVE | pre-derivation check for the Q44 law-error budget
%   (memory project-formC-inject-response-y1-amplifies-2026-08-26)
%
% METHOD. Paired canonical-deep runs, arm 'best' (b free, sqrt(P55[0]) = b_half)
% vs arm 'bmid' (b locked at the SAME seed 8/9, P55 = 0). Everything else is
% identical, so the difference in P44 along the run is exactly what the b
% route contributes. Reported alongside: sqrt(P55) itself, P41, K1(4), and the
% hold-end bias, so the b route's share of the covariance AND of the answer
% are both visible.
%
% READING. If sqrt(P44) is nearly identical between the arms, the b route
% carries almost nothing and a law-error term in Q44 would not double count.
% If 'best' has a much larger P44 near the wall, the budget is already there
% and the question becomes why it is not winning the P(4,1) sign competition.

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'b_route_P44');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  ns = numel(seeds);

    A = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false));
    B = run_formC_b(struct('arm', 'bmid', 'ap_src', 'post', 'seeds', seeds, 'verbose', false));
    assert(abs(A.runs{1}.ctrl_const.b_init - B.runs{1}.ctrl_const.b_init) < 1e-12, ...
           'arms seed b differently; the comparison is not paired');

    t = A.runs{1}.tout(:);  N = numel(t);  a_nom = A.runs{1}.a_nom;
    assert(numel(A.runs) == ns, 'expected %d runs, got %d', ns, numel(A.runs));
    % O.runs is a COLUMN cell; force a row so cell2mat gives N x ns, not (N*ns) x 1
    G = @(O, f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
    sP44_a = G(A, 'P_a_out') / a_nom;   sP44_b = G(B, 'P_a_out') / a_nom;   % driver stores sqrt
    sP55_a = G(A, 'P_b_out');           sP55_b = G(B, 'P_b_out');
    P41_a  = G(A, 'P41_out');           P41_b  = G(B, 'P41_out');
    K1_a   = G(A, 'K_a_y1_out');        K1_b   = G(B, 'K_a_y1_out');
    ah_a   = G(A, 'a_bar_hat_out');     ah_b   = G(B, 'a_bar_hat_out');
    at_a   = G(A, 'a_true_out') / a_nom; at_b  = G(B, 'a_true_out') / a_nom;
    bh_a   = G(A, 'b_hat_out');

    kk = 2:N;  tt = t(kk);
    SEG = {'hold start', tt > 0.05 & tt < 0.50; 'descend', tt > 0.55 & tt < 1.45; ...
           'oscillate', tt > 1.60 & tt < 3.40; 'hold end', tt > 3.70};
    fprintf('\nb route into P44: arm best (b free) vs bmid (b locked, same seed %.4f)\n', ...
            A.runs{1}.ctrl_const.b_init);
    fprintf('%-11s %10s %10s %8s | %10s %8s | %10s %10s | %9s %9s\n', 'segment', ...
            'sqrtP44 free', 'locked', 'ratio', 'sqrtP55', 'b_hat', 'P41 free', 'locked', 'K1 free', 'locked');
    for q = 1:size(SEG,1)
        m = SEG{q,2};  r = @(X) mean(X(kk(m),:), 'all');
        fprintf('%-11s %10.5f %10.5f %8.3f | %10.4f %8.4f | %+10.2e %+10.2e | %+9.4f %+9.4f\n', ...
                SEG{q,1}, r(sP44_a), r(sP44_b), r(sP44_a)/r(sP44_b), r(sP55_a), r(bh_a), ...
                r(P41_a), r(P41_b), r(K1_a), r(K1_b));
    end
    m = SEG{end,2};
    ea = mean(ah_a(kk(m),:) ./ at_a(kk(m),:) - 1, 1);  eb = mean(ah_b(kk(m),:) ./ at_b(kk(m),:) - 1, 1);
    fprintf('\nhold-end bias:  best %+.2f+-%.2f %%   bmid %+.2f+-%.2f %%   paired %+.2f+-%.2f %%\n', ...
            100*mean(ea), 100*std(ea)/sqrt(ns), 100*mean(eb), 100*std(eb)/sqrt(ns), ...
            100*mean(ea-eb), 100*std(ea-eb)/sqrt(ns));
    % how much of the shape-error band does P55 cover, and what does F_e(4,5) make of it
    fprintf('sqrt(P55): [0] %.4f -> [end] %.4f ; b_true band 0.867..0.928 = +-%.4f about 8/9\n', ...
            sP55_a(1,1), mean(sP55_a(end,:)), (0.928-0.867)/2);

    out = struct('t', tt, 'sP44_a', sP44_a(kk,:), 'sP44_b', sP44_b(kk,:), 'sP55_a', sP55_a(kk,:), ...
                 'P41_a', P41_a(kk,:), 'P41_b', P41_b(kk,:), 'K1_a', K1_a(kk,:), 'K1_b', K1_b(kk,:), ...
                 'ea', ea, 'eb', eb);
    local_fig(out, fullfile(od, 'b_route_P44.png'));
    fprintf('figure -> %s\n', od);
end

function local_fig(o, fpath)
    FS = 17; AXLW = 2.0; BLU = [0 0.2 0.9]; ORG = [0.9 0.5 0]; GRY = [0.5 0.5 0.5];
    f = figure('Position',[40 40 1300 1000],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1,o.t,mean(o.sP44_a,2),'-','Color',BLU,'LineWidth',2.2,'DisplayName','\surdP_{44}  b free (best)');
    plot(a1,o.t,mean(o.sP44_b,2),'-','Color',ORG,'LineWidth',2.2,'DisplayName','\surdP_{44}  b locked (bmid)');
    set(a1,'YScale','log'); ylabel(a1,'\surdP_{44}  [a_o]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    plot(a2,o.t,mean(o.P41_a,2),'-','Color',BLU,'LineWidth',2.2,'DisplayName','P_{41}  b free');
    plot(a2,o.t,mean(o.P41_b,2),'-','Color',ORG,'LineWidth',2.2,'DisplayName','P_{41}  b locked');
    ylabel(a2,'P_{41}');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    plot(a3,o.t,movmean(mean(o.K1_a,2),51),'-','Color',BLU,'LineWidth',2.2,'DisplayName','K_1(4)  b free');
    plot(a3,o.t,movmean(mean(o.K1_b,2),51),'-','Color',ORG,'LineWidth',2.2,'DisplayName','K_1(4)  b locked');
    xlabel(a3,'t [s]'); ylabel(a3,'K_1(4)');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
