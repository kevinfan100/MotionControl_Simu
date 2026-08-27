function out = compare_consider_b(seeds)
%COMPARE_CONSIDER_B  Stage-2 acceptance of consider-b on the canonical deep band.
%
%   out = compare_consider_b(1:8);
%
% STATUS: ACTIVE | acceptance for formC_state_b.tex "Consider b" (2026-08-27)
%
% Three paired arms differing only in slot 5:
%   best       b estimated   (K_5 free,  P55 shrinks)          -- production so far
%   bmid       b locked      (K_5 = 0,   P55 = 0)
%   bconsider  b considered  (K_5 = 0,   P55 = P_bb carried)   -- the candidate
%
% Registered before the run (README + inject_prereg.txt):
%   A1 fixture: arm best, seed 7, a_bar_hat_z[end] = 0.107505 bit-for-bit
%   A2 consider mechanics: b_hat constant = 8/9 to 1e-15; sqrt(P55) constant =
%      b_half to 1e-12; P45 nonzero and ~ (1-a_hat)/b_hat * P_bb (sign and shape)
%   A3 trough bias falls toward the x3 arm (~ +7 %) from +22.65 %
%   A4 TOTAL honesty sqrt(bias^2+spread^2)/sqrtP44 in the end hold in [0.3, 1.2]
%      (an upper-bound container; spread/sqrtP alone is NOT the criterion here)
%   A5 per-seed sd of the end-hold bias < 12 %
%   A6 K1(4) on the descent >= the 'best' value (sign competition moved)

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'consider_b');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  ns = numel(seeds);

    % ---- A1 fixture ------------------------------------------------------
    c7 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false));
    v = c7.runs{1}.a_bar_hat_out(end, ax);
    fprintf('[A1] fixture a_bar_hat_z[end] = %.6f (expect 0.107505)  %s\n', v, local_pf(abs(v-0.107505) < 5e-7));

    ARMS = {'best', 'bmid', 'bconsider'};
    R = struct();
    for a = 1:3
        O = run_formC_b(struct('arm', ARMS{a}, 'ap_src', 'post', 'seeds', seeds, 'verbose', false));
        assert(numel(O.runs) == ns);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        a_nom = O.runs{1}.a_nom;
        F = struct('t', O.runs{1}.tout(:), 'ah', G('a_bar_hat_out'), 'at', G('a_true_out')/a_nom, ...
                   'sP44', G('P_a_out')/a_nom, 'sP55', G('P_b_out'), 'P41', G('P41_out'), ...
                   'K1', G('K_a_y1_out'), 'K2', G('K_a_y2_out'), 'bh', G('b_hat_out'));
        pb = O.runs{1}.ctrl_const.Pf_b_std;           % scalar from the driver, 3x1 inside the controller
        F.b_half = pb(min(ax, numel(pb)));
        R.(ARMS{a}) = F;
    end
    save(fullfile(od, 'compare_consider_b.mat'), '-struct', 'R');

    % ---- A2 consider mechanics -------------------------------------------
    C = R.bconsider;  kk = 2:numel(C.t);
    db = max(abs(C.bh(kk,:) - 8/9), [], 'all');
    dP55 = max(abs(C.sP55(kk,:) - C.b_half), [], 'all');
    fprintf('[A2] bconsider: max|b_hat - 8/9| = %.2e  %s ; max|sqrtP55 - b_half| = %.2e  %s (b_half %.4f)\n', ...
            db, local_pf(db < 1e-12), dP55, local_pf(dP55 < 1e-10), C.b_half);
    % P45 is not logged; test the induced P44 shape instead: sqrtP44(consider) vs
    % sqrt(sqrtP44(bmid)^2 + [(1-a_hat)/b_hat]^2 P_bb) along the run
    pred = sqrt(R.bmid.sP44(kk,:).^2 + ((1 - C.ah(kk,:)) / (8/9)).^2 * C.b_half^2);
    ratio = mean(C.sP44(kk,:), 2) ./ mean(pred, 2);
    tt = C.t(kk);
    SEG = {'hold start', tt > 0.05 & tt < 0.50; 'descend', tt > 0.55 & tt < 1.45; ...
           'oscillate', tt > 1.60 & tt < 3.40; 'hold end', tt > 3.70};
    fprintf('[A2] sqrtP44(consider) / first-order prediction:  ');
    for g = 1:4; fprintf('%s %.3f  ', SEG{g,1}, mean(ratio(SEG{g,2}))); end; fprintf('\n');

    % ---- A3-A6 table -----------------------------------------------------
    fprintf('\n%-10s %-11s %9s %8s %9s %9s %9s %9s %9s\n', 'arm', 'segment', 'bias %', 'SEM', ...
            'sd %', 'sqrtP44', 'spr/sP', 'TOTAL/sP', 'K1(4)');
    T = struct();
    for a = 1:3
        F = R.(ARMS{a});
        for g = 1:4
            m = SEG{g,2};
            e = F.ah(kk,:) ./ F.at(kk,:) - 1;   eb = mean(e(m,:), 1);
            d = F.ah(kk,:) - F.at(kk,:);
            bias = mean(mean(d(m,:), 1));  spr = mean(std(d(m,:), 0, 2));  sP = mean(F.sP44(kk(m),:), 'all');
            tot = sqrt(bias^2 + spr^2) / sP;
            fprintf('%-10s %-11s %+9.2f %8.2f %9.2f %9.4f %9.2f %9.2f %+9.4f\n', ARMS{a}, SEG{g,1}, ...
                    100*mean(eb), 100*std(eb)/sqrt(ns), 100*std(eb), sP, spr/sP, tot, mean(F.K1(kk(m),:), 'all'));
            if g == 4; T.(ARMS{a}) = [mean(eb) std(eb)/sqrt(ns) std(eb) sP spr/sP tot]; end
        end
    end
    m = SEG{4,2};
    e1 = mean((R.best.ah(kk,:) ./ R.best.at(kk,:) - 1) .* m, 1) / mean(m);
    e3 = mean((R.bconsider.ah(kk,:) ./ R.bconsider.at(kk,:) - 1) .* m, 1) / mean(m);
    d = e3 - e1;
    fprintf('\npaired end-hold bias, bconsider - best: %+.2f +- %.2f %%  (t = %+.1f)\n', ...
            100*mean(d), 100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));
    fprintf('[A3] %s   [A4] TOTAL/sP = %.2f %s   [A5] sd = %.1f %% %s\n', ...
            local_pf(T.bconsider(1) < 0.10), T.bconsider(6), local_pf(T.bconsider(6) >= 0.3 && T.bconsider(6) <= 1.2), ...
            100*T.bconsider(3), local_pf(T.bconsider(3) < 0.12));

    out = struct('R', R, 'T', T, 'SEG', {SEG});
    local_fig(R, ARMS, fullfile(od, 'compare_consider_b.png'));
    fprintf('\nfigure -> %s\n', od);
end

function s = local_pf(ok); if ok; s = 'PASS'; else; s = 'FAIL'; end; end

% =======================================================================
function local_fig(R, ARMS, fpath)
    FS = 17; AXLW = 2.0; GRY = [0.5 0.5 0.5];
    COL = [0 0.2 0.9; 0.9 0.5 0; 0.1 0.6 0.2];
    f = figure('Position',[40 40 1300 1050],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    t = R.best.t(2:end);

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:3
        F = R.(ARMS{a});
        plot(a1,t,movmean(100*(mean(F.ah(2:end,:),2)./mean(F.at(2:end,:),2)-1),51),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',ARMS{a});
    end
    ylabel(a1,'a_{hat}/a_{true} - 1  [%]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    for a = 1:3
        F = R.(ARMS{a});
        plot(a2,t,mean(F.sP44(2:end,:),2),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',['\surdP_{44}  ' ARMS{a}]);
    end
    C = R.bconsider;
    plot(a2,t,(1-mean(C.ah(2:end,:),2))/(8/9)*C.b_half,':','Color',[0.55 0.3 0.75],'LineWidth',2.2, ...
         'DisplayName','(1-a_{hat})/b_{hat} \cdot \surdP_{bb}');
    set(a2,'YScale','log'); ylabel(a2,'\surdP_{44}  [a_o]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',11);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:3
        F = R.(ARMS{a});
        plot(a3,t,movmean(mean(F.K1(2:end,:),2),51),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',['K_1(4)  ' ARMS{a}]);
    end
    xlabel(a3,'t [s]'); ylabel(a3,'K_1(4)');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
