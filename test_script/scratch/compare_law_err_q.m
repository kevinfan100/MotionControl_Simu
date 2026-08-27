function out = compare_law_err_q(seeds)
%COMPARE_LAW_ERR_Q  Level-2 acceptance: the correlated law-error container q_law.
%
%   out = compare_law_err_q(1:8);
%
% STATUS: ACTIVE | acceptance for formC_state_b.tex "law error as process noise"
%
% Arms (canonical deep, paired):
%   best            production
%   best  + lawq    the candidate
%   bmid  + lawq    b locked at the anchor + the container (is the b state needed?)
%   best  + lawq, a_cov x2 and /2   parameter-leakage check (rule 2)
% Registered predictions P-L2-1..7 are in inject_prereg.txt / the ref tex.

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'law_err_q');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  ns = numel(seeds);

    % ---- P-L2-7 negative controls -----------------------------------------
    c7 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false));
    v = c7.runs{1}.a_bar_hat_out(end, ax);
    fprintf('[P-L2-7] fixture (flag off) a_bar_hat_z[end] = %.6f  %s\n', v, local_pf(abs(v-0.107505) < 5e-7));
    s0 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, 'scenario', 'shallow'));
    s1 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, 'scenario', 'shallow', 'law_err_q', false));
    d = max(abs(s0.runs{1}.a_bar_hat_out(:) - s1.runs{1}.a_bar_hat_out(:)));
    fprintf('[P-L2-7] shallow, flag off twice: max diff %.1e  %s\n', d, local_pf(d == 0));

    ARMS = {'best', struct('arm','best'); ...
            'best+lawq', struct('arm','best','law_err_q',true); ...
            'bmid+lawq', struct('arm','bmid','law_err_q',true); ...
            'best+lawq acov x2', struct('arm','best','law_err_q',true,'a_cov_scale',2); ...
            'best+lawq acov /2', struct('arm','best','law_err_q',true,'a_cov_scale',0.5)};
    R = struct();  keys = cell(size(ARMS,1),1);
    for a = 1:size(ARMS,1)
        o = ARMS{a,2};  o.ap_src = 'post';  o.seeds = seeds;  o.verbose = false;
        O = run_formC_b(o);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        a_nom = O.runs{1}.a_nom;
        F = struct('name', ARMS{a,1}, 't', O.runs{1}.tout(:), 'ah', G('a_bar_hat_out'), ...
                   'at', G('a_true_out')/a_nom, 'sP44', G('P_a_out')/a_nom, ...
                   'K1', G('K_a_y1_out'), 'Q44', G('Q44_out'));
        keys{a} = matlab.lang.makeValidName(ARMS{a,1});
        R.(keys{a}) = F;
    end
    save(fullfile(od, 'compare_law_err_q.mat'), 'R', 'keys');

    kk = 2:numel(R.(keys{1}).t);  tt = R.(keys{1}).t(kk);
    SEG = {'hold start', tt > 0.05 & tt < 0.50; 'descend', tt > 0.55 & tt < 1.45; ...
           'oscillate', tt > 1.60 & tt < 3.40; 'hold end', tt > 3.70};
    fprintf('\n%-20s %-11s %9s %7s %8s %9s %9s %9s %9s\n', 'arm', 'segment', 'bias %', 'SEM', ...
            'sd %', 'sqrtP44', 'spr/sP', 'TOTAL/sP', 'K1(4)');
    T = struct();
    for a = 1:numel(keys)
        F = R.(keys{a});
        for g = 1:4
            m = SEG{g,2};
            e = F.ah(kk,:) ./ F.at(kk,:) - 1;   eb = mean(e(m,:), 1);
            dd = F.ah(kk,:) - F.at(kk,:);
            bias = mean(mean(dd(m,:),1));  spr = mean(std(dd(m,:),0,2));  sP = mean(F.sP44(kk(m),:),'all');
            tot = sqrt(bias^2 + spr^2)/sP;
            fprintf('%-20s %-11s %+9.2f %7.2f %8.2f %9.4f %9.2f %9.2f %+9.4f\n', F.name, SEG{g,1}, ...
                    100*mean(eb), 100*std(eb)/sqrt(ns), 100*std(eb), sP, spr/sP, tot, mean(F.K1(kk(m),:),'all'));
            if g == 4; T.(keys{a}) = [mean(eb) std(eb)/sqrt(ns) std(eb) sP spr/sP tot]; end
            if g == 2; T.([keys{a} '_desc']) = mean(F.K1(kk(m),:),'all'); end
        end
    end
    m = SEG{4,2};
    eb0 = mean((R.(keys{1}).ah(kk,:)./R.(keys{1}).at(kk,:)-1).*m,1)/mean(m);
    fprintf('\npaired end-hold bias vs best:\n');
    for a = 2:numel(keys)
        e = mean((R.(keys{a}).ah(kk,:)./R.(keys{a}).at(kk,:)-1).*m,1)/mean(m);  d = e - eb0;
        fprintf('  %-20s %+7.2f +- %.2f %%  (t = %+.1f)\n', R.(keys{a}).name, 100*mean(d), 100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));
    end
    k2 = keys{2};
    fprintf('\n[P-L2-1] bias %+.2f %%  %s   [P-L2-2] TOTAL/sP %.2f  %s   [P-L2-3] sd %.1f %%  %s   [P-L2-4] desc K1 %+.3f  %s\n', ...
        100*T.(k2)(1), local_pf(T.(k2)(1) >= 0.05 && T.(k2)(1) <= 0.10), T.(k2)(6), local_pf(T.(k2)(6) >= 0.8 && T.(k2)(6) <= 1.5), ...
        100*T.(k2)(3), local_pf(T.(k2)(3) <= 0.12), T.([k2 '_desc']), local_pf(T.([k2 '_desc']) >= 0.07));
    dB = 100*(T.(keys{4})(1) - T.(k2)(1)); dB2 = 100*(T.(keys{5})(1) - T.(k2)(1));
    dH = T.(keys{4})(6) - T.(k2)(6);  dH2 = T.(keys{5})(6) - T.(k2)(6);
    fprintf('[P-L2-6] a_cov x2: bias %+.2f pp, honesty %+.2f ; /2: bias %+.2f pp, honesty %+.2f   %s\n', ...
        dB, dH, dB2, dH2, local_pf(max(abs([dB dB2])) < 1 && max(abs([dH dH2])) < 0.1));

    out = struct('R', R, 'keys', {keys}, 'T', T);
    local_fig(R, keys, fullfile(od, 'compare_law_err_q.png'));
    fprintf('\nfigure -> %s\n', od);
end

function s = local_pf(ok); if ok; s = 'PASS'; else; s = 'FAIL'; end; end

function local_fig(R, keys, fpath)
    FS = 17; AXLW = 2.0; GRY = [0.5 0.5 0.5];
    COL = [0 0.2 0.9; 0.85 0.1 0.1; 0.1 0.6 0.2; 0.9 0.5 0; 0.55 0.3 0.75];
    f = figure('Position',[40 40 1300 1050],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    t = R.(keys{1}).t(2:end);
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:3
        F = R.(keys{a});
        plot(a1,t,movmean(100*(mean(F.ah(2:end,:),2)./mean(F.at(2:end,:),2)-1),51),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',F.name);
    end
    ylabel(a1,'a_{hat}/a_{true} - 1  [%]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    for a = 1:3
        F = R.(keys{a});
        plot(a2,t,mean(F.sP44(2:end,:),2),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',['\surdP_{44}  ' F.name]);
    end
    F = R.(keys{2});
    plot(a2,t,sqrt(mean(F.Q44(2:end,:),2)),':','Color',COL(2,:),'LineWidth',2.0,'DisplayName','\surdQ_{44} per step  best+lawq');
    F = R.(keys{1});
    plot(a2,t,sqrt(mean(F.Q44(2:end,:),2)),':','Color',COL(1,:),'LineWidth',2.0,'DisplayName','\surdQ_{44} per step  best');
    set(a2,'YScale','log'); ylabel(a2,'[a_o]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',10,'NumColumns',3);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:3
        F = R.(keys{a});
        plot(a3,t,movmean(mean(F.K1(2:end,:),2),51),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',['K_1(4)  ' F.name]);
    end
    xlabel(a3,'t [s]'); ylabel(a3,'K_1(4)');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
