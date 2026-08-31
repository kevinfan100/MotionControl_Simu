function out = check_lawq_btrue_floor(seeds)
%CHECK_LAWQ_BTRUE_FLOOR  Closure check C (2026-08-30): is the residual bias that
%   survives the q_law container the law error itself (b = 8/9 vs b_true(a))?
%
%   out = check_lawq_btrue_floor(1:8);
%
% STATUS: ACTIVE | closes (or reopens) the law-error-budget line
%
% Arms (canonical deep, paired seeds):
%   best                 production
%   best + lawq          level-2 container
%   btrue(cmd)           law reads b_true(w_bar_d) exactly, slot 5 locked (no law error)
%   btrue(cmd) + lawq    container fires although the law is right
%
% Registered predictions (written before the run):
%   P-C1  btrue+lawq end-hold bias within +-3 % (relative): the container carries
%         no bias of its own; anything left is the amplifier acting on zero error.
%   P-C2  btrue alone within +-3 %: the amplifier only amplifies law error
%         (a'_true arm on the Meng ramp gave -0.002 a_o at the cmd point, 08-26).
%   P-C3  paired (best+lawq) - (btrue+lawq) in [6, 13] %: the law floor, to be
%         compared with the integrated law-alone error +9.7 % of the trough
%         (check_delta_b_curve, 08-27).
%   P-C4  fixture: best, seed 7, a_bar_hat_z[end] = 0.108275 (post-IF(1) base).
% Verdict rule: C1 & C2 & C3 PASS -> residual = law floor, the loop is closed;
% C1 or C2 FAIL -> something other than law error survives, the loop stays open.

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'law_err_q');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  ns = numel(seeds);

    c7 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false));
    v = c7.runs{1}.a_bar_hat_out(end, ax);
    fprintf('[P-C4] fixture a_bar_hat_z[end] = %.6f  (expect 0.108275)  %s\n', v, local_pf(abs(v - 0.108275) < 5e-7));

    ARMS = {'best',            struct('arm','best'); ...
            'best+lawq',       struct('arm','best','law_err_q',true); ...
            'btrue(cmd)',      struct('arm','best','b_true',true,'b_true_at','cmd'); ...
            'btrue(cmd)+lawq', struct('arm','best','b_true',true,'b_true_at','cmd','law_err_q',true)};
    R = struct();  keys = cell(size(ARMS,1),1);
    for a = 1:size(ARMS,1)
        o = ARMS{a,2};  o.ap_src = 'post';  o.seeds = seeds;  o.verbose = false;
        O = run_formC_b(o);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        a_nom = O.runs{1}.a_nom;
        F = struct('name', ARMS{a,1}, 't', O.runs{1}.tout(:), 'ah', G('a_bar_hat_out'), ...
                   'at', G('a_true_out')/a_nom, 'sP44', G('P_a_out')/a_nom, ...
                   'K1', G('K_a_y1_out'), 'bh', G('b_hat_out'));
        keys{a} = matlab.lang.makeValidName(ARMS{a,1});
        R.(keys{a}) = F;
        fprintf('  arm %-16s b_hat used: min %.4f max %.4f\n', F.name, min(F.bh(:)), max(F.bh(:)));
    end
    save(fullfile(od, 'check_lawq_btrue_floor.mat'), 'R', 'keys');

    kk = 2:numel(R.(keys{1}).t);  tt = R.(keys{1}).t(kk);
    SEG = {'hold start', tt > 0.05 & tt < 0.50; 'descend', tt > 0.55 & tt < 1.45; ...
           'oscillate', tt > 1.60 & tt < 3.40; 'hold end', tt > 3.70};
    fprintf('\n%-18s %-11s %9s %7s %8s %9s %9s %9s\n', 'arm', 'segment', 'bias %', 'SEM', 'sd %', 'sqrtP44', 'TOTAL/sP', 'K1(4)');
    EB = struct();
    for a = 1:numel(keys)
        F = R.(keys{a});
        for g = 1:4
            m = SEG{g,2};
            e = F.ah(kk,:) ./ F.at(kk,:) - 1;   eb = mean(e(m,:), 1);
            dd = F.ah(kk,:) - F.at(kk,:);
            bias = mean(mean(dd(m,:),1));  spr = mean(std(dd(m,:),0,2));  sP = mean(F.sP44(kk(m),:),'all');
            fprintf('%-18s %-11s %+9.2f %7.2f %8.2f %9.4f %9.2f %+9.4f\n', F.name, SEG{g,1}, ...
                    100*mean(eb), 100*std(eb)/sqrt(ns), 100*std(eb), sP, sqrt(bias^2+spr^2)/sP, mean(F.K1(kk(m),:),'all'));
            if g == 4; EB.(keys{a}) = eb; end
        end
    end
    fprintf('\npaired end-hold differences:\n');
    P = {2, 4, '(best+lawq) - (btrue+lawq)   [law floor]'; 1, 3, '(best) - (btrue)             [law floor x amplifier]'; ...
         3, 4, '(btrue) - (btrue+lawq)       [container alone, no law error]'};
    D = zeros(3,1);
    for i = 1:3
        d = EB.(keys{P{i,1}}) - EB.(keys{P{i,2}});  D(i) = mean(d);
        fprintf('  %-52s %+7.2f +- %.2f %%  (t = %+.1f)\n', P{i,3}, 100*mean(d), 100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));
    end
    b1 = 100*mean(EB.(keys{4}));  b2 = 100*mean(EB.(keys{3}));  b3 = 100*D(1);
    fprintf('\n[P-C1] btrue+lawq bias %+.2f %%  %s   [P-C2] btrue bias %+.2f %%  %s   [P-C3] law floor %+.2f %% (integrated law-alone +9.7)  %s\n', ...
            b1, local_pf(abs(b1) <= 3), b2, local_pf(abs(b2) <= 3), b3, local_pf(b3 >= 6 && b3 <= 13));
    out = struct('R', R, 'keys', {keys}, 'EB', EB, 'D', D);
    local_fig(R, keys, fullfile(od, 'check_lawq_btrue_floor.png'));
    fprintf('\nfigure -> %s\n', od);
end

function s = local_pf(ok); if ok; s = 'PASS'; else; s = 'FAIL'; end; end

function local_fig(R, keys, fpath)
    FS = 17; AXLW = 2.0; GRY = [0.5 0.5 0.5];
    COL = [0 0.2 0.9; 0.85 0.1 0.1; 0.1 0.6 0.2; 0.9 0.5 0];
    LS = {'-', '-', '--', '--'};
    f = figure('Position',[40 40 1300 900],'Color','w','Visible','off');
    tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
    t = R.(keys{1}).t(2:end);
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:4
        F = R.(keys{a});
        plot(a1,t,movmean(100*(mean(F.ah(2:end,:),2)./mean(F.at(2:end,:),2)-1),51),LS{a},'Color',COL(a,:),'LineWidth',2.2,'DisplayName',F.name);
    end
    ylabel(a1,'a_{hat}/a_{true} - 1  [%]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    for a = 1:4
        F = R.(keys{a});
        plot(a2,t,mean(F.sP44(2:end,:),2),LS{a},'Color',COL(a,:),'LineWidth',2.2,'DisplayName',['\surdP_{44}  ' F.name]);
    end
    set(a2,'YScale','log'); xlabel(a2,'t [s]'); ylabel(a2,'[a_o]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',11,'NumColumns',4);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
