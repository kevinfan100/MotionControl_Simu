function out = compare_law_exact_step(seeds)
%COMPARE_LAW_EXACT_STEP  Does the exact (quadrature-free) law step remove the
%   Euler ratchet measured in ledger_hold_ahat_legs?
%
%   out = compare_law_exact_step(1:8);
%
% STATUS: ACTIVE | acceptance for ctrl_const.law_exact_step (2026-08-30)
%
% Arms (canonical deep, paired seeds):
%   best                 production (forward-Euler law step)
%   best + exact         1/(1-A[k+1]) = 1/(1-A[k]) + b M   (exact step of dA/dw = b(1-A)^2)
%   best + exact + lawq  exact step plus the P44 lever, to see what is left for the y1 leg
%
% Registered predictions (written before the run):
%   P-X0  flag off reproduces the fixture bit for bit (seed 7: 0.108275).
%   P-X1  oscillate-segment law-leg excess (law leg - d a_true) drops from
%         +0.013 a_o to within +-0.004 a_o (the ratchet +0.0093 is gone; what
%         remains is the a_hat != a_true evaluation error, not quadrature).
%   P-X2  end-hold bias drops from +23 % to 8..14 % (removing 0.009..0.013 a_o
%         of the 0.020 a_o excess); paired diff t < -4.
%   P-X3  the y1 leg over the oscillation is NOT reduced by the exact step
%         (it is the amplifier, a different mechanism): |change| < 0.004 a_o.
%   P-X4  exact + lawq: bias below +5 %.
% Verdict rule: P-X2 FAIL (bias unchanged) => the ratchet is a ledger
% correlate, not a cause; stop here.

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'law_exact_step');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  ns = numel(seeds);
    pc = physical_constants();  R = pc.R;  lam = 0.7;

    c7 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, ...
                            'ctrl_const_override', struct('law_exact_step', false)));
    v = c7.runs{1}.a_bar_hat_out(end, ax);
    fprintf('[P-X0] flag off, fixture a_bar_hat_z[end] = %.6f  (expect 0.108275)  %s\n', v, local_pf(abs(v - 0.108275) < 5e-7));

    ARMS = {'best',            struct(); ...
            'best+exact',      struct('law_exact_step', true); ...
            'best+exact+lawq', struct('law_exact_step', true)};
    LAWQ = [false false true];
    R_ = struct();  keys = cell(size(ARMS,1),1);  LEG = struct();
    for a = 1:size(ARMS,1)
        o = struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                   'ctrl_const_override', ARMS{a,2}, 'law_err_q', LAWQ(a));
        O = run_formC_b(o);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
        F = struct('name', ARMS{a,1}, 't', t, 'ah', G('a_bar_hat_out'), 'at', G('a_true_out')/a_nom, ...
                   'sP44', G('P_a_out')/a_nom, 'K1', G('K_a_y1_out'), 'bh', G('b_hat_out'), ...
                   'dx3', G('delta_x_hat_3_out')/R, 'n1', G('innov_y1_out'), ...
                   'K2', G('K_a_y2_out'), 'n2', G('innov_y2_out'), 'hd', O.runs{1}.h_bar_d_out(:));
        keys{a} = matlab.lang.makeValidName(ARMS{a,1});
        R_.(keys{a}) = F;
        LEG.(keys{a}) = local_legs(F, lam, ns);
    end
    save(fullfile(od, 'compare_law_exact_step.mat'), 'R_', 'keys', 'LEG');

    % ---- segment table --------------------------------------------------
    kk = 2:numel(R_.(keys{1}).t);  tt = R_.(keys{1}).t(kk);
    SEG = {'hold start', tt > 0.05 & tt < 0.50; 'descend', tt > 0.55 & tt < 1.45; ...
           'oscillate', tt > 1.60 & tt < 3.40; 'hold end', tt > 3.70};
    fprintf('\n%-18s %-11s %9s %7s %8s %9s %9s %9s\n', 'arm', 'segment', 'bias %', 'SEM', 'sd %', 'sqrtP44', 'TOTAL/sP', 'K1(4)');
    EB = struct();
    for a = 1:numel(keys)
        F = R_.(keys{a});
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

    % ---- ledger legs over the oscillation ---------------------------------
    fprintf('\noscillation legs (a_o units, mean over seeds; SEM in brackets):\n');
    fprintf('%-18s %10s %10s %10s %10s %10s %10s %10s\n', 'arm', 'd a_hat', 'd a_true', 'law leg', 'y1 leg', 'y2 leg', 'resid', 'ratchet');
    for a = 1:numel(keys)
        L = LEG.(keys{a});
        fprintf('%-18s %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f %+10.4f\n', R_.(keys{a}).name, ...
                mean(L.dobs), mean(L.dtru), mean(L.law), mean(L.y1), mean(L.y2), mean(L.res), mean(L.rat));
        fprintf('%-18s %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n', '', ...
                std(L.dobs)/sqrt(ns), std(L.dtru)/sqrt(ns), std(L.law)/sqrt(ns), std(L.y1)/sqrt(ns), std(L.y2)/sqrt(ns), std(L.res)/sqrt(ns), std(L.rat)/sqrt(ns));
    end

    fprintf('\npaired end-hold differences vs best:\n');
    D = struct();
    for a = 2:numel(keys)
        d = EB.(keys{a}) - EB.(keys{1});  D.(keys{a}) = d;
        fprintf('  %-18s %+7.2f +- %.2f %%  (t = %+.1f)\n', R_.(keys{a}).name, 100*mean(d), 100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));
    end

    L0 = LEG.(keys{1});  L1 = LEG.(keys{2});
    ex0 = mean(L0.law - L0.dtru);  ex1 = mean(L1.law - L1.dtru);
    dy1 = mean(L1.y1 - L0.y1);
    b1 = 100*mean(EB.(keys{2}));  b2 = 100*mean(EB.(keys{3}));
    d2 = D.(keys{2});  t2 = mean(d2)/(std(d2)/sqrt(ns));
    fprintf('\n[P-X1] law-leg excess %+.4f -> %+.4f a_o  %s\n', ex0, ex1, local_pf(abs(ex1) <= 0.004));
    fprintf('[P-X2] end-hold bias %+.2f %%, paired t = %+.1f  %s\n', b1, t2, local_pf(b1 >= 8 && b1 <= 14 && t2 < -4));
    fprintf('[P-X3] y1-leg change %+.4f a_o  %s\n', dy1, local_pf(abs(dy1) < 0.004));
    fprintf('[P-X4] exact+lawq bias %+.2f %%  %s\n', b2, local_pf(b2 < 5));

    out = struct('R', R_, 'keys', {keys}, 'LEG', LEG, 'EB', EB);
    local_fig(R_, keys, fullfile(od, 'compare_law_exact_step.png'));
    fprintf('\nfigure -> %s\n', od);
end

function L = local_legs(F, lam, ns)
    t = F.t;  dwd = [0; diff(F.hd)];
    idx = find(t > 1.60 & t < 3.40);  k0 = idx(1);  k1 = idx(end);  kk = (k0+1):k1;
    dw  = dwd(kk) + (1 - lam) * F.dx3(kk-1, :);
    ap  = F.bh(kk-1, :) .* (1 - F.ah(kk-1, :)).^2;
    L.law  = sum(ap .* dw, 1);
    L.y1   = sum(F.K1(kk, :) .* F.n1(kk, :), 1);
    L.y2   = sum(F.K2(kk, :) .* F.n2(kk, :), 1);
    L.dobs = F.ah(k1, :) - F.ah(k0, :);
    L.dtru = F.at(k1, :) - F.at(k0, :);
    L.res  = L.dobs - (L.law + L.y1 + L.y2);
    L.rat  = sum(2 * F.bh(kk-1,:).^2 .* (1 - F.ah(kk-1,:)).^3 .* dw.^2, 1);
    assert(numel(L.dobs) == ns);
end

function s = local_pf(ok); if ok; s = 'PASS'; else; s = 'FAIL'; end; end

function local_fig(R_, keys, fpath)
    FS = 17; AXLW = 2.0; GRY = [0.5 0.5 0.5];
    COL = [0 0.2 0.9; 0.85 0.1 0.1; 0.1 0.6 0.2];
    f = figure('Position',[40 40 1300 900],'Color','w','Visible','off');
    tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
    t = R_.(keys{1}).t(2:end);
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:numel(keys)
        F = R_.(keys{a});
        plot(a1,t,movmean(100*(mean(F.ah(2:end,:),2)./mean(F.at(2:end,:),2)-1),51),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',F.name);
    end
    ylabel(a1,'a_{hat}/a_{true} - 1  [%]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:numel(keys)
        F = R_.(keys{a});
        plot(a2,t,mean(F.ah(2:end,:)-F.at(2:end,:),2),'-','Color',COL(a,:),'LineWidth',2.2,'DisplayName',['a_{hat} - a_{true}  ' F.name]);
    end
    xlabel(a2,'t [s]'); ylabel(a2,'[a_o]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
