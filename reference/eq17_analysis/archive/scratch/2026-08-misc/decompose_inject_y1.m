function out = decompose_inject_y1(S)
%DECOMPOSE_INJECT_Y1  Where does the y1 path's AMPLIFYING response come from?
%
%   S   = load('test_results/inject_response/inject_response.mat');
%   out = decompose_inject_y1(S);
%
% STATUS: ACTIVE | post-processing of run_inject_response (no new runs)
%
% run_inject_response measured that after a known +-5 % injection into a_bar,
% the y1 leg of the update pushes the injected arm FURTHER from baseline
% (cumulative +1.6..+2.5 x injected), i.e. it amplifies instead of correcting.
% The leg is K1(4)*innov1. Its paired difference splits exactly into
%
%     d(K*i) = dK * i_base  +  K_base * di  +  dK * di
%              [gain route]   [innovation route]  [cross]
%
% gain route      : the Kalman gain changed because a_hat changed
%                   (K1(4) = P41/S1, P41 built by F_e(4,3) ~ a_bar'(a_hat))
%                   -> EKF linearisation / rectification family
% innovation route: the position innovation changed because the CONTROLLER
%                   uses a_hat (pole moved), or because the filter predicts
%                   with the wrong a_bar -> closed-loop coupling family
%
% Both routes are read from the same log with the same K and innov the
% controller actually used, so the split is an identity, not a model.

    t = S.t;  kk = 2:numel(t);  t = t(kk);  ns = S.ns;  frac = S.frac;
    B = S.base;
    here = fileparts(mfilename('fullpath'));
    tg = ''; if isfield(S, 'tag'); tg = S.tag; end
    od = fullfile(fileparts(fileparts(here)), 'test_results', ['inject_response' tg]);

    Kb = B.K_a_y1_out(kk,:);  ib = B.innov_y1_out(kk,:);
    ab = B.a_bar_hat_out(kk,:);
    dj = mean(S.plus.a_bar_hat_out(kk,:) - ab, 2); [~, kj] = max(abs(diff(dj))); kj = kj + 1;

    R = struct();
    for c = {'plus', 'minus'}
        F = S.(c{1});
        Ka = F.K_a_y1_out(kk,:);  ia = F.innov_y1_out(kk,:);
        aa  = F.a_bar_hat_out(kk,:);                             % same row frame as ab
        amt = mean(aa(kj,:) - ab(kj,:));                         % signed injected amount
        dK = Ka - Kb;  di = ia - ib;
        g  = cumsum(mean(dK .* ib, 2)) / amt;      % gain route
        n  = cumsum(mean(Kb .* di, 2)) / amt;      % innovation route
        x  = cumsum(mean(dK .* di, 2)) / amt;      % cross
        tot = cumsum(mean(Ka .* ia - Kb .* ib, 2)) / amt;
        % the two ingredients themselves, so the routes can be read physically
        R.(c{1}) = struct('g', g, 'n', n, 'x', x, 'tot', tot, ...
                          'dK_rel', mean(dK, 2) ./ mean(abs(Kb), 2), ...
                          'di_nm',  mean(di, 2) * 2250, ...
                          'ib_nm',  mean(ib, 2) * 2250, 'amt', amt);
    end

    fprintf('\n[decompose] y1 leg response, cumulative / injected (identity: g + n + x = tot)\n');
    fprintf('%-8s %10s %10s %10s %10s %10s\n', 'arm', 'gain', 'innov', 'cross', 'sum', 'measured');
    for c = {'plus', 'minus'}
        r = R.(c{1});
        fprintf('%-8s %+10.3f %+10.3f %+10.3f %+10.3f %+10.3f\n', c{1}, ...
                r.g(end), r.n(end), r.x(end), r.g(end)+r.n(end)+r.x(end), r.tot(end));
    end
    % windows: growth (injection -> peak of r) and recovery (peak -> trough)
    rp = (S.plus.a_bar_hat_out(kk,:) - ab) ./ (ab(kj,:) * frac);
    [~, kpk] = max(mean(rp, 2));
    ktr = find(S.h_d(kk) <= 1.12, 1);
    fprintf('\nwindows: growth %.2f-%.2f s (w_bar_d %.2f -> %.2f), recovery %.2f-%.2f s\n', ...
            t(kj), t(kpk), S.h_d(kj+1), S.h_d(kpk+1), t(kpk), t(ktr));
    fprintf('%-8s %-10s %10s %10s %10s\n', 'arm', 'window', 'gain', 'innov', 'cross');
    for c = {'plus', 'minus'}
        r = R.(c{1});
        fprintf('%-8s %-10s %+10.3f %+10.3f %+10.3f\n', c{1}, 'growth', ...
                r.g(kpk)-r.g(kj), r.n(kpk)-r.n(kj), r.x(kpk)-r.x(kj));
        fprintf('%-8s %-10s %+10.3f %+10.3f %+10.3f\n', c{1}, 'recovery', ...
                r.g(ktr)-r.g(kpk), r.n(ktr)-r.n(kpk), r.x(ktr)-r.x(kpk));
    end
    % sign check on the ingredients, growth window
    m = kj:kpk;
    fprintf('\ningredients over the growth window (means):\n');
    fprintf('  baseline  K1(4) = %+.4f    innov1 = %+.4f nm\n', ...
            mean(mean(Kb(m,:))), mean(mean(ib(m,:)))*2250);
    for c = {'plus', 'minus'}
        r = R.(c{1});
        fprintf('  %-6s dK1/|K1| = %+.3f    d innov1 = %+.4f nm\n', c{1}, ...
                mean(r.dK_rel(m)), mean(r.di_nm(m)));
    end

    out = struct('t', t, 'kj', kj, 'kpk', kpk, 'ktr', ktr, 'R', R, 'h_d', S.h_d(kk));
    local_fig(out, fullfile(od, 'inject_y1_decompose.png'));
    fprintf('\nfigure -> %s\n', od);
end

% =======================================================================
function local_fig(o, fpath)
    FS = 17; AXLW = 2.0;
    ORG=[0.9 0.5 0]; GRN=[0.1 0.6 0.2]; GRY=[0.5 0.5 0.5]; BLU=[0 0.2 0.9]; RED=[0.85 0.1 0.1];
    f = figure('Position',[40 40 1300 1000],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    tj = o.t(o.kj); tpk = o.t(o.kpk); ttr = o.t(o.ktr);
    NM = {'plus','minus'}; CL = {ORG, GRN}; LB = {'+5%','-5%'};

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for c = 1:2
        r = o.R.(NM{c});
        plot(a1,o.t,r.tot,'-','Color',CL{c},'LineWidth',2.6,'DisplayName',[LB{c} ' total y_1 leg']);
        plot(a1,o.t,r.g,'--','Color',CL{c},'LineWidth',2.0,'DisplayName',[LB{c} ' gain route  \DeltaK_1\cdot innov_{base}']);
        plot(a1,o.t,r.n,':','Color',CL{c},'LineWidth',2.4,'DisplayName',[LB{c} ' innovation route  K_{1,base}\cdot\Deltainnov']);
    end
    xline(a1,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xline(a1,tpk,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xline(a1,ttr,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xlim(a1,[tj-0.5 o.t(end)]); ylim(a1,[-1 3]);
    ylabel(a1,'cumulative / injected');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',10,'NumColumns',3);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for c = 1:2
        plot(a2,o.t,movmean(o.R.(NM{c}).dK_rel,101),'-','Color',CL{c},'LineWidth',2.2, ...
             'DisplayName',[LB{c} '  \DeltaK_1(4) / |K_1(4)|_{base}']);
    end
    xline(a2,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xlim(a2,[tj-0.5 o.t(end)]); ylabel(a2,'[-]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    plot(a3,o.t,movmean(o.R.plus.ib_nm,101),'-','Color',BLU,'LineWidth',2.0,'DisplayName','innov_1 baseline [nm]');
    for c = 1:2
        plot(a3,o.t,movmean(o.R.(NM{c}).di_nm,101),'-','Color',CL{c},'LineWidth',2.2, ...
             'DisplayName',[LB{c} '  \Delta innov_1 [nm]']);
    end
    xline(a3,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xlim(a3,[tj-0.5 o.t(end)]); xlabel(a3,'t [s]'); ylabel(a3,'[nm]  (movmean 101)');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
