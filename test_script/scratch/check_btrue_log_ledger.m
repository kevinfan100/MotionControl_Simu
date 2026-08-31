function out = check_btrue_log_ledger(seeds)
%CHECK_BTRUE_LOG_LEDGER  Under the b_true oracle (the law is exactly right),
%   what does the log say moves a_hat away from a_true?
%
%   out = check_btrue_log_ledger(1:8);
%
% STATUS: ACTIVE | confidence check requested 2026-08-30
%
% Four arms, all with the filter reading the exact b_true(w_bar_d) (cheat):
%   noisy   Euler       canonical, 8 seeds
%   noisy   exact step
%   det     Euler       thermal off, meas noise off, y2 off, seed 7 (nothing random left)
%   det     exact step
% For each: end-hold bias; the oscillation split into law / y1 / y2 legs
% (identity residual must be ~0 for Euler; for the exact arm the residual IS
% the quadrature the exact step removed, and must equal -ratchet); per-cycle
% increments; and cumulative-leg curves in the figure.
%   ratchet = -1/2 sum a'' dw^2 = sum b^2 (1-a)^3 dw^2  (Euler's left-endpoint error)

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'btrue_log_ledger');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  R = physical_constants().R;  lam = 0.7;
    CO_det = struct('thermal_enable', false, 'meas_noise_enable', false);
    ARMS = {'noisy Euler', seeds, true,  struct(),                       struct(); ...
            'noisy exact', seeds, true,  struct('law_exact_step', true), struct(); ...
            'det Euler',   7,     false, struct(),                       CO_det; ...
            'det exact',   7,     false, struct('law_exact_step', true), CO_det};
    out = struct();  keys = cell(4,1);
    for a = 1:4
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', ARMS{a,2}, 'verbose', false, ...
                               'b_true', true, 'b_true_at', 'cmd', 'y2_on', ARMS{a,3}, ...
                               'ctrl_const_override', ARMS{a,4}, 'config_override', ARMS{a,5}));
        ns = numel(O.runs);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
        F = struct('name', ARMS{a,1}, 'ns', ns, 't', t, 'ah', G('a_bar_hat_out'), 'at', G('a_true_out')/a_nom, ...
                   'bh', G('b_hat_out'), 'dx3', G('delta_x_hat_3_out')/R, 'hd', O.runs{1}.h_bar_d_out(:), ...
                   'K1', G('K_a_y1_out'), 'n1', G('innov_y1_out'), 'K2', G('K_a_y2_out'), 'n2', G('innov_y2_out'));
        % per-step legs (posterior a_hat[k] - a_hat[k-1] = law_k + y1_k + y2_k + resid_k)
        kk = 2:numel(t);  dwd = [0; diff(F.hd)];
        F.dw   = dwd(kk) + (1 - lam) * F.dx3(kk-1, :);
        F.law  = F.bh(kk-1,:) .* (1 - F.ah(kk-1,:)).^2 .* F.dw;
        F.y1   = F.K1(kk,:) .* F.n1(kk,:);
        F.y2   = F.K2(kk,:) .* F.n2(kk,:);
        F.dah  = F.ah(kk,:) - F.ah(kk-1,:);
        F.dat  = F.at(kk,:) - F.at(kk-1,:);
        F.res  = F.dah - (F.law + F.y1 + F.y2);
        F.rat  = F.bh(kk-1,:).^2 .* (1 - F.ah(kk-1,:)).^3 .* F.dw.^2;
        F.tt   = t(kk);
        keys{a} = matlab.lang.makeValidName(ARMS{a,1});
        out.(keys{a}) = F;
        ho = F.tt > 3.70;
        fprintf('\n=== b_true oracle, %s (%d seed%s) ===\n', F.name, ns, char('s'*(ns>1)));
        fprintf('  b_hat used: %.4f .. %.4f    end-hold a_hat/a_true - 1 = %+.2f %%\n', min(F.bh(:)), max(F.bh(:)), ...
                100*(mean(mean(F.ah(kk(ho),:),1)./mean(F.at(kk(ho),:),1)) - 1));
        SEG = {'descend', F.tt > 0.5 & F.tt < 1.5; 'cycle 1', F.tt >= 1.5 & F.tt < 2.5; 'cycle 2', F.tt >= 2.5 & F.tt < 3.5; ...
               'hold end', F.tt >= 3.5; 'osc 1.6-3.4', F.tt > 1.60 & F.tt < 3.40};
        fprintf('  %-12s %9s %9s | %9s %9s %9s %9s | %9s   (a_o, mean over seeds)\n', 'segment', 'd a_hat', 'd a_true', 'law', 'y1', 'y2', 'resid', 'ratchet');
        for g = 1:size(SEG,1)
            m = SEG{g,2};  S = @(v) mean(sum(v(m,:),1));
            fprintf('  %-12s %+9.4f %+9.4f | %+9.4f %+9.4f %+9.4f %+9.4f | %+9.4f\n', SEG{g,1}, ...
                    S(F.dah), S(F.dat), S(F.law), S(F.y1), S(F.y2), S(F.res), S(F.rat));
        end
    end
    save(fullfile(od, 'check_btrue_log_ledger.mat'), 'out', 'keys');
    local_fig(out, keys, fullfile(od, 'check_btrue_log_ledger.png'));
    fprintf('\nfigure -> %s\n', od);
end

function local_fig(out, keys, fpath)
    FS = 16; AXLW = 2.0; GRY = [0.5 0.5 0.5];
    COL = [0 0.2 0.9; 0.85 0.1 0.1; 0 0.2 0.9; 0.85 0.1 0.1];  LS = {'-', '-', '--', '--'};
    f = figure('Position',[40 40 1300 1150],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    % (1) a_hat - a_true, four arms
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:4
        F = out.(keys{a});
        plot(a1,F.tt,mean(F.ah(2:end,:)-F.at(2:end,:),2),LS{a},'Color',COL(a,:),'LineWidth',2.2,'DisplayName',F.name);
    end
    ylabel(a1,'a_{hat} - a_{true}  [a_o]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    % (2) cumulative legs, noisy Euler (mean over seeds)
    F = out.(keys{1});
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    plot(a2,F.tt,cumsum(mean(F.law - F.dat,2)),'-','Color',[0.1 0.6 0.2],'LineWidth',2.2,'DisplayName','law leg - true change');
    plot(a2,F.tt,cumsum(mean(F.y1,2)),'-','Color',[0.9 0.5 0],'LineWidth',2.2,'DisplayName','y_1 leg');
    plot(a2,F.tt,cumsum(mean(F.y2,2)),'-','Color',[0.4 0.7 1.0],'LineWidth',2.2,'DisplayName','y_2 leg');
    plot(a2,F.tt,cumsum(mean(F.rat,2)),':','Color',[0 0 0],'LineWidth',2.4,'DisplayName','ratchet -1/2\Sigma a'''' dw^2');
    plot(a2,F.tt,mean(F.ah(2:end,:)-F.at(2:end,:),2)-mean(F.ah(2,:)-F.at(2,:)),'-','Color',COL(1,:),'LineWidth',1.6,'DisplayName','a_{hat}-a_{true} (sum)');
    ylabel(a2,'noisy Euler, cumulative [a_o]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',11);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    % (3) cumulative legs, det Euler
    F = out.(keys{3});
    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    plot(a3,F.tt,cumsum(F.law - F.dat),'-','Color',[0.1 0.6 0.2],'LineWidth',2.2,'DisplayName','law leg - true change');
    plot(a3,F.tt,cumsum(F.y1),'-','Color',[0.9 0.5 0],'LineWidth',2.2,'DisplayName','y_1 leg');
    plot(a3,F.tt,cumsum(F.rat),':','Color',[0 0 0],'LineWidth',2.4,'DisplayName','ratchet');
    plot(a3,F.tt,(F.ah(2:end)-F.at(2:end))-(F.ah(2)-F.at(2)),'-','Color',COL(3,:),'LineWidth',1.6,'DisplayName','a_{hat}-a_{true} (sum)');
    xlabel(a3,'t [s]'); ylabel(a3,'det Euler, cumulative [a_o]');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',11);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
