function out = check_true_ctrl_gain_nu()
%CHECK_TRUE_CTRL_GAIN_NU  Where does the phase-locked mean innovation come from?
%   Discriminator: feed the EXACT gain to the control law (a_ctrl_override =
%   'true') while the estimator runs unchanged. The force is then right, so any
%   tracking lag caused by a gain ERROR is gone.
%     mean(nu) vanishes  -> the y1 push is the amplifier acting on a seed
%                           (gain error -> wrong force -> lag -> nu -> a_hat up)
%     mean(nu) remains   -> the position model itself is biased (lambda_eff,
%                           command lag, MA/echo), independent of the gain
%   All arms: b_true oracle + exact law step.  det = thermal off, meas noise
%   off, y2 off, seed 7.  noisy = canonical, 8 seeds.
% STATUS: ACTIVE | 2026-08-30

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'btrue_log_ledger');
    ax = 3;  R = physical_constants().R;  lam = 0.7;
    CO_det = struct('thermal_enable', false, 'meas_noise_enable', false);
    ARMS = {'det, ctrl a_hat',   7,   false, [],     CO_det; ...
            'det, ctrl a_true',  7,   false, 'true', CO_det; ...
            'noisy, ctrl a_hat',  1:8, true,  [],     struct(); ...
            'noisy, ctrl a_true', 1:8, true,  'true', struct()};
    out = struct();  keys = cell(4,1);
    for a = 1:4
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', ARMS{a,2}, 'verbose', false, ...
                               'b_true', true, 'b_true_at', 'cmd', 'y2_on', ARMS{a,3}, 'a_ctrl_override', ARMS{a,4}, ...
                               'config_override', ARMS{a,5}, 'ctrl_const_override', struct('law_exact_step', true)));
        ns = numel(O.runs);  a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        F = struct('name', ARMS{a,1}, 'ns', ns, 't', t, 'ah', G('a_bar_hat_out'), 'at', G('a_true_out')/a_nom, ...
                   'bh', G('b_hat_out'), 'dx3', G('delta_x_hat_3_out')/R, 'hd', O.runs{1}.h_bar_d_out(:), ...
                   'K1', G('K_a_y1_out'), 'n1', G('innov_y1_out'), 'K2', G('K_a_y2_out'), 'n2', G('innov_y2_out'));
        kk = 2:numel(t);  F.tt = t(kk);  dwd = [0; diff(F.hd)];
        F.dw  = dwd(kk) + (1 - lam) * F.dx3(kk-1, :);
        F.law = F.bh(kk-1,:) .* (1 - F.ah(kk-1,:)).^2 .* F.dw;
        F.y1  = F.K1(kk,:) .* F.n1(kk,:);   F.y2 = F.K2(kk,:) .* F.n2(kk,:);
        F.dah = F.ah(kk,:) - F.ah(kk-1,:);  F.dat = F.at(kk,:) - F.at(kk-1,:);
        F.res = F.dah - (F.law + F.y1 + F.y2);
        keys{a} = matlab.lang.makeValidName(ARMS{a,1});  out.(keys{a}) = F;
        m = F.tt >= 1.5 & F.tt < 3.5;  ho = F.tt > 3.7;  de = F.tt > 0.5 & F.tt < 1.5;
        S = @(v, mm) mean(sum(v(mm,:),1));
        fprintf('\n=== %s (%d seed%s) ===\n', F.name, ns, char('s'*(ns>1)));
        fprintf('  end-hold bias %+.2f %%   descend excess %+.4f   oscillation excess %+.4f  (a_o)\n', ...
                100*(mean(mean(F.ah(kk(ho),:),1)./mean(F.at(kk(ho),:),1)) - 1), S(F.dah,de)-S(F.dat,de), S(F.dah,m)-S(F.dat,m));
        fprintf('  oscillation legs: law %+.4f  y1 %+.4f  y2 %+.4f  resid %+.4f   | mean nu %+.2e  mean |nu| %.2e\n', ...
                S(F.law,m), S(F.y1,m), S(F.y2,m), S(F.res,m), mean(F.n1(kk(m),:),'all'), mean(abs(F.n1(kk(m),:)),'all'));
        ph = mod(F.tt(m) - 1.5, 1);  edges = 0:0.25:1;  s = '';
        for b = 1:4
            sel = ph >= edges(b) & ph < edges(b+1);  idx = find(m);  idx = idx(sel);
            s = [s sprintf('  ph %.2f-%.2f: nu %+.2e K1 %+.3f y1 %+.4f', edges(b), edges(b+1), ...
                 mean(F.n1(kk(idx),:),'all'), mean(F.K1(kk(idx),:),'all'), mean(sum(F.y1(idx,:),1)))]; %#ok<AGROW>
        end
        fprintf('%s\n', s);
    end
    save(fullfile(od, 'check_true_ctrl_gain_nu.mat'), 'out', 'keys');
    local_fig(out, keys, fullfile(od, 'check_true_ctrl_gain_nu.png'));
    fprintf('\nfigure -> %s\nTRUE CTRL DONE\n', od);
end

function local_fig(out, keys, fpath)
    FS = 16; AXLW = 2.0; GRY = [0.5 0.5 0.5];
    COL = [0 0.2 0.9; 0.85 0.1 0.1; 0 0.2 0.9; 0.85 0.1 0.1];  LS = {'-', '-', '--', '--'};
    f = figure('Position',[40 40 1300 1150],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:4
        F = out.(keys{a});
        plot(a1,F.tt,mean(F.ah(2:end,:)-F.at(2:end,:),2),LS{a},'Color',COL(a,:),'LineWidth',2.2,'DisplayName',F.name);
    end
    ylabel(a1,'a_{hat} - a_{true}  [a_o]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:2
        F = out.(keys{a});
        plot(a2,F.tt,F.n1(2:end),LS{a},'Color',COL(a,:),'LineWidth',1.8,'DisplayName',['\nu_1  ' F.name]);
    end
    ylabel(a2,'innovation y_1 (det)');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 3:4
        F = out.(keys{a});
        plot(a3,F.tt,movmean(mean(F.n1(2:end,:),2),81),LS{a},'Color',COL(a,:),'LineWidth',2.0,'DisplayName',['mean \nu_1 (8 seeds, 50 ms avg)  ' F.name]);
    end
    xlabel(a3,'t [s]'); ylabel(a3,'innovation y_1 (noisy)');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
