function out = compare_exact_step_btrue_fig(seeds)
%COMPARE_EXACT_STEP_BTRUE_FIG  Euler vs exact law step, both under the b_true
%   oracle with b evaluated at the particle's TRUE height (b_true_at = 'true').
%   The law is as right as it can possibly be in both arms; the ONLY difference
%   is how the same ODE is integrated:
%       Euler :  a[k+1] = a[k] + b (1-a[k])^2 dw
%       exact :  1/(1-a[k+1]) = 1/(1-a[k]) + b dw
%   Arms: noisy canonical (8 seeds) and deterministic (thermal off, meas noise
%   off, y2 off, seed 7), each Euler vs exact.
% STATUS: ACTIVE | requested comparison figure (2026-08-31)

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'law_exact_step');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  ns = numel(seeds);
    CO_det = struct('thermal_enable', false, 'meas_noise_enable', false);
    ARMS = {'noisy Euler', seeds, true,  struct(),                       struct(); ...
            'noisy exact', seeds, true,  struct('law_exact_step', true), struct(); ...
            'det Euler',   7,     false, struct(),                       CO_det; ...
            'det exact',   7,     false, struct('law_exact_step', true), CO_det};
    out = struct();  keys = cell(4,1);
    for a = 1:4
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', ARMS{a,2}, 'verbose', false, ...
                               'b_true', true, 'b_true_at', 'true', 'y2_on', ARMS{a,3}, ...
                               'ctrl_const_override', ARMS{a,4}, 'config_override', ARMS{a,5}));
        a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        F = struct('name', ARMS{a,1}, 't', t, 'ah', G('a_bar_hat_out'), 'at', G('a_true_out')/a_nom, 'bh', G('b_hat_out'));
        keys{a} = matlab.lang.makeValidName(ARMS{a,1});  out.(keys{a}) = F;
        kk = 2:numel(t);  tt = t(kk);  ho = tt > 3.7;  m = tt >= 1.5 & tt < 3.5;
        eb = mean(F.ah(kk(ho),:),1) ./ mean(F.at(kk(ho),:),1) - 1;
        fprintf('%-12s  b_hat used %.4f..%.4f | trough bias %+6.2f %% (SEM %.2f) | osc excess %+8.4f a_o\n', ...
                F.name, min(F.bh(:)), max(F.bh(:)), 100*mean(eb), 100*std(eb)/sqrt(numel(eb)), ...
                mean(sum(F.ah(kk(m),:)-F.ah(kk(m)-1,:),1) - sum(F.at(kk(m),:)-F.at(kk(m)-1,:),1)));
        if a <= 2; EB{a} = eb; end %#ok<AGROW>
    end
    d = EB{2} - EB{1};
    fprintf('paired trough diff (exact - Euler, noisy): %+.2f +- %.2f %%  (t = %+.1f)\n', ...
            100*mean(d), 100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));
    save(fullfile(od, 'compare_exact_step_btrue.mat'), 'out', 'keys');

    FS = 16; AXLW = 2.0; GRY = [0.5 0.5 0.5];  BLU = [0 0.2 0.9];  RED = [0.85 0.1 0.1];
    f = figure('Position',[40 40 1300 900],'Color','w','Visible','off');
    tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:2
        F = out.(keys{a});  c = BLU; if a == 2; c = RED; end
        plot(a1,F.t(2:end),movmean(100*(mean(F.ah(2:end,:),2)./mean(F.at(2:end,:),2)-1),51),'-','Color',c,'LineWidth',2.2,'DisplayName',F.name);
    end
    ylabel(a1,'a_{hat}/a_{true} - 1  [%]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);
    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 3:4
        F = out.(keys{a});  c = BLU; if a == 4; c = RED; end
        plot(a2,F.t(2:end),F.ah(2:end)-F.at(2:end),'-','Color',c,'LineWidth',2.2,'DisplayName',F.name);
    end
    xlabel(a2,'t [s]'); ylabel(a2,'a_{hat} - a_{true}  [a_o]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    fp = fullfile(od, 'compare_exact_step_btrue.png');
    exportgraphics(f,fp,'Resolution',150); close(f);
    fprintf('figure -> %s\nBTRUE FIG DONE\n', fp);
end
