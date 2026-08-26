function out = sweep_q44_scale(scenario, seeds, scales)
%SWEEP_Q44_SCALE  Does admitting more gain uncertainty (P44) move the TARGET
%   metrics -- trough bias, honesty, per-seed spread -- and at what cost?
%
%   out = sweep_q44_scale('meng', 1:8, [1 3 10]);
%   out = sweep_q44_scale('canonical', 1:8, [1 3 10]);
%
% STATUS: ACTIVE | lever test for the P(4,1) sign-competition mechanism
%   (run_inject_response + decompose_inject_y1, 2026-08-26). Uses the
%   diagnostic knob ctrl_const.q44_scale (multiplies Q(4,4) only). This is
%   NOT a fix: the right Q44 budget must come from a derivation. The sweep
%   only asks whether the lever exists on the target metrics before that
%   derivation is spent.
%
% READ TOGETHER: bias (accuracy), spread/sqrtP (honesty), and per-seed sd.
% A bias reduction bought with honesty << 1 is over-inflation, not a gain.

    if nargin < 1 || isempty(scenario); scenario = 'meng'; end
    if nargin < 2 || isempty(seeds);    seeds = 1:8;        end
    if nargin < 3 || isempty(scales);   scales = [1 3 10];  end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', ['q44_sweep_' scenario]);
    if ~exist(od, 'dir'); mkdir(od); end

    base = struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false);
    switch lower(scenario)
        case 'meng'
            pc = physical_constants();
            base.config_override = struct('trajectory_type', 'osc', 'h_init', 15.0, ...
                'h_bottom', 2.5, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, ...
                't_hold', 0.5, 't_descend_override', 10.0, 'T_sim', 12.5, 'h_min', 1.1*pc.R);
            SEG = {'ramp', @(t) t > 0.6 & t < 10.4; 'hold end', @(t) t > 11.6};
        case 'canonical'
            SEG = {'descend', @(t) t > 0.55 & t < 1.45; 'oscillate', @(t) t > 1.60 & t < 3.40; ...
                   'hold end', @(t) t > 3.70};
        otherwise
            error('scenario must be meng|canonical');
    end

    ax = 3;  ns = numel(seeds);  nsc = numel(scales);
    R = cell(nsc, 1);
    for q = 1:nsc
        o = base;
        if scales(q) ~= 1; o.ctrl_const_override = struct('q44_scale', scales(q)); end
        fprintf('[sweep] %s  q44_scale = %g\n', scenario, scales(q));
        O = run_formC_b(o);
        r1 = O.runs{1};  N = numel(r1.tout);  a_nom = r1.a_nom;
        F = struct('t', r1.tout(:), 'ah', zeros(N,ns), 'at', zeros(N,ns), 'sP', zeros(N,ns), 'K1', zeros(N,ns));
        for s = 1:ns
            r = O.runs{s};
            F.ah(:,s) = r.a_bar_hat_out(:,ax);
            F.at(:,s) = r.a_true_out(:,ax) / a_nom;
            F.sP(:,s) = r.P_a_out(:,ax) / a_nom;        % driver stores sqrt(P)
            F.K1(:,s) = r.K_a_y1_out(:,ax);
        end
        R{q} = F;
    end
    save(fullfile(od, 'q44_sweep.mat'), 'R', 'scales', 'seeds', 'scenario', 'SEG');

    % ---- table -----------------------------------------------------------
    t = R{1}.t(2:end);
    fprintf('\n%-10s %6s %12s %10s %10s %10s\n', 'segment', 'scale', 'bias %', 'SEM %', 'seed sd %', 'spread/sqrtP');
    fprintf('%-10s %6s %12s %10s %10s %10s %10s\n', '', '', '', '', '', '', 'K1(4)');
    T = struct();
    for g = 1:size(SEG,1)
        m = SEG{g,2}(t);
        for q = 1:nsc
            F = R{q};  e = F.ah(2:end,:) ./ F.at(2:end,:) - 1;
            eb = mean(e(m,:), 1);                                 % per seed
            hon = mean(std(F.ah(2:end,:) - F.at(2:end,:), 0, 2) ./ mean(F.sP(2:end,:), 2));
            hs = std(F.ah(2:end,:) - F.at(2:end,:), 0, 2) ./ mean(F.sP(2:end,:), 2);
            fprintf('%-10s %6g %+12.2f %10.2f %10.2f %10.2f %+10.4f\n', SEG{g,1}, scales(q), ...
                    100*mean(eb), 100*std(eb)/sqrt(ns), 100*std(eb), mean(hs(m)), mean(mean(F.K1(find(m)+1,:))));
            T.(sprintf('s%d_%s', q, strrep(SEG{g,1},' ','_'))) = [mean(eb) std(eb)/sqrt(ns) std(eb) mean(hs(m))];
        end
    end
    % paired differences vs scale 1, end hold
    m = SEG{end,2}(t);
    e1 = mean((R{1}.ah(2:end,:) ./ R{1}.at(2:end,:) - 1) .* m, 1) / mean(m);
    fprintf('\npaired end-hold bias vs scale 1:\n');
    for q = 2:nsc
        eq_ = mean((R{q}.ah(2:end,:) ./ R{q}.at(2:end,:) - 1) .* m, 1) / mean(m);
        d = eq_ - e1;
        fprintf('  scale %3g : %+7.2f +- %.2f %%   (t = %+.1f)\n', scales(q), 100*mean(d), ...
                100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));
    end

    out = struct('R', {R}, 'scales', scales, 'T', T);
    local_fig(R, scales, scenario, fullfile(od, 'q44_sweep.png'));
    fprintf('\nfigure -> %s\n', od);
end

% =======================================================================
function local_fig(R, scales, scenario, fpath)
    FS = 17; AXLW = 2.0;  GRY = [0.5 0.5 0.5];
    COL = [0 0.2 0.9; 0.9 0.5 0; 0.1 0.6 0.2; 0.55 0.3 0.75];
    f = figure('Position',[40 40 1300 1000],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    t = R{1}.t(2:end);

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for q = 1:numel(R)
        e = 100*(mean(R{q}.ah(2:end,:),2) ./ mean(R{q}.at(2:end,:),2) - 1);
        plot(a1,t,movmean(e,51),'-','Color',COL(q,:),'LineWidth',2.2,'DisplayName',sprintf('Q_{44} x %g', scales(q)));
    end
    ylabel(a1,'a_{hat}/a_{true} - 1  [%]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,1,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for q = 1:numel(R)
        h = std(R{q}.ah(2:end,:) - R{q}.at(2:end,:), 0, 2) ./ mean(R{q}.sP(2:end,:), 2);
        plot(a2,t,movmean(h,51),'-','Color',COL(q,:),'LineWidth',2.2,'DisplayName',sprintf('spread/\\surdP_{44}  x %g', scales(q)));
    end
    set(a2,'YScale','log'); ylim(a2,[0.1 10]);
    ylabel(a2,'honesty');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for q = 1:numel(R)
        plot(a3,t,movmean(mean(R{q}.K1(2:end,:),2),51),'-','Color',COL(q,:),'LineWidth',2.2,'DisplayName',sprintf('K_1(4)  x %g', scales(q)));
    end
    xlabel(a3,'t [s]'); ylabel(a3,'K_1(4)  (sign)');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
