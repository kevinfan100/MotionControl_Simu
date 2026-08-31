function out = compare_mean_corr_hold(seeds)
%COMPARE_MEAN_CORR_HOLD  Acceptance for the hold-gated noise-mean correction
%   (ctrl_const.mean_corr_hold; derivation: T2 closed form, T12 cleaned ledger).
%
%   out = compare_mean_corr_hold(1:8);
%
% Arms (canonical deep, PRODUCTION stack: b estimated, exact law step ON):
%   base   best + law_exact_step
%   corr   base + mean_corr_hold
%
% Registered predictions (written before the run):
%   P-M0  flag off reproduces the fixture bit for bit (seed 7, exact-step base).
%   P-M1  end-hold (trough) bias: base ~+14.4 % -> corr 7 +- 2.5 %
%         (the correction cancels the hold-segment row-3 mean forcing, whose
%         adjoint share is -0.0061 of the -0.0121 total e4).
%   P-M2  seed spread unchanged within 20 % (the correction is a mean, not a
%         variance device; a spread change flags overcorrection).
%   P-M3  oscillation-segment bias change < 1.5 pp (correction is hold-gated).
%   P-M4  first-hold bias change < 0.5 pp (far field: a' tiny, correction ~0).
% STATUS: ACTIVE | closed-loop mean-bias line, first correction increment

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'mean_corr_hold');
    if ~exist(od, 'dir'); mkdir(od); end
    ax = 3;  ns = numel(seeds);

    c7 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, ...
                            'ctrl_const_override', struct('law_exact_step', true, 'mean_corr_hold', false)));
    v7 = c7.runs{1}.a_bar_hat_out(end, ax);
    c7b = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, ...
                             'ctrl_const_override', struct('law_exact_step', true)));
    d0 = max(abs(c7.runs{1}.a_bar_hat_out(:) - c7b.runs{1}.a_bar_hat_out(:)));
    fprintf('[P-M0] flag-off vs absent: max diff %.1e  %s\n', d0, local_pf(d0 == 0));

    ARMS = {'base (exact)', struct('law_exact_step', true); ...
            'base + mean corr', struct('law_exact_step', true, 'mean_corr_hold', true)};
    R = struct();  keys = cell(2,1);
    for a = 1:2
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                               'ctrl_const_override', ARMS{a,2}));
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        a_nom = O.runs{1}.a_nom;
        F = struct('name', ARMS{a,1}, 't', O.runs{1}.tout(:), 'ah', G('a_bar_hat_out'), 'at', G('a_true_out')/a_nom, ...
                   'sP44', G('P_a_out')/a_nom);
        keys{a} = matlab.lang.makeValidName(ARMS{a,1});  R.(keys{a}) = F;
    end
    save(fullfile(od, 'compare_mean_corr_hold.mat'), 'R', 'keys');

    kk = 2:numel(R.(keys{1}).t);  tt = R.(keys{1}).t(kk);
    SEG = {'hold start', tt > 0.05 & tt < 0.50; 'descend', tt > 0.55 & tt < 1.45; ...
           'oscillate', tt > 1.60 & tt < 3.40; 'hold end', tt > 3.70};
    fprintf('\n%-18s %-11s %9s %7s %8s %9s\n', 'arm', 'segment', 'bias %', 'SEM', 'sd %', 'sqrtP44');
    EB = struct();  OS = struct();  H1 = struct();
    for a = 1:2
        F = R.(keys{a});
        for g = 1:4
            m = SEG{g,2};
            e = F.ah(kk,:) ./ F.at(kk,:) - 1;   eb = mean(e(m,:), 1);
            fprintf('%-18s %-11s %+9.2f %7.2f %8.2f %9.4f\n', F.name, SEG{g,1}, ...
                    100*mean(eb), 100*std(eb)/sqrt(ns), 100*std(eb), mean(F.sP44(kk(m),:),'all'));
            if g == 4; EB.(keys{a}) = eb; end
            if g == 3; OS.(keys{a}) = eb; end
            if g == 1; H1.(keys{a}) = eb; end
        end
    end
    d = EB.(keys{2}) - EB.(keys{1});
    fprintf('\npaired end-hold diff (corr - base): %+.2f +- %.2f %%  (t = %+.1f)\n', ...
            100*mean(d), 100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));
    b1 = 100*mean(EB.(keys{2}));
    sdr = std(EB.(keys{2})) / max(std(EB.(keys{1})), eps);
    dos = 100*abs(mean(OS.(keys{2})) - mean(OS.(keys{1})));
    dh1 = 100*abs(mean(H1.(keys{2})) - mean(H1.(keys{1})));
    fprintf('[P-M1] corr trough bias %+.2f %%  %s   [P-M2] spread ratio %.2f  %s   [P-M3] osc change %.2f pp  %s   [P-M4] hold1 change %.2f pp  %s\n', ...
        b1, local_pf(b1 >= 4.5 && b1 <= 9.5), sdr, local_pf(abs(sdr - 1) < 0.2), ...
        dos, local_pf(dos < 1.5), dh1, local_pf(dh1 < 0.5));
    out = struct('R', R, 'keys', {keys});
    local_fig(R, keys, fullfile(od, 'compare_mean_corr_hold.png'));
    fprintf('figure -> %s\n', od);
end

function s = local_pf(ok); if ok; s = 'PASS'; else; s = 'FAIL'; end; end

function local_fig(R, keys, fpath)
    FS = 16; AXLW = 2.0; GRY = [0.5 0.5 0.5];  BLU = [0 0.2 0.9];  RED = [0.85 0.1 0.1];
    f = figure('Position',[40 40 1200 700],'Color','w','Visible','off');
    a1 = axes(f); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    for a = 1:2
        F = R.(keys{a});  c = BLU; if a == 2; c = RED; end
        plot(a1,F.t(2:end),movmean(100*(mean(F.ah(2:end,:),2)./mean(F.at(2:end,:),2)-1),51),'-','Color',c,'LineWidth',2.2,'DisplayName',F.name);
    end
    xlabel(a1,'t [s]'); ylabel(a1,'a_{hat}/a_{true} - 1  [%]');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',13);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
