% STATUS: ACTIVE (scratch) | PURPOSE: how much the a_hat error differs from
%   seed to seed as the trajectory gets slower -- canonical 1 Hz, canonical
%   with the oscillation slowed to 0.2 Hz, and the Meng 10 s ramp -- and
%   whether the sign of the y1 feedback gain l_41 goes with it.
%   Production arm (a_pd 0.05, a_cov 0.05), z axis, house 6 seeds by default.
%
%   out = run_traj_seed_spread();            % runs + figure + console
%   out = run_traj_seed_spread([], true);    % replot from the saved .mat
%
%   Figure (one column per trajectory, rows share y across columns):
%     row 1  a_hat - a_true, every seed thin, seed mean thick
%     row 2  sd_seeds(a_hat)  vs  mean sqrt(P44)        (honesty)
%     row 3  l_41 (y1 gain on a_hat), seed mean, zero line   (sign = amplify / correct)
function out = run_traj_seed_spread(seeds, replot)

    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777]; end
    if nargin < 2 || isempty(replot); replot = false; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); fn = fullfile(od, 'traj_seed_spread.mat');
    ax = 3; pc = physical_constants();

    SC = { '1 Hz  (canonical deep)',   struct();
           '0.2 Hz (canonical deep)',  struct('frequency', 0.2, 'n_cycles', 2, 'T_sim', 0.5 + 1.0 + 10.0 + 1.3);
           'Meng 10 s ramp',           struct('trajectory_type','osc','h_init',15.0,'h_bottom',2.5,'amplitude',0, ...
                                              'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10.0, ...
                                              'T_sim',12.5,'h_min',1.1*pc.R) };
    if replot
        S = load(fn); O = S.O; seeds = S.seeds;
    else
        O = cell(1, size(SC,1));
        for c = 1:size(SC,1)
            clear run_formC_b motion_control_law_formC_b;
            evalc("O{c} = run_formC_b(struct('arm','best','seeds',seeds,'config_override',SC{c,2}));");
            fprintf('ran %-26s T = %.1f s\n', SC{c,1}, O{c}.runs{1}.tout(end));
        end
        save(fn, 'O', 'seeds', 'SC', '-v7.3');
    end

    nS = numel(seeds); D = cell(1, 3);
    for c = 1:3
        t = O{c}.runs{1}.tout(:); N = numel(t);
        E = zeros(N, nS); AH = E; P44 = E; L1 = E; AT = E;
        for q = 1:nS
            r = O{c}.runs{q}; ad = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
            AT(:,q) = r.a_true_out(:,ax)/ad; AH(:,q) = r.a_bar_hat_out(:,ax);
            E(:,q) = AH(:,q) - AT(:,q); P44(:,q) = r.P_a_out(:,ax); L1(:,q) = r.K_a_y1_out(:,ax);
        end
        D{c} = struct('t', t, 'E', E, 'at', mean(AT,2), 'sd', std(AH,0,2), 'sP', sqrt(mean(P44,2)), 'l41', mean(L1,2), 'name', SC{c,1});
    end

    % ---------------- console ------------------------------------------------
    fprintf('\n%d seeds, production arm, z.  Windows: descent = t_hold..t_hold+t_desc, motion = the oscillation/ramp, hold = last 1 s\n', nS);
    fprintf('%-26s %-8s %10s %10s %9s %10s %8s\n', 'trajectory', 'window', 'mean err', 'sd seeds', 'honesty', 'mean l41', 'frac<0');
    for c = 1:3
        d = D{c}; t = d.t; T = t(end);
        switch c
            case 1; W = [0.5 1.5; 1.5 3.5; T-1 T];
            case 2; W = [0.5 1.5; 1.5 11.5; T-1 T];
            case 3; W = [0.5 5.5; 5.5 10.5; T-1 T];
        end
        WN = {'descent', 'motion', 'hold'};
        for w = 1:3
            m = t >= W(w,1) & t <= W(w,2);
            fprintf('%-26s %-8s %+10.4f %10.4f %9.2f %+10.4f %8.2f\n', d.name, WN{w}, mean(mean(d.E(m,:),2)), mean(d.sd(m)), ...
                    mean(d.sd(m)) / mean(d.sP(m)), mean(d.l41(m)), mean(d.l41(m) < 0));
        end
    end

    % ---------------- figure -------------------------------------------------
    C_M = [0.55 0.78 1.0]; C_E = [0 0.2 0.9]; C_T = [0.8 0 0]; FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position',[10 10 2200 1400],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,3,3,'TileSpacing','compact','Padding','compact'); A = gobjects(3,3);
    yl1 = 0; yl2 = 0; yl3 = [0 0];
    for c = 1:3; d = D{c}; yl1 = max(yl1, max(abs(d.E(:)))); yl2 = max(yl2, max([d.sd; d.sP])); yl3 = [min(yl3(1), min(d.l41)), max(yl3(2), max(d.l41))]; end
    for c = 1:3
        d = D{c};
        a = nexttile(tl, c); A(1,c) = a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
        h1 = plot(a, d.t, d.E, '-', 'Color', C_M, 'LineWidth', 0.7);
        h2 = plot(a, d.t, mean(d.E,2), '-', 'Color', C_E, 'LineWidth', 2.4);
        legend(a,[h1(1) h2],{sprintf('\\^a_h - a   each seed (%d)', nS), 'seed mean'},'Location','northoutside', ...
               'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        title(a, d.name, 'FontSize', LFS+1, 'FontWeight', 'bold');
        if c == 1; ylabel(a,'\^a_h - a','FontSize',FS,'FontWeight','bold'); end
        ylim(a, 1.05*[-yl1 yl1]);
        a = nexttile(tl, 3+c); A(2,c) = a; hold(a,'on');
        h1 = plot(a, d.t, d.sd, '-', 'Color', C_E, 'LineWidth', 2.2);
        h2 = plot(a, d.t, d.sP, '--', 'Color', C_T, 'LineWidth', 2.2);
        legend(a,[h1 h2],{'sd_{seeds}(\^a_h)', 'mean \surdP_{44}'},'Location','northoutside', ...
               'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a,'spread','FontSize',FS,'FontWeight','bold'); end
        ylim(a, [0 1.05*yl2]);
        a = nexttile(tl, 6+c); A(3,c) = a; hold(a,'on');
        yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
        h1 = plot(a, d.t, d.l41, '-', 'Color', C_E, 'LineWidth', 2.2);
        legend(a,h1,{'{\itl}_{41}   seed mean'},'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a,'l_{41}','FontSize',FS,'FontWeight','bold'); end
        ylim(a, [1.05*yl3(1) 1.05*yl3(2)]);
        xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
    end
    for k = 1:9
        set(A(k),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(k),'off');
        [row, col] = ind2sub([3 3], k); xlim(A(k), [0 D{col}.t(end)]);
        if row < 3; set(A(k),'XTickLabel',[]); end
        if col > 1; set(A(k),'YTickLabel',[]); end
    end
    fig = fullfile(od, 'traj_seed_spread.png'); exportgraphics(f, fig, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fig);
    out = struct('D', {D}, 'seeds', seeds, 'file', fig);
end
