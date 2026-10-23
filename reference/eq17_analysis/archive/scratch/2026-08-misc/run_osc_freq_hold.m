% STATUS: ACTIVE (scratch) | PURPOSE: oscillation frequency x final hold, on the
%   canonical deep band (w_bar 22.2 -> [1.10, 3.32]), TRUE-b arm evaluated at
%   the particle's true height (opts.b_true, b_true_at 'true') -- the same arm
%   as arm_btrue_trueheight_10s_5s_gains.png, so the two pages read side by side.
%     col 1  1 Hz   x 2 cycles, run ends at the last trough (no hold)
%     col 2  1 Hz   x 2 cycles + 1.3 s hold at the trough      (= canonical)
%     col 3  0.2 Hz x 2 cycles, run ends at the last trough
%     col 4  0.2 Hz x 2 cycles + 1.3 s hold at the trough
%   Descent is the canonical 1 s cosine in all four. Rows as the reference page:
%   a_hat - a (each seed + mean) | l31 | l32 | l41 | l42 | trajectory h/R.
%
%   out = run_osc_freq_hold();                 % run + figure + console (b_true_at 'true')
%   out = run_osc_freq_hold([], true);         % replot from the saved .mat
%   out = run_osc_freq_hold([], false, 'cmd'); % b_true evaluated at the COMMANDED height (no jitter)
function out = run_osc_freq_hold(seeds, replot, bt_at)
    if nargin < 1 || isempty(seeds);  seeds  = [7 11 23 42 101 777]; end
    if nargin < 2 || isempty(replot); replot = false; end
    if nargin < 3 || isempty(bt_at);  bt_at  = 'true'; end
    assert(any(strcmp(bt_at, {'true','cmd'})), 'bt_at must be ''true'' or ''cmd''.');
    tag = ''; if strcmp(bt_at, 'cmd'); tag = '_cmd'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    fn = fullfile(od, ['osc_freq_hold' tag '.mat']); fig = fullfile(od, ['osc_freq_hold' tag '_gains.png']);
    ax = 3; T_HOLD = 1.3; T0 = 0.5; TD = 1.0; NC = 2;

    F  = [1 1 0.2 0.2]; HOLD = [false true false true]; nC = 4;
    NAMES = cell(1,nC); OV = cell(1,nC); T3 = zeros(1,nC);
    for c = 1:nC
        T3(c) = T0 + TD + NC/F(c);
        OV{c} = struct('frequency', F(c), 'n_cycles', NC, 't_hold', T0, 't_descend_override', TD, ...
                       'T_sim', T3(c) + HOLD(c)*T_HOLD);
        if HOLD(c); hs = '+ hold'; else; hs = '(no hold)'; end
        if strcmp(bt_at,'cmd'); hn = 'cmd h'; else; hn = 'true h'; end
        NAMES{c} = sprintf('b_{true}@%s,  %g Hz x %d  %s', hn, F(c), NC, hs);
    end

    if replot
        S = load(fn); O = S.O; seeds = S.seeds;
    else
        O = cell(1,nC);
        for c = 1:nC
            clear run_formC_b motion_control_law_formC_b;
            evalc("O{c} = run_formC_b(struct('arm','best','seeds',seeds,'b_true',true,'b_true_at',bt_at,'config_override',OV{c}));");
            fprintf('ran col %d  %-38s T = %.2f s  lock_b = %d\n', c, NAMES{c}, O{c}.runs{1}.tout(end), O{c}.runs{1}.ctrl_const.lock_b);
        end
        save(fn, 'O', 'seeds', 'OV', 'NAMES', 'F', 'HOLD', 'T3', 'bt_at', '-v7.3');
    end

    % ---------------- extract ---------------------------------------------
    nS = numel(seeds); D = cell(1,nC);
    for c = 1:nC
        t = O{c}.runs{1}.tout(:); N = numel(t);
        E = zeros(N,nS); AH = E; P44 = E; L31 = E; L32 = E; L41 = E; L42 = E; AT = E;
        for q = 1:nS
            r = O{c}.runs{q}; ad = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
            AT(:,q) = r.a_true_out(:,ax)/ad; AH(:,q) = r.a_bar_hat_out(:,ax); E(:,q) = AH(:,q) - AT(:,q);
            P44(:,q) = r.P_a_out(:,ax);
            L31(:,q) = r.K_dx_y1_out(:,ax); L32(:,q) = r.K_dx_y2_out(:,ax);
            L41(:,q) = r.K_a_y1_out(:,ax);  L42(:,q) = r.K_a_y2_out(:,ax);
        end
        r1 = O{c}.runs{1};
        D{c} = struct('t',t,'E',E,'sd',std(AH,0,2),'sP',sqrt(mean(P44,2)), ...
                      'l31',mean(L31,2),'l32',mean(L32,2),'l41',mean(L41,2),'l42',mean(L42,2), ...
                      'L31',L31,'L32',L32,'L41',L41,'L42',L42, ...
                      'hd',r1.p_d_out(:,ax)/r1.R,'ht',r1.h_bar_true_out(:,1),'name',NAMES{c});
    end

    % ---------------- console ---------------------------------------------
    fprintf('\n%d seeds, z, TRUE-b arm @ %s height.  windows: descent %.1f-%.1f | motion | hold (last %.1f s, if any)\n', nS, bt_at, T0, T0+TD, T_HOLD);
    fprintf('%-40s %-8s %10s %10s %9s %10s %8s\n', 'column', 'window', 'mean err', 'sd seeds', 'honesty', 'mean l41', 'frac<0');
    for c = 1:nC
        d = D{c}; t = d.t;
        W = [T0, T0+TD; T0+TD, T3(c)]; WN = {'descent','motion'};
        if HOLD(c); W = [W; T3(c), t(end)]; WN{end+1} = 'hold'; end
        for w = 1:size(W,1)
            m = t >= W(w,1) & t <= W(w,2);
            fprintf('%-40s %-8s %+10.4f %10.4f %9.2f %+10.4f %8.2f\n', d.name, WN{w}, mean(mean(d.E(m,:),2)), ...
                    mean(d.sd(m)), mean(d.sd(m))/mean(d.sP(m)), mean(d.l41(m)), mean(d.l41(m) < 0));
        end
        % seed-mean error at each trough visit (trough times: T0+TD + n/F, n = 0..NC) and at the end
        tv = T0 + TD + (0:NC)/F(c); ev = zeros(size(tv));
        for i = 1:numel(tv); [~,k] = min(abs(t - tv(i))); ev(i) = mean(mean(d.E(max(k-25,1):min(k+25,end),:),2)); end
        fprintf('%-40s trough visits t = %s  ->  err %s ;  end %+.4f\n', '', mat2str(tv,3), mat2str(ev,3), mean(mean(d.E(end-50:end,:),2)));
    end

    % ---------------- figure ----------------------------------------------
    C_M = [0.55 0.78 1.0]; C_E = [0 0.2 0.9]; C_T = [0.8 0 0]; FS = 18; LFS = 12; AXLW = 2.0;
    nR = 6; f = figure('Position',[10 10 560*nC 2100],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f, nR, nC, 'TileSpacing','compact','Padding','compact'); A = gobjects(nR,nC);
    ROWS = {'l31','l32','l41','l42'}; RL = {'l_{31}','l_{32}','l_{41}','l_{42}'};
    yl1 = 0; ylr = zeros(4,2); ylh = 0;
    for c = 1:nC
        d = D{c}; yl1 = max(yl1, max(abs(d.E(:)))); ylh = max(ylh, max(d.hd));
        for i = 1:4; v = d.(upper(ROWS{i})); ylr(i,:) = [min(ylr(i,1),min(v(:))), max(ylr(i,2),max(v(:)))]; end
    end
    for c = 1:nC
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
        for i = 1:4
            a = nexttile(tl, i*nC + c); A(i+1,c) = a; hold(a,'on');
            yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
            h0 = plot(a, d.t, d.(upper(ROWS{i})), '-', 'Color', C_M, 'LineWidth', 0.7);
            h1 = plot(a, d.t, d.(ROWS{i}), '-', 'Color', C_E, 'LineWidth', 2.2);
            legend(a,[h0(1) h1],{sprintf('{\\it%s}   each seed (%d)', RL{i}, nS), 'seed mean'},'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',LFS,'FontWeight','bold','Box','on');
            if c == 1; ylabel(a,RL{i},'FontSize',FS,'FontWeight','bold'); end
            pad = 0.05*diff(ylr(i,:)); ylim(a, [ylr(i,1)-pad, ylr(i,2)+pad]);
        end
        a = nexttile(tl, 5*nC + c); A(6,c) = a; hold(a,'on');
        h1 = plot(a, d.t, d.hd, '-', 'Color', C_T, 'LineWidth', 2.4);
        h2 = plot(a, d.t, d.ht, '-', 'Color', C_E, 'LineWidth', 1.0);
        legend(a,[h1 h2],{'w_d  (command)', 'w  true, seed 7'},'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel(a,'h / R','FontSize',FS,'FontWeight','bold'); end
        ylim(a, [0 1.05*ylh]); xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
    end
    for k = 1:nR*nC
        set(A(k),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(k),'off');
        [row, col] = ind2sub([nR nC], k); xlim(A(k), [0 D{col}.t(end)]);
        if row < nR; set(A(k),'XTickLabel',[]); end
        if col > 1; set(A(k),'YTickLabel',[]); end
    end
    exportgraphics(f, fig, 'Resolution', 150); close(f);
    fprintf('figure -> %s\n', fig);
    out = struct('D', {D}, 'seeds', seeds, 'file', fig);
end
