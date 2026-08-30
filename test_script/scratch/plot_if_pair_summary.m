% STATUS: ACTIVE (scratch figure) | PURPOSE: the N-seed summary of the paired
%   "R2 colour factor IF kept vs removed" runs (run_if_pair_meng), Meng 10 s
%   ramp, z axis. Two pages, both arms overlaid: IF on = blue, IF off = green,
%   truth = red. Every curve is the mean over seeds.
%
%   page A  if_pair_mechanism.png   what the filter did with y2
%     1  l_42                       willingness to listen to e_ah
%     2  e_ah share  |l42 e_ah| / (|l41 e_h1| + |l42 e_ah|), 0.25 s mean
%     3  sum l_42 e_ah              net push on a_hat from y2
%   page B  if_pair_result.png      what came out
%     1  a_hat - a_true
%     2  b_hat - b_true
%     3  honesty  sd_seeds(a_hat) / mean sqrt(P44)     (1 = honest)
%   Console: paired (off - on) differences, mean +- SE over seeds, three
%   windows (far 1-4 s, descent 6-9 s, hold 10.5-12.5 s).
function out = plot_if_pair_summary(mat)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(mat); mat = fullfile(od, 'pair_if_meng_100.mat'); end
    S = load(mat); out = S.out; ax = 3; nS = numel(out.seeds); W = 400;
    WIN = [1 4; 6 9; 10.5 12.5]; WN = {'far 1-4 s', 'descent 6-9 s', 'hold 10.5-12.5 s'};

    D = cell(1, 2);
    for c = 1:2
        r0 = out.O{c}.runs{1}; t = r0.tout(:); N = numel(t);
        F = struct('ah',zeros(N,nS),'at',zeros(N,nS),'bh',zeros(N,nS),'bt',zeros(N,nS),'l42',zeros(N,nS), ...
                   'l41',zeros(N,nS),'sh',zeros(N,nS),'C2',zeros(N,nS),'C1',zeros(N,nS),'P44',zeros(N,nS), ...
                   'e2',zeros(N,nS),'R2',zeros(N,nS),'trk',zeros(N,nS));
        for q = 1:nS
            r = out.O{c}.runs{q};
            F.ah(:,q) = r.a_bar_hat_out(:,ax);
            F.at(:,q) = r.a_true_out(:,ax) / (r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax));
            F.bh(:,q) = r.b_hat_out(:,ax);
            F.bt(:,q) = local_b_true(r.p_true_out(:,ax)/r.R);
            F.l42(:,q) = r.K_a_y2_out(:,ax);  F.l41(:,q) = r.K_a_y1_out(:,ax);
            c1 = r.K_a_y1_out(:,ax).*r.innov_y1_out(:,ax); c2 = r.K_a_y2_out(:,ax).*r.innov_y2_out(:,ax); c1(1)=0; c2(1)=0;
            F.C1(:,q) = cumsum(c1); F.C2(:,q) = cumsum(c2);
            F.sh(:,q) = movmean(abs(c2)./max(abs(c1)+abs(c2), realmin), W);
            F.P44(:,q) = r.P_a_out(:,ax); F.e2(:,q) = r.innov_y2_out(:,ax); F.R2(:,q) = r.R2_out(:,ax);
            F.trk(:,q) = r.p_d_out(:,ax) - [r.p_true_out(1,ax); r.p_true_out(1:end-1,ax)];
        end
        F.t = t; D{c} = F;
    end

    % ---------------- console: paired differences ----------------------------
    fprintf('\n%d seeds, paired (off - on), mean +- SE\n', nS);
    for w = 1:size(WIN,1)
        m = D{1}.t >= WIN(w,1) & D{1}.t <= WIN(w,2);
        fprintf('\n--- %s ---\n', WN{w});
        fprintf('%-32s %12s %12s %14s\n', 'metric', 'IF on', 'IF off', 'off - on');
        mets = {
          'a_hat bias  (mean a_hat-a_true)', @(F) mean(F.ah(m,:) - F.at(m,:), 1);
          'b_hat bias  (mean b_hat-b_true)', @(F) mean(F.bh(m,:) - F.bt(m,:), 1);
          'mean l_42',                       @(F) mean(F.l42(m,:), 1);
          'mean |l_41|',                     @(F) mean(abs(F.l41(m,:)), 1);
          'e_ah share',                      @(F) mean(F.sh(m,:), 1);
          'var(e_ah)/R2',                    @(F) var(F.e2(m,:), 0, 1) ./ mean(F.R2(m,:), 1);
          'sum l42 e_ah over window',        @(F) F.C2(find(m,1,'last'),:) - F.C2(find(m,1),:);
          'sum l41 e_h1 over window',        @(F) F.C1(find(m,1,'last'),:) - F.C1(find(m,1),:);
          'tracking RMS [nm]',               @(F) 1e3*sqrt(mean(F.trk(m,:).^2, 1));
        };
        for i = 1:size(mets,1)
            x1 = mets{i,2}(D{1}); x2 = mets{i,2}(D{2}); d = x2 - x1;
            fprintf('%-32s %12.5f %12.5f %+9.5f +- %.5f\n', mets{i,1}, mean(x1), mean(x2), mean(d), std(d)/sqrt(nS));
        end
        h1 = std(D{1}.ah(m,:), 0, 2) ./ sqrt(mean(D{1}.P44(m,:), 2));
        h2 = std(D{2}.ah(m,:), 0, 2) ./ sqrt(mean(D{2}.P44(m,:), 2));
        fprintf('%-32s %12.3f %12.3f\n', 'honesty sd(a_hat)/sqrtP44', mean(h1), mean(h2));
    end

    % ---------------- figures ------------------------------------------------
    C_ON = [0 0.2 0.9]; C_OFF = [0.10 0.55 0.15]; FS = 18; LFS = 13; AXLW = 2.0;
    t = D{1}.t; tl_ = [0 t(end)];
    mn = @(x) mean(x, 2);

    % page A
    f = figure('Position',[10 10 1400 1400],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact'); A = gobjects(3,1);
    rows = {
      'l_{42}',                 @(F) mn(F.l42),   'l_{42}';
      'e_{ah} share',           @(F) mn(F.sh),    '|{\itl}_{42} e_{ah}| / ( |{\itl}_{41} e_{h1}| + |{\itl}_{42} e_{ah}| )';
      '\Sigma {\itl}_{42} e_{ah}', @(F) mn(F.C2), '\Sigma {\itl}_{42} e_{ah}';
    };
    for i = 1:3
        a = nexttile(tl); A(i) = a; hold(a,'on');
        if i == 3; yline(a,0,'-','Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off'); end
        h1 = plot(a, t, rows{i,2}(D{1}), '-', 'Color', C_ON,  'LineWidth', 2.2);
        h2 = plot(a, t, rows{i,2}(D{2}), '-', 'Color', C_OFF, 'LineWidth', 2.2);
        legend(a,[h1 h2],{[rows{i,3} '    IF on'], 'IF off'},'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        ylabel(a, strrep(rows{i,1},'{\itl}','l'), 'FontSize', FS, 'FontWeight', 'bold');
    end
    xlabel(A(3),'time  [s]','FontSize',FS,'FontWeight','bold');
    local_style(A, tl_, FS, AXLW);
    fnA = fullfile(od,'if_pair_mechanism.png'); exportgraphics(f,fnA,'Resolution',150); close(f);

    % page B
    f = figure('Position',[10 10 1400 1400],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact'); A = gobjects(3,1);
    hon = @(F) std(F.ah,0,2) ./ sqrt(mean(F.P44,2));
    rows = {
      '\^a_h - a   (mean over seeds)', @(F) mn(F.ah - F.at), {'\^a_h - a  (\muean over seeds)'};
      '\^b - b',                       @(F) mn(F.bh - F.bt), {'\^b - b'};
      'sd(\^a_h) / \surdP_{44}',       hon,                  {'sd_{seeds}(\^a_h) / mean \surdP_{44}'};
    };
    for i = 1:3
        a = nexttile(tl); A(i) = a; hold(a,'on');
        yline(a, double(i==3), '-', 'Color',[0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
        h1 = plot(a, t, rows{i,2}(D{1}), '-', 'Color', C_ON,  'LineWidth', 2.2);
        h2 = plot(a, t, rows{i,2}(D{2}), '-', 'Color', C_OFF, 'LineWidth', 2.2);
        lab = rows{i,3}{1}; lab = strrep(lab, '\muean', 'mean');
        legend(a,[h1 h2],{[lab '    IF on'], 'IF off'},'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        ylabel(a, rows{i,1}, 'FontSize', FS, 'FontWeight', 'bold');
    end
    xlabel(A(3),'time  [s]','FontSize',FS,'FontWeight','bold');
    local_style(A, tl_, FS, AXLW);
    fnB = fullfile(od,'if_pair_result.png'); exportgraphics(f,fnB,'Resolution',150); close(f);
    fprintf('\nfigures -> %s\n            %s\n', fnA, fnB);
    out.D = D;
end

function local_style(A, tl_, FS, AXLW)
    for i = 1:numel(A)
        set(A(i),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(i),'off'); xlim(A(i), tl_);
        if i < numel(A); set(A(i),'XTickLabel',[]); end
    end
end

function b = local_b_true(w)
    persistent wq bq
    if isempty(wq)
        wq = linspace(1.05, 30, 4000); cp = zeros(size(wq));
        for i = 1:numel(wq); [~, c] = calc_correction_functions(wq(i)); cp(i) = c; end
        a_tr = 1 ./ cp; bq = gradient(a_tr, wq) ./ (1 - a_tr).^2;
    end
    b = interp1(wq, bq, w, 'linear', 'extrap');
end
