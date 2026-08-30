% STATUS: ACTIVE (scratch) | PURPOSE: steps 0-2 of the 2026-08-28 check list on
%   the oscillation x hold runs of run_osc_freq_hold (TRUE-b arm, 6 seeds):
%     0  instrument: the exact a_hat identity, with the predict increment
%        REBUILT from the logs,  LAW_model = a_bar' (Dw_d[k-1] + (1-lc) dw3_hat),
%        must reproduce the by-difference LAW (residual = MA2 memory term only;
%        the index shift is searched, not assumed);
%     1  the three-term error ledger per TROUGH VISIT, seed mean +- sd:
%        de_a = (LAW - Da_true) + l41 e_h1 + l42 e_ah ;
%     2  the predict term split four ways:
%        (a'_used - a'_true) Dw_d | a'_true (Dw_d - Dw_true) | a'_true Dw_true - Da_true | te = a'_used (1-lc) dw3_hat.
%   Figure: one column per run set, row 1 cumulative three terms + e_a,
%   row 2 cumulative four predict sub-terms, row 3 trajectory.
%   out = ledger_osc_visits();  out = ledger_osc_visits('cmd');
function out = ledger_osc_visits(tag)
    if nargin < 1 || isempty(tag); tag = ''; else; tag = ['_' tag]; end
    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    S = load(fullfile(od, ['osc_freq_hold' tag '.mat'])); O = S.O; F = S.F; HOLD = S.HOLD; NAMES = S.NAMES;
    ax = 3; nC = numel(O); nS = numel(O{1}.runs); T0 = 0.5; TD = 1.0; NC = 2; lc = O{1}.runs{1}.ctrl_const.lambda_c;
    D = cell(1, nC);
    for c = 1:nC
        t = O{c}.runs{1}.tout(:); N = numel(t); Ts = t(2) - t(1);
        Z = zeros(N, nS); EA = Z; LAW = Z; C1 = Z; C2 = Z; T1 = Z; T2 = Z; T3 = Z; T4 = Z; LM = Z; DAT = Z;
        best = [];
        for q = 1:nS
            r = O{c}.runs{q}; R = r.R; ad = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax);
            ah = r.a_bar_hat_out(:,ax); at = r.a_true_out(:,ax)/ad;
            apu = r.a_prime_out(:,ax)/ad; apt = r.a_prime_true_out(:,ax)/ad;     % a_bar' used / true-at-particle
            dwd = [0; diff(r.p_d_out(:,ax))]/R;                                    % Dw_d[k] = w_d[k] - w_d[k-1]
            dwt = [0; diff(r.h_bar_true_out(:))];
            dw3 = r.delta_x_hat_3_out(:,ax)/R;
            c1 = r.K_a_y1_out(:,ax).*r.innov_y1_out(:,ax); c2 = r.K_a_y2_out(:,ax).*r.innov_y2_out(:,ax);
            law = [0; diff(ah)] - c1 - c2;                                         % predict, by difference (exact)
            % ---- step 0: rebuild LAW from the logs; search the index shifts once (seed 1)
            if isempty(best)
                best = struct('res', Inf);
                for sa = -2:1, for sd = -2:1, for s3 = -2:1
                    lm = shiftv(apu, sa) .* (shiftv(dwd, sd) + (1-lc)*shiftv(dw3, s3));
                    m = 200:N-5; res = sqrt(mean((law(m) - lm(m)).^2));
                    if res < best.res; best = struct('res', res, 'sa', sa, 'sd', sd, 's3', s3); end
                end, end, end
            end
            lm = shiftv(apu, best.sa) .* (shiftv(dwd, best.sd) + (1-lc)*shiftv(dw3, best.s3));
            apu_s = shiftv(apu, best.sa); dwd_s = shiftv(dwd, best.sd); dw3_s = shiftv(dw3, best.s3);
            apt_s = shiftv(apt, best.sa); dwt_s = shiftv(dwt, best.sd); dat = [0; diff(at)]; dat_s = shiftv(dat, best.sd);
            EA(:,q) = ah - at; LAW(:,q) = law; C1(:,q) = c1; C2(:,q) = c2; LM(:,q) = lm; DAT(:,q) = dat;
            T1(:,q) = (apu_s - apt_s).*dwd_s;            % slope-value error
            T2(:,q) = apt_s.*(dwd_s - dwt_s);            % path difference (command vs true)
            T3(:,q) = apt_s.*dwt_s - dat_s;              % quadrature error of the true law
            T4(:,q) = apu_s.*(1-lc).*dw3_s;              % te: tracking-error feed
        end
        % windows: descent | in i = previous far turn -> trough i (quarter cycle, i=1: last half of the descent)
        %          | out i = trough i -> next far turn | hold = from the last trough to the end
        P = 1/F(c); tv = T0 + TD + (0:NC)*P;
        W = {'descent', [T0, T0+TD]};
        for i = 1:numel(tv)
            if i == 1; W(end+1,:) = {'in  1', [T0+TD/2, tv(1)]}; else; W(end+1,:) = {sprintf('in  %d', i), [tv(i)-P/2, tv(i)]}; end
            if i < numel(tv); W(end+1,:) = {sprintf('out %d', i), [tv(i), tv(i)+P/2]}; end
        end
        if HOLD(c); W(end+1,:) = {'hold', [tv(end), t(end)]}; end
        % self-loop fingerprint: (a'_used - a'_true) vs -2 b_true (1-a) e_a, per step
        BT = zeros(N, nS); for q = 1:nS; r = O{c}.runs{q}; BT(:,q) = r.b_hat_out(:,ax); end   % b the law used (= b_true at particle)
        AT_all = zeros(N, nS); for q = 1:nS; r = O{c}.runs{q}; AT_all(:,q) = r.a_true_out(:,ax)/(r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax)); end
        APD = zeros(N, nS); for q = 1:nS; r = O{c}.runs{q}; ad = r.a_hat_out(1,ax)/r.a_bar_hat_out(1,ax); APD(:,q) = (r.a_prime_out(:,ax) - r.a_prime_true_out(:,ax))/ad; end
        PRED = -2*BT.*(1-AT_all).*EA;
        D{c} = struct('t', t, 'EA', EA, 'LAW', LAW, 'C1', C1, 'C2', C2, 'LM', LM, 'DAT', DAT, ...
                      'T1', T1, 'T2', T2, 'T3', T3, 'T4', T4, 'W', {W}, 'name', NAMES{c}, 'best', best, ...
                      'hd', O{c}.runs{1}.p_d_out(:,ax)/O{c}.runs{1}.R);
        % ---- console
        res_all = LAW - LM; 
        fprintf('\n=== %s ===\n', NAMES{c});
        fprintf('step 0  shifts (a'',Dw_d,dw3) = (%+d,%+d,%+d)  rms(LAW - LAW_model) = %.2e  max = %.2e  vs rms(LAW) = %.2e   [residual = MA2 memory term]\n', ...
                best.sa, best.sd, best.s3, sqrt(mean(res_all(200:end-5,:).^2,'all')), max(abs(res_all(200:end-5,:)),[],'all'), sqrt(mean(LAW(200:end,:).^2,'all')));
        fprintf('%-8s %-11s | %8s | %8s %8s %8s | %8s %8s %8s %8s | %8s | %6s %6s | %7s\n', 'window', 't [s]', 'd e_a', 'LAW-Da', 'Sl41e1', 'Sl42e2', '(a''-a''t)Dwd', 'a''t(Dwd-Dwt)', 'quad', 'te', 'te+Sl41e', 'mean e_a', 'ratio', 'sd(de_a)');
        for w = 1:size(W,1)
            m = t >= W{w,2}(1) & t <= W{w,2}(2);
            f = @(X) mean(sum(X(m,:),1));
            dea = EA(find(m,1,'last'),:) - EA(find(m,1),:);
            sel = m & abs(mean(EA,2)) > 0.003; ratio = NaN;
            if nnz(sel) > 50; ratio = median(mean(APD(sel,:),2) ./ mean(PRED(sel,:),2)); end
            fprintf('%-8s %5.2f-%5.2f | %+8.4f | %+8.4f %+8.4f %+8.4f | %+8.4f %+8.4f %+8.4f %+8.4f | %+8.4f | %+6.3f %6.2f | %7.4f\n', W{w,1}, W{w,2}, mean(dea), ...
                    f(LAW-DAT), f(C1), f(C2), f(T1), f(T2), f(T3), f(T4), f(T4)+f(C1), mean(mean(EA(m,:),2)), ratio, std(dea));
        end
    end
    % ---- figure
    C_E = [0 0.2 0.9]; C_T = [0.8 0 0]; FS = 16; LFS = 11; AXLW = 2.0; nR = 3;
    fig = fullfile(od, ['osc_visit_ledger' tag '.png']);
    f = figure('Position',[10 10 560*nC 1500],'Color','w','NumberTitle','off','Visible','off');
    tl = tiledlayout(f, nR, nC, 'TileSpacing','compact','Padding','compact'); A = gobjects(nR, nC);
    y1 = 0; y2 = 0;
    for c = 1:nC; d = D{c}; y1 = max(y1, max(abs([mean(d.EA,2); cumsum(mean(d.LAW-d.DAT,2)); cumsum(mean(d.C1,2)); cumsum(mean(d.C2,2))])));
        y2 = max(y2, max(abs([cumsum(mean(d.T1,2)); cumsum(mean(d.T2,2)); cumsum(mean(d.T3,2)); cumsum(mean(d.T4,2))]))); end
    for c = 1:nC
        d = D{c}; t = d.t;
        a = nexttile(tl, c); A(1,c) = a; hold(a,'on'); yline(a,0,'-','Color',[0.5 0.5 0.5],'HandleVisibility','off');
        h = gobjects(1,4);
        h(1) = plot(a, t, mean(d.EA,2), 'k--', 'LineWidth', 2.2);
        h(2) = plot(a, t, cumsum(mean(d.LAW-d.DAT,2)), '-', 'Color', [0.55 0.3 0.85], 'LineWidth', 2.2);
        h(3) = plot(a, t, cumsum(mean(d.C1,2)), '-', 'Color', C_E, 'LineWidth', 2.2);
        h(4) = plot(a, t, cumsum(mean(d.C2,2)), '-', 'Color', [0.1 0.6 0.15], 'LineWidth', 2.2);
        legend(a, h, {'e_a = \^a - a', '\Sigma (LAW - \Deltaa_{true})', '\Sigma l_{41} e_{h1}', '\Sigma l_{42} e_{ah}'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        title(a, d.name, 'FontSize', LFS+1, 'FontWeight', 'bold'); ylim(a, 1.05*[-y1 y1]);
        if c == 1; ylabel(a, 'seed-mean cumulative  [a-bar]', 'FontSize', FS, 'FontWeight', 'bold'); end
        a = nexttile(tl, nC + c); A(2,c) = a; hold(a,'on'); yline(a,0,'-','Color',[0.5 0.5 0.5],'HandleVisibility','off');
        h(1) = plot(a, t, cumsum(mean(d.T1,2)), '-', 'Color', [0.85 0.4 0.1], 'LineWidth', 2.2);
        h(2) = plot(a, t, cumsum(mean(d.T2,2)), '-', 'Color', [0.2 0.6 0.8], 'LineWidth', 2.2);
        h(3) = plot(a, t, cumsum(mean(d.T3,2)), '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 2.2);
        h(4) = plot(a, t, cumsum(mean(d.T4,2)), '-', 'Color', [0.7 0.1 0.5], 'LineWidth', 2.2);
        legend(a, h, {'(a''_{used}-a''_{true})\Deltaw_d', 'a''_{true}(\Deltaw_d-\Deltaw_{true})', 'a''_{true}\Deltaw_{true}-\Deltaa_{true}', 'te = a''(1-\lambda_c)\deltaw_3'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS-1,'FontWeight','bold','Box','on');
        ylim(a, 1.05*[-y2 y2]);
        if c == 1; ylabel(a, 'predict split, cumulative', 'FontSize', FS, 'FontWeight', 'bold'); end
        a = nexttile(tl, 2*nC + c); A(3,c) = a; hold(a,'on');
        plot(a, t, d.hd, '-', 'Color', C_T, 'LineWidth', 2.2); ylim(a, [0 1.05*max(d.hd)]);
        if c == 1; ylabel(a, 'w_d  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel(a, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    end
    for k = 1:nR*nC
        set(A(k),'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid(A(k),'off');
        [row, col] = ind2sub([nR nC], k); xlim(A(k), [0 D{col}.t(end)]);
        if row < nR; set(A(k),'XTickLabel',[]); end
        if col > 1 && row < 3; set(A(k),'YTickLabel',[]); end
    end
    exportgraphics(f, fig, 'Resolution', 150); close(f); fprintf('figure -> %s\n', fig);
    out = struct('D', {D}, 'file', fig);
end

function y = shiftv(x, s)
% y[k] = x[k-s]  (s > 0: use the value from s steps earlier); zero-padded
    y = zeros(size(x)); n = numel(x);
    if s >= 0; y(1+s:n) = x(1:n-s); else; y(1:n+s) = x(1-s:n); end
end
