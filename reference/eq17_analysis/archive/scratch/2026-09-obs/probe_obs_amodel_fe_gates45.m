% PURPOSE (2026-09-11, step 3 of the green plan; 2026-09-12 extended with the orange variant): observability gates 4-5
%   on the PRODUCTION linearisation (9 slots, free [1 2 3 4 5 8 9], negative control = inert slot 7) for three arms --
%   original, green (amodel_fe), orange (amodel_fe + amodel_h) -- canon and Meng, seed 7,
%   chord seed with Pf_w0_std = 0 (same arm as plot_obs_b_5state_two_traj.m). verify_state_observability.m gives gates
%   4-5 (rank + negative control, CRLB vs prior, channel and segment splits); this script adds the whitened SVD
%   O_w = R^{-1/2} [H Phi] per window on the free block (sigma_1..sigma_7 and V; the tool returns neither, and its srat
%   is taken on the UNWHITENED O) and the filter's own sqrt(P55)(t).
%   Record order (formC_b.m:1807 then :1815): record k = predict with F[k] from x_upd[k-1], then update with H[k], so
%   y[k0+j] = H[k0+j] F[k0+j] ... F[k0+1] x[k0]. The tool applies F[k0] after H[k0] (one record early).
%   Health checks written before the run: (H1) negative control CRLB = Inf in both arms (tool); (H2a) my CRLB_b built
%   with the TOOL's F order equals the tool's crlb(:,b) to 1e-6 on the windows the tool resolves (a window it flags
%   'beyond numerical resolution' has G conditioned past 1e12 and is excluded); (H2b) report how far the
%   record-order difference moves CRLB_b (expected small: F changes slowly); (H3) flag off reproduces 09-09 production:
%   canon best CRLB_b/prior 0.009, Meng 0.57, sqrt(P55)[end] 0.0087 / 0.0149.
%   Written before the orange run (0912 tex S7): its a_w column is zero in BOTH rows and F keeps e_4 invariant, so gate 4
%   must report rank 6 of 7 in EVERY window and CRLB(a_w) = Inf everywhere -- that is the derivation, not a broken
%   instrument; and b should also become identifiable in hold (H25 keeps the Grad-free term (1-a)/b).
%   Output printed + test_results/apd_acov_meng/obs_amodel_fe_gates45.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = probe_obs_amodel_fe_gates45(seed, win, out_in)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'canon', 'meng'};  NAME = {'original', 'green', 'orange'};
    FLAG = {struct(), struct('amodel_fe', true), struct('amodel_fe', true, 'amodel_h', true)};  nA = 3;
    if nargin >= 3 && ~isempty(out_in); out = out_in; local_fig(out, TR, fullfile(od, 'obs_amodel_fe_gates45.png')); return; end
    free = [1 2 3 4 5 8 9];  lab = {'dw1','dw2','dw3','a_w','b','m1','m2'};  JB = 5;  JA = 4;
    out = struct();
    for c = 1:2
        for f = 1:nA
            [L, S, SEG, p0b] = local_run(TR{c}, seed, FLAG{f});
            prior = nan(1, numel(free));  prior(JB) = p0b;
            vcfg = struct('axis', 3, 'free', free, 'neg_ctrl', 7, 'prior_std', prior, 'labels', {lab}, ...
                          'window', win, 't_end', S.tout(end), 'segments', {SEG});
            fprintf('\n################ %s | %s ################\n', TR{c}, NAME{f});
            vo = verify_state_observability(L, vcfg);
            [sv, V, cb_mine] = local_svd(L, free, win, vo.starts, 'record');
            [~, ~, cb_tool] = local_svd(L, free, win, vo.starts, 'tool');
            rel = abs(cb_tool(:) ./ vo.crlb(:, JB, 1) - 1);  ok = ~vo.near(:, JB, 1);
            d2a = max(rel(ok));  d2b = max(abs(cb_mine(:) ./ vo.crlb(:, JB, 1) - 1));
            fprintf('H2a my CRLB_b (tool F order) vs tool: max rel diff %.2e over %d resolved windows (all windows %.2e) -> %s\n', ...
                    d2a, sum(ok), max(rel), local_pf(d2a < 1e-6));
            fprintf('H2b record F order vs tool F order: max rel change of CRLB_b %.2e (report only)\n', d2b);
            P55 = arrayfun(@(s) s.P_upd(5,5), L);  P44 = arrayfun(@(s) s.P_upd(4,4), L);  t_rec = (double([L.k]) - double(L(1).k)) * vo.Ts;
            [spQ, sp0] = local_replay(L);
            dQ = max(abs(spQ(:) - sqrt(P55(:))) ./ sqrt(P55(:)));
            fprintf('H4 replay with recovered Q reproduces sqrt(P55): max rel err %.2e -> %s | Q = 0 replay: sqrt(P55)[end] %.3g (filter %.3g, %.0fx)\n', ...
                    dQ, local_pf(dQ < 1e-9), sp0(end), sqrt(P55(end)), sqrt(P55(end))/sp0(end));
            out.(TR{c})(f) = struct('vo', vo, 'sv', sv, 'V', {V}, 'cb', cb_mine(:), 'tw', vo.t_win(:), ...
                                    'sqrtP55', sqrt(P55(:)), 'sqrtP44', sqrt(P44(:)), 'ahat', arrayfun(@(s) s.x_upd(4), L(:)), 'sp0', sp0(:), 't_rec', t_rec(:), 'wd', S.h_bar_d_out(:), 't', S.tout(:), ...
                                    'SEG', {SEG}, 'prior', p0b, 'bhat', S.b_hat_out(:,3));
        end
        A = out.(TR{c})(1);
        fprintf('\n=== %s summary (window %d steps, CRLB in record F order) ===\n', TR{c}, win);
        fprintf('%-8s | %s\n', 'segment', 'best CRLB_b/prior  (sigma_7/sigma_1)  per arm: original | green | orange');
        for s = 1:size(A.SEG, 1)
            m = find(A.tw >= A.SEG{s,2} & A.tw <= A.SEG{s,3});
            if isempty(m); continue; end
            fprintf('%-8s |', A.SEG{s,1});
            for f = 1:nA
                X = out.(TR{c})(f);
                fprintf(' %9.4f (%8.2e) |', min(X.cb(m))/X.prior, median(X.sv(m,end)./X.sv(m,1)));
            end
            fprintf('\n');
        end
        fprintf('best CRLB_b/prior whole run :');  for f = 1:nA; fprintf(' %s %.4f |', NAME{f}, min(out.(TR{c})(f).cb)/out.(TR{c})(f).prior); end; fprintf('\n');
        fprintf('median CRLB a_w (y1+y2)     :');  for f = 1:nA; fprintf(' %s %.4g |', NAME{f}, median(out.(TR{c})(f).vo.crlb(:,JA,1))); end; fprintf('\n');
        fprintf('rank over windows           :');  for f = 1:nA; fprintf(' %s %d..%d |', NAME{f}, min(out.(TR{c})(f).vo.rank(:,1)), max(out.(TR{c})(f).vo.rank(:,1))); end; fprintf('\n');
        fprintf('filter sqrt(P55) end        :');  for f = 1:nA; fprintf(' %s %.5f |', NAME{f}, out.(TR{c})(f).sqrtP55(end)); end; fprintf('\n');
        fprintf('Q = 0 replay sqrt(P55) end  :');  for f = 1:nA; fprintf(' %s %.3g |', NAME{f}, out.(TR{c})(f).sp0(end)); end; fprintf('\n');
        fprintf('b_hat end                   :');  for f = 1:nA; fprintf(' %s %.4f |', NAME{f}, out.(TR{c})(f).bhat(end)); end; fprintf('\n');
        te = A.SEG{3,2};  [~, ke] = min(abs(A.t_rec - te));
        fprintf('filter sqrt(P44)/a_hat at %.1f s :', te);  for f = 1:nA; X = out.(TR{c})(f); fprintf(' %s %.4f |', NAME{f}, X.sqrtP44(ke)/X.ahat(ke)); end; fprintf('\n');
        [~, q] = min(A.cb);
        fprintf('\n--- %s window %d (t %.2f s): sigma and V on the free block [%s] ---\n', TR{c}, q, A.tw(q), strjoin(lab, ' '));
        for f = 1:nA
            X = out.(TR{c})(f);
            fprintf('%s sigma: %s\n', NAME{f}, sprintf('%10.4g', X.sv(q,:)));
            fprintf('%s V (rows = slots, cols = v_1..v_7):\n', NAME{f});
            for j = 1:numel(free); fprintf('   %-4s %s\n', lab{j}, sprintf('%8.3f', X.V{q}(j,:))); end
            fprintf('   CRLB_b %.3g  (1/sigma_7 %.3g)\n', X.cb(q), 1/X.sv(q,end));
        end
    end
    local_fig(out, TR, fullfile(od, 'obs_amodel_fe_gates45.png'));
end

function [L, S, SEG, p0b] = local_run(traj, seed, flags)
    pc = physical_constants();
    cfg = canonical_scenario(0.05, 1.1, 'deep');
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                        'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
            fn = fieldnames(OV); for i = 1:numel(fn); cfg.(fn{i}) = OV.(fn{i}); end
            SEG = {'hold0',0,0.5; 'far',0.5,7; 'near',7,10.5; 'hold',10.5,12.5};  p0 = 3e-4;
        otherwise
            OV = struct();
            SEG = {'hold0',0,0.5; 'descent',0.5,1.5; 'osc',1.5,3.5; 'hold',3.5,4.8};  p0 = 1e-5;
    end
    w0 = cfg.h_init / pc.R;  [~, cp] = calc_correction_functions(w0, true);  a0 = 1/cp;
    b_ch = (1/(1 - a0) - 1) / (w0 - 1.0);  ws_ch = 2 - 1/b_ch;
    cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws_ch,'obs_dump',true);
    fn = fieldnames(flags); for i = 1:numel(fn); cc.(fn{i}) = flags.(fn{i}); end
    o  = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep', ...
                'verbose',false,'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;
    evalc('Rr = run_formC_b(o);');
    S = Rr.runs{1};  p0b = S.ctrl_const.Pf_b_std;
    L = obs_dump('get');  L = L([L.ax] == 3);
end

function [sv, V, cb] = local_svd(L, free, W, starts, order)
    % whitened O on the free block per window; CRLB_b from G = O_w' O_w (eig, as the tool)
    n = size(L(1).F, 1);  nw = numel(starts);  nf = numel(free);  jb = find(free == 5);
    sv = zeros(nw, nf);  V = cell(nw, 1);  cb = zeros(nw, 1);
    for q = 1:nw
        k0 = starts(q);  Phi = eye(n);  O = zeros(0, n);
        for k = k0:(k0 + W - 1)
            if strcmp(order, 'record') && k > k0; Phi = L(k).F * Phi; end
            for j = 1:numel(L(k).H)
                Hj = L(k).H{j};  if isempty(Hj); continue; end
                O(end+1, :) = (Hj * Phi) / sqrt(L(k).R(j)); %#ok<AGROW>
            end
            if strcmp(order, 'tool'); Phi = L(k).F * Phi; end
        end
        Of = O(:, free);  [~, Sm, Vm] = svd(Of, 'econ');  sv(q, :) = diag(Sm).';  V{q} = Vm;
        G = Of' * Of;  G = 0.5*(G + G');  [Ve, De] = eig(G);  dv = real(diag(De));
        tol = max(dv) * 1e-15;  ok = dv > tol;  wj = real(Ve(jb, :)).^2;      % same cut as verify_state_observability
        if any(wj(~ok) > 1e-12); cb(q) = Inf; else; cb(q) = sqrt(sum(wj(ok) ./ dv(ok).')); end
    end
end

function [spQ, sp0] = local_replay(L)
    % the filter's covariance recursion replayed from the dump (copied from scratch/audit_fehrq_consistency.m, 09-09):
    % with Q recovered as P_pred - F P_upd F' it must reproduce sqrt(P55); with Q = 0 it is what the CRLB assumes
    N = numel(L);  n = size(L(1).F, 1);
    lockz = find(all(abs(L(end).P_upd) < 1e-300, 1) & all(abs(L(end).P_upd) < 1e-300, 2).');
    spQ = zeros(N, 1);  sp0 = zeros(N, 1);  PQ = L(1).P_upd;  P0 = PQ;  spQ(1) = sqrt(PQ(5,5));  sp0(1) = spQ(1);
    for k = 2:N
        F = L(k).F;  Q = L(k).P_pred - F * L(k-1).P_upd * F';
        PQ = F*PQ*F' + Q;  P0 = F*P0*F';  PQ = 0.5*(PQ+PQ');  P0 = 0.5*(P0+P0');
        for j = 1:numel(L(k).H)
            Hj = L(k).H{j};  if isempty(Hj); continue; end
            [PQ] = local_joseph(PQ, Hj, L(k).R(j), lockz, n);  [P0] = local_joseph(P0, Hj, L(k).R(j), lockz, n);
        end
        spQ(k) = sqrt(max(PQ(5,5), 0));  sp0(k) = sqrt(max(P0(5,5), 0));
    end
end

function P = local_joseph(P, Hj, Rj, lockz, n)
    S = Hj*P*Hj' + Rj;  K = (P*Hj')/S;  K(lockz) = 0;
    P = (eye(n) - K*Hj)*P*(eye(n) - K*Hj)' + K*Rj*K';  P = 0.5*(P+P');
end

function local_fig(out, TR, fn)
    FS = 15; LFS = 11; AXLW = 1.8;  COL = {[0 0.2 0.9], [0 0.6 0.25], [0.9 0.5 0]};  NAME = {'original', 'green', 'orange'};  nA = 3;
    fig = figure('Units','inches','Position',[0 0 12 10.5], 'Color','w', 'Visible','off');
    tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        A = out.(TR{c})(1);  T = A.t(end);
        nexttile(c); hold on; box on;
        plot(A.t, A.wd, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]);
        if c == 1; ylabel('$\bar{w}_d$', 'Interpreter','latex', 'FontSize', FS+2); end
        legend({TR{c}}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        nexttile(2 + c); hold on; box on;  H = gobjects(1, nA);
        for f = 1:nA
            B = out.(TR{c})(f);
            H(f) = semilogy(B.tw, B.sv(:, end) ./ B.sv(:, 1), '-o', 'Color', COL{f}, 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', COL{f});
        end
        set(gca, 'YScale', 'log', 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW); ylim([1e-20 1]); set(gca, 'YTick', 10.^(-20:4:0)); xlim([0 T]);
        if c == 1; ylabel('$\sigma_7 / \sigma_1$', 'Interpreter','latex', 'FontSize', FS+2); end
        legend(H, NAME, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        nexttile(4 + c); hold on; box on;  H = gobjects(1, 2*nA);
        for f = 1:nA
            B = out.(TR{c})(f);
            H(f)    = semilogy(B.tw, B.cb / B.prior, 'o', 'Color', COL{f}, 'MarkerSize', 6, 'MarkerFaceColor', COL{f});
            H(nA+f) = semilogy(B.t_rec, B.sqrtP55 / B.sqrtP55(1), '-', 'Color', COL{f}, 'LineWidth', 2.2);
        end
        plot([0 T], [1 1], 'k:', 'LineWidth', 1.5);
        set(gca, 'YScale', 'log', 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW); ylim([1e-3 1e2]); set(gca, 'YTick', 10.^(-3:2)); xlim([0 T]);
        if c == 1; ylabel('error bar of $b$ $/\sqrt{P_{55}[0]}$', 'Interpreter','latex', 'FontSize', FS+2); end
        xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold');
        legend(H, {'original CRLB', 'green CRLB', 'orange CRLB', 'original filter', 'green filter', 'orange filter'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',3);
    end
    exportgraphics(fig, fn, 'Resolution', 150);  close(fig);
    fprintf('\nfigure: %s\n', fn);
end

function s = local_pf(b); if b; s = 'PASS'; else; s = 'FAIL'; end; end
