% PURPOSE (2026-09-10, user: "if the wall is known, is the substituted 4-state [dw1 dw2 dw3 b] feasible?"): two numbers from
%   the production linearisation (obs_dump), no controller change. Same arm as audit_fehrq_consistency.m (chord seed through
%   contact w = 1, Pf_w0_std = 0, law_exact_step + pred_mean2 + nw_mcorr + pred_mean2_e4, seed 7).
%   D1  does the CURRENT 5-slot filter keep the wall known?  derived wall  w_wall = w_d - dw3 - a/(b(1-a))  (09-08 3x3),
%       its std sqrt(t P t') with  t = d w_wall / d x = [0 0 -1  -1/(b(1-a)^2)  a/(b^2(1-a))  0 ...].
%   D2  the 4-state = the user's substitution applied to the production matrices: in every column replace
%           e_a = -a'(e_dw3 + e_anchor) + lever * e_b ,  e_anchor == 0 (the anchor is known),  slot 4 removed,
%       anchor 'wc' = contact (lever a(1-a)/b, the physical wall) or 'ws' = w_s (lever (1-a)/b, what Pf_w0_std = 0 pins).
%       F4 = F(keep,keep) + F(keep,4) Tinv(4,keep) at x_upd[k-1];  H4 = H(keep) + H(4) Tinv(4,keep) at x_pred[k];
%       Q4 = Q(keep,keep) (the gain-row noise has no home: the thermal kick moves the particle and the gain follows the law);
%       P4[0] = P_upd[1](keep,keep) (b keeps its own prior 0.0389, the anchor is simply not carried).
%   Information only, at the production linearisation point. The representation error of the pinned line is
%   compare_one_param_families.m family A.
%   2026-09-10 first version pinned P on t e = 0 after EVERY predict: invalid (sqrt(P55) -> 0 inside hold0). The production
%   row 4 is the Euler Jacobian, which does not conserve the anchor, so every re-pin was a fresh perfect measurement; and
%   Pf_w0_std = 0 already pins w_s, so pinning the contact as well fixes b = 1/(w_c - w_s) outright.
%   Health checks: (H1) plain replay reproduces the recorded sqrt(P55) to 1e-9; (H2) derived wall at k = 1 is 1.000 (the
%   chord goes through contact); (H3) derived-anchor std at k = 1: w_s ~ sqrt(P33 + floor^2/a'^2) (pinned by the prior),
%   contact ~ sqrt(P33 + P55/b^4 + floor^2/a'^2) (moves with b); (H4) 4-state hold0 must NOT collapse (lever 0.05 far field).
%   Output printed + test_results/apd_acov_meng/wall_known_4state.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = probe_wall_known_4state(seed)
    if nargin < 1 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    TR = {'canon', 'meng'};  out = struct();  FIG = cell(2, 1);
    for it = 1:2
        traj = TR{it};
        switch traj   % identical to audit_fehrq_consistency.m
            case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV; p0 = 3e-4;
                         SEG = {'hold0',0,0.5; 'far',0.5,7; 'near',7,10.5; 'hold',10.5,12.5};
            case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
                         SEG = {'hold0',0,0.5; 'descent',0.5,1.5; 'osc',1.5,3.5; 'hold',3.5,4.8};
        end
        w0 = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0, true); a0 = 1/cp;
        b_ch = (1/(1-a0) - 1)/(w0 - 1.0);  ws0 = 1 + 1.0 - 1/b_ch;
        cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1, ...
                    'b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws0,'obs_dump',true);
        o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get'); L = L([L.ax] == 3);  r = R.runs{1};  t = r.tout(:);  N = numel(L);  n = size(L(1).F, 1);
        Ts = t(2) - t(1);  kk = double([L.k]);  tr = t(1) + (kk(:) - kk(1)) * Ts;
        lockz = find(all(abs(L(end).P_upd) < 1e-300, 1) & all(abs(L(end).P_upd) < 1e-300, 2).');
        Qs = cell(N, 1);
        for k = 2:N; Q = L(k).P_pred - L(k).F * L(k-1).P_upd * L(k).F'; Qs{k} = 0.5*(Q + Q'); end
        act = arrayfun(@(s) sqrt(s.P_upd(5,5)), L); act = act(:);
        % ---- D1: the current filter's own derived-wall std ----
        swall = zeros(N, 1);
        for k = 1:N; tk = local_t(L(k).x_upd, n); swall(k) = sqrt(max(tk * L(k).P_upd * tk', 0)); end
        a1 = L(1).x_upd(4); b1 = L(1).x_upd(5); apr1 = b1*(1-a1)^2;
        wwall1 = (w0 - L(1).x_upd(3)) - a1/(b1*(1-a1));
        % ---- D2: replays ----
        sp_plain = local_replay(L, Qs, lockz, n, 'both');
        sp_cur1  = local_replay(L, Qs, lockz, n, 'y1');
        sp_cur2  = local_replay(L, Qs, lockz, n, 'y2');
        [sc, sc1, sc2] = deal(local_replay4(L, Qs, lockz, n, 'both', 'wc'), local_replay4(L, Qs, lockz, n, 'y1', 'wc'), local_replay4(L, Qs, lockz, n, 'y2', 'wc'));
        [ss, ss1, ss2] = deal(local_replay4(L, Qs, lockz, n, 'both', 'ws'), local_replay4(L, Qs, lockz, n, 'y1', 'ws'), local_replay4(L, Qs, lockz, n, 'y2', 'ws'));
        tws = local_t(L(1).x_upd, n, 'ws'); sws1 = sqrt(tws * L(1).P_upd * tws');
        P1 = L(1).P_upd; e_ws = sqrt(P1(3,3) + p0^2/apr1^2); e_wc = sqrt(P1(3,3) + P1(5,5)/b1^4 + p0^2/apr1^2);
        fprintf('\n===== [%s seed %d] wall known = 4-state [dw1 dw2 dw3 b], %d records, n = %d =====\n', traj, seed, N, n);
        fprintf('H1  plain replay vs recorded sqrt(P55): max rel err %.2e -> %s\n', max(abs(sp_plain - act)./act), local_pf(max(abs(sp_plain - act)./act) < 1e-9));
        fprintf('H2  derived wall at k = 1: %.4f R (chord through contact => 1.000) -> %s\n', wwall1, local_pf(abs(wwall1 - 1) < 2e-3));
        fprintf('H3  anchor std at k = 1:  w_s %.4f R (expect ~%.4f)  contact %.4f R (expect ~%.4f) -> %s\n', sws1, e_ws, swall(1), e_wc, ...
                local_pf(abs(sws1/e_ws - 1) < 0.1 && abs(swall(1)/e_wc - 1) < 0.1));
        m0 = find(tr < SEG{1,3}, 1, 'last');
        fprintf('H4  4-state at end of hold0: contact %.5f  w_s %.5f  (prior %.5f; collapse to 0 = instrument broken) -> %s\n', sc(m0), ss(m0), act(1), ...
                local_pf(sc(m0) > 0.2*act(1) && ss(m0) > 0.2*act(1)));
        fprintf('    a_hat[0] %.4f  b_hat[0] %.4f  a''[0] %.3e  prior sqrt(P55[0]) %.4f\n', a1, b1, apr1, act(1));
        fprintf('sqrt(P55) at segment ends (both | y1 only / y2 only)\n%-8s | %-6s | %-26s | %-26s | %-26s | %s\n', 'segment', 'end t', 'current 5-slot', '4-state contact known', '4-state w_s known', 'current: contact std [R]');
        for sg = 1:size(SEG, 1)
            m = find(tr < SEG{sg,3}, 1, 'last');
            fprintf('%-8s | %-6.2f | %.5f  %.5f/%.5f | %.5f  %.5f/%.5f | %.5f  %.5f/%.5f | %.4f\n', SEG{sg,1}, tr(m), act(m), sp_cur1(m), sp_cur2(m), ...
                    sc(m), sc1(m), sc2(m), ss(m), ss1(m), ss2(m), swall(m));
        end
        fprintf('end: prior %.4f | current %.5f (/%.1f) | contact known %.5f (/%.1f) | w_s known %.5f (/%.1f)\n', ...
                act(1), act(end), act(1)/act(end), sc(end), act(1)/sc(end), ss(end), act(1)/ss(end));
        out.(traj) = struct('t', tr, 'act', act, 'known', sc, 'known_ws', ss, 'known1', sc1, 'known2', sc2, 'swall', swall, ...
                            'hbar', r.h_bar_true_out(min(kk, numel(r.h_bar_true_out))), 'SEG', {SEG});
        FIG{it} = out.(traj);
    end
    local_fig(FIG, TR, fullfile(od, 'wall_known_4state.png'));
end

function tk = local_t(x, n, anchor)   % d(anchor)/dx ; contact w_c = w_d - dw3 - a/(b(1-a)), w_s = w_c - 1/b
    if nargin < 3; anchor = 'wc'; end
    a = x(4); b = x(5); tk = zeros(1, n);
    tk(3) = -1;  tk(4) = -1/(b*(1-a)^2);
    if strcmp(anchor, 'wc'); tk(5) = a/(b^2*(1-a)); else; tk(5) = 1/(b^2*(1-a)); end
end

function sp = local_replay4(L, Qs, lockz, n, chan, anchor)   % the substituted filter: slot 4 gone, anchor known
    keep = setdiff(1:n, 4);  lz = find(ismember(keep, lockz));  m = numel(keep);  N = numel(L);
    P = L(1).P_upd(keep, keep);  sp = zeros(N, 1);  sp(1) = sqrt(P(4,4));      % keep(4) = slot 5 = b
    for k = 2:N
        Ti = inv(local_T(L(k-1).x_upd, n, anchor));  F = L(k).F;
        F4 = F(keep, keep) + F(keep, 4) * Ti(4, keep);
        P = F4 * P * F4';  if ~isempty(Qs{k}); P = P + Qs{k}(keep, keep); end;  P = 0.5*(P + P');
        Tp = inv(local_T(L(k).x_pred, n, anchor));
        for j = 1:numel(L(k).H)
            if strcmp(chan,'y1') && j ~= 1; continue; end
            if strcmp(chan,'y2') && j ~= 2; continue; end
            Hj = L(k).H{j}; if isempty(Hj); continue; end
            H4 = Hj(keep) + Hj(4) * Tp(4, keep);
            S = H4*P*H4' + L(k).R(j); K = (P*H4')/S; K(lz) = 0;
            P = (eye(m) - K*H4)*P*(eye(m) - K*H4)' + K*L(k).R(j)*K'; P = 0.5*(P + P');
        end
        sp(k) = sqrt(max(P(4,4), 0));
    end
end

function T = local_T(x, n, anchor); T = eye(n); T(4, :) = local_t(x, n, anchor); end

function sp = local_replay(L, Qs, lockz, n, chan)
    N = numel(L); P = L(1).P_upd;
    sp = zeros(N, 1); sp(1) = sqrt(P(5,5));
    for k = 2:N
        P = L(k).F * P * L(k).F'; if ~isempty(Qs{k}); P = P + Qs{k}; end; P = 0.5*(P + P');
        for j = 1:numel(L(k).H)
            if strcmp(chan,'y1') && j ~= 1; continue; end
            if strcmp(chan,'y2') && j ~= 2; continue; end
            Hj = L(k).H{j}; if isempty(Hj); continue; end
            S = Hj*P*Hj' + L(k).R(j); K = (P*Hj')/S; K(lockz) = 0;
            P = (eye(n) - K*Hj)*P*(eye(n) - K*Hj)' + K*L(k).R(j)*K'; P = 0.5*(P + P');
        end
        sp(k) = sqrt(max(P(5,5), 0));
    end
end

function local_fig(FIG, TR, fn)
    FS = 15; LFS = 11; AXLW = 1.8;
    f = figure('Units','inches','Position',[0 0 13 7.5],'Color','w','Visible','off');
    tl = tiledlayout(2, 2, 'TileSpacing','compact','Padding','compact');
    for c = 1:2
        D = FIG{c};
        nexttile(tl, c); hold on; box on;
        h1 = semilogy(D.t, D.act,   '-', 'Color', [0 0.35 0.85], 'LineWidth', 2.0);
        h2 = semilogy(D.t, D.known, '-', 'Color', [0 0.55 0.2],  'LineWidth', 2.0);
        h4 = semilogy(D.t, D.known_ws, '-', 'Color', [0.9 0.5 0],  'LineWidth', 1.6);
        h3 = yline(D.act(1), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4);
        set(gca, 'YScale', 'log', 'FontSize', FS, 'LineWidth', AXLW); xlim([D.t(1) D.t(end)]);
        for s = 2:size(D.SEG,1); xline(D.SEG{s,2}, ':', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); end
        if c == 1; ylabel('\surdP_{55}  (b)', 'FontSize', FS); end
        legend([h1 h2 h4 h3], {'current (5 slots)', '4-state, contact known', '4-state, $\bar{w}_s$ known', 'prior'}, 'Interpreter','latex', 'Location','northoutside','Orientation','horizontal','FontSize', LFS);
        nexttile(tl, c + 2); hold on; box on;
        yyaxis left;  plot(D.t, D.swall, '-', 'Color', [0 0.35 0.85], 'LineWidth', 2.0);
        ylabel('current: std of contact $\bar{w}_{\rm wall}$', 'Interpreter','latex', 'FontSize', FS - 2);
        yyaxis right; plot(D.t, D.hbar, '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.2); ylabel('$\bar{w}$ (true)', 'Interpreter','latex', 'FontSize', FS - 2);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW); xlim([D.t(1) D.t(end)]); xlabel(sprintf('t [s]   (%s)', TR{c}), 'FontSize', FS);
        ax = gca; ax.YAxis(1).Color = [0 0.35 0.85]; ax.YAxis(2).Color = [0.45 0.45 0.45];
        for s = 2:size(D.SEG,1); xline(D.SEG{s,2}, ':', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); end
    end
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure: %s\n', fn);
end

function s = local_pf(b); if b; s = 'PASS'; else; s = 'FAIL'; end; end
