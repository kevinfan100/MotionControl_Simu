% PURPOSE (2026-09-10, user: "does putting e_b into F_e(3,5) improve b?"): information only, from the production
%   linearisation (obs_dump), no controller change. Same arm as audit_fehrq_consistency.m (chord seed, seed 7).
%   The (B') rows of 0910_estb_delayed_gain_row.tex are added to the recorded F as a perturbation of e_Dw:
%       D1 = -(1-lc) a' f[k-2],  D2 = -(1-lc) a' f[k-1],  D3 = (1-lc) a' (f[k-1] + f[k-2]),
%       D5 = -(1-lc)(1-a)^2 ( f[k-1] Dwh[k-1] + f[k-2](Dwh[k-1] + Dwh[k-2]) )
%       F(3,[1 2 3 5]) -= D,  F(4,[1 2 3 5]) += a' D          (row 3 = e3 - e_Dw, row 4 = ... + a' e_Dw)
%   f = the deterministic-mirror force the filter uses (recovered from F_dw = -F(3,4) by the F_dw recursion),
%   Dwh = the filter's own step (F(4,5)/(1-a)^2). Replay P with the recovered Q, F vs F + (B').
%   Health checks written before the run: (H1) plain replay = recorded sqrt(P55) to 1e-9; (H2) F(3,3) = lambda_c = 0.7;
%   (H3) recovered force ~0 in the first hold; (H4) |Dwh| peak ~ commanded peak step (canon 2 pi A f Ts = 4.4e-3 R).
%   Output printed + test_results/apd_acov_meng/delayed_gain_row_bprime.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = probe_delayed_gain_row_bprime(seed)
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
        % ---- rebuild the ingredients: record j is the transition (j-1) -> j ----
        lc = L(2).F(3,3);  omlc = 1 - lc;
        Fdw = arrayfun(@(s) -s.F(3,4), L); Fdw = Fdw(:);          % F_dw at time j-1
        f = zeros(N, 1);                                          % deterministic force at time j-1
        for j = 1:N
            fm1 = 0; fm2 = 0; if j > 1; fm1 = f(j-1); end; if j > 2; fm2 = f(j-2); end
            f(j) = Fdw(j) - omlc*(fm1 + fm2);
        end
        M = zeros(N, 1);                                          % filter's own step at time j-1
        for j = 2:N; a = L(j-1).x_upd(4); M(j) = L(j).F(4,5) / (1 - a)^2; end
        % ---- (B') perturbation of F ----
        Fb = cell(N, 1);  f35 = zeros(N, 1);  ind = zeros(N, 1);
        for j = 1:N
            F = L(j).F;
            if j >= 4
                a = L(j-1).x_upd(4); b = L(j-1).x_upd(5); apr = b*(1-a)^2;
                D = zeros(1, n);
                D(1) = -omlc*apr*f(j-2);  D(2) = -omlc*apr*f(j-1);  D(3) = omlc*apr*(f(j-1) + f(j-2));
                D(5) = -omlc*(1-a)^2*( f(j-1)*M(j-1) + f(j-2)*(M(j-1) + M(j-2)) );
                F(3,:) = F(3,:) - D;  F(4,:) = F(4,:) + apr*D;
                f35(j) = -D(5);  ind(j) = F(3,4) * L(j).F(4,5);     % one-step path e_b -> e_a -> e_dw3
            end
            Fb{j} = F;
        end
        sp_plain = local_replay(L, Qs, lockz, n, cellfun(@(s) s, {L.F}, 'UniformOutput', false));
        sp_b     = local_replay(L, Qs, lockz, n, Fb);
        % ---- report ----
        m0 = find(tr < SEG{1,3}, 1, 'last');  osc = tr > SEG{2,2} & tr < SEG{3,3};
        fprintf('\n===== [%s seed %d] F_e(3,5) from the delayed gains, %d records =====\n', traj, seed, N);
        fprintf('H1  plain replay vs recorded sqrt(P55): max rel err %.2e -> %s\n', max(abs(sp_plain - act)./act), local_pf(max(abs(sp_plain - act)./act) < 1e-9));
        fprintf('H2  lambda_c from F(3,3) = %.4f -> %s\n', lc, local_pf(abs(lc - 0.7) < 1e-12));
        fprintf('H3  |force| max in hold0 %.2e vs max overall %.2e -> %s\n', max(abs(f(1:m0))), max(abs(f)), local_pf(max(abs(f(1:m0))) < 1e-3*max(abs(f))));
        fprintf('H4  |Dwh| max %.2e R/step (canon commanded peak 4.4e-3)\n', max(abs(M)));
        fprintf('new F(3,5): max |.| %.3e | median over motion %.3e | one-step path |F(3,4)F(4,5)| median %.3e | ratio median %.2f\n', ...
                max(abs(f35)), median(abs(f35(osc))), median(abs(ind(osc))), median(abs(f35(osc))./max(abs(ind(osc)), eps)));
        fprintf('%-8s | %-6s | %-12s | %-12s | %s\n', 'segment', 'end t', 'sqrt(P55) (A)', 'with (B'')', 'change');
        for sg = 1:size(SEG, 1)
            m = find(tr < SEG{sg,3}, 1, 'last');
            fprintf('%-8s | %-6.2f | %.6f     | %.6f     | %+.3f%%\n', SEG{sg,1}, tr(m), sp_plain(m), sp_b(m), 100*(sp_b(m)/sp_plain(m) - 1));
        end
        out.(traj) = struct('t', tr, 'A', sp_plain, 'B', sp_b, 'f35', f35, 'ind', ind, 'SEG', {SEG});
        FIG{it} = out.(traj);
    end
    local_fig(FIG, TR, fullfile(od, 'delayed_gain_row_bprime.png'));
end

function sp = local_replay(L, Qs, lockz, n, Fs)
    N = numel(L); P = L(1).P_upd; sp = zeros(N, 1); sp(1) = sqrt(P(5,5));
    for k = 2:N
        P = Fs{k} * P * Fs{k}'; if ~isempty(Qs{k}); P = P + Qs{k}; end; P = 0.5*(P + P');
        for j = 1:numel(L(k).H)
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
        h1 = plot(D.t, D.A, '-', 'Color', [0 0.35 0.85], 'LineWidth', 2.2);
        h2 = plot(D.t, D.B, '--', 'Color', [0.9 0.5 0], 'LineWidth', 1.8);
        set(gca, 'YScale', 'log', 'FontSize', FS, 'LineWidth', AXLW); xlim([D.t(1) D.t(end)]);
        for s = 2:size(D.SEG,1); xline(D.SEG{s,2}, ':', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); end
        if c == 1; ylabel('$\sqrt{P_{55}}$ ($\hat b$)', 'Interpreter','latex', 'FontSize', FS); end
        legend([h1 h2], {'(A) current $F_e$', '(B$''$) with $F_e(3,5)$'}, 'Interpreter','latex', 'Location','northoutside','Orientation','horizontal','FontSize', LFS);
        nexttile(tl, c + 2); hold on; box on;
        h3 = plot(D.t, abs(D.ind), '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.4);
        h4 = plot(D.t, abs(D.f35), '-', 'Color', [0.9 0.5 0], 'LineWidth', 1.8);
        top = max([abs(D.f35); abs(D.ind)]);
        set(gca, 'YScale', 'log', 'FontSize', FS, 'LineWidth', AXLW); xlim([D.t(1) D.t(end)]); ylim([1e-4*top, 2*top]);
        xlabel(sprintf('t [s]   (%s)', TR{c}), 'FontSize', FS);
        if c == 1; ylabel('$|F_e|$ entry', 'Interpreter','latex', 'FontSize', FS); end
        legend([h3 h4], {'$|F_e(3,4)\,F_e(4,5)|$', 'new $|F_e(3,5)|$'}, 'Interpreter','latex', 'Location','northoutside','Orientation','horizontal','FontSize', LFS);
    end
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('figure: %s\n', fn);
end

function s = local_pf(b); if b; s = 'PASS'; else; s = 'FAIL'; end; end
