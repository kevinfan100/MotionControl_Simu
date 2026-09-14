function out = plot_obs_b_amodel_vs_core(seed, win)
% PURPOSE (2026-09-10, user: "run the observability for the a-model version the same way"): the 09-09 5-state
%   observability (integration/plot_obs_b_5state_two_traj.m: production linearisation point, chord seed, 500-step
%   windows, O_w = R_w^{-1/2}[H Phi], SVD) for TWO (F_e, H) pairs at the same points:
%     core   = 0907 reading copy S4/S5 (a state enters row 3 via -F_dw and y2 via H24)
%     amodel = 0910_estb_5state_amodel.tex (e_a replaced through the a model in the true dynamics AND in y2:
%              F(3,4) = 0, H(2,4) = 0) -> the a-state column of O is identically zero (the part r of e_a that the a model
%              does not explain has no outlet), so the 5-column rank is 4; the capability numbers are taken on the
%              observable columns [1 2 3 5].
%   Written before the run: amodel rank(5 cols) = 4 in EVERY window with max|O(:,4)| = 0 (else the column is wired);
%   core reproduces the 09-09 console (canon descent window sigma_min 241, CRLB_b/prior 0.10).
%   Output printed + test_results/apd_acov_meng/obs_b_amodel_vs_core.png | EXPIRES: with the b line | 產線改動不會自動跟上
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win  = 500; end
    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng'); pc = physical_constants();
    TR = {'canon', 'meng'};  NAME = {'canon deep', 'Meng ramp'};
    COL_OLD = [0 0.2 0.9]; COL_NEW = [0.9 0.5 0];
    FS = 15; LFS = 11; AXLW = 1.8; K_INIT = 4;
    fig = figure('Units','inches','Position',[0 0 11 7.6], 'Color','w', 'Visible','off');
    tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    out = struct();
    for c = 1:2
        [S, cfg, SEG] = local_run(TR{c}, seed);
        prior_b = S.ctrl_const.Pf_b_std;
        R = pc.R; Ts = pc.Ts; lc = cfg.lambda_c; d = 2;
        wd  = S.h_bar_d_out(:);  ah = S.a_bar_hat_out(:, 3);  bh = S.b_hat_out(:, 3);
        dw3 = S.delta_x_hat_3_out(:, 3) / R;  fb = S.f_bar_out(:, 3);  R2 = S.R2_out(:, 3);
        n = numel(wd);  t = (0:n-1)' * Ts;  R1 = (cfg.meas_noise_std(3) / R)^2;
        dwd = [diff(wd); 0];  dwd(1:K_INIT) = 0;
        Fdw = fb + (1 - lc) * ([0; fb(1:end-1)] + [0; 0; fb(1:end-2)]);
        grad = [zeros(d, 1); wd(d+1:end) - wd(1:end-d)];  grad(1:K_INIT) = 0;
        starts = K_INIT:win:(n - win - 1);  nw = numel(starts);  tw = t(starts);
        RES = struct();
        for m = 1:3
            rk5 = zeros(nw,1); col4 = zeros(nw,1); smin = zeros(nw,1); smax = zeros(nw,1); vb = zeros(nw,1); cb = zeros(nw,1);
            S5 = cell(nw,1); V5 = cell(nw,1); Sc = cell(nw,1); Vc = cell(nw,1);
            for q = 1:nw
                k0 = starts(q);  Ow = zeros(2*(win+1), 5);  Phi = eye(5);
                for i = 0:win
                    k = k0 + i;  M = dwd(k) + (1-lc)*dw3(k);  om = 1 - ah(k);  b_ = bh(k);  F_ = Fdw(k);  g_ = grad(k);
                    if m == 1      % core (0907 S4/S5)
                        F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc -F_ 0;
                             0 0 (1-lc)*b_*om^2, 1 + b_*om^2*F_ - 2*b_*om*M, om^2*M;  0 0 0 0 1];
                        H = [1 0 0 0 0; 0 0 0, 1 + 2*b_*om*g_, -om^2*g_];
                    else           % a model F_e (0910_estb_5state_amodel S4)
                        F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc + F_*b_*om^2, 0, -F_*om/b_;
                             0 0 (1-lc)*b_*om^2 - F_*b_^2*om^4, 1 - 2*b_*om*M, om^2*M + F_*om^3;  0 0 0 0 1];
                        if m == 2  % with the a-model H (S5)
                            H = [1 0 0 0 0; 0 0 -(1 + 2*b_*om*g_)*b_*om^2, 0, om/b_ + om^2*g_];
                        else       % m == 3: with the ORIGINAL H (user: "what if we keep the previous H?")
                            H = [1 0 0 0 0; 0 0 0, 1 + 2*b_*om*g_, -om^2*g_];
                        end
                    end
                    Ow(2*i + (1:2), :) = diag([1/sqrt(R1), 1/sqrt(max(R2(k), eps))]) * (H * Phi);
                    Phi = F * Phi;
                end
                [~, S5m, V5m] = svd(Ow, 'econ');  s5 = diag(S5m);  rk5(q) = sum(s5 > max(size(Ow)) * eps(s5(1)));  col4(q) = max(abs(Ow(:,4)));
                S5{q} = s5;  V5{q} = V5m;
                if m == 2; cols = [1 2 3 5]; else; cols = 1:5; end
                [~, Sv, V] = svd(Ow(:, cols), 'econ');  s = diag(Sv);
                smax(q) = s(1);  smin(q) = s(end);  jb = find(cols == 5);  vb(q) = abs(V(jb, end));  Sc{q} = s;  Vc{q} = V;
                if sum(s > max(size(Ow)) * eps(s(1))) == numel(cols); cb(q) = sqrt(sum((V(jb,:).^2) ./ (s.^2)')); else; cb(q) = Inf; end
            end
            RES(m).rk5 = rk5; RES(m).col4 = col4; RES(m).smin = smin; RES(m).smax = smax; RES(m).vb = vb; RES(m).cb = cb;
            RES(m).S5 = S5; RES(m).V5 = V5; RES(m).Sc = Sc; RES(m).Vc = Vc;
        end
        out.(TR{c}) = struct('t_win', tw, 'core', RES(1), 'amodel', RES(2), 'mixed', RES(3), 'seg', {SEG}, 'prior', prior_b);
        % ---- console ----
        fprintf('\n[%s] seed %d | window %d steps = %.2f s | sqrt(P55[0]) %.4f\n', TR{c}, seed, win, win*Ts, prior_b);
        fprintf('check: amodel rank of the 5-column O = %s (expect 4 everywhere) | max|O_w(:,4)| = %.1e (expect 0)\n', mat2str(unique(RES(2).rk5)'), max(RES(2).col4));
        fprintf('   %6s %-8s | %-35s | %-35s\n', '', '', 'core (a state in row 3 and y2)', 'a model (observable cols 1 2 3 5)');
        fprintf('   %6s %-8s | %5s %9s %9s %9s | %5s %9s %9s %9s\n', 't0','segment','rank','sig_min','|v_b|','CRLB/pr','rank','sig_min','|v_b|','CRLB/pr');
        for q = 1:nw
            sg = 'n/a';
            for r2 = 1:size(SEG,1); if tw(q) >= SEG{r2,2} && tw(q) < SEG{r2,3}; sg = SEG{r2,1}; end; end
            fprintf('   %6.2f %-8s | %5d %9.2e %9.3f %9.3f | %5d %9.2e %9.3f %9.3f\n', tw(q), sg, ...
                    RES(1).rk5(q), RES(1).smin(q), RES(1).vb(q), RES(1).cb(q)/prior_b, RES(2).rk5(q), RES(2).smin(q), RES(2).vb(q), RES(2).cb(q)/prior_b);
        end
        fprintf('windows where the data beat the prior (CRLB_b < sqrt(P55[0])): core %d/%d | a model %d/%d\n', ...
                sum(RES(1).cb < prior_b), nw, sum(RES(2).cb < prior_b), nw);
        fprintf('best window CRLB_b/prior: core %.3f | a model %.3f\n', min(RES(1).cb)/prior_b, min(RES(2).cb)/prior_b);
        % ---- figure ----
        tb = local_bounds(cfg);
        ax1 = nexttile(c); hold on;
        hw = plot(t(K_INIT:end), wd(K_INIT:end), '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0); yl = [0 1.05*max(wd)];
        for i = 1:numel(tb); plot([tb(i) tb(i)], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(hw, {NAME{c}}, 'Location','northoutside','Orientation','horizontal', 'FontSize', LFS, 'FontWeight','bold', 'Box','on');
        if c == 1; ylabel('w_d / R', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 t(end)]); ylim(yl);
        s1c = cellfun(@(x) x(1), RES(1).S5);  s5c = cellfun(@(x) x(end), RES(1).S5);
        s1a = cellfun(@(x) x(1), RES(2).S5);  s5a = cellfun(@(x) x(end), RES(2).S5);
        s4a = cellfun(@(x) x(end), RES(2).Sc);  s1a4 = cellfun(@(x) x(1), RES(2).Sc);   % a model without the dead a_w column
        FL = 1e-7;  zr = (s5a ./ s1a) < 1e3*eps;
        ax2 = nexttile(2 + c); hold on;
        h1 = semilogy(tw, s5c ./ s1c, '-o', 'Color', COL_OLD, 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', COL_OLD);
        h2 = semilogy(tw(zr), FL*ones(sum(zr),1), 's', 'Color', COL_NEW, 'MarkerSize', 7, 'MarkerFaceColor', COL_NEW);
        s1m = cellfun(@(x) x(1), RES(3).S5);  s5m = cellfun(@(x) x(end), RES(3).S5);
        h5 = semilogy(tw, s5m ./ s1m, '-^', 'Color', [0 0.6 0.25], 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', [0 0.6 0.25]);
        set(gca, 'YScale', 'log'); ylim([FL 1]); set(gca, 'YTick', 10.^(-7:0));
        for i = 1:numel(tb); plot([tb(i) tb(i)], [FL 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend([h1 h2 h5], {'original', 'a model (= 0, drawn at 10^{-7})', 'a-model F_e + original H'}, ...
               'Location','northoutside','NumColumns', 2, 'FontSize',LFS-1,'FontWeight','bold','Box','on');
        if c == 1; ylabel('\sigma_5 / \sigma_1   [-]', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 t(end)]);
        xlabel(sprintf('time [s]   window start k_0, N = %d steps', win), 'FontSize', FS, 'FontWeight','bold');
        ax3 = ax2;
        linkaxes([ax1 ax2 ax3], 'x');
    end
    exportgraphics(fig, fullfile(od, 'obs_b_amodel_vs_core_ratio3.png'), 'Resolution', 150);
    fprintf('\nsaved %s\n', fullfile(od, 'obs_b_amodel_vs_core_ratio3.png'));
end

function [S, cfg, SEG] = local_run(traj, seed)
    pc = physical_constants();
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                        'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
            cfg = canonical_scenario(0.05, 1.1, 'deep');
            f = fieldnames(OV); for i = 1:numel(f); cfg.(f{i}) = OV.(f{i}); end
            SEG = {'hold0',0,0.5; 'far',0.5,7; 'near',7,10.5; 'hold',10.5,12.5};  p0 = 3e-4;
        otherwise
            OV = struct(); cfg = canonical_scenario(0.05, 1.1, 'deep');
            SEG = {'hold0',0,0.5; 'descent',0.5,1.5; 'osc',1.5,3.5; 'hold',3.5,4.8};  p0 = 1e-5;
    end
    w0 = cfg.h_init / pc.R;  [~, cp] = calc_correction_functions(w0, true);  a0 = 1/cp;
    b_ch = (1/(1 - a0) - 1) / (w0 - 1.0);  ws_ch = 1 + 1.0 - 1/b_ch;
    cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws_ch);
    o  = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep', ...
                'verbose',false,'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;
    evalc('Rr = run_formC_b(o);');
    S = Rr.runs{1};
end

function tb = local_bounds(cfg)
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override;
    if cfg.amplitude > 0; tb = [t1 t2 t2 + cfg.n_cycles / cfg.frequency]; else; tb = [t1 t2]; end
end
