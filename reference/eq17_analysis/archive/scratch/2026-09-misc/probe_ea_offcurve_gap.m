% PURPOSE (2026-09-10, user: "replace the e_a in the true-dynamics particle row by e_b and e_dw3, keeping the a state"):
%   linearising the true gain a(w, b, w_s) about the estimator's (w_hat, b_hat, w_s) gives
%       e_a = -a' e_dw3 + ((1-a)/b) e_b + g + (true gain minus the model curve),   g := a(w_hat, b_hat, w_s) - a_hat
%   g is how far the a STATE sits off its own curve; the estimator can compute it. This measures g along the production
%   chord-seed arm (Pf_w0_std = 0, so w_s = 1 - 1/b_chord is the known anchor), next to the actual e_a = a_true - a_hat.
%   Health checks written before the run: (H1) g at k = 1 must be ~0 (the chord seed puts a_hat[0] on the curve);
%   (H2, the 09-10 bug) the dump record k and the per-step log are offset -- the log row is found by matching the
%   LOGGED delta_w3 against the dump's own x_upd(3), and the match must be at rounding level (both are the same number, up to the um<->R
%   conversion), not merely close. The first version used row = k and read h_bar_d_out row 1, which is init-only (= 0), giving g[1] = -8.6.
%   Output printed + test_results/apd_acov_meng/ea_offcurve_gap.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = probe_ea_offcurve_gap(seed)
    if nargin < 1 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    TR = {'canon', 'meng'};  out = struct();  FIG = cell(2,1);
    for it = 1:2
        traj = TR{it};
        switch traj
            case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV; p0 = 3e-4;
                         SEG = {'hold0',0,0.5; 'far',0.5,7; 'near',7,10.5; 'hold',10.5,12.5};
            case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
                         SEG = {'hold0',0,0.5; 'descent',0.5,1.5; 'osc',1.5,3.5; 'hold',3.5,4.8};
        end
        w0 = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0, true); a0 = 1/cp;
        b_ch = (1/(1-a0) - 1)/(w0 - 1.0);  ws0 = 2 - 1/b_ch;  ws = ws0 - 1;          % the law's w_s (u = 0 point)
        cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws0,'obs_dump',true);
        o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;  evalc('R = run_formC_b(o);');
        L = obs_dump('get'); L = L([L.ax] == 3);  r = R.runs{1};  t = r.tout(:);  N = numel(L);
        kk = double([L.k]);
        hd = r.h_bar_d_out;  if size(hd, 2) == 3; hd = hd(:, 3); end
        ht = r.h_bar_true_out;  if size(ht, 2) == 3; ht = ht(:, 3); end
        d3_log = r.delta_x_hat_3_out(:, 3) / r.R;  d3_dump = arrayfun(@(q) q.x_upd(3), L(:));
        best = Inf;  off = 0;
        for oo = -3:3
            rows = kk(:) + oo;  ok = rows >= 1 & rows <= numel(d3_log);
            if ~any(ok); continue; end
            e = max(abs(d3_log(rows(ok)) - d3_dump(ok)));
            if e < best; best = e; off = oo; end
        end
        idx = min(max(kk(:) + off, 1), numel(t));  tr = t(idx);
        fprintf('\nH2 [%s] dump-to-log offset %d, max |logged dw3 - dumped dw3| = %.2e -> %s\n', traj, off, best, local_pf(best <= 1e-15));
        ah = zeros(N,1); bh = ah; d3 = ah; g = ah; ea = ah; p44 = ah;
        for k = 1:N
            x = L(k).x_upd;  ah(k) = x(4);  bh(k) = x(5);  d3(k) = x(3);  p44(k) = sqrt(L(k).P_upd(4,4));
            wh = hd(idx(k)) - d3(k);
            g(k) = (1 - 1/(bh(k)*(wh - ws))) - ah(k);
            [~, c] = calc_correction_functions(max(ht(idx(k)), 1.001), true);  ea(k) = 1/c - ah(k);
        end
        fprintf('\n===== [%s seed %d] how far the a state sits off its own curve =====\n', traj, seed);
        fprintf('H1  g at k = 1: %+.2e (chord seed on the curve => ~0) -> %s\n', g(1), local_pf(abs(g(1)) < 1e-4));
        fprintf('%-8s | %-9s | %-9s | %-9s | %-9s | %-9s | %s\n', 'segment', 'rms e_a', 'rms g', 'max |g|', 'g/e_a', 'g as e_b', 'sqrtP44 med');
        gb = g ./ ((1 - ah) ./ bh);            % the b error that would produce the same off-curve gain
        for s = 1:size(SEG,1)
            m = tr >= SEG{s,2} & tr < SEG{s,3};
            fprintf('%-8s | %.2e  | %.2e  | %.2e  | %8.3f  | %.2e  | %.2e\n', SEG{s,1}, sqrt(mean(ea(m).^2)), sqrt(mean(g(m).^2)), ...
                    max(abs(g(m))), sqrt(mean(g(m).^2))/sqrt(mean(ea(m).^2)), sqrt(mean(gb(m).^2)), median(p44(m)));
        end
        FIG{it} = struct('t', tr, 'ea', ea, 'g', g, 'gb', gb, 'hd', hd(idx), 'SEG', {SEG});  out.(traj) = FIG{it};
    end
    FS = 15; LFS = 11; AXLW = 1.8;
    f = figure('Units','inches','Position',[0 0 13 6.5],'Color','w','Visible','off');
    tl = tiledlayout(2, 2, 'TileSpacing','compact','Padding','compact');
    for c = 1:2
        D = FIG{c};
        nexttile(tl, c); hold on; box on;
        h1 = plot(D.t, D.ea, '-', 'Color', [0 0.35 0.85], 'LineWidth', 1.6);
        h2 = plot(D.t, D.g, '-', 'Color', [0.9 0.5 0], 'LineWidth', 1.8);
        yline(0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off');
        set(gca, 'FontSize', FS, 'LineWidth', AXLW); xlim([D.t(1) D.t(end)]);
        for s = 2:size(D.SEG,1); xline(D.SEG{s,2}, ':', 'Color', [0.6 0.6 0.6], 'HandleVisibility','off'); end
        legend([h1 h2], {'$e_{\bar a_w} = \bar a_w - \hat{\bar a}_w$', '$\bar a(\hat{\bar w}, \hat b, \bar w_s) - \hat{\bar a}_w$'}, ...
               'Interpreter','latex', 'Location','northoutside','Orientation','horizontal','FontSize', LFS);
        nexttile(tl, c + 2); hold on; box on;
        plot(D.t, D.hd, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.6);
        set(gca, 'FontSize', FS, 'LineWidth', AXLW); xlim([D.t(1) D.t(end)]);
        ylabel('$\bar w_d$', 'Interpreter','latex', 'FontSize', FS); xlabel(sprintf('t [s]   (%s)', TR{c}), 'FontSize', FS);
    end
    exportgraphics(f, fullfile(od, 'ea_offcurve_gap.png'), 'Resolution', 150); close(f);
    fprintf('figure: %s\n', fullfile(od, 'ea_offcurve_gap.png'));
end
function s = local_pf(b); if b; s = 'PASS'; else; s = 'FAIL'; end; end
