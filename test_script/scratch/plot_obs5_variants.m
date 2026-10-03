% PURPOSE (2026-09-14): the two observability figures of the comparison reading copy (0915_estb_variants_compare.tex),
%   from one pair of runs (canon and Meng, production chord-seed arm, seed 7, 500-step windows):
%     obs5_ratio.png    0907 house layout -- row 1 the commanded height with the segment dividers; row 2 sigma_5/sigma_1
%                       for the two 5-state versions (the substituted one sits at the floor: its sigma_5 is exactly 0,
%                       the dead a_w direction); row 3 the like-for-like comparison, the weakest direction the data can
%                       actually see -- sigma_5/sigma_1 for the original against sigma_4/sigma_1 for the substituted
%                       (the 4-state's sigma_4/sigma_1 is the same curve, it is that version with the dead column
%                       deleted), all against the window start
%     obs5_sigma_V.png  |V| at one window per trajectory (canon descent t = 1.25 s, Meng near wall t = 8.75 s), with
%                       each direction labelled by its own sigma
%   Three versions, all built on the SAME linearisation point (a real production run):
%     original     0907 core: a_w is a state of its own
%     substituted  e_a -> -a' e_dw3 + ((1-a)/b) e_b in F_e AND in the y2 row, a_w kept as a slot
%     4-state      no a_w slot, the gain is the law value at the estimator's own height
%   The ratio is a rank indicator only (= 0 iff a column is dead), not a quality measure -- see 09-09: the windows that
%   beat the prior and those that lose overlap in it.
%   Output reference/eq17_analysis/derivation/figures/ | EXPIRES: with the b line | 產線改動不會自動跟上
function out = plot_obs5_variants(seed, win)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  TR = {'canon', 'meng'};  T_SHOW = [1.25, 8.75];
    NAME = {'original', 'substituted', '4-state'};  COL = {[0 0.2 0.9], [0.9 0.5 0], [0.6 0 0.6]};
    LAB = {'\deltaw_1', '\deltaw_2', '\deltaw_3', 'a_w', 'b'};
    FS = 15;  LFS = 12;  AXLW = 1.8;  K_INIT = 4;  out = struct();
    figA = figure('Units','inches','Position',[0 0 12 9], 'Color','w', 'Visible','off');
    tlA = tiledlayout(figA, 3, 2, 'TileSpacing','compact', 'Padding','compact');
    figB = figure('Units','inches','Position',[0 0 12 6.4], 'Color','w', 'Visible','off');
    tlB = tiledlayout(figB, 2, 3, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        [S, cfg, tb] = local_run(TR{c}, seed);
        R = S.R;  Ts = pc.Ts;  lc = canonical_scenario(0.05, 1.1, 'deep').lambda_c;  d = 2;
        wd = S.h_bar_d_out(:);  ah = S.a_bar_hat_out(:,3);  bh = S.b_hat_out(:,3);  dw3 = S.delta_x_hat_3_out(:,3)/R;
        fb = S.f_bar_out(:,3);  R2 = S.R2_out(:,3);  n = numel(wd);  t = (0:n-1)'*Ts;
        R1 = (canonical_scenario(0.05, 1.1, 'deep').meas_noise_std(3) / R)^2;
        dwd = [diff(wd); 0];  dwd(1:K_INIT) = 0;
        Fdw = fb + (1 - lc) * ([0; fb(1:end-1)] + [0; 0; fb(1:end-2)]);
        grad = [zeros(d,1); wd(d+1:end) - wd(1:end-d)];  grad(1:K_INIT) = 0;
        starts = K_INIT:win:(n - win - 1);  nw = numel(starts);  tw = t(starts);
        rat = nan(nw, 3);  rat4 = nan(nw, 3);  SV = cell(1, 3);  VV = cell(1, 3);
        kq = max(K_INIT, round(T_SHOW(c)/Ts));  [~, iq] = min(abs(starts - kq));  starts(iq) = kq;
        for m = 1:3
            sv_all = nan(nw, 5);
            for q = 1:nw
                [sv, V] = local_svd(starts(q), win, m, lc, dwd, dw3, ah, bh, Fdw, grad, R1, R2);
                sv_all(q, 1:numel(sv)) = sv(:).';
                rat(q, m) = sv(end) / sv(1);
                rat4(q, m) = sv(min(4, numel(sv))) / sv(1);       % the weakest LIVE direction of this version
                if q == iq; SV{m} = sv;  VV{m} = V; end
            end
            out.(TR{c})(m) = struct('tw', tw, 'ratio', rat(:, m), 'ratio4', rat4(:, m), 'sv', SV{m}, 'V', VV{m}, ...
                                    't_show', t(starts(iq)));
        end
        % ---- figure A ----
        ax1 = nexttile(tlA, c); hold on; box on;
        hw = plot(t(K_INIT:end), wd(K_INIT:end), '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0);
        yl = [0 1.05*max(wd)];
        for i = 1:numel(tb); plot([tb(i) tb(i)], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(hw, {TR{c}}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('w_d / R', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 t(end)]); ylim(yl);
        ax2 = nexttile(tlA, 2 + c); hold on; box on;  H = gobjects(1, 2);
        for m = 1:2
            H(m) = semilogy(tw, max(rat(:, m), 1e-20), '-o', 'Color', COL{m}, 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', COL{m});
        end
        set(gca, 'YScale','log', 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW);
        ylim([1e-20 1]); set(gca, 'YTick', 10.^(-20:5:0)); xlim([0 t(end)]);
        for i = 1:numel(tb); plot([tb(i) tb(i)], [1e-20 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(H, NAME(1:2), 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('\sigma_5 / \sigma_1', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca, 'XTickLabel', []);
        ax3 = nexttile(tlA, 4 + c); hold on; box on;  H = gobjects(1, 3);
        H(1) = semilogy(tw, rat(:, 1),  '-o', 'Color', COL{1}, 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', COL{1});
        H(2) = semilogy(tw, rat4(:, 2), '-o', 'Color', COL{2}, 'LineWidth', 1.8, 'MarkerSize', 5, 'MarkerFaceColor', COL{2});
        H(3) = semilogy(tw, rat4(:, 3), '--', 'Color', COL{3}, 'LineWidth', 1.6);
        set(gca, 'YScale','log', 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW);
        ylim([1e-6 1]); set(gca, 'YTick', 10.^(-6:2:0)); xlim([0 t(end)]);
        for i = 1:numel(tb); plot([tb(i) tb(i)], [1e-6 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(H, {sprintf('%s  \\sigma_5/\\sigma_1', NAME{1}), sprintf('%s  \\sigma_4/\\sigma_1', NAME{2}), ...
                   sprintf('%s  \\sigma_4/\\sigma_1', NAME{3})}, 'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('weakest live direction', 'FontSize', FS-2, 'FontWeight','bold'); end
        xlabel(sprintf('time [s]   window start k_0, N = %d steps', win), 'FontSize', FS, 'FontWeight','bold');
        linkaxes([ax1 ax2 ax3], 'x');
        % ---- figure B ----
        for m = 1:3
            sv = SV{m}(:).';  V = VV{m};  nn = numel(sv);  lab = LAB;
            if m == 3; lab = LAB([1 2 3 5]); end
            nexttile(tlB); hold on; box on;
            imagesc(abs(V), [0 1]);  colormap(flipud(gray));
            for i = 1:nn
                for j = 1:nn
                    tc = 'k';  if abs(V(i,j)) > 0.55; tc = 'w'; end
                    text(j, i, sprintf('%.2f', V(i,j)), 'HorizontalAlignment','center', 'FontSize', 11, 'FontWeight','bold', 'Color', tc);
                end
            end
            xt = cell(1, nn);
            for j = 1:nn; xt{j} = sprintf('v_%d\\newline%s', j, local_num(sv(j))); end
            set(gca, 'YDir','reverse', 'XTick', 1:nn, 'YTick', 1:nn, 'YTickLabel', lab, 'XTickLabel', xt, ...
                     'XTickLabelRotation', 0, 'FontSize', 13, 'FontWeight','bold', 'LineWidth', 1.4, 'TickLength', [0 0]);
            xlabel('\sigma_i', 'FontSize', 13, 'FontWeight','bold');
            xlim([0.5 nn+0.5]);  ylim([0.5 nn+0.5]);
            h = plot(NaN, NaN, 'w.');
            legend(h, {sprintf('%s   %s   t_0 = %.2f s', TR{c}, NAME{m}, t(starts(iq)))}, 'Location','northoutside', ...
                   'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        end
    end
    exportgraphics(figA, fullfile(od, 'obs5_ratio.png'), 'Resolution', 150);  close(figA);
    exportgraphics(figB, fullfile(od, 'obs5_sigma_V.png'), 'Resolution', 150);  close(figB);
    fprintf('figures: %s\n         %s\n', fullfile(od, 'obs5_ratio.png'), fullfile(od, 'obs5_sigma_V.png'));
end

function [sv, V] = local_svd(k0, win, m, lc, dwd, dw3, ah, bh, Fdw, grad, R1, R2)
    nx = 5 - (m == 3);  Ow = zeros(2*(win+1), nx);  Phi = eye(nx);
    for i = 0:win
        k = k0 + i;  M = dwd(k) + (1-lc)*dw3(k);  om = 1 - ah(k);  b_ = bh(k);  F_ = Fdw(k);  g_ = grad(k);
        switch m
            case 1
                F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc -F_ 0;
                     0 0 (1-lc)*b_*om^2, 1 + b_*om^2*F_ - 2*b_*om*M, om^2*M;  0 0 0 0 1];
                H = [1 0 0 0 0; 0 0 0, 1 + 2*b_*om*g_, -om^2*g_];
            case 2
                F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc + F_*b_*om^2, 0, -F_*om/b_;
                     0 0 (1-lc)*b_*om^2 - F_*b_^2*om^4, 1 - 2*b_*om*M, om^2*M + F_*om^3;  0 0 0 0 1];
                H = [1 0 0 0 0; 0 0 -(1 + 2*b_*om*g_)*b_*om^2, 0, om/b_ + om^2*g_];
            otherwise
                om_d = om + b_*om^2*g_;
                F = [0 1 0 0; 0 0 1 0; 0 0 lc + F_*b_*om^2, -F_*om/b_;  0 0 0 1];
                H = [1 0 0 0; 0 0 -b_*om_d^2, om_d/b_];
        end
        Ow(2*i + (1:2), :) = diag([1/sqrt(R1), 1/sqrt(max(R2(k), eps))]) * (H * Phi);
        Phi = F * Phi;
    end
    [~, Sm, V] = svd(Ow, 'econ');  sv = diag(Sm);
end

function [S, cfg, tb] = local_run(traj, seed)
    pc = physical_constants();
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                        'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
            cfg = canonical_scenario(0.05, 1.1, 'deep');
            f = fieldnames(OV); for i = 1:numel(f); cfg.(f{i}) = OV.(f{i}); end
            p0 = 3e-4;  tb = [0.5 7 10.5];
        otherwise
            OV = struct(); cfg = canonical_scenario(0.05, 1.1, 'deep');  p0 = 1e-5;  tb = [0.5 1.5 3.5];
    end
    w0 = cfg.h_init / pc.R;  [~, cp] = calc_correction_functions(w0, true);  a0 = 1/cp;
    b_ch = (1/(1 - a0) - 1) / (w0 - 1.0);
    cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',2 - 1/b_ch);
    o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false, ...
               'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;  evalc('Rr = run_formC_b(o);');  S = Rr.runs{1};
end

function s = local_num(x)
    if x < 1e-6;      s = '0';
    elseif x >= 1e4;  s = sprintf('%.1fe4', x/1e4);
    else;             s = sprintf('%.0f', x); end
end
