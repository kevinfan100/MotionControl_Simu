% PURPOSE (2026-09-16): the sliding-window observability figure of the [1/1] two-version reading copy
%   (0916_estb_11_two_versions.tex), in the layout of obs5_ratio.png (plot_obs5_variants.m):
%     row 1  the commanded height with the segment dividers
%     row 2  plane plant: sigma_5/sigma_1 of version A (c0 state, gain computed) and B (a slot along the law)
%     row 3  cell plant : the same two curves, linearised at the true [1/1] cell law along the same commands
%   all against the window start k0, N = 500 steps, non-overlapping windows. The ratio is a rank indicator (= 0 iff a
%   direction is dead), not a quality measure -- the CRLB against the prior is printed by probe_obs_11_two_versions.
%   Both versions have five live states, so sigma_5/sigma_1 is the like-for-like quantity (no dead column to skip).
%   Linearisation data and version matrices as in probe_obs_11_two_versions.m (seed 7).
%   Output reference/eq17_analysis/derivation/figures/obs11_ratio.png | EXPIRES: with the [1/1] line
function out = plot_obs11_ratio(seed, win)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  TR = {'canon', 'meng'};  C0_CELL = 1.056;  D1_CELL = 2.197;   % Pade coefficients (c0, d1)
    NAME = {'A  c_0 state', 'B  a slot'};  COL = {[0 0.2 0.9], [0.8 0 0]};
    FS = 15;  LFS = 12;  AXLW = 1.8;  K_INIT = 4;  out = struct();
    fig = figure('Units','inches','Position',[0 0 12 9], 'Color','w', 'Visible','off');
    tl = tiledlayout(fig, 3, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        [S, tb] = local_run(TR{c}, seed);
        R = S.R;  Ts = pc.Ts;  lc = canonical_scenario(0.05, 1.1, 'deep').lambda_c;  d = 2;
        wd = S.h_bar_d_out(:);  n = numel(wd);  t = (0:n-1)'*Ts;
        R1 = (canonical_scenario(0.05, 1.1, 'deep').meas_noise_std(3) / R)^2;
        dwd = [diff(wd); 0];  dwd(1:K_INIT) = 0;
        grad = [zeros(d,1); wd(d+1:end) - wd(1:end-d)];  grad(1:K_INIT) = 0;
        P = struct();
        P.plane = struct('ah', S.a_bar_hat_out(:,3), 'd1', 1 ./ S.b_hat_out(:,3), 'dw3', S.delta_x_hat_3_out(:,3)/R, ...
                         'fb', S.f_bar_out(:,3), 'R2', S.R2_out(:,3), 'c0', 0);          % production law = [1/1] with c0 = 0, d1 = 1/b
        gg = wd - 1.0;  a_c = (gg + C0_CELL) ./ (gg + D1_CELL);
        [au, iu] = unique(P.plane.ah);  R2_of_a = @(a) interp1(au, P.plane.R2(iu), min(max(a, au(1)), au(end)));
        P.cell = struct('ah', a_c, 'd1', D1_CELL*ones(n,1), 'dw3', zeros(n,1), 'fb', dwd ./ a_c, 'R2', R2_of_a(a_c), 'c0', C0_CELL);
        starts = K_INIT:win:(n - win - 1);  nw = numel(starts);  tw = t(starts);
        pn = {'plane', 'cell'};  rat = struct();
        for ip = 1:2
            q = P.(pn{ip});  q.Fdw = q.fb + (1 - lc) * ([0; q.fb(1:end-1)] + [0; 0; q.fb(1:end-2)]);
            r = nan(nw, 2);
            for m = 1:2
                for iw = 1:nw
                    sv = local_svd(starts(iw), win, m, lc, dwd, grad, wd, q, R1);
                    r(iw, m) = sv(end) / sv(1);
                end
            end
            rat.(pn{ip}) = r;
        end
        out.(TR{c}) = struct('tw', tw, 'plane', rat.plane, 'cell', rat.cell);
        % ---- row 1: trajectory ----
        ax1 = nexttile(tl, c); hold on; box on;
        hw = plot(t(K_INIT:end), wd(K_INIT:end), '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0);
        yl = [0 1.05*max(wd)];
        for i = 1:numel(tb); plot([tb(i) tb(i)], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
        legend(hw, {TR{c}}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('w_d / R', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 t(end)]); ylim(yl);
        % ---- rows 2, 3: plane, cell ----
        AX = gobjects(1, 2);
        for ip = 1:2
            AX(ip) = nexttile(tl, 2*ip + c); hold on; box on;  H = gobjects(1, 2);
            for m = 1:2
                H(m) = semilogy(tw, max(rat.(pn{ip})(:, m), 1e-20), '-o', 'Color', COL{m}, 'LineWidth', 1.8, ...
                                'MarkerSize', 5, 'MarkerFaceColor', COL{m});
            end
            set(gca, 'YScale','log', 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW);
            ylim([1e-20 1]); set(gca, 'YTick', 10.^(-20:5:0)); xlim([0 t(end)]);
            for i = 1:numel(tb); plot([tb(i) tb(i)], [1e-20 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility','off'); end
            legend(H, {sprintf('%s   %s', pn{ip}, NAME{1}), sprintf('%s   %s', pn{ip}, NAME{2})}, 'Location','northoutside', ...
                   'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if c == 1; ylabel(sprintf('\\sigma_5 / \\sigma_1   (%s)', pn{ip}), 'FontSize', FS, 'FontWeight','bold'); end
            if ip == 1; set(gca, 'XTickLabel', []); end
        end
        xlabel(sprintf('time [s]   window start k_0, N = %d steps', win), 'FontSize', FS, 'FontWeight','bold');
        linkaxes([ax1 AX], 'x');
    end
    fn = fullfile(od, 'obs11_ratio.png');  exportgraphics(fig, fn, 'Resolution', 150);  close(fig);
    fprintf('figure: %s\n', fn);
end

function sv = local_svd(k0, win, m, lc, dwd, grad, wd, q, R1)
    Ow = zeros(2*(win+1), 5);  Phi = eye(5);
    for i = 0:win
        k = k0 + i;  M = dwd(k) + (1-lc)*q.dw3(k);  ah = q.ah(k);  om = 1 - ah;  d_ = q.d1(k);  c_ = q.c0;  F_ = q.Fdw(k);  g_ = grad(k);
        kap = d_ - c_;
        if m == 1      % A: (c0, d1) states, gain computed
            ap = om^2/kap;
            F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc + F_*ap, -F_*om/kap, F_*ah*om/kap; 0 0 0 1 0; 0 0 0 0 1];
            gd = (wd(k) - q.dw3(k) - g_ - 1.0);  ad = (gd + c_) / (gd + d_);  ad = max(min(ad, 1 - 1e-6), 0);  omd = 1 - ad;
            H = [1 0 0 0 0; 0 0 -omd^2/kap, omd/kap, -ad*omd/kap];
        else           % B: a slot along the law, d1 state, c0 fixed
            F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc -F_ 0;
                 0 0 (1-lc)*om^2/kap, 1 + om^2/kap*F_ - 2*om/kap*M, -om^2/kap^2*M;  0 0 0 0 1];
            H = [1 0 0 0 0; 0 0 0, 1 + 2*om/kap*g_, om^2/kap^2*g_];
        end
        Ow(2*i + (1:2), :) = diag([1/sqrt(R1), 1/sqrt(max(q.R2(k), eps))]) * (H * Phi);
        Phi = F * Phi;
    end
    sv = svd(Ow, 'econ');
end

function [S, tb] = local_run(traj, seed)
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
