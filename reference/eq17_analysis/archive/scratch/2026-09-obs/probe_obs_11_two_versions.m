% PURPOSE (2026-09-16, user: "[1/1] law, two versions -- with and without an a state, pure (no m1 m2) -- same style as
%   the e_a / e_dw3 comparison, show V and max/min"): the whitened observability SVD of the two 5-state reading copies of
%   0916_estb_11_two_versions.tex, one window per trajectory segment.
%     A   x = [dw1 dw2 dw3 c0 b]   contact offset c0 as a constant state, gain COMPUTED from the law (no a slot)
%     B   x = [dw1 dw2 dw3 a_w b]  a slot carried along the law a' = b (1-a)^2, c0 only in the seed (production structure)
%   Two plants on the same commanded trajectories (canon deep, Meng ramp), w_bar_s = 1 = actual wall:
%     plane  c0 = 0, linearised at the production run's own estimates (a_hat, b_hat, dw3_hat, f_bar, R2) -- so version B
%            must reproduce the 09-09 / 09-14 console (canon descent sigma = [13526 948 680 680 241], CRLB_b/prior 0.105):
%            that is the health check of the wiring
%     cell   c0 = 1.056, 1/b = 1.141 (Fig. 11 [1/1] fit), linearised at the TRUE law values along the same commands
%            (dw3 = 0, f_bar = dw_d / a, R2 read off the plane run as a function of a_hat)
%   Windows: descent (t = 1.25 s canon / 8.75 s Meng, 500 steps, as before) and the final hold (last 500 steps).
%   Printed per case: rank, sigma_1..5, sigma_max/sigma_min, V (rows = states), 1/sigma, CRLB of slots 4 and 5 against
%   their priors, negative control (synthetic zero column, must be Inf).
%   Output: console + reference/eq17_analysis/derivation/figures/obs11_sigma.png | EXPIRES: with the [1/1] line
function out = probe_obs_11_two_versions(seed, win)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  TR = {'canon', 'meng'};  T_SHOW = [1.25, 8.75];
    C0_CELL = 1.056;  D1_CELL = 2.197;  SIG_C0 = 0.5;          % cell [1/1] band fit (09-16), Pade coefficients (c0, d1); c0 prior = half the plane-cell spread, nominal
    SIG_B_OLD = 0.0389;                                          % the derived prior of the old shape constant; mapped onto d1 by d1 = c0 + 1/b: sigma_d1 = sigma_b (d1 - c0)^2
    LABA = {'dw1','dw2','dw3','c0','d1'};  LABB = {'dw1','dw2','dw3','a_w','d1'};
    out = struct();  SV = struct();
    for c = 1:2
        switch TR{c}
            case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                                     'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg = OV; p0 = 3e-4;
            otherwise;   OV = struct(); cfg = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
        end
        w0 = cfg.h_init / pc.R;  [~, cp] = calc_correction_functions(w0, true);  a0 = 1/cp;
        b_ch = (1/(1 - a0) - 1) / (w0 - 1.0);
        cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',2 - 1/b_ch);
        o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false, ...
                   'seeds',seed,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;  evalc('Rr = run_formC_b(o);');  S = Rr.runs{1};
        R = S.R;  Ts = pc.Ts;  lc = canonical_scenario(0.05, 1.1, 'deep').lambda_c;  d = 2;  K_INIT = 4;
        wd = S.h_bar_d_out(:);  n = numel(wd);  t = (0:n-1)'*Ts;
        R1 = (canonical_scenario(0.05, 1.1, 'deep').meas_noise_std(3) / R)^2;
        dwd = [diff(wd); 0];  dwd(1:K_INIT) = 0;
        grad = [zeros(d,1); wd(d+1:end) - wd(1:end-d)];  grad(1:K_INIT) = 0;
        % --- two plants, Pade coefficients (c0, d1) --------------------------------------------------------
        P = struct();
        % plane: the run's own estimates; production's law 1/(1-a) = b (w - w0) is the [1/1] with c0 = 0, d1 = 1/b
        P.plane = struct('ah', S.a_bar_hat_out(:,3), 'd1', 1 ./ S.b_hat_out(:,3), 'dw3', S.delta_x_hat_3_out(:,3)/R, ...
                         'fb', S.f_bar_out(:,3), 'R2', S.R2_out(:,3), 'c0', 0);
        % cell: true [1/1] values along the same commands
        gg = wd - 1.0;  a_c = (gg + C0_CELL) ./ (gg + D1_CELL);
        [au, iu] = unique(P.plane.ah);  R2_of_a = @(a) interp1(au, P.plane.R2(iu), min(max(a, au(1)), au(end)));
        P.cell = struct('ah', a_c, 'd1', D1_CELL*ones(n,1), 'dw3', zeros(n,1), 'fb', dwd ./ a_c, 'R2', R2_of_a(a_c), 'c0', C0_CELL);
        WIN = struct('descent', max(K_INIT, round(T_SHOW(c)/Ts)), 'hold', n - win - 1);
        pn = fieldnames(P);  wn = fieldnames(WIN);
        for ip = 1:numel(pn)
            q = P.(pn{ip});  Fdw = q.fb + (1 - lc) * ([0; q.fb(1:end-1)] + [0; 0; q.fb(1:end-2)]);
            for iw = 1:numel(wn)
                k0 = WIN.(wn{iw});
                fprintf('\n================ %s | %s plant | %s window: %d steps from t = %.2f s (seed %d) ================\n', ...
                        TR{c}, pn{ip}, wn{iw}, win, t(k0), seed);
                for m = 1:2
                    Ow = zeros(2*(win+1), 5);  Phi = eye(5);
                    for i = 0:win
                        k = k0 + i;  M = dwd(k) + (1-lc)*q.dw3(k);  ah = q.ah(k);  om = 1 - ah;  d_ = q.d1(k);  c_ = q.c0;  F_ = Fdw(k);  g_ = grad(k);
                        kap = d_ - c_;                                     % d1 - c0, the law's slope scale
                        if m == 1      % A: (c0, d1) states, gain computed; y2 read at the delayed height exactly (tex sections 4, 7)
                            ap = om^2/kap;
                            F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc + F_*ap, -F_*om/kap, F_*ah*om/kap; 0 0 0 1 0; 0 0 0 0 1];
                            gd = (wd(k) - q.dw3(k) - g_ - 1.0);                                   % delayed-height gap, w_s = 1
                            ad = (gd + c_) / (gd + d_);  ad = max(min(ad, 1 - 1e-6), 0);  omd = 1 - ad;
                            H = [1 0 0 0 0; 0 0 -omd^2/kap, omd/kap, -ad*omd/kap];
                        else           % B: a slot along the law, d1 state, c0 fixed (tex sections 6, 7)
                            F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc -F_ 0;
                                 0 0 (1-lc)*om^2/kap, 1 + om^2/kap*F_ - 2*om/kap*M, -om^2/kap^2*M;  0 0 0 0 1];
                            H = [1 0 0 0 0; 0 0 0, 1 + 2*om/kap*g_, om^2/kap^2*g_];
                        end
                        Ow(2*i + (1:2), :) = diag([1/sqrt(R1), 1/sqrt(max(q.R2(k), eps))]) * (H * Phi);
                        Phi = F * Phi;
                    end
                    LAB = LABA;  if m == 2; LAB = LABB; end
                    [~, Sm, Vm] = svd(Ow, 'econ');  sv = diag(Sm);  rk = sum(sv > max(size(Ow))*eps(sv(1)));
                    Own = [Ow, zeros(size(Ow,1), 1)];                                  % synthetic negative control column
                    G = Own'*Own;  [Ve, De] = eig(0.5*(G+G'));  dv = real(diag(De));  tol = max(dv)*1e-15;  ok = dv > tol;
                    crlb = @(j) local_crlb(real(Ve(j,:)).^2, dv, ok);
                    c4 = crlb(4);  c5 = crlb(5);  cn = crlb(6);
                    kap0 = q.d1(k0) - q.c0;  sig_d1 = SIG_B_OLD * kap0^2;                          % prior of d1 mapped from the derived old prior
                    if m == 1; pr4 = SIG_C0; else; pr4 = (1 - q.ah(k0))/kap0 * SIG_C0; end          % c0 prior mapped into the a slot for B (tex, seed and prior)
                    NAME = {'A: (c0, d1) states, gain computed', 'B: a slot along the law, d1 state'};
                    fprintf('\n--- %s | rank %d of 5 | sigma_max/sigma_min = %.3g | CRLB(%s) = %.4g (prior %.4g, ratio %.3f) | CRLB(d1) = %.4g (prior %.4f, ratio %.3f) | neg ctrl %.3g\n', ...
                            NAME{m}, rk, sv(1)/sv(end), LAB{4}, c4, pr4, c4/pr4, c5, sig_d1, c5/sig_d1, cn);
                    fprintf('  sigma  :%s\n', sprintf('%12.4g', sv));
                    fprintf('  1/sigma:%s\n', sprintf('%12.4g', 1./sv));
                    fprintf('  V (rows = state, cols = v1..v5):\n');
                    for j = 1:5; fprintf('   %-4s %s\n', LAB{j}, sprintf('%9.3f', Vm(j,:))); end
                    out.(TR{c}).(pn{ip}).(wn{iw})(m) = struct('sv', sv, 'V', Vm, 'rank', rk, 'crlb4', c4, 'crlb5', c5, 'neg', cn, 't', t(k0));
                    SV.(TR{c}).(pn{ip}).(wn{iw})(:, m) = sv;
                end
            end
        end
    end
    % --- figure: sigma spectra, A vs B, per trajectory / plant / window ---------------------------------
    COL_A = [0 0.2 0.9];  COL_B = [0.8 0 0];  FS = 13;  LFS = 11;  AXLW = 1.6;
    f = figure('Position', [40 40 1500 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');  k = 0;
    for c = 1:2
        for ip = 1:2
            for iw = 1:2
                k = k + 1;  nexttile(k); hold on; box on;
                s = SV.(TR{c}).(pn{ip}).(wn{iw});
                hA = plot(1:5, s(:,1), 'o-', 'Color', COL_A, 'LineWidth', 1.8, 'MarkerFaceColor', COL_A, 'DisplayName', 'A: $c_0$ state');
                hB = plot(1:5, s(:,2), 's--', 'Color', COL_B, 'LineWidth', 1.8, 'MarkerFaceColor', COL_B, 'DisplayName', 'B: $a$ slot');
                set(gca, 'YScale', 'log', 'XTick', 1:5, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;
                xlim([0.5 5.5]);
                title(sprintf('%s, %s, %s', TR{c}, pn{ip}, wn{iw}), 'FontSize', FS, 'FontWeight', 'normal');
                if iw == 1 && ip == 1; ylabel('$\sigma_i$', 'Interpreter', 'latex', 'FontSize', FS + 4); end
                if c == 2; xlabel('$i$', 'Interpreter', 'latex', 'FontSize', FS + 2); end
                if k == 1; legend([hA hB], 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on'); end
            end
        end
    end
    fn = fullfile(od, 'obs11_sigma.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('\nfigure: %s\n', fn);
end

function cval = local_crlb(w, dv, ok)
    if any(w(~ok) > 1e-12); cval = Inf; else; cval = sqrt(sum(w(ok) ./ dv(ok).')); end
end
