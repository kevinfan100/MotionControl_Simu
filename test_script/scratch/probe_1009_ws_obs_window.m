% STATUS: SCRATCH -- 2026-10-11, 1009_conly_wall_position.tex, descent only (user 10-11: "singular values, fixed window, square matrix").
%   Run: run_1009_ws 'desc' settings (22.2 -> 1.10 R in 1 s, parked 1 s; c and w_s, q55 = P_fix^2 I, q66 = 0, seed 1) with obs_dump.
%   Fixed window of N steps sliding along the run (as the 10-10 probe_ws_obs_ramp row 4):
%     O_win = [H_j Phi(i, k0) / sqrt(R_j)]  (2N x n, whitened)   ->   G = O_win' O_win  (n x n, square; eig(G) = sigma(O_win)^2)
%     free slots [dx1 dx2 dx3 a c delta]; (c, delta) block after the others are taken as unknown (Schur complement) = 2 x 2
%     prior-scaled: S G_s S, S = diag(sqrt(p55[0]), sqrt(p77[0])) = diag(0.10, 0.64)  ->  sigma_1 >= sigma_2;  sigma > 1 <=> the
%     window alone pins that direction better than the prior.
%   Weak direction (unscaled G_s, written in (c, w_s) with w_s = -delta): dc / dw_s of the eigenvector with the smaller eigenvalue,
%   next to the one-height alias 2 (1 - c-hat) / (h_d - w_s-hat) (1009 v1 S3: at one height the w_s column = -2(1-c)/h x the c column).
%   Negative control: inert slot 6 must come out unobservable (CRLB Inf, eigen-decomposition, no pinv).
%   Whole descent (window from t = 0 to the end of the ramp): sigma, weak direction, the filter's posterior long axis, and the
%   direction from the estimate to the F2 band-fit pair (cell (-0.163, -1.238), plane (-0.085, -0.104)).
%   Output test_results/temp_1009_ws/obswin_<plant>.mat.
function probe_1009_ws_obs_window(N, plants)
    if nargin < 1 || isempty(N); N = 200; end
    if nargin < 2 || isempty(plants); plants = {'plane', 'cell'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    pc = physical_constants();  R = pc.R;
    SIG_C = 0.10;  SIG_D = 0.64;  STRIDE = 4;
    BAND = struct('plane', [-0.085, -0.104], 'cell', [-0.163, -1.238]);   % run_plaw_walls f2 CFIT / DFIT, (c, w_s = -delta)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                'c_init', 0, 'Pf_c_std', SIG_C, 'Pf_a_floor', 0.002, 'plaw_seed_cross', true, ...
                'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, ...
                'plaw_dc_slope', true, 'plaw_p', 2, 'delta_init', 0, 'Pf_delta_std', SIG_D, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-0.9, 3], ...
                'obs_dump', true);
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * R, 't_hold', 0, 't_descend_override', 1.0, ...
                'T_sim', 2.0, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, 'lambda_c', 0.4);
    for ip = 1:numel(plants)
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                   'seeds', 1, 'log_P_full', false);
        if strcmp(plants{ip}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
        clear run_formC_b motion_control_law_formC_b;
        obs_dump('reset');
        evalc('Rr = run_formC_b(o);');
        L = obs_dump('get');  S1 = Rr.runs{1};
        Lz = L([L.ax] == 3);  n = numel(Lz);  ns = size(Lz(1).F, 1);
        wd = S1.h_bar_d_out(:);  hd = nan(n, 1);
        for i = 1:n; k = Lz(i).k + 1; if k >= 1 && k <= numel(wd); hd(i) = wd(k); end; end
        Ts = pc.Ts;  t = (1:n)' * Ts;
        free = [1 2 3 4 5 7];  io = 1:4;  it = 5:6;                     % within free: others = dx1..3, a; target = c, delta
        % negative control over the whole run
        [Gall, ~] = local_gram(Lz, 1, n, ns);
        crn = local_crlb(0.5 * (Gall([free 6], [free 6]) + Gall([free 6], [free 6])'));
        fprintf('[%s] n = %d, ns = %d | negative control (inert slot 6) CRLB = %g (must be Inf)\n', plants{ip}, n, ns, crn(end));
        % sliding window
        K0 = 1:STRIDE:(n - N + 1);  tc = nan(numel(K0), 1);  hc = tc;  sig = nan(numel(K0), 2);  rat = tc;  alias = tc;
        for q = 1:numel(K0)
            k0 = K0(q);  G = local_gram(Lz, k0, k0 + N - 1, ns);
            [s2, ratio] = local_cw(G, free, io, it, SIG_C, SIG_D);
            sig(q, :) = s2;  rat(q) = ratio;  im = k0 + floor(N / 2);  tc(q) = t(im);  hc(q) = hd(im);
            ch = Lz(im).x_pred(5);  dl = Lz(im).x_pred(7);
            alias(q) = 2 * (1 - ch) / (hd(im) + dl);                      % one-height alias dc/dw_s = 2(1-c)/(h - w_s), h - w_s = h + delta
        end
        % whole descent (from t = 0 to the end of the ramp)
        iend = find(hd <= min(hd) + 1e-9, 1);
        G = local_gram(Lz, 1, iend, ns);  [s2, ratio] = local_cw(G, free, io, it, SIG_C, SIG_D);
        Pp = Lz(iend).P_upd([5 7], [5 7]);  Pw = diag([1 -1]) * Pp * diag([1 -1]);   % to (c, w_s)
        [V, D] = eig(0.5 * (Pw + Pw'));  [~, j] = max(diag(D));  rP = V(1, j) / V(2, j);
        ce = S1.b_hat_out(end, 3);  we = -S1.ws_hat_out(end, 3);  b = BAND.(plants{ip});  rB = (ce - b(1)) / (we - b(2));
        fprintf('  whole descent: sigma (prior-scaled) %.3g / %.3g | weak dc/dw_s %.3f | filter long axis dc/dw_s %.3f | estimate (%+.3f, %+.3f) -> band fit (%+.3f, %+.3f): dc/dw_s %.3f\n', ...
                s2(1), s2(2), ratio, rP, ce, we, b(1), b(2), rB);
        fprintf('  sliding N = %d (%.0f ms):\n    h_d    sigma1   sigma2  | weak dc/dw_s  alias\n', N, 1e3 * N * Ts);
        for hm = [15 10 6 4 3 2 1.5 1.2]
            q = find(hc <= hm, 1);  if isempty(q); continue; end
            fprintf('    %5.2f  %7.3g  %7.3g | %7.3f      %7.3f\n', hc(q), sig(q, 1), sig(q, 2), rat(q), alias(q));
        end
        res = struct('t', tc, 'hd', hc, 'sig', sig, 'ratio', rat, 'alias', alias, 'N', N, 'Ts', Ts, 't_full', t, 'hd_full', hd, ...
                     'sig_whole', s2, 'ratio_whole', ratio, 'ratio_post', rP, 'ratio_band', rB, 'plant', plants{ip});
        save(fullfile(od, sprintf('obswin_%s.mat', plants{ip})), '-struct', 'res');
    end
end

function [G, Phi] = local_gram(Lz, k0, k1, ns)
    Phi = eye(ns);  G = zeros(ns);
    for i = k0:k1
        for j = 1:2
            Hj = Lz(i).H{j};  if isempty(Hj); continue; end
            M = (Hj / sqrt(Lz(i).R(j))) * Phi;  G = G + M' * M;
        end
        Phi = Lz(i).F * Phi;
    end
end

function [s2, ratio] = local_cw(G, free, io, it, sc, sd)
    Gf = 0.5 * (G(free, free) + G(free, free)');
    Gs = Gf(it, it) - Gf(it, io) * (Gf(io, io) \ Gf(io, it));            % (c, delta) block, the others unknown
    Gs = 0.5 * (Gs + Gs');
    S = diag([sc, sd]);  e = sort(max(eig(S * Gs * S), 0), 'descend');  s2 = sqrt(e(:)');
    Gw = diag([1 -1]) * Gs * diag([1 -1]);                                % (c, delta) -> (c, w_s)
    [V, D] = eig(Gw);  [~, j] = min(diag(D));  ratio = V(1, j) / V(2, j); % weak direction dc / dw_s
end

function c = local_crlb(G)
    [V, D] = eig(G);  dv = real(diag(D));  m = size(G, 1);  tol = max(dv) * 1e-15;  ok = dv > tol;  c = zeros(1, m);
    for j = 1:m
        wj = real(V(j, :)).^2;
        if any(wj(~ok) > 1e-12); c(j) = Inf; else; c(j) = sqrt(sum(wj(ok) ./ dv(ok).')); end
    end
end
