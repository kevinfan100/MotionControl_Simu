function obs_walk_5state()
%OBS_WALK_5STATE  Gates 4-5 done by hand on the 5-state core of 0907_estb_5state_core.tex, one step at a time.
%   PURPOSE (2026-09-09, teaching walk): build F_e[k], H[k] from the 0907 formulas along the canon deep command
%   trajectory with the exact law open loop (no filter, no noise, dw3_hat = 0, b_hat = chord seed), stack O on a
%   window, SVD -> rank / sigma_min/sigma_max, pinned-slot control, G -> CRLB, channel split, victim. Prints every
%   intermediate object. R2 is the logged median 8.8e-3 (08-24) as a stand-in for the IF_var formula.
%   EXPIRES: teaching script, with the 0907 reading copy | 產線改動不會自動跟上
    here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    cfg = canonical_scenario(0.05, 1.1, 'deep');  params = calc_simulation_params(cfg);  P = params.Value;
    Ts = P.common.Ts;  R = P.common.R;  N = round(cfg.T_sim / Ts) + 1;  lc = cfg.lambda_c;  d = 2;
    w_hat = P.wall.w_hat(:);  pz = P.wall.pz;
    % ---------------- step 1: the command trajectory ----------------
    clear trajectory_generator;  t = (0:N-1)' * Ts;  wd = zeros(N, 1);  pd_k = P.common.p0;
    for k = 1:N; [pd_kp1, ~] = trajectory_generator(t(k), P); wd(k) = (dot(pd_k, w_hat) - pz) / R; pd_k = pd_kp1; end
    clear trajectory_generator;
    fprintf('STEP 1  trajectory: N = %d steps, Ts = %.4g s, hold %.1f s -> descend %.1f s -> osc %d x %g Hz -> hold to %.1f s\n', ...
            N, Ts, cfg.t_hold, cfg.t_descend_override, cfg.n_cycles, cfg.frequency, cfg.T_sim);
    % ---------------- step 2: per-step quantities the Jacobians need ----------------
    [~, cp0] = calc_correction_functions(wd(1), true);  a0 = 1 / cp0;
    b_hat = (1/(1 - a0) - 1) / (wd(1) - 1.0);                                  % chord seed (rule B.8)
    dwd = [diff(wd); 0];                                                        % Delta_wbar_d[k] = wd[k+1] - wd[k]
    grad = [zeros(d, 1); wd(d+1:end) - wd(1:end-d)];                           % nabla_d wbar_d[k] = wd[k] - wd[k-d]
    u = zeros(N, 1);  u(1) = 1/(1 - a0);  for k = 1:N-1; u(k+1) = u(k) + b_hat * dwd(k); end
    ah = 1 - 1 ./ u;                                                            % a_hat along the exact law, open loop
    fd = dwd ./ ah;                                                             % commanded force f_bar_d: a_bar f_bar = Delta w_bar
    Fdw = fd + (1 - lc) * ([0; fd(1:end-1)] + [0; 0; fd(1:end-2)]);           % F_dw[k] = f[k] + (1-lc)(f[k-1] + f[k-2])
    dw3h = zeros(N, 1);                                                         % dw3_hat = 0 in this walk
    fprintf('STEP 2  b_hat (chord) = %.4f\n', b_hat);
    fprintf('        %6s %8s %8s %10s %10s %10s\n', 't [s]', 'wbar_d', 'a_hat', 'Dwbar_d', 'grad_d', 'F_dw');
    for ts = [0.2 1.0 1.41 2.0 4.0]
        k = find(t >= ts, 1); fprintf('        %6.2f %8.3f %8.4f %10.2e %10.2e %10.2e\n', t(k), wd(k), ah(k), dwd(k), grad(k), Fdw(k));
    end
    % ---------------- step 3: F_e[k], H[k] from the 0907 formulas ----------------
    Fe = @(k) [0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0];   % placeholder (overwritten below)
    function [F, H] = jac(k)
        M  = dwd(k) + (1 - lc) * dw3h(k);
        om = 1 - ah(k);
        F = [0 1 0 0 0;
             0 0 1 0 0;
             0 0 lc -Fdw(k) 0;
             0 0 (1-lc)*b_hat*om^2, 1 + b_hat*om^2*Fdw(k) - 2*b_hat*om*M, om^2*M;
             0 0 0 0 1];
        H = [1 0 0 0 0;
             0 0 0 1 + 2*b_hat*om*grad(k), -om^2*grad(k)];
    end
    for ts = [1.0 4.0]
        k = find(t >= ts, 1); [F, H] = jac(k);
        fprintf('STEP 3  t = %.2f s (%s)\n', t(k), local_phase(t(k), cfg));
        fprintf('        F_e[k] =\n'); disp(F);
        fprintf('        H[k] =\n'); disp(H);
        fprintf('        b column of F_e: F_e(4,5) = %.3e ; b column of H: H_25 = %.3e\n', F(4,5), H(2,5));
    end
    % ---------------- step 4-6 on one window ----------------
    sig_n = cfg.meas_noise_std(3) / R;  R1 = sig_n^2;  R2 = 8.8e-3;  Nw = 500;  prior_b = 0.039;
    k0 = find(t >= 1.5, 1);
    fprintf('STEP 4  window k0 at t = %.2f s (%s), N = %d steps\n', t(k0), local_phase(t(k0), cfg), Nw);
    [O, Rrow, Ochan] = local_stack(k0, Nw, @jac);
    fprintf('        size(O) = %d x %d ; column norms |O(:,j)| = %s\n', size(O,1), size(O,2), mat2str(vecnorm(O), 3));
    fprintf('        rows at k0 (y1, y2):\n'); disp(O(1:2, :));
    kk = 2 * 250 + (1:2);
    fprintf('        rows at k0+250 (y1, y2):\n'); disp(O(kk, :));
    fprintf('        rows at k0+500 (y1, y2):\n'); disp(O(end-1:end, :));
    s = svd(O);  tol = max(size(O)) * eps(max(s));  rk = sum(s > tol);
    fprintf('STEP 5  singular values of O = %s\n        rank = %d (tol %.1e), sigma_min/sigma_max = %.2e\n', mat2str(s', 3), rk, tol, s(end)/s(1));
    On = [O, zeros(size(O, 1), 1)];  sn = svd(On);
    fprintf('        pinned-slot control: append a zero column -> singular values %s -> rank %d of 6\n', mat2str(sn', 3), sum(sn > tol));
    G = O' * (O ./ Rrow);  lam = eig((G + G')/2);
    crlb = local_crlb(G);
    fprintf('STEP 6  G = O^T R^-1 O, eigenvalues = %s\n', mat2str(lam', 3));
    fprintf('        CRLB = sqrt(diag(G^-1)) = %s  (dw1 dw2 dw3 a_w b)\n', mat2str(crlb', 3));
    fprintf('        CRLB_b / sqrt(P55[0]) = %.3g / %.3f = %.3g\n', crlb(5), prior_b, crlb(5) / prior_b);
    fprintf('        others KNOWN: 1/sqrt(G_jj) = %s ; others UNKNOWN: sqrt([G^-1]_jj) = %s\n', mat2str(1 ./ sqrt(diag(G))', 3), mat2str(crlb', 3));
    fprintf('        check: sigma_n/R = %.3g ; y1 sees dw1 once at gain 1 -> CRLB_dw1 = %.3g\n', sig_n, crlb(1));
    G1 = O(1:2:end, :)' * (O(1:2:end, :) / R1);  G2 = O(2:2:end, :)' * (O(2:2:end, :) / R2);
    c1 = local_crlb(G1);  c2 = local_crlb(G2);
    fprintf('        channel split: CRLB_b from y1 only = %.3g (rank G1 = %d), from y2 only = %.3g (rank G2 = %d)\n', c1(5), rank(G1), c2(5), rank(G2));
    c4 = local_crlb(G(1:4, 1:4));
    fprintf('        victim: CRLB_a_w with b free = %.3g, with b known = %.3g\n', crlb(4), c4(4));
    % ---------------- step 7: the same on four windows ----------------
    fprintf('STEP 7  same recipe on four windows\n');
    fprintf('        %6s %-8s %5s %12s %12s %12s %12s\n', 't0', 'phase', 'rank', 'smin/smax', 'CRLB_b/pr', 'CRLB_a_w', 'b from y1/y2');
    for ts = [0.16 0.8 1.5 4.0]
        k0 = find(t >= ts, 1);  [O, Rrow] = local_stack(k0, Nw, @jac);
        s = svd(O);  tol = max(size(O)) * eps(max(s));  rk = sum(s > tol);
        G = O' * (O ./ Rrow);  c = local_crlb(G);
        c1 = local_crlb(O(1:2:end, :)' * (O(1:2:end, :) / R1));  c2 = local_crlb(O(2:2:end, :)' * (O(2:2:end, :) / R2));
        fprintf('        %6.2f %-8s %5d %12.2e %12.3g %12.3g %6.2g/%-6.2g\n', t(k0), local_phase(t(k0), cfg), rk, s(end)/s(1), c(5)/prior_b, c(4), c1(5)/prior_b, c2(5)/prior_b);
    end
    function [O, Rrow, Ochan] = local_stack(k0, Nw, jacf)
        Phi = eye(5);  O = zeros(2 * (Nw + 1), 5);  Rrow = zeros(2 * (Nw + 1), 1);
        for i = 0:Nw
            [F, H] = jacf(k0 + i);
            O(2*i + (1:2), :) = H * Phi;  Rrow(2*i + (1:2)) = [R1; R2];
            Phi = F * Phi;
        end
        Ochan = {O(1:2:end, :), O(2:2:end, :)};
    end
    function c = local_crlb(G)
        % eig, not pinv: a null direction is CRLB = Inf, never 0. Null vectors aligned with a coordinate axis
        % mark that slot Inf and the complementary block is inverted; a null vector mixing slots marks all
        % slots it touches Inf (the pair is observable only as a combination).
        n = size(G, 1);  c = inf(n, 1);
        [V, L] = eig((G + G')/2);  lam = diag(L);  tol = n * eps(max(lam));
        null_ix = find(lam <= tol);  dead = false(n, 1);
        for q = null_ix'; dead = dead | (abs(V(:, q)) > 1e-6); end
        live = ~dead;
        if any(live); Gi = inv(G(live, live)); c(live) = sqrt(diag(Gi)); end
    end
end
function s = local_phase(tt, cfg)
    t1 = cfg.t_hold; t2 = t1 + cfg.t_descend_override; t3 = t2 + cfg.n_cycles / cfg.frequency;
    if tt < t1; s = 'hold0'; elseif tt < t2; s = 'descent'; elseif tt < t3; s = 'osc'; else; s = 'hold'; end
end
