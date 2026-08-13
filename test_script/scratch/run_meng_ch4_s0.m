% STATUS: ACTIVE (scratch) | PURPOSE: Scenario 0 = Meng thesis 4.3.2 Experiment 1
%   (his only published simulation, Fig 28/29): two-region drag step, z
%   full-cosine 0 -> -8 -> 0 um over 10 s, boundary z_b = -3.2 um.
%   RESPEC (ledger section 8): the thesis-literal pair (lambda_c = 0,
%   drag ratio 0.494) is NOT reproducible under the faithful (4.4)+estimator
%   loop with d = 2 -- its closed-loop spectrum is 1.173/step unstable for
%   lambda_c below ~0.35. Main arms therefore run the journal's
%   lambda_c = 0.4 with the step deepened to 0.35 (< the 0.4226 bound), which
%   preserves the Fig 28/29 contrast: fixed-gain arm diverges, estimation arm
%   rescues. The literal pair is kept as arm E0lit with divergence as the
%   PREDICTED outcome.
%   Arms:
%     N     : law (3.6), a_hat frozen at a_N, suppress_xD  -> PREDICT diverge
%     E98   : law (4.4) ('ch4'), lambda_f = 0.98 (Ch6 anchor) -> PREDICT stable
%     E1    : law (4.4), lambda_f = 1 (journal (20)), Pf init at 0.98 equil
%     E0lit : thesis-literal (lambda_c ~ 0, ratio 0.494), law (4.4)
%             -> PREDICT diverge at ~1.17/step from t = 0 (ledger section 8)
%   EXPIRES: Scenario-0 adjudication vs thesis Fig 28/29
%            (spec: reference/eq17_analysis/meng_ch4_spec_ledger.md)
function out = run_meng_ch4_s0(seeds)

    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42]; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));

    pc  = physical_constants();
    Ts  = pc.Ts;  kBT = pc.k_B * pc.T;
    gN  = pc.gamma_N;  a_N = Ts / gN;
    LC     = 0.4;                          % journal "all experiments"
    RATIO  = 0.35;                         % main-arm drag step (bound at LC: 0.4226)
    Z_B    = -3.2;                         % [um]
    T_SIM  = 10;  N = round(T_SIM / Ts);
    SIGMA_N = [0.0007; 0.0007; 0.0023];    % [um] journal p.3

    t  = (0:N)' * Ts;
    zd = -4 * (1 - cos(2*pi*t/T_SIM));
    pd_arr = [zeros(N+1, 2), zd];
    tc = T_SIM/(2*pi) * acos(1 + Z_B/4);   % 2.183 s

    % --- pre-registered predictions --------------------------------------
    grow = @(lc, g) max(abs(roots([1, (1-lc)-1, 0, (1-lc)*(g-1)])));
    sig  = @(lc, a) sqrt((2 + 1/(1-lc^2))*4*kBT*a + ((1-lc)/(1+lc))*SIGMA_N(3)^2);
    fprintf('=== Scenario 0 (respec: lc=%.1f, ratio=%.2f; ledger sections 7-8) ===\n', LC, RATIO);
    fprintf('crossings %.3f / %.3f s\n', tc, T_SIM - tc);
    fprintf('P1  arm N growth  = %.5f/step after first crossing\n', grow(LC, 1/RATIO));
    fprintf('P2  sigma_dz (11) = %.1f nm (region 1) / %.1f nm (region 2)\n', ...
            1e3*sig(LC, a_N), 1e3*sig(LC, a_N/RATIO));
    fprintf('P3  arm E: zero-mean dz; a_hat_z tracks %.5f -> %.5f\n', a_N, a_N/RATIO);
    fprintf('P4  arm E0lit diverges at ~%.3f/step from t=0 (thesis-literal not reproducible)\n', 1.173);

    params = struct();
    params.ctrl   = struct('enable', 1, 'Ts', Ts, 'k_B', pc.k_B, 'T', pc.T, ...
                           'gamma', gN, 'sigma2_noise', SIGMA_N.^2);
    params.common = struct('R', pc.R, 'p0', [0; 0; 0]);
    params.wall   = struct('w_hat', [0; 0; 1], 'pz', 0, 'enable_wall_effect', 0);
    params.traj   = struct('amplitude', 4, 'frequency', 1/T_SIM);

    mk = @(lc) build_eq17_constants(struct('lambda_c', lc, ...
             'sigma2_n_s', SIGMA_N.^2, 'kBT', kBT, 't_warmup_kf', 0));
    cc4 = mk(LC);    cc4.iir_warmup_mode = 'prefill';
    cc0 = mk(1e-9);  cc0.iir_warmup_mode = 'prefill';

    % (4.4)-loop variance-channel calibration (calc_cdpmr_ch4, ledger section 9):
    % the journal reuses the (3.6)-loop formula (11) for the (4.4) system, which
    % reads a_xm high by C_dpmr44/C_dpmr36 ~ 1.8x here. Injected for the ch4
    % arms only (E0lit stays journal-literal: its loop has no stationary
    % variance anyway).
    cd44 = calc_cdpmr_ch4(LC, 0.98, RATIO);
    cc4ch = cc4;
    cc4ch.C_dpmr_eff = cd44.C_dpmr(1:3);
    cc4ch.C_np_eff   = cd44.C_n(1:3);
    cc4ch.xi_per_axis = (cd44.C_n(1:3) ./ cd44.C_dpmr(1:3)) .* SIGMA_N.^2 / (4*kBT);

    arms = struct('name', {}, 'cc', {}, 'ratio', {}, 'nseed', {});
    c = cc4; c.a_hat_freeze = a_N*[1;1;1]; c.suppress_xD = true;
    arms(1) = struct('name', 'N',     'cc', c, 'ratio', RATIO, 'nseed', 1);
    c = cc4ch; c.control_law = 'ch4'; c.lambda_f = 0.98;
    arms(2) = struct('name', 'E98',   'cc', c, 'ratio', RATIO, 'nseed', numel(seeds));
    c = cc4ch; c.control_law = 'ch4'; c.lambda_f = 1; c.Pf_init_lambda_f = 0.98;
    arms(3) = struct('name', 'E1',    'cc', c, 'ratio', RATIO, 'nseed', numel(seeds));
    c = cc0; c.control_law = 'ch4'; c.lambda_f = 0.98;
    arms(4) = struct('name', 'E0lit', 'cc', c, 'ratio', 0.494, 'nseed', 1);

    out = struct('t', t(1:N), 'zd', zd(1:N), 'tc', tc, 'seeds', seeds, ...
                 'lc', LC, 'ratio', RATIO, 'a_N', a_N, ...
                 'sig11', [sig(LC, a_N), sig(LC, a_N/RATIO)]);
    for ia = 1:numel(arms)
        nm = arms(ia).name;
        runs = cell(arms(ia).nseed, 1);
        for is = 1:arms(ia).nseed
            runs{is} = local_run_once(pd_arr, params, arms(ia).cc, seeds(is), ...
                                      Ts, kBT, gN, a_N, arms(ia).ratio, Z_B, N);
        end
        out.(nm) = runs;
        r = runs{1};  ke = r.kend;
        tt = (1:ke)' * Ts;
        w1 = tt < tc - 0.2;  w2 = tt > tc + 0.5 & tt < T_SIM - tc - 0.5;
        dz = r.p_d_used(1:ke, 3) - r.p_true(1:ke, 3);
        fprintf('\n--- arm %-5s (seed %d, kend %d) ---\n', nm, seeds(1), ke);
        fprintf('std dz r1 = %.1f nm | r2 = %.1f nm | max|dz| = %.4g um\n', ...
                1e3*std(dz(w1)), 1e3*std(dz(w2)), max(abs(dz)));
        fprintf('mean dz r1/r2 = %+.2f / %+.2f nm; a_hat_z r2/end = %.5f / %.5f (a2 = %.5f)\n', ...
                1e3*mean(dz(w1)), 1e3*mean(dz(w2)), ...
                mean(r.a_hat(w2, 3)), r.a_hat(ke, 3), a_N/arms(ia).ratio);
        fprintf('y2 gate duty (z): G1 %.3f  G2 %.3f  G3 %.3f\n', ...
                mean(r.G(1:ke, 1)), mean(r.G(1:ke, 2)), mean(r.G(1:ke, 3)));
    end
end


function r = local_run_once(pd_arr, params, cc, seed, Ts, kBT, gN, a_N, ratio, z_b, N)
%LOCAL_RUN_ONCE  One closed-loop realization with the two-region plant and a
%   true d = 2 measurement delay (sample-instant convention: entering iter k
%   the plant holds p[k], so the two-back ENTRY value is p[k-2]).
    rng(seed);
    clear motion_control_law_eq17_core            % reset persistents
    sigma_n = sqrt(params.ctrl.sigma2_noise(:));

    p_true = zeros(N, 3);  a_hat = zeros(N, 3);  G = false(N, 3);
    xD_hat = zeros(N, 3);  a_true_z = zeros(N, 1);
    p = [0; 0; 0];
    pe1 = p;  pe2 = p;
    kend = N;
    for k = 1:N
        pd     = pd_arr(k, :)';
        del_pd = pd_arr(k+1, :)' - pd;
        p_entry = p;
        p_m     = pe2 + sigma_n .* randn(3, 1);

        [f_d, ~, dg] = motion_control_law_eq17_core(del_pd, pd, p_m, params, cc);

        % plant: region drag from the TRUE position (self-limiting divergence)
        if p(3) > z_b; gam = gN; else; gam = gN * ratio; end
        a_gain = Ts / gam;
        f_T = sqrt(4*kBT*gam/Ts) * randn(3, 1);
        p = p + a_gain * (f_d + f_T);

        p_true(k, :)  = p';
        a_hat(k, :)   = dg.a_hat';
        xD_hat(k, :)  = dg.x_D_hat';
        a_true_z(k)   = a_gain;
        G(k, :)       = dg.guards_individual(:, 3)';
        pe2 = pe1;  pe1 = p_entry;
        if ~isfinite(p(3)) || abs(p(3)) > 1e8
            kend = k;  break;
        end
    end
    r = struct('p_true', p_true, 'p_d_used', pd_arr(1:N, :), 'a_hat', a_hat, ...
               'xD_hat', xD_hat, 'a_true_z', a_true_z, 'G', G, 'seed', seed, ...
               'kend', kend, 'a2', a_N/ratio);
end
