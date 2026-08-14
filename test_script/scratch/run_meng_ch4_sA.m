% STATUS: ACTIVE (scratch) | PURPOSE: Scenario A = Meng journal section IV-A /
%   thesis 4.3.2 experiment 2: z ramps h = 15 -> 2.5 um over 10 s
%   (h_bar 6.667 -> 1.111), x and y each 1 Hz 9 um sines, REAL wall-effect
%   plant (calc_correction_functions), lambda_c = 0.4 (journal), true d = 2
%   measurement delay, paper noise 0.7/0.7/2.3 nm.
%   Plant note: per-axis discrete map p[k+1] = p[k] + a_axis(h_bar)*(f_d+f_T)
%   with a evaluated at the current position -- this IS Meng's model (4.3);
%   the RK4 mobility plant is a later robustness arm, not this rung.
%   Arms:
%     N   : law (3.6), a_hat frozen at a_N (free-space), suppress_xD
%           -> journal Fig 5-right: bounded, GROWING deterministic error
%              toward the wall, 1 Hz line in x/y (g = 1/c < 1 = stable side)
%     E98 : law (4.4) ch4, lambda_f = 0.98, C_dpmr44 calibration
%           -> journal Fig 5-left/Fig 6: zero-mean error, a_hat tracks
%              a_z 0.01225 -> 0.00141, a_xy 0.01346 -> 0.00636 (memory-
%              verified mapping), no 1 Hz line in the residual
%   Gates: h_bar_safe = 1 override (2026-08-12 ruling); G1/G2/G3 duty printed.
%   EXPIRES: Scenario-A adjudication vs journal Fig 4-6
%            (spec: reference/eq17_analysis/meng_ch4_spec_ledger.md)
function out = run_meng_ch4_sA(seeds, lambda_f, IF_override, plant_mode, ch4_fdet)

    if nargin < 1 || isempty(seeds); seeds = 7; end
    if nargin < 2 || isempty(lambda_f); lambda_f = 0.98; end
    if nargin < 3; IF_override = []; end   % 3x1 IF_eff_per_axis (diagnostic: 1e12 kills y2 on that axis)
    if nargin < 4 || isempty(plant_mode); plant_mode = 'map'; end  % 'map' = Meng (4.3) | 'rk4' = house continuous physics
    if nargin < 5 || isempty(ch4_fdet); ch4_fdet = false; end       % ledger 19/20 exogenous-regressor fix

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));

    pc  = physical_constants();
    Ts  = pc.Ts;  kBT = pc.k_B * pc.T;  R = pc.R;
    gN  = pc.gamma_N;  a_N = Ts / gN;
    LC  = 0.4;
    T_SIM = 10;  N = round(T_SIM / Ts);
    SIGMA_N = [0.0007; 0.0007; 0.0023];    % [um] journal p.3
    H0 = 15;  H1 = 2.5;                    % [um] ramp endpoints (probe centre)

    % trajectory arrays (world coords; wall at z = 0, w_hat = [0;0;1])
    t  = (0:N)' * Ts;
    hz = max(H0 - (H0 - H1) * t / T_SIM, H1);
    pd_arr = [9*sin(2*pi*1*t), 9*sin(2*pi*1*t), hz];

    % predictions (printed before running)
    [cpar0, cper0] = calc_correction_functions(H0/R);
    [cpar1, cper1] = calc_correction_functions(H1/R);
    fprintf('=== Scenario A (lc=%.1f, ledger sections 1/8/9) ===\n', LC);
    fprintf('P1  a_z true: %.5f -> %.5f um/pN; a_xy: %.5f -> %.5f\n', ...
            a_N/cper0, a_N/cper1, a_N/cpar0, a_N/cpar1);
    fprintf('P2  arm N: bounded growing deterministic error toward the wall,\n');
    fprintf('    1 Hz line in x/y (fixed a_N is the overdamped side, no divergence)\n');
    fprintf('P3  arm E98: zero-mean dz; no 1 Hz line; a_hat tracks P1 within the\n');
    fprintf('    +12%% systematic residual measured on Scenario 0\n');

    params = struct();
    params.ctrl   = struct('enable', 1, 'Ts', Ts, 'k_B', pc.k_B, 'T', pc.T, ...
                           'gamma', gN, 'sigma2_noise', SIGMA_N.^2);
    params.common = struct('R', R, 'p0', [0; 0; H0], 'gamma_N', gN);
    params.wall   = struct('w_hat', [0; 0; 1], 'pz', 0, 'enable_wall_effect', 1);
    params.traj   = struct('amplitude', 9, 'frequency', 1);   % Q77 bound reads (inert: Q77 forced 0)

    cc = build_eq17_constants(struct('lambda_c', LC, ...
             'sigma2_n_s', SIGMA_N.^2, 'kBT', kBT, 't_warmup_kf', 0));
    cc.iir_warmup_mode = 'prefill';
    cc.force_Q77_zero  = true;             % paper (21): Q33-only
    cc.h_bar_safe      = 1;                % G3 off (2026-08-12 ruling)

    % per-axis lambda_f (scalar broadcasts); calibration computed per unique value
    lfv = lambda_f(:);  if isscalar(lfv); lfv = lfv * ones(3, 1); end
    Cd = zeros(3, 1);  Cn = zeros(3, 1);
    for u = unique(lfv)'
        cdu = calc_cdpmr_ch4(LC, u, 0.35);
        m = (lfv == u);
        Cd(m) = cdu.C_dpmr(m);
        Cn(m) = cdu.C_n(m);
    end
    ccE = cc;
    ccE.control_law = 'ch4';  ccE.lambda_f = lfv;
    pfl = lfv;  pfl(pfl >= 1) = 0.98;
    ccE.Pf_init_lambda_f = pfl;
    ccE.C_dpmr_eff  = Cd;
    ccE.C_np_eff    = Cn;
    ccE.xi_per_axis = (Cn ./ Cd) .* SIGMA_N.^2 / (4*kBT);
    if ~isempty(IF_override); ccE.IF_eff_per_axis = IF_override(:); end
    ccE.ch4_fdet = ch4_fdet;

    ccN = cc;
    ccN.a_hat_freeze = a_N * [1; 1; 1];
    ccN.suppress_xD  = true;

    arms = struct('name', {'N', 'E98'}, 'cc', {ccN, ccE}, ...
                  'nseed', {1, numel(seeds)});

    out = struct('t', t(1:N), 'pd', pd_arr(1:N, :), 'seeds', seeds, ...
                 'a_N', a_N, 'lc', LC, 'lambda_f', lambda_f);
    for ia = 1:numel(arms)
        nm = arms(ia).name;
        runs = cell(arms(ia).nseed, 1);
        for is = 1:arms(ia).nseed
            runs{is} = local_run_once(pd_arr, params, arms(ia).cc, ...
                                      seeds(is), pc, N, plant_mode);
        end
        out.(nm) = runs;
        r = runs{1};  ke = r.kend;
        tt = (1:ke)' * Ts;
        wf = tt < 3;  wn = tt > 8 & tt <= min(10, ke*Ts);   % far / near windows
        for ax = [1 3]
            d = r.p_d_used(1:ke, ax) - r.p_true(1:ke, ax);
            fprintf('%-4s ax%d: std far/near = %.1f / %.1f nm  mean = %+.1f / %+.1f nm  max|d| = %.3g um\n', ...
                nm, ax, 1e3*std(d(wf)), 1e3*std(d(wn)), ...
                1e3*mean(d(wf)), 1e3*mean(d(wn)), max(abs(d)));
        end
        af = mean(r.a_hat(wf, 3));  an = mean(r.a_hat(wn, 3));
        fprintf('%-4s a_hat_z far/near = %.5f / %.5f (true %.5f / %.5f)\n', ...
                nm, af, an, mean(r.a_true(wf, 3)), mean(r.a_true(wn, 3)));
        fprintf('%-4s gate duty z: G1 %.3f G2 %.3f G3 %.3f | kend %d\n', ...
                nm, mean(r.G(1:ke, 1)), mean(r.G(1:ke, 2)), mean(r.G(1:ke, 3)), ke);
    end
end


function r = local_run_once(pd_arr, params, cc, seed, pc, N, plant_mode)
%LOCAL_RUN_ONCE  Wall-effect plant (per-axis Meng-(4.3) map) + d = 2 delay.
    rng(seed);
    clear motion_control_law_eq17_core
    Ts = pc.Ts;  kBT = pc.k_B * pc.T;  gN = pc.gamma_N;  R = pc.R;
    sigma_n = sqrt(params.ctrl.sigma2_noise(:));

    p_true = zeros(N, 3);  a_hat = zeros(N, 3);  a_true = zeros(N, 3);
    P_a = zeros(N, 3);  G = false(N, 3);
    p = params.common.p0;
    pe1 = p;  pe2 = p;
    kend = N;
    for k = 1:N
        pd     = pd_arr(k, :)';
        del_pd = pd_arr(k+1, :)' - pd;
        p_entry = p;
        p_m     = pe2 + sigma_n .* randn(3, 1);

        [f_d, ~, dg] = motion_control_law_eq17_core(del_pd, pd, p_m, params, cc);

        % plant: per-axis gain at the TRUE height (curve clamped at its
        % validity floor h_bar = 1.001; the commanded minimum is 1.111)
        h_bar = max(p(3) / R, 1.001);
        [c_par, c_per] = calc_correction_functions(h_bar);
        gam = gN * [c_par; c_par; c_per];
        a_ax = Ts ./ gam;
        f_T = sqrt(4*kBT*gam/Ts) .* randn(3, 1);
        if strcmpi(plant_mode, 'rk4')
            % house continuous physics: RK4 on the full mobility (ZOH force)
            p = step_dynamics(p, f_d + f_T, params, Ts);
        else
            p = p + a_ax .* (f_d + f_T);
        end

        p_true(k, :) = p';
        a_hat(k, :)  = dg.a_hat';
        a_true(k, :) = a_ax';
        P_a(k, :)    = dg.P_a';
        G(k, :)      = dg.guards_individual(:, 3)';
        pe2 = pe1;  pe1 = p_entry;
        if ~isfinite(p(3)) || abs(p(3)) > 1e8
            kend = k;  break;
        end
    end
    r = struct('p_true', p_true, 'p_d_used', pd_arr(1:N, :), 'a_hat', a_hat, ...
               'a_true', a_true, 'P_a', P_a, 'G', G, 'seed', seed, 'kend', kend);
end
