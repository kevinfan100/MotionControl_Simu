% PURPOSE (2026-09-17): the [1/1] two-wall test of 0916_estb_11_two_versions.tex -- both versions of the pure 5-state
%   estimator on both plants, three seed arms, paired seeds, canon deep trajectory.
%     versions  A  c0_state: x = [dw1 dw2 dw3 c0 d1], gain computed from the law (ctrl_const.c0_state + gain_algebraic)
%               B  production structure with the four production flags OFF (pure Euler a slot): x = [dw1 dw2 dw3 a_w b];
%                  reported in the Pade coefficients through c0 = w_s - w0 - 1/b (c0 fixed by the seed), d1 = c0 + 1/b
%     plants    plane  Brenner polynomial (production plant)
%               cell   plane polynomial slid down 1.05 R (cell_cperp_delta.m, 2026-09-20; Fig 11 data fit cell_cperp_fig11 via opts.plant_cperp_cell); the thermal force follows the same c_perp
%     arms      T  seed = the plant's own band fit, tight prior (sanity: constants must stay within sqrt(P))
%               P  plane seed on the cell plant, wall-family prior (the question: is c0 learned in one descent)
%               N  cell seed on the plane plant, wall-family prior (negative control: d1_hat must walk to d1_eff(w))
%   Band fits (0916_abar_gap_polynomial.tex S5-S6, w_s = wall surface, 2026-09-18): plane (c0, d1) = (-0.984, 0.161),
%   cell (0.056, 1.197). The estimator's law is a = ((w - w_s) + c0)/((w - w_s) + d1); the controller parameter ws0_perp
%   receives w_s directly (plane 0). Old coordinate (w_s = contact height 1): every constant +1, results bit-identical.
%   Priors: tight = 0.05 on both (the fit residual level); wall family = half the plane-cell spread, 0.52 on both.
%   Health printed before any statistic: seed a_bar at the band floor 1.1, floor hits, NaN, min(d1 - c0), min true height.
%   Output: test_results/temp_11_two_walls/<plant>_<version>_<arm>.mat (gitignored) + console; figures by plot_11_ladder.m.
%   EXPIRES: with the [1/1] two-wall line | 產線改動不會自動跟上
function out = run_11_two_walls(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:10; end
    if ~isfield(opts, 'plants');   opts.plants   = {'plane', 'cell'}; end
    if ~isfield(opts, 'versions'); opts.versions = {'A', 'B'}; end
    if ~isfield(opts, 'arms');     opts.arms     = {'T', 'P', 'N'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_11_two_walls');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;
    if ~isfield(opts, 'W_S');        opts.W_S        = 0.0; end   % WALL SURFACE in the w coordinate (plane 0; 2026-09-18, was the contact height 1)
    if ~isfield(opts, 'seed_shift'); opts.seed_shift = 0;   end   % added to BOTH seed constants (w_s -> w_s + s, c0/d1 -> c0/d1 - s is the identity)
    W_S = opts.W_S;
    FIT = struct('plane', [-0.984, 0.161], 'cell', [0.0501, 1.1871]);       % (c0, d1) band fits, w_s = wall surface; cell = fit on cell_cperp_delta (2026-09-20; data-fit plant: 0.056, 1.197)
    SIG_TIGHT = [0.05, 0.05];  SIG_FAMILY = [0.52, 0.52];                    % (sigma_c0, sigma_d1)
    if isfield(opts, 'sig_tight'); SIG_TIGHT = opts.sig_tight; end          % 2026-09-20: T arm with the family prior (fair pair to the delta law's 0.5), tag it via tag_suffix
    SEG = [0.5, 1.5, 3.5];                                                    % canon deep: hold | descent | oscillation | hold
    out = struct();
    if ~isfield(opts, 'tag_suffix'); opts.tag_suffix = ''; end                     % appended to the output tag (diagnostic plants)
    if ~isfield(opts, 'traj'); opts.traj = 'canon'; end                            % 2026-09-20: 'meng' = 15 um -> 2.5 um ramp over 10 s, hold to 12.5 s
    OV = struct();
    if strcmp(opts.traj, 'meng')
        OV  = struct('trajectory_type', 'osc', 'h_init', 15, 'h_bottom', 2.5, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, ...
                     't_hold', 0.5, 't_descend_override', 10, 'T_sim', 12.5, 'h_min', 2.475);
        SEG = [0.5, 10.5, 10.5];  opts.tag_suffix = [opts.tag_suffix '_meng'];
    end
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        plant_cperp = [];  if strcmp(pl, 'cell'); plant_cperp = @cell_cperp_delta; end   % 2026-09-20: default cell = plane slid 1.05 R (cell_cperp_delta); data fit via opts.plant_cperp_cell
        if strcmp(pl, 'cell') && isfield(opts, 'plant_cperp_cell'); plant_cperp = opts.plant_cperp_cell; end   % diagnostic: another cell plant
        for ia = 1:numel(opts.arms)
            arm = opts.arms{ia};
            if strcmp(arm, 'P') && ~strcmp(pl, 'cell');  continue; end
            if strcmp(arm, 'N') && ~strcmp(pl, 'plane'); continue; end
            switch arm
                case 'T'; seed_cd = FIT.(pl);          sig = SIG_TIGHT;
                case 'P'; seed_cd = FIT.plane;         sig = SIG_FAMILY;
                case 'N'; seed_cd = FIT.cell;          sig = SIG_FAMILY;
            end
            seed_cd = seed_cd + opts.seed_shift;
            for iv = 1:numel(opts.versions)
                ver = opts.versions{iv};
                cc = local_ctrl_const(ver, seed_cd, sig, W_S);
                tag = sprintf('%s_%s_%s%s', pl, ver, arm, opts.tag_suffix);
                o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                           'seeds', opts.seeds, 'log_P_full', false);
                if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
                a11 = (1.1 - W_S + seed_cd(1)) / (1.1 - W_S + seed_cd(2));
                fprintf('\n=== %s | w_s = %.2f, seed (c0, d1) = (%.3f, %.3f), a_bar(1.1) = %.3f, prior (%.2f, %.2f), %d seeds ===\n', ...
                        tag, W_S, seed_cd(1), seed_cd(2), a11, sig(1), sig(2), numel(opts.seeds));
                clear run_formC_b motion_control_law_formC_b;
                tic; evalc('Rr = run_formC_b(o);'); tr = toc;
                res = local_collect(Rr, ver, seed_cd, W_S, plant_cperp, Ts);
                res.tag = tag; res.plant = pl; res.version = ver; res.arm = arm; res.seed_cd = seed_cd; res.sig = sig; res.seg = SEG;
                local_report(res, SEG);
                fprintf('    (%.0f s)\n', tr);
                save(fullfile(od, [tag '.mat']), '-struct', 'res');
                out.(tag) = res;
            end
        end
    end
end

function cc = local_ctrl_const(ver, seed_cd, sig, W_S)
    c0 = seed_cd(1);  d1 = seed_cd(2);  kap = d1 - c0;
    pure = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                  'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 1e-5);
    switch ver
        case 'A'      % (c0, d1) states, gain computed
            cc = pure;
            cc.c0_state = true;  cc.gain_algebraic = true;  cc.amodel_fe = true;  cc.amodel_h = true;
            cc.c0_init = c0;  cc.d1_init = d1;  cc.Pf_c0_std = sig(1);  cc.Pf_d1_std = sig(2);
            cc.ws0_perp = W_S;  cc.Pf_w0_std = 0;
            cc.b_init = 1 / kap;  cc.Pf_b_std = sig(2);            % slot-5 seed/prior are overwritten by c0_state; kept consistent
        case 'B'      % production structure: a slot along the law, slot 5 = b = 1/(d1 - c0), origin w0 = w_s - d1
            cc = pure;
            cc.b_init = 1 / kap;  cc.ws0_perp = W_S - d1 + 1;      % w0 = ws0_perp - 1 = w_s - d1
            cc.Pf_b_std = sig(2) / kap^2;                            % sigma_d1 -> sigma_b at fixed c0
            cc.Pf_w0_std = sig(1);                                   % sigma_c0 = sigma_w0 at fixed b (c0 = w_s - w0 - 1/b)
    end
end

function res = local_collect(Rr, ver, seed_cd, W_S, plant_cperp, Ts)
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:), 'ns', ns);
    res.h_true = zeros(n, ns); res.a_true = zeros(n, ns); res.a_hat = zeros(n, ns);
    res.c0_hat = zeros(n, ns); res.d1_hat = zeros(n, ns); res.sP_c0 = zeros(n, ns); res.sP_d1 = zeros(n, ns);
    res.sP_a = zeros(n, ns); res.innov1 = zeros(n, ns); res.innov2 = zeros(n, ns); res.S1 = zeros(n, ns); res.S2 = zeros(n, ns);
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);
        at = zeros(n, 1);
        for i = 1:n
            h = max(hb(i), 1.001);
            if isempty(plant_cperp); [~, c] = calc_correction_functions(h); else; c = plant_cperp(h); end
            at(i) = 1 / c;
        end
        res.h_true(:, q) = hb;  res.a_true(:, q) = at;  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        switch ver
            case 'A'
                res.c0_hat(:, q) = S.ws_hat_out(:, 3);  res.d1_hat(:, q) = S.b_hat_out(:, 3);
                res.sP_c0(:, q)  = S.P_ws_out(:, 3);    res.sP_d1(:, q)  = S.P_b_out(:, 3);
            case 'B'      % c0 is fixed by the seed; d1 = c0 + 1/b_hat; sigma_d1 = sigma_b / b^2
                b = S.b_hat_out(:, 3);
                res.c0_hat(:, q) = seed_cd(1) * ones(n, 1);  res.d1_hat(:, q) = seed_cd(1) + 1 ./ b;
                res.sP_c0(:, q)  = zeros(n, 1);              res.sP_d1(:, q)  = S.P_b_out(:, 3) ./ b.^2;
        end
        res.sP_a(:, q) = sqrt(max(S.P_a_out(:, 3), 0)) / 1;   % P_a_out is already sqrt(P_a) [um/pN]; converted below if needed
        res.innov1(:, q) = S.innov_y1_out(:, 3);  res.innov2(:, q) = S.innov_y2_out(:, 3);
        if isfield(S, 'S1_out'); res.S1(:, q) = S.S1_out(:, 3);  res.S2(:, q) = S.S2_out(:, 3); end
    end
    res.W_S = W_S;
end

function local_report(res, SEG)
    t = res.t;  ea = res.a_true - res.a_hat;                     % e_a = a_true - a_hat, absolute
    m = mean(ea, 2);  s = std(ea, 0, 2);
    desc = t >= SEG(1) & t < SEG(2);  osc = t >= SEG(2) & t < SEG(3);  hold = t >= SEG(3);
    [~, iw] = max(abs(m) .* desc);
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d | min(d1-c0) %.4f\n', min(res.h_true(:)), ...
            nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), min(res.d1_hat(:) - res.c0_hat(:)));
    fprintf('  e_a    : descent worst %+.4f at t = %.2f s (sd %.4f) | descent mean %+.4f sd %.4f | osc mean %+.4f sd %.4f | hold mean %+.4f sd %.4f\n', ...
            m(iw), t(iw), s(iw), mean(m(desc)), mean(s(desc)), mean(m(osc)), mean(s(osc)), mean(m(hold)), mean(s(hold)));
    for th = {'c0', 'd1'}
        x = res.([th{1} '_hat']);  sp = res.(['sP_' th{1}]);
        walk = max(abs(x - x(1, :)), [], 1);                       % per-seed sup|theta_hat(t) - theta_hat(0)|
        fprintf('  %-2s     : start %.3f | end %.3f +- %.3f (seed sd) | sup|walk| %.3f (seeds mean) | sqrtP end %.3f | honesty sd/sqrtP %.2f\n', ...
                th{1}, x(1, 1), mean(x(end, :)), std(x(end, :)), mean(walk), mean(sp(end, :)), std(x(end, :)) / max(mean(sp(end, :)), eps));
    end
end
