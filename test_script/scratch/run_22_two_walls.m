% PURPOSE (2026-09-18): the [2/2] two-wall test of 0918_estb_22_versionA.tex -- version A only (four constant states
%   c0, c1, d1, d2, gain computed from the law; ctrl_const.pade22 + c0_state + gain_algebraic), both plants, three seed
%   arms, paired seeds, canon deep trajectory. Same layout as run_11_two_walls.m so the two orders can be read side by side.
%     plants    plane  Brenner polynomial (production plant); the plane IS in the [2/2] two-point family to 1%
%               cell   Meng Fig. 11 cell, data fit (cell_cperp_fig11.m); in the [2/2] band fit to 0.5%
%     arms      T  seed = the plant's own [2/2] constants, tight prior (sanity: constants must stay within sqrt(P))
%               P  plane seed on the cell plant, wall-family prior
%               N  cell seed on the plane plant, wall-family prior
%   Seeds (0916_abar_gap_polynomial.tex S5-S6, w_s = wall surface): plane (102/127, -229/127, -689/1016, -1/8),
%   cell (0.353, -0.571, 0.484, 0.058). Priors: tight = 0.05 on all four; wall family = half the plane-cell spread per
%   coefficient in this basis (0.225, 0.616, 0.581, 0.092). The denominator guard is placed at the band floor (pade22_gmin).
%   Output: test_results/temp_22_two_walls/<plant>_A_<arm>.mat (gitignored) + console; figures by plot_22_ladder.m.
%   EXPIRES: with the [1/1]/[2/2] two-wall line | 產線改動不會自動跟上
function out = run_22_two_walls(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = 1:10; end
    if ~isfield(opts, 'plants'); opts.plants = {'plane', 'cell'}; end
    if ~isfield(opts, 'arms');   opts.arms   = {'T', 'P', 'N'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_22_two_walls');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  W_S = 0.0;                       % w_s = wall surface (2026-09-18; was the contact height 1)
    % (c0, c1, d1, d2) in x = w - w_s = h_bar: plane two-point (102/127, -229/127, -689/1016, -1/8), cell band fit
    % (0.353, -0.571, 0.484, 0.058). Old contact-height coordinate: (c0 + c1 + 1, c1 + 2, d1 + 2, d2 + d1 + 1).
    FIT = struct('plane', [102/127, -229/127, -689/1016, -1/8], 'cell', [0.353, -0.571, 0.484, 0.058]);
    SIG_TIGHT  = 0.05 * ones(1, 4);
    SIG_FAMILY = abs(FIT.cell - FIT.plane) / 2;      % half the plane-cell spread IN THIS BASIS: (0.225, 0.616, 0.581, 0.092);
                                                     % the [2/2] basis change is not a pure shift (c0, d2 mix with c1, d1), so the
                                                     % family prior differs from the 09-18 contact-height run (0.391, 0.616, 0.581, 0.673)
    SEG = [0.5, 1.5, 3.5];
    G_MIN = 1.1 - W_S;                                % band floor in the estimator's gap variable (denominator must stay positive there)
    law22 = @(g, th) (g.^2 + th(2) * g + th(1)) ./ (g.^2 + th(3) * g + th(4));
    out = struct();
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        plant_cperp = [];  if strcmp(pl, 'cell'); plant_cperp = @cell_cperp_fig11; end   % [2/2] line is closed (C59): keeps the data-fit cell its constants were fitted on; the new default test cell is cell_cperp_delta
        for ia = 1:numel(opts.arms)
            arm = opts.arms{ia};
            if strcmp(arm, 'P') && ~strcmp(pl, 'cell');  continue; end
            if strcmp(arm, 'N') && ~strcmp(pl, 'plane'); continue; end
            switch arm
                case 'T'; seed = FIT.(pl);   sig = SIG_TIGHT;
                case 'P'; seed = FIT.plane;  sig = SIG_FAMILY;
                case 'N'; seed = FIT.cell;   sig = SIG_FAMILY;
            end
            cc = local_ctrl_const(seed, sig, W_S);  cc.pade22_gmin = G_MIN;
            tag = sprintf('%s_A_%s', pl, arm);
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'scenario', 'deep', 'verbose', false, ...
                       'seeds', opts.seeds, 'log_P_full', true);
            if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
            fprintf('\n=== %s | w_s = %.1f, seed (c0, c1, d1, d2) = (%.3f, %.3f, %.3f, %.3f), a_bar(1.1) = %.3f, prior (%.2f, %.2f, %.2f, %.2f), %d seeds ===\n', ...
                    tag, W_S, seed, law22(1.1 - W_S, seed), sig, numel(opts.seeds));
            clear run_formC_b motion_control_law_formC_b;
            tic; evalc('Rr = run_formC_b(o);'); tr = toc;
            res = local_collect(Rr, plant_cperp, Ts);
            res.tag = tag; res.plant = pl; res.version = 'A'; res.arm = arm; res.seed = seed; res.sig = sig; res.seg = SEG;
            res.true_th = FIT.(pl);  res.W_S = W_S;
            local_report(res, SEG);
            fprintf('    (%.0f s)\n', tr);
            save(fullfile(od, [tag '.mat']), '-struct', 'res');
            out.(tag) = res;
        end
    end
end

function cc = local_ctrl_const(seed, sig, W_S)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 1e-5);
    cc.c0_state = true;  cc.pade22 = true;  cc.gain_algebraic = true;  cc.amodel_fe = true;  cc.amodel_h = true;
    cc.c0_init = seed(1);  cc.c1_init = seed(2);  cc.d1_init = seed(3);  cc.d2_init = seed(4);
    cc.Pf_c0_std = sig(1);  cc.Pf_c1_std = sig(2);  cc.Pf_d1_std = sig(3);  cc.Pf_d2_std = sig(4);
    cc.ws0_perp = W_S;  cc.Pf_w0_std = 0;
    cc.b_init = 1 / max(seed(3) - seed(2), 0.05);  cc.Pf_b_std = sig(3);   % slot-5 seed/prior are overwritten by c0_state; kept finite
end

function res = local_collect(Rr, plant_cperp, Ts)
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:), 'ns', ns);
    res.h_true = zeros(n, ns); res.a_true = zeros(n, ns); res.a_hat = zeros(n, ns);
    TH = {'c0', 'c1', 'd1', 'd2'};  SLOT = [7, 8, 5, 9];
    for j = 1:4; res.([TH{j} '_hat']) = zeros(n, ns); res.(['sP_' TH{j}]) = zeros(n, ns); end
    res.innov1 = zeros(n, ns); res.innov2 = zeros(n, ns); res.S1 = zeros(n, ns); res.S2 = zeros(n, ns);
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);
        at = zeros(n, 1);
        for i = 1:n
            h = max(hb(i), 1.001);
            if isempty(plant_cperp); [~, c] = calc_correction_functions(h); else; c = plant_cperp(h); end
            at(i) = 1 / c;
        end
        res.h_true(:, q) = hb;  res.a_true(:, q) = at;  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        res.c0_hat(:, q) = S.ws_hat_out(:, 3);  res.c1_hat(:, q) = S.c1_hat_out(:, 3);
        res.d1_hat(:, q) = S.b_hat_out(:, 3);   res.d2_hat(:, q) = S.d2_hat_out(:, 3);
        PF = S.P_full_out(:, :, :, 3);
        for j = 1:4; res.(['sP_' TH{j}])(:, q) = sqrt(max(PF(:, SLOT(j), SLOT(j)), 0)); end
        res.innov1(:, q) = S.innov_y1_out(:, 3);  res.innov2(:, q) = S.innov_y2_out(:, 3);
        if isfield(S, 'S1_out'); res.S1(:, q) = S.S1_out(:, 3);  res.S2(:, q) = S.S2_out(:, 3); end
    end
end

function local_report(res, SEG)
    t = res.t;  ea = res.a_true - res.a_hat;
    m = mean(ea, 2);  s = std(ea, 0, 2);
    desc = t >= SEG(1) & t < SEG(2);  osc = t >= SEG(2) & t < SEG(3);  hold = t >= SEG(3);
    [~, iw] = max(abs(m) .* desc);
    Dfloor = 1.1^2 + res.d1_hat * 1.1 + res.d2_hat;                       % denominator at the band floor x = 1.1 (w_s = 0)
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d | min D(1.1) %.4f (guard 0.002)\n', min(res.h_true(:)), ...
            nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), min(Dfloor(:)));
    fprintf('  e_a    : descent worst %+.4f at t = %.2f s (sd %.4f) | descent mean %+.4f sd %.4f | osc mean %+.4f sd %.4f | hold mean %+.4f sd %.4f\n', ...
            m(iw), t(iw), s(iw), mean(m(desc)), mean(s(desc)), mean(m(osc)), mean(s(osc)), mean(m(hold)), mean(s(hold)));
    TH = {'c0', 'c1', 'd1', 'd2'};
    for j = 1:4
        x = res.([TH{j} '_hat']);  sp = res.(['sP_' TH{j}]);
        walk = max(abs(x - x(1, :)), [], 1);
        fprintf('  %-2s     : start %.3f | end %.3f +- %.3f (seed sd) | true %.3f | sup|walk| %.3f | sqrtP end %.3f | honesty sd/sqrtP %.2f\n', ...
                TH{j}, x(1, 1), mean(x(end, :)), std(x(end, :)), res.true_th(j), mean(walk), mean(sp(end, :)), std(x(end, :)) / max(mean(sp(end, :)), eps));
    end
    S2 = res.S2;  nis2 = res.innov2.^2 ./ S2;  nis2 = nis2(isfinite(nis2) & S2 > 0);
    if ~isempty(nis2); fprintf('  NIS2   : mean %.2f\n', mean(nis2)); end
end
