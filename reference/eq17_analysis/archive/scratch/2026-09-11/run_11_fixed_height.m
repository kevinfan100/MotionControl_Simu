% PURPOSE (2026-09-18, user: "fixed-height test first: can the information at one height alone converge the
%   constants?"): the [1/1] two-version estimator held at ONE height for the whole run (trajectory 'positioning', no
%   descent, no velocity), wrong-wall seeds, both plants, three heights of the canon deep trajectory.
%     plants / seeds   cell plant  <- plane seed (-0.984, 0.161)    (the P question)      [w_s = wall surface, 2026-09-18]
%                      plane plant <- cell  seed (0.056, 1.197)     (the N question)      [old coordinate: +1 on each constant]
%     heights          22.2 R (start of the trajectory), 2.0 R (band middle), 1.10 R (band floor)
%     versions         A  (c0, d1) states, gain computed (c0_state)   B  a slot along the law (production pure)
%   Pre-registered (0916_estb_11_two_versions.tex S9): at one height y2 gives ONE number a(h), so only the combination
%   c0 - a d1 = g (a - 1) is learnable -- (c0_hat, d1_hat) move along that line and stop where the prior puts them,
%   a_hat -> a_true, the (c0, d1) block of P keeps one direction at the prior; B cannot learn d1 at all in a hold
%   (F_e(4,5) ~ M = 0, H25 ~ grad = 0), only its a slot follows.
%   Printed per run: e_a steady state (last 1 s), sqrt(P_a); the along-line residual r = c0_hat - a d1_hat - g (a - 1)
%   and its sqrt(P); the orthogonal direction's sqrt(P) against the prior; NIS2.
%   Output: test_results/temp_11_fixed_height/<plant>_<version>_h<height>.mat + console; figures by plot_11_fixed_height.m.
%   EXPIRES: with the [1/1] two-wall line | 產線改動不會自動跟上
function out = run_11_fixed_height(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:10; end
    if ~isfield(opts, 'plants');   opts.plants   = {'cell', 'plane'}; end
    if ~isfield(opts, 'versions'); opts.versions = {'A', 'B'}; end
    if ~isfield(opts, 'heights');  opts.heights  = [22.2, 2.0, 1.10]; end     % [R]
    if ~isfield(opts, 'T_hold');   opts.T_hold   = 3.0; end                    % [s]
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_11_fixed_height');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;  W_S = 0.0;          % w_s = wall surface (2026-09-18; was the contact height 1)
    FIT = struct('plane', [-0.984, 0.161], 'cell', [0.0501, 1.1871]);        % (c0, d1), w_s = wall surface; cell = band fit on cell_cperp_delta (2026-09-20; data-fit plant: 0.056, 1.197)
    SIG = [0.52, 0.52];
    out = struct();
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        plant_cperp = [];  if strcmp(pl, 'cell'); plant_cperp = @cell_cperp_delta; end   % 2026-09-20: default cell = plane slid 1.05 R (cell_cperp_delta); data fit via opts.plant_cperp_cell
        if strcmp(pl, 'cell'); seed_cd = FIT.plane; else; seed_cd = FIT.cell; end     % the OTHER wall's constants
        for ih = 1:numel(opts.heights)
            h = opts.heights(ih);
            if isempty(plant_cperp); [~, c] = calc_correction_functions(h); else; c = plant_cperp(h); end
            a_true_h = 1 / c;  g = h - W_S;
            for iv = 1:numel(opts.versions)
                ver = opts.versions{iv};
                cc = local_ctrl_const(ver, seed_cd, SIG, W_S);
                % positioning = hold at h_init for the whole run; the timing fields below only place the driver's
                % metric windows (descent [0, 0] must exist or its per-seed table gets an empty column)
                OV = struct('trajectory_type', 'positioning', 'h_init', h * R, 'T_sim', opts.T_hold, ...
                            'h_bottom', h * R, 'amplitude', 0, 't_hold', 0, 't_descend_override', 0);
                tag = sprintf('%s_%s_h%s', pl, ver, strrep(sprintf('%.2f', h), '.', 'p'));
                o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', ...
                           'verbose', false, 'seeds', opts.seeds, 'log_P_full', true);
                if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
                fprintf('\n=== %s | hold at w = %.2f R (a_true %.4f) | seed (c0, d1) = (%.3f, %.3f), prior (%.2f, %.2f), %d seeds ===\n', ...
                        tag, h, a_true_h, seed_cd(1), seed_cd(2), SIG(1), SIG(2), numel(opts.seeds));
                clear run_formC_b motion_control_law_formC_b;
                tic; evalc('Rr = run_formC_b(o);'); tr = toc;
                res = local_collect(Rr, ver, seed_cd, W_S, plant_cperp, Ts);
                res.tag = tag; res.plant = pl; res.version = ver; res.h = h; res.g = g; res.a_true_h = a_true_h;
                res.seed_cd = seed_cd; res.sig = SIG; res.true_cd = FIT.(pl);
                local_report(res);
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
        case 'A'
            cc = pure;
            cc.c0_state = true;  cc.gain_algebraic = true;  cc.amodel_fe = true;  cc.amodel_h = true;
            cc.c0_init = c0;  cc.d1_init = d1;  cc.Pf_c0_std = sig(1);  cc.Pf_d1_std = sig(2);
            cc.ws0_perp = W_S;  cc.Pf_w0_std = 0;
            cc.b_init = 1 / kap;  cc.Pf_b_std = sig(2);
        case 'B'
            cc = pure;
            cc.b_init = 1 / kap;  cc.ws0_perp = W_S - d1 + 1;
            cc.Pf_b_std = sig(2) / kap^2;  cc.Pf_w0_std = sig(1);
    end
end

function res = local_collect(Rr, ver, seed_cd, W_S, plant_cperp, Ts)
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:), 'ns', ns);
    res.h_true = zeros(n, ns); res.a_true = zeros(n, ns); res.a_hat = zeros(n, ns);
    res.c0_hat = zeros(n, ns); res.d1_hat = zeros(n, ns); res.sP_c0 = zeros(n, ns); res.sP_d1 = zeros(n, ns);
    res.sP_a = zeros(n, ns); res.innov2 = zeros(n, ns); res.S2 = nan(n, ns);  res.dw3_hat = zeros(n, ns);
    res.P_cd = zeros(n, 3, ns);                       % A: [P_c0c0, P_c0d1, P_d1d1] in (c0, d1); B: same block mapped from (w0, b)
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);  at = zeros(n, 1);
        for i = 1:n
            hh = max(hb(i), 1.001);
            if isempty(plant_cperp); [~, c] = calc_correction_functions(hh); else; c = plant_cperp(hh); end
            at(i) = 1 / c;
        end
        res.h_true(:, q) = hb;  res.a_true(:, q) = at;  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        res.dw3_hat(:, q) = S.delta_x_hat_3_out(:, 3) / S.R;     % logged in um -> R
        PF = S.P_full_out(:, :, :, 3);                % n x np x np, z axis
        switch ver
            case 'A'
                res.c0_hat(:, q) = S.ws_hat_out(:, 3);  res.d1_hat(:, q) = S.b_hat_out(:, 3);
                res.sP_c0(:, q)  = S.P_ws_out(:, 3);    res.sP_d1(:, q)  = S.P_b_out(:, 3);
                res.P_cd(:, :, q) = [PF(:, 7, 7), PF(:, 7, 5), PF(:, 5, 5)];
                % gain uncertainty rebuilt from the constants: a = (g + c0)/(g + d1), J = [1, -a]/(g + d1)
                gg = S.h_bar_d_out(:) - res.dw3_hat(:, q) - W_S;  den = max(gg + res.d1_hat(:, q), 1e-2);
                J1 = 1 ./ den;  J2 = -res.a_hat(:, q) ./ den;
                res.sP_a(:, q) = sqrt(max(J1.^2 .* PF(:, 7, 7) + 2 * J1 .* J2 .* PF(:, 7, 5) + J2.^2 .* PF(:, 5, 5), 0)) * S.a_nom;
            case 'B'
                b = S.b_hat_out(:, 3);
                res.c0_hat(:, q) = seed_cd(1) * ones(n, 1);  res.d1_hat(:, q) = seed_cd(1) + 1 ./ b;
                res.sP_c0(:, q)  = zeros(n, 1);              res.sP_d1(:, q)  = S.P_b_out(:, 3) ./ b.^2;
                res.P_cd(:, :, q) = [zeros(n, 1), zeros(n, 1), PF(:, 5, 5) ./ b.^4];
                res.sP_a(:, q) = S.P_a_out(:, 3);                  % already sqrt(P_a) [um/pN] per driver
        end
        res.a_nom = S.a_nom;
        res.innov2(:, q) = S.innov_y2_out(:, 3);
        if isfield(S, 'S2_out'); res.S2(:, q) = S.S2_out(:, 3); end
    end
    res.W_S = W_S;
end

function local_report(res)
    t = res.t;  last = t >= t(end) - 1.0;
    ea = res.a_true - res.a_hat;  m = mean(ea, 2);  s = std(ea, 0, 2);
    a = res.a_true_h;  g = res.g;
    r = res.c0_hat - a * res.d1_hat - g * (a - 1);               % along-line residual: 0 when the law reproduces a(h)
    % orthogonal (unlearnable) direction [a, 1]/sqrt(1+a^2) applied to the deviation from the seed
    u = [a, 1] / sqrt(1 + a^2);
    dev = u(1) * (res.c0_hat - res.seed_cd(1)) + u(2) * (res.d1_hat - res.seed_cd(2));
    % sqrt(P) along the unlearnable direction u = [a, 1] and along the learnable one v = [1, -a], at the end
    Pe = squeeze(res.P_cd(end, :, :));  if isvector(Pe); Pe = Pe(:); end          % 3 x ns
    v = [1, -a] / sqrt(1 + a^2);
    sPu = sqrt(max(u(1)^2 * Pe(1, :) + 2 * u(1) * u(2) * Pe(2, :) + u(2)^2 * Pe(3, :), 0));
    sPv = sqrt(max(v(1)^2 * Pe(1, :) + 2 * v(1) * v(2) * Pe(2, :) + v(2)^2 * Pe(3, :), 0));
    P0u = sqrt(u(1)^2 * res.sig(1)^2 + u(2)^2 * res.sig(2)^2);  P0v = sqrt(v(1)^2 * res.sig(1)^2 + v(2)^2 * res.sig(2)^2);
    tr = t <= 0.1;
    fprintf('  health : min h_true %.4f (first 0.1 s %.4f) | NaN %d | floor hits %d | min(d1-c0) %.4f | dw3_hat end %+.4f R\n', ...
            min(res.h_true(:)), min(min(res.h_true(tr, :))), nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), ...
            min(res.d1_hat(:) - res.c0_hat(:)), mean(res.dw3_hat(end, :)));
    fprintf('  e_a    : steady (last 1 s) mean %+.4f sd_seed %.4f | worst |mean| %.4f at t = %.3f s | after 0.1 s worst |mean| %.4f | sqrtP_a end %.4f\n', ...
            mean(m(last)), mean(s(last)), max(abs(m)), t(find(abs(m) == max(abs(m)), 1)), max(abs(m(~tr))), mean(res.sP_a(end, :)) / res.a_nom);
    fprintf('  line   : residual r = c0 - a d1 - g(a-1): start %+.3f -> end %+.4f +- %.4f (seed sd) | true point r = %+.4f | sqrtP along [1,-a]: %.3f (prior %.2f)\n', ...
            mean(r(1, :)), mean(r(end, :)), std(r(end, :)), res.true_cd(1) - a * res.true_cd(2) - g * (a - 1), mean(sPv), P0v);
    fprintf('  orth   : movement along the unlearnable direction [a, 1]: end %+.3f +- %.3f | sqrtP along it %.3f (prior %.2f)\n', ...
            mean(dev(end, :)), std(dev(end, :)), mean(sPu), P0u);
    for th = {'c0', 'd1'}
        x = res.([th{1} '_hat']);  sp = res.(['sP_' th{1}]);
        fprintf('  %-2s     : start %.3f | end %.3f +- %.3f (seed sd) | true %.3f | sqrtP end %.3f\n', ...
                th{1}, x(1, 1), mean(x(end, :)), std(x(end, :)), res.true_cd(strcmp(th{1}, 'd1') + 1), mean(sp(end, :)));
    end
    nis2 = res.innov2.^2 ./ res.S2;  nis2 = nis2(isfinite(nis2));
    if ~isempty(nis2); fprintf('  NIS2   : mean %.2f (design ~0.3 with the IF colour, 1.0 white)\n', mean(nis2)); end
end
