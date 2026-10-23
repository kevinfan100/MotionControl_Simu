% PURPOSE (2026-09-18, user: "speed scan, three clearly different speeds, simple judgement first"): the [1/1] version-A
%   estimator (c0, d1 states, gain computed) driven over the SAME height interval at three descent speeds, wrong-wall
%   seeds, both plants. Separates "the heights walked through" (one line c0 - a d1 = g (a - 1) per height, speed-free,
%   measured by the fixed-height test) from "the speed" (the y1 path F_e(3,4:5) ~ F_dw ~ velocity, zero in a hold).
%     trajectory   hold 0.5 s at h_init = 5.5 R (band top) -> cosine descent to 1.10 R over T_d -> hold 2 s
%     T_d          0.2 s (peak 35 R/s, canon-like) | 2 s (3.5 R/s) | 20 s (0.35 R/s, Meng-ramp-like)
%     plants/seeds cell <- plane seed (-0.984, 0.161); plane <- cell seed (0.056, 1.197); prior 0.52; version A only
%                  (w_s = wall surface, 2026-09-18; old contact-height coordinate: +1 on each constant, bit-identical)
%   Pre-registered: y1 information ~ v^2 per step x steps ~ 1/v => ~ v (faster better); y2 gives one line per height,
%   ~ 1/v (slower better). The end (c0_hat, d1_hat) scatter versus T_d should show both; the channel split of the
%   constants' movement (slot 7 via dws_y1/dws_y2, slot 5 via K_b_y1*innov1 / K_b_y2*innov2) is the direct measure of
%   the speed's share (y1 share = speed share).
%   Output test_results/temp_11_speed_scan/<plant>_<ver>_Td<T>.mat + console; figures by plot_11_speed_scan.m.
%   EXPIRES: with the [1/1] two-wall line | 產線改動不會自動跟上
function out = run_11_speed_scan(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:10; end
    if ~isfield(opts, 'plants');   opts.plants   = {'cell', 'plane'}; end
    if ~isfield(opts, 'versions'); opts.versions = {'A'}; end
    if ~isfield(opts, 'T_d');      opts.T_d      = [0.2, 2, 20]; end          % [s]
    if ~isfield(opts, 'h_init');   opts.h_init   = 5.5; end                   % [R]
    if ~isfield(opts, 'h_bottom'); opts.h_bottom = 1.10; end                  % [R]
    if ~isfield(opts, 'T_after');  opts.T_after  = 2.0; end                   % [s] hold after the descent
    if ~isfield(opts, 't_hold');   opts.t_hold   = 0.5; end                   % [s] hold before the descent
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_11_speed_scan');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;  W_S = 0.0;          % w_s = wall surface (2026-09-18; was the contact height 1)
    FIT = struct('plane', [-0.984, 0.161], 'cell', [0.0501, 1.1871]);        % (c0, d1), w_s = wall surface; cell = band fit on cell_cperp_delta (2026-09-20; data-fit plant: 0.056, 1.197)
    SIG = [0.52, 0.52];
    out = struct();
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        plant_cperp = [];  if strcmp(pl, 'cell'); plant_cperp = @cell_cperp_delta; end   % 2026-09-20: default cell = plane slid 1.05 R (cell_cperp_delta); data fit via opts.plant_cperp_cell
        if strcmp(pl, 'cell'); seed_cd = FIT.plane; else; seed_cd = FIT.cell; end
        for it = 1:numel(opts.T_d)
            Td = opts.T_d(it);
            for iv = 1:numel(opts.versions)
                ver = opts.versions{iv};
                cc = local_ctrl_const(ver, seed_cd, SIG, W_S);
                T_sim = opts.t_hold + Td + opts.T_after;
                OV = struct('h_init', opts.h_init * R, 'h_bottom', opts.h_bottom * R, 'amplitude', 0, 'n_cycles', 0, ...
                            't_hold', opts.t_hold, 't_descend_override', Td, 'T_sim', T_sim);
                tag = sprintf('%s_%s_Td%s', pl, ver, strrep(sprintf('%g', Td), '.', 'p'));
                o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', ...
                           'verbose', false, 'seeds', opts.seeds, 'log_P_full', true);
                if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
                fprintf('\n=== %s | %.1f R -> %.2f R over T_d = %g s (peak %.1f R/s), hold %.1f s after | seed (c0, d1) = (%.3f, %.3f), prior %.2f, %d seeds ===\n', ...
                        tag, opts.h_init, opts.h_bottom, Td, pi/2 * (opts.h_init - opts.h_bottom) / Td, opts.T_after, seed_cd(1), seed_cd(2), SIG(1), numel(opts.seeds));
                clear run_formC_b motion_control_law_formC_b;
                tic; evalc('Rr = run_formC_b(o);'); tr = toc;
                res = local_collect(Rr, ver, seed_cd, W_S, plant_cperp, Ts);
                res.tag = tag; res.plant = pl; res.version = ver; res.Td = Td; res.h_init = opts.h_init; res.h_bottom = opts.h_bottom;
                res.t_hold = opts.t_hold; res.T_after = opts.T_after;
                res.seed_cd = seed_cd; res.sig = SIG; res.true_cd = FIT.(pl);
                res.a_true_top = local_atrue(pl, opts.h_init, plant_cperp);  res.a_true_bot = local_atrue(pl, opts.h_bottom, plant_cperp);
                local_report(res);
                fprintf('    (%.0f s)\n', tr);
                save(fullfile(od, [tag '.mat']), '-struct', 'res');
                out.(tag) = res;
            end
        end
    end
end

function at = local_atrue(pl, h, plant_cperp)
    if strcmp(pl, 'cell'); at = 1 / plant_cperp(h); else; [~, c] = calc_correction_functions(h); at = 1 / c; end
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
    res.P_cd = zeros(n, 3, ns);
    % per-step increments of the two constants from each channel (A: slot 7 = c0, slot 5 = d1)
    res.dc0_y1 = zeros(n, ns); res.dc0_y2 = zeros(n, ns); res.dd1_y1 = zeros(n, ns); res.dd1_y2 = zeros(n, ns);
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);  at = zeros(n, 1);
        for i = 1:n
            hh = max(hb(i), 1.001);
            if isempty(plant_cperp); [~, c] = calc_correction_functions(hh); else; c = plant_cperp(hh); end
            at(i) = 1 / c;
        end
        res.h_true(:, q) = hb;  res.a_true(:, q) = at;  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        res.dw3_hat(:, q) = S.delta_x_hat_3_out(:, 3) / S.R;
        PF = S.P_full_out(:, :, :, 3);
        switch ver
            case 'A'
                res.c0_hat(:, q) = S.ws_hat_out(:, 3);  res.d1_hat(:, q) = S.b_hat_out(:, 3);
                res.sP_c0(:, q)  = S.P_ws_out(:, 3);    res.sP_d1(:, q)  = S.P_b_out(:, 3);
                res.P_cd(:, :, q) = [PF(:, 7, 7), PF(:, 7, 5), PF(:, 5, 5)];
                gg = S.h_bar_d_out(:) - res.dw3_hat(:, q) - W_S;  den = max(gg + res.d1_hat(:, q), 1e-2);
                J1 = 1 ./ den;  J2 = -res.a_hat(:, q) ./ den;
                res.sP_a(:, q) = sqrt(max(J1.^2 .* PF(:, 7, 7) + 2 * J1 .* J2 .* PF(:, 7, 5) + J2.^2 .* PF(:, 5, 5), 0)) * S.a_nom;
                res.dc0_y1(:, q) = S.dws_y1_out(:, 3);  res.dc0_y2(:, q) = S.dws_y2_out(:, 3);
                res.dd1_y1(:, q) = S.K_b_y1_out(:, 3) .* S.innov_y1_out(:, 3);
                res.dd1_y2(:, q) = S.K_b_y2_out(:, 3) .* S.innov_y2_out(:, 3);
            case 'B'
                b = S.b_hat_out(:, 3);
                res.c0_hat(:, q) = seed_cd(1) * ones(n, 1);  res.d1_hat(:, q) = seed_cd(1) + 1 ./ b;
                res.sP_c0(:, q)  = zeros(n, 1);              res.sP_d1(:, q)  = S.P_b_out(:, 3) ./ b.^2;
                res.P_cd(:, :, q) = [zeros(n, 1), zeros(n, 1), PF(:, 5, 5) ./ b.^4];
                res.sP_a(:, q) = S.P_a_out(:, 3);
                res.dd1_y1(:, q) = -(S.K_b_y1_out(:, 3) .* S.innov_y1_out(:, 3)) ./ b.^2;
                res.dd1_y2(:, q) = -(S.K_b_y2_out(:, 3) .* S.innov_y2_out(:, 3)) ./ b.^2;
        end
        res.a_nom = S.a_nom;
        res.innov2(:, q) = S.innov_y2_out(:, 3);
        if isfield(S, 'S2_out'); res.S2(:, q) = S.S2_out(:, 3); end
    end
    res.W_S = W_S;
end

function local_report(res)
    t = res.t;  t1 = res.t_hold;  t2 = t1 + res.Td;
    desc = t >= t1 & t <= t2;  after = t > t2;  last = t >= t(end) - 1.0;
    ea = res.a_true - res.a_hat;  m = mean(ea, 2);  s = std(ea, 0, 2);
    [wm, iw] = max(abs(m(desc)));  td = t(desc);
    % time after arrival until |mean e_a| stays below 0.01
    ma = abs(m(after));  ta = t(after);  k = find(ma > 0.01, 1, 'last');
    if isempty(k); t_in = ta(1) - t2; elseif k == numel(ma); t_in = NaN; else; t_in = ta(k + 1) - t2; end
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d | min(d1-c0) %.4f | max |dw3_hat| %.4f R\n', min(res.h_true(:)), ...
            nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), min(res.d1_hat(:) - res.c0_hat(:)), max(abs(res.dw3_hat(:))));
    fprintf('  e_a    : descent worst |mean| %.4f at t = %.3f s (w_d %.2f R), descent sd_seed max %.4f | after arrival: into 0.01 after %.2f s | hold (last 1 s) %+.4f sd_seed %.4f\n', ...
            wm, td(iw), res.wd(find(desc, 1) + iw - 1), max(s(desc)), t_in, mean(m(last)), mean(s(last)));
    for th = {'c0', 'd1'}
        x = res.([th{1} '_hat']);  sp = res.(['sP_' th{1}]);
        fprintf('  %-2s     : start %.3f | end of descent %.3f +- %.3f | end %.3f +- %.3f (seed sd) | true (band fit) %.3f | sqrtP end %.3f\n', ...
                th{1}, x(1, 1), mean(x(find(desc, 1, 'last'), :)), std(x(find(desc, 1, 'last'), :)), mean(x(end, :)), std(x(end, :)), ...
                res.true_cd(strcmp(th{1}, 'd1') + 1), mean(sp(end, :)));
    end
    % residuals on the two boundary lines
    HA = [res.h_init, res.a_true_top; res.h_bottom, res.a_true_bot];
    for i = 1:2
        h = HA(i, 1); a = HA(i, 2); g = h - res.W_S;
        r = res.c0_hat(end, :) - a * res.d1_hat(end, :) - g * (a - 1);
        fprintf('  line %.2f R: r = c0 - a d1 - g(a-1) end %+.4f +- %.4f (band-fit point %+.4f)\n', h, mean(r), std(r), res.true_cd(1) - a * res.true_cd(2) - g * (a - 1));
    end
    % channel split of the constants' movement over the descent and over the whole run
    for th = {'c0', 'd1'}
        y1 = res.(['d' th{1} '_y1']);  y2 = res.(['d' th{1} '_y2']);
        n1d = mean(sum(y1(desc, :), 1)); n2d = mean(sum(y2(desc, :), 1));
        n1a = mean(sum(y1, 1));          n2a = mean(sum(y2, 1));
        q1d = mean(sqrt(sum(y1(desc, :).^2, 1))); q2d = mean(sqrt(sum(y2(desc, :).^2, 1)));
        fprintf('  %-2s move: descent net y1 %+.3f, y2 %+.3f (rms-sum y1 %.3f, y2 %.3f, y1 share %.2f) | whole run net y1 %+.3f, y2 %+.3f\n', ...
                th{1}, n1d, n2d, q1d, q2d, q1d^2 / max(q1d^2 + q2d^2, eps), n1a, n2a);
    end
    nis2 = res.innov2.^2 ./ res.S2;  nis2 = nis2(isfinite(nis2));
    if ~isempty(nis2); fprintf('  NIS2   : mean %.2f\n', mean(nis2)); end
end
