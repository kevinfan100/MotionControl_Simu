% PURPOSE (2026-09-20): the two convergence tests of the offset law (0920_estb_delta_law.tex, version A) that the [1/1]
%   line ran as run_11_fixed_height.m / run_11_speed_scan.m, now with the seed = plant delta +- 0.5 R (plant-based init
%   with a known perturbation, user 09-20):
%     fixed   positioning at ONE height for T_hold s (no velocity): heights 2.0 R and 1.10 R above the reference surface.
%             Paper gate 3: y1 alone loses delta in a hold (F_dw = 0) but the y2 row keeps it wherever a'(g_d) != 0, so
%             delta_hat must converge from the level alone -- unlike [1/1], where one height pins only a line (C57).
%             Reported: time for |delta_hat - target| to fall below sqrt(P) and below 0.05 R; end error, sqrt(P), e_a last 1 s.
%     speed   hold 0.5 s at 5.5 R -> cosine descent to 1.10 R over T_d in {0.2, 2, 20} s -> hold 2 s (as run_11_speed_scan).
%             Reported: delta_hat at the end of the descent and at the end, e_a per segment; the y1/y2 share is read from
%             the fixed test (y2 only) against the fast descent (y1 dominated), not from a channel split.
%   Both plants, seed = plant delta (sign 'T'; 'Tp'/'Tm' = +-0.5 R perturbation arms kept as an option), 10 seeds, canon deep noise. Output test_results/temp_delta_two_walls/
%   fixed_<plant>_<sign>_h<h>.mat, speed_<plant>_<sign>_Td<T>.mat + console; figures by plot_delta_fixed_speed.m.
%   EXPIRES: with the delta-law line | 產線改動不會自動跟上
function out = run_delta_fixed_speed(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds   = 1:10; end
    if ~isfield(opts, 'plants');  opts.plants  = {'plane', 'cell'}; end
    if ~isfield(opts, 'signs');   opts.signs   = {'T'}; end                  % 'T' = seed = plant delta (user 09-20: every test seeded from the plant); 'Tp'/'Tm' = +-0.5 R perturbation arms
    if ~isfield(opts, 'heights'); opts.heights = [2.0, 1.10]; end             % [R], fixed test
    if ~isfield(opts, 'T_hold');  opts.T_hold  = 3.0; end                     % [s], fixed test
    if ~isfield(opts, 'T_d');     opts.T_d     = [0.2, 2, 20]; end            % [s], speed test
    if ~isfield(opts, 'do_fixed'); opts.do_fixed = true; end
    if ~isfield(opts, 'do_speed'); opts.do_speed = true; end
    if ~isfield(opts, 'a_ctrl_override'); opts.a_ctrl_override = []; end   % 'true' = the CONTROL applies the exact gain while the estimator runs freely (oracle-control discriminator)
    if ~isfield(opts, 'tag_suffix'); opts.tag_suffix = ''; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_delta_two_walls');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    W_S = 0.0;  DEL = struct('plane', 0.0, 'cell', 1.05);  SIG = 0.5;  OFF = 0.5;
    H_TOP = 5.5;  H_BOT = 1.10;  T_HOLD0 = 0.5;  T_AFTER = 2.0;
    out = struct();
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        plant_cperp = [];  if strcmp(pl, 'cell'); plant_cperp = @cell_cperp_delta; end
        for is = 1:numel(opts.signs)
            sg = opts.signs{is};  dl_seed = DEL.(pl) + OFF * (strcmp(sg, 'Tp') - strcmp(sg, 'Tm'));   % T: 0, Tp: +OFF, Tm: -OFF
            if startsWith(sg, 'PI'); dl_seed = DEL.plane; end
            if startsWith(sg, 'MID'); dl_seed = (DEL.plane + DEL.cell) / 2; end                    % family-midpoint init: plane and cell each ~1 sigma away                                       % 'PI_iekf' adds the iterated y2 update (O38)                                              % PI: PLANE INIT, delta_hat[0] = 0 whatever the plant (user 09-20)
            cc = local_ctrl_const(dl_seed, SIG, W_S);  cc.delta_iekf = contains(sg, 'iekf') || contains(sg, 'slin');  cc.delta_slin = contains(sg, 'slin');
            if opts.do_fixed
                for ih = 1:numel(opts.heights)
                    h = opts.heights(ih);
                    OV = struct('trajectory_type', 'positioning', 'h_init', h * R, 'T_sim', opts.T_hold, ...
                                'h_bottom', h * R, 'amplitude', 0, 't_hold', 0, 't_descend_override', 0);
                    tag = sprintf('fixed_%s_%s_h%s', pl, sg, strrep(sprintf('%.2f', h), '.', 'p'));
                    tag = [tag opts.tag_suffix];
                    o = o_base(cc, OV, opts.seeds, plant_cperp);  if ~isempty(opts.a_ctrl_override); o.a_ctrl_override = opts.a_ctrl_override; end
                    res = local_run(o, plant_cperp, Ts);
                    res.tag = tag; res.plant = pl; res.sign = sg; res.h = h; res.dl_seed = dl_seed; res.dl_target = DEL.(pl); res.sig = SIG;
                    res.kind = 'fixed';  res.seg = [0, 0, 0];
                    fprintf('\n=== %s | hold at w = %.2f R (a_true %.4f, a'' %.4f) | seed delta %.2f -> target %.2f, prior %.2f, %d seeds ===\n', ...
                            tag, h, local_a(h - W_S + DEL.(pl)), local_ap(h - W_S + DEL.(pl)), dl_seed, DEL.(pl), SIG, numel(opts.seeds));
                    local_report_fixed(res);
                    save(fullfile(od, [tag '.mat']), '-struct', 'res');  out.(tag) = res;
                end
            end
            if opts.do_speed
                for it = 1:numel(opts.T_d)
                    Td = opts.T_d(it);  T_sim = T_HOLD0 + Td + T_AFTER;
                    OV = struct('h_init', H_TOP * R, 'h_bottom', H_BOT * R, 'amplitude', 0, 'n_cycles', 0, ...
                                't_hold', T_HOLD0, 't_descend_override', Td, 'T_sim', T_sim);
                    tag = sprintf('speed_%s_%s_Td%s', pl, sg, strrep(sprintf('%g', Td), '.', 'p'));
                    res = local_run(o_base(cc, OV, opts.seeds, plant_cperp), plant_cperp, Ts);
                    res.tag = tag; res.plant = pl; res.sign = sg; res.Td = Td; res.dl_seed = dl_seed; res.dl_target = DEL.(pl); res.sig = SIG;
                    res.kind = 'speed';  res.seg = [T_HOLD0, T_HOLD0 + Td, T_sim];
                    fprintf('\n=== %s | %.1f R -> %.2f R over T_d = %g s (peak %.1f R/s), hold %.1f s after | seed delta %.2f -> target %.2f, %d seeds ===\n', ...
                            tag, H_TOP, H_BOT, Td, pi/2 * (H_TOP - H_BOT) / Td, T_AFTER, dl_seed, DEL.(pl), numel(opts.seeds));
                    local_report_speed(res);
                    save(fullfile(od, [tag '.mat']), '-struct', 'res');  out.(tag) = res;
                end
            end
        end
    end
end

function o = o_base(cc, OV, seeds, plant_cperp)
    o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', ...
               'verbose', false, 'seeds', seeds, 'log_P_full', false);
    if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
end

function cc = local_ctrl_const(dl_seed, sig, ws)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', true, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 1e-5);
    cc.delta_state = true;  cc.gain_algebraic = true;  cc.amodel_fe = true;  cc.amodel_h = true;
    cc.delta_init = dl_seed;  cc.Pf_delta_std = sig;  cc.ws0_perp = ws;  cc.Pf_w0_std = 0;
    cc.b_init = 8/9;  cc.Pf_b_std = 0;
end

function a = local_a(g)
    u = 1 / max(g, 1);  a = 1 - (9/8)*u + (1/2)*u^3 - (57/100)*u^4 + (1/5)*u^5 + (7/200)*u^11 - (1/25)*u^12;
end
function ap = local_ap(g)
    u = 1 / max(g, 1);  Dp = -(9/8) + (3/2)*u^2 - (228/100)*u^3 + u^4 + (77/200)*u^10 - (12/25)*u^11;  ap = -u^2 * Dp;
end

function res = local_run(o, plant_cperp, Ts)
    clear run_formC_b motion_control_law_formC_b;
    tic; evalc('Rr = run_formC_b(o);'); tr = toc;
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:), 'ns', ns, 'runtime_s', tr);
    res.h_true = zeros(n, ns); res.a_true = zeros(n, ns); res.a_hat = zeros(n, ns); res.dl_hat = zeros(n, ns); res.sP_dl = zeros(n, ns);
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);  at = zeros(n, 1);
        for i = 1:n
            h = max(hb(i), 1.001);
            if isempty(plant_cperp); [~, c] = calc_correction_functions(h); else; c = plant_cperp(h); end
            at(i) = 1 / c;
        end
        res.h_true(:, q) = hb;  res.a_true(:, q) = at;  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        res.dl_hat(:, q) = S.ws_hat_out(:, 3);  res.sP_dl(:, q) = S.P_ws_out(:, 3);
    end
end

function local_report_fixed(res)
    t = res.t;  e = res.dl_hat - res.dl_target;  sp = res.sP_dl;  ea = res.a_true - res.a_hat;
    last = t >= t(end) - 1.0;
    t_sp = nan(1, res.ns);  t_05 = nan(1, res.ns);
    for q = 1:res.ns
        k = find(abs(e(:, q)) <= sp(:, q), 1, 'first');   if ~isempty(k); t_sp(q) = t(k); end
        k = find(abs(e(:, q)) <= 0.05, 1, 'first');       if ~isempty(k); t_05(q) = t(k); end
    end
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d\n', min(res.h_true(:)), nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9));
    fprintf('  delta  : end error %+.3f +- %.3f (seed sd) | sqrtP end %.3f | honesty sd/sqrtP %.2f | t(|err| < sqrtP) %.2f s (median, %d/%d reached) | t(|err| < 0.05) %.2f s (median, %d/%d)\n', ...
            mean(e(end, :)), std(e(end, :)), mean(sp(end, :)), std(e(end, :)) / max(mean(sp(end, :)), eps), ...
            median(t_sp, 'omitnan'), nnz(isfinite(t_sp)), res.ns, median(t_05, 'omitnan'), nnz(isfinite(t_05)), res.ns);
    fprintf('  e_a    : last 1 s mean %+.4f sd %.4f | worst |mean| %.4f at t = %.2f s\n', mean(mean(ea(last, :), 2)), mean(std(ea(last, :), 0, 2)), ...
            max(abs(mean(ea, 2))), t(find(abs(mean(ea, 2)) == max(abs(mean(ea, 2))), 1)));
end

function local_report_speed(res)
    t = res.t;  seg = res.seg;  e = res.dl_hat - res.dl_target;  sp = res.sP_dl;  ea = res.a_true - res.a_hat;
    desc = t >= seg(1) & t < seg(2);  hold = t >= seg(2);  kd = find(desc, 1, 'last');
    m = mean(ea, 2);  s = std(ea, 0, 2);  [~, iw] = max(abs(m) .* desc);
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d\n', min(res.h_true(:)), nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9));
    fprintf('  delta  : error at descent end %+.3f +- %.3f (sqrtP %.3f) | at run end %+.3f +- %.3f (sqrtP %.3f) | honesty end sd/sqrtP %.2f\n', ...
            mean(e(kd, :)), std(e(kd, :)), mean(sp(kd, :)), mean(e(end, :)), std(e(end, :)), mean(sp(end, :)), std(e(end, :)) / max(mean(sp(end, :)), eps));
    fprintf('  e_a    : descent worst %+.4f at t = %.2f s (sd %.4f) | descent mean %+.4f sd %.4f | hold mean %+.4f sd %.4f\n', ...
            m(iw), t(iw), s(iw), mean(m(desc)), mean(s(desc)), mean(m(hold)), mean(s(hold)));
end
