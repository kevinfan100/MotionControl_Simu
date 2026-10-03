% PURPOSE (2026-09-20): the two-wall test of the offset law (0920_estb_delta_law.tex, version A): one position-type state
%   delta = depth of the effective wall below the reference surface w_s, gain = exact plane polynomial at (w - w_s) + delta.
%     plants    plane  Brenner polynomial (production plant), delta_true = 0
%               cell   plane polynomial slid down 1.05 R (cell_cperp_delta.m), delta_true = 1.05; thermal force follows c_perp
%     arms      T   seed = the plant's own delta, prior 0.5 (health: delta_hat must stay within sqrt(P), no drift)
%               Tp  seed = delta_true + 0.5   (does one descent bring it back? which segment does the work?)
%               Tm  seed = delta_true - 0.5
%               W   reference surface w_s mis-set by +1 R, seed = delta_true (the estimator's target becomes delta_true + 1:
%                   delta_hat must absorb the reference error and a_hat must match arm T)
%               PI  PLANE INIT (the main question, user 09-20): delta_hat[0] = 0 (rigid plane assumed) on the CELL plant, prior 0.5 --
%                   does one descent find the effective wall 1.05 R below the visible surface, and what does the gain cost meanwhile
%   Controller flags: delta_state + gain_algebraic + amodel_fe + amodel_h, pure form (ma2_aug / nw_mcorr / pred_mean2 /
%   law_exact_step off), slot 5 locked. Paired seeds, canon deep. e_a = a_true - a_hat absolute; motion segments first.
%   Output: test_results/temp_delta_two_walls/<plant>_<arm>.mat (gitignored) + console; figures by plot_delta_ladder.m.
%   EXPIRES: with the delta-law line | 產線改動不會自動跟上
function out = run_delta_two_walls(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = 1:10; end
    if ~isfield(opts, 'plants'); opts.plants = {'plane', 'cell'}; end
    if ~isfield(opts, 'arms');   opts.arms   = {'T', 'Tp', 'Tm', 'W'}; end
    if ~isfield(opts, 'tag_suffix'); opts.tag_suffix = ''; end
    if ~isfield(opts, 'traj'); opts.traj = 'canon'; end               % 'canon' (deep) | 'meng' (15 um -> 2.5 um ramp over 10 s, hold to 12.5 s; probe_obs_11_two_versions.m)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_delta_two_walls');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;
    W_S   = 0.0;                                            % reference surface (plane: the wall; cell: the visible surface)
    DEL   = struct('plane', 0.0, 'cell', 1.05);             % plant offsets (cell_cperp_delta.m)
    SIG   = 0.5;                                            % prior: half the plane-cell spread, two-sided
    OFF   = 0.5;                                            % seed perturbation of the T+/- arms
    W_ERR = 1.0;                                            % reference error of arm W (R)
    SEG   = [0.5, 1.5, 3.5];  OV = struct();                % canon deep: hold | descent | oscillation | hold
    if strcmp(opts.traj, 'meng')
        OV  = struct('trajectory_type', 'osc', 'h_init', 15, 'h_bottom', 2.5, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, ...
                     't_hold', 0.5, 't_descend_override', 10, 'T_sim', 12.5, 'h_min', 2.475);
        SEG = [0.5, 10.5, 10.5];                            % hold | ramp | (no oscillation) | hold
        opts.tag_suffix = [opts.tag_suffix '_meng'];
    end
    out = struct();
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        plant_cperp = [];  if strcmp(pl, 'cell'); plant_cperp = @cell_cperp_delta; end
        if strcmp(pl, 'cell') && isfield(opts, 'plant_cperp_cell'); plant_cperp = opts.plant_cperp_cell; end
        for ia = 1:numel(opts.arms)
            arm = opts.arms{ia};  ws = W_S;  dl_seed = DEL.(pl);  dl_target = DEL.(pl);
            switch arm
                case 'T'
                case 'Tp'; dl_seed = DEL.(pl) + OFF;
                case 'Tm'; dl_seed = DEL.(pl) - OFF;
                case 'W';  ws = W_S + W_ERR;  dl_target = DEL.(pl) + W_ERR;   % g = (w - ws) + delta must still be w - W_S + DEL
                case 'MID'; dl_seed = (DEL.plane + DEL.cell) / 2;                % wall-family MIDPOINT init (C68): the prior for an unknown wall type
                case {'PI', 'PI_iekf', 'PI_slin'}; dl_seed = DEL.plane;                     % PLANE INIT ('PI_iekf' adds the iterated y2 update, O38) (user 09-20): the estimator starts as if the wall were the rigid plane (delta_hat[0] = 0); on the cell plant the seed is 1.05 R off
                otherwise; error('unknown arm %s', arm);
            end
            cc = local_ctrl_const(dl_seed, SIG, ws);  cc.delta_iekf = contains(arm, 'iekf') || contains(arm, 'slin');  cc.delta_slin = contains(arm, 'slin');   % 'PI_slin' = iterated y2 update + statistical linearisation
            tag = sprintf('%s_%s%s', pl, arm, opts.tag_suffix);
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                       'seeds', opts.seeds, 'log_P_full', false);
            if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
            a11 = local_a_plane(1.1 - W_S + DEL.(pl));  a11_seed = local_a_plane(1.1 - ws + dl_seed);
            fprintf('\n=== %s | w_s = %.2f, seed delta = %.3f (target %.3f), prior %.2f | a_bar(1.1): plant %.3f, seed %.3f | %d seeds ===\n', ...
                    tag, ws, dl_seed, dl_target, SIG, a11, a11_seed, numel(opts.seeds));
            clear run_formC_b motion_control_law_formC_b;
            tic; evalc('Rr = run_formC_b(o);'); tr = toc;
            res = local_collect(Rr, plant_cperp, Ts);
            res.tag = tag; res.plant = pl; res.arm = arm; res.W_S = W_S; res.ws = ws; res.dl_seed = dl_seed; res.dl_target = dl_target;
            res.sig = SIG; res.seg = SEG;
            local_report(res, SEG);
            fprintf('    (%.0f s)\n', tr);
            save(fullfile(od, [tag '.mat']), '-struct', 'res');
            out.(tag) = res;
        end
    end
end

function a = local_a_plane(g)
    u = 1 / max(g, 1);
    a = 1 - (9/8)*u + (1/2)*u^3 - (57/100)*u^4 + (1/5)*u^5 + (7/200)*u^11 - (1/25)*u^12;
end

function cc = local_ctrl_const(dl_seed, sig, ws)
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', true, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 1e-5);
    cc.delta_state = true;  cc.gain_algebraic = true;  cc.amodel_fe = true;  cc.amodel_h = true;
    cc.delta_init = dl_seed;  cc.Pf_delta_std = sig;
    cc.ws0_perp = ws;  cc.Pf_w0_std = 0;
    cc.b_init = 8/9;  cc.Pf_b_std = 0;                    % slot 5 is locked and inert under delta_state
end

function res = local_collect(Rr, plant_cperp, Ts)
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:), 'ns', ns);
    res.h_true = zeros(n, ns); res.a_true = zeros(n, ns); res.a_hat = zeros(n, ns);
    res.dl_hat = zeros(n, ns); res.sP_dl = zeros(n, ns);
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
        res.dl_hat(:, q) = S.ws_hat_out(:, 3);  res.sP_dl(:, q) = S.P_ws_out(:, 3);     % slot 7 = delta; P_ws_out is sqrt(P77)
        res.innov1(:, q) = S.innov_y1_out(:, 3);  res.innov2(:, q) = S.innov_y2_out(:, 3);
        if isfield(S, 'S1_out'); res.S1(:, q) = S.S1_out(:, 3);  res.S2(:, q) = S.S2_out(:, 3); end
    end
end

function local_report(res, SEG)
    t = res.t;  ea = res.a_true - res.a_hat;                     % e_a = a_true - a_hat, absolute
    m = mean(ea, 2);  s = std(ea, 0, 2);
    desc = t >= SEG(1) & t < SEG(2);  osc = t >= SEG(2) & t < SEG(3);  hold = t >= SEG(3);
    [~, iw] = max(abs(m) .* desc);
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d | delta_hat range [%.3f, %.3f]\n', min(res.h_true(:)), ...
            nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), min(res.dl_hat(:)), max(res.dl_hat(:)));
    fprintf('  e_a    : descent worst %+.4f at t = %.2f s (sd %.4f) | descent mean %+.4f sd %.4f | osc mean %+.4f sd %.4f | hold mean %+.4f sd %.4f\n', ...
            m(iw), t(iw), s(iw), mean(m(desc)), mean(s(desc)), mean(m(osc)), mean(s(osc)), mean(m(hold)), mean(s(hold)));
    x = res.dl_hat;  sp = res.sP_dl;  e = x - res.dl_target;
    walk = max(abs(x - x(1, :)), [], 1);
    fprintf('  delta  : seed %.3f target %.3f | end %.3f +- %.3f (seed sd), error %+.3f | at descent end %.3f | sup|walk| %.3f | sqrtP end %.3f | honesty |err|/sqrtP %.2f, sd/sqrtP %.2f\n', ...
            res.dl_seed, res.dl_target, mean(x(end, :)), std(x(end, :)), mean(e(end, :)), mean(x(find(t < SEG(2), 1, 'last'), :)), ...
            mean(walk), mean(sp(end, :)), abs(mean(e(end, :))) / max(mean(sp(end, :)), eps), std(x(end, :)) / max(mean(sp(end, :)), eps));
    dt = t(2) - t(1);  hs = find(hold);
    slope = mean((x(hs(end), :) - x(hs(1), :)) / (numel(hs) * dt));
    fprintf('  hold   : delta_hat slope %+.5f /s (seed mean)\n', slope);
end
