function R = run_menq_exchange_rate(max_new_runs)
%RUN_MENQ_EXCHANGE_RATE  Closed-loop (b, w_s) exchange rate of the Form B filter.
%
% STATUS: ACTIVE -- closed-loop (b, w_s) exchange rate of the production Form B filter.
%   See memory project-formb-7a-falsified-quadrature-ratchet-2026-08-06.
%
% PURPOSE
%   Prof. Menq: "the influence of b and w_s to h cannot be separated" and
%   "the variation of w_s should be near zero". Offline algebra already shows
%   the two sensitivity directions are parallel up to a scalar,
%       dA/db = ((w_bar - w_s)/b) * dA/dw_s ,
%   so a b error and a w_s error are indistinguishable AT THE GAIN LEVEL.
%   This script measures what the REAL closed-loop EKF does about it: the
%   TRUE z curve is replaced by a Form B curve with a known b_plant (true
%   w_s = 1, true p = 1, plant side only, opts.plant_gain_law), w_s is
%   released (tier t2) and p is locked, and we regress the steady-state
%   estimates on b_plant. The two slopes ARE the exchange rate:
%       d b_hat / d b_plant   ~ 1  and  d ws_hat / d b_plant ~ 0  => separable
%       d ws_hat / d b_plant  significantly non-zero               => compensation
%
% SCOPE
%   z axis (wall-normal) only; canonical 4.8 s scenario of run_formB_ws;
%   x / y keep the production parallel law and are untouched.
%   NO production file is modified: the plant curve, the tier and the prior
%   widths all go in through documented run_formB_ws options.
%
% ARMS (the prior widths are the confound, so both are run)
%   'prod'  : Pf_b_std = envelope prior (0.0157), Pf_ws_std = 0.028
%             = the calibration-grade production pair the team asked for.
%   'loose' : Pf_b_std = 1/8  (the driver's own documented global-sup
%             fallback on w_bar in [1.1, 10], PRIOR_STD_BP),
%             Pf_ws_std = 0.111 (the house placeholder used by the c3 prior
%             sweep). Neither number is invented here; this arm exists so a
%             small d b_hat / d b_plant in 'prod' cannot be blamed on prior
%             stiffness alone.
%
% CRITERION (stated before the run)
%   Let g = w_bar - 1 be the true gap in the readout window and b ~ 9/8.
%   If the filter matches the observed gain at all, the two slopes must obey
%       d ws_hat / d b_plant = (g/b) * (1 - d b_hat / d b_plant).
%   COMPENSATION CONFIRMED if d ws_hat / d b_plant is significantly > 0
%   (95 % CI excludes 0). SEPARABLE if that CI contains 0 while
%   d b_hat / d b_plant has a CI containing 1.
%   The posterior correlation rho(b, w_s) of the live 7-state covariance is
%   reported as the direct non-identifiability readout (|rho| -> 1 = ridge).
%
% OUTPUT (test_results/ is gitignored)
%   temp_menq_exchange_rate.mat  cache + all collected metrics
%   temp_menq_exchange_rate.png  two-panel figure (.claude/rules/figure-style.md)
%
%   R = run_menq_exchange_rate()      % run everything (resumes from cache)
%   R = run_menq_exchange_rate(12)    % do at most 12 NEW sim runs, then stop

    if nargin < 1 || isempty(max_new_runs); max_new_runs = Inf; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(fullfile(proj, 'model')));
    addpath(fullfile(proj, 'test_script', 'integration'));
    res_dir  = fullfile(proj, 'test_results');
    if ~exist(res_dir, 'dir'); mkdir(res_dir); end
    cache_f  = fullfile(res_dir, 'temp_menq_exchange_rate.mat');
    fig_f    = fullfile(res_dir, 'temp_menq_exchange_rate.png');

    % ---------------- named constants (no magic numbers) ----------------
    AX_Z       = 3;                                      % wall-normal axis
    B_ANCHOR   = 9/8;                                    % derived far-field anchor
    B_GRID     = [1.0000 1.0625 1.1250 1.1875 1.2500];   % b_plant sweep [-]
    WS_TRUE    = 1;                                      % true wall position [-]
    P_TRUE     = 1;                                      % true exponent [-]
    SEEDS      = [7 11 3001 3002 3003 3004];             % 6 seeds
    ARMS       = {'prod', 'loose'};
    PF_B_PROD  = [];        % [] = keep the driver's envelope prior (0.0157)
    PF_B_LOOSE = 1/8;       % driver constant PRIOR_STD_BP (global-sup fallback)
    PF_WS_PROD = 0.028;     % calibration-grade w_s prior (project value)
    PF_WS_LOOSE = 0.111;    % house placeholder w_s prior (project value)
    IX_B       = 5;         % EKF slot of b   (state = [dw1 dw2 dw3 a b p ws ...])
    IX_WS      = 7;         % EKF slot of w_s
    T_CRIT_5   = 2.5706;    % t_{0.975, 5} for the 6-seed per-seed slope CI

    nb = numel(B_GRID); ns = numel(SEEDS); na = numel(ARMS);

    % ---------------- cache ----------------
    if exist(cache_f, 'file')
        S = load(cache_f); R = S.R;
    else
        R = struct();
        R.b_grid = B_GRID; R.seeds = SEEDS; R.arms = {ARMS};
        R.done    = false(na, nb, ns);
        R.ws_hold = nan(na, nb, ns);   % mean ws_hat over the final-hold window
        R.b_hold  = nan(na, nb, ns);   % mean b_hat  over the final-hold window
        R.ws_end  = nan(na, nb, ns);
        R.b_end   = nan(na, nb, ns);
        R.sqP_ws  = nan(na, nb, ns);   % sqrt P_ws at hold-window mean
        R.sqP_b   = nan(na, nb, ns);
        R.g_hold  = nan(na, nb, ns);   % mean (h_bar_true - 1) over hold [-]
        R.desc_pk = nan(na, nb, ns);   % z gain error, descent peak |e_a| [%]
        R.hold_mu = nan(na, nb, ns);   % z gain error, hold mean e_a [%]
        R.osc_rms = nan(na, nb, ns);
        R.warn    = cell(na, nb, ns);
    end

    % ---------------- sweep ----------------
    n_new = 0;
    for ia = 1:na
        for ib = 1:nb
            for is = 1:ns
                if R.done(ia, ib, is); continue; end
                if n_new >= max_new_runs
                    fprintf('run budget reached (%d new runs); re-call to continue\n', n_new);
                    save(cache_f, 'R'); return;
                end
                opt_cell = local_arm_opts(ARMS{ia}, B_GRID(ib), WS_TRUE, P_TRUE, SEEDS(is), ...
                                          PF_B_PROD, PF_B_LOOSE, PF_WS_PROD, PF_WS_LOOSE);
                [out, txt] = local_quiet_run(opt_cell);
                r   = out.runs{1};
                w   = out.metrics.windows.hold;
                t   = r.tout(:);
                m   = t > w(1);
                R.ws_hold(ia, ib, is) = mean(r.ws_hat_out(m, AX_Z));
                R.b_hold(ia, ib, is)  = mean(r.b_hat_out(m, AX_Z));
                R.ws_end(ia, ib, is)  = r.ws_hat_out(end, AX_Z);
                R.b_end(ia, ib, is)   = r.b_hat_out(end, AX_Z);
                R.sqP_ws(ia, ib, is)  = mean(r.P_ws_out(m, AX_Z));
                R.sqP_b(ia, ib, is)   = mean(r.P_b_out(m, AX_Z));
                R.g_hold(ia, ib, is)  = mean(r.h_bar_true_out(m)) - WS_TRUE;
                R.desc_pk(ia, ib, is) = out.metrics.rows(1, 1);
                R.osc_rms(ia, ib, is) = out.metrics.rows(1, 2);
                R.hold_mu(ia, ib, is) = out.metrics.rows(1, 3);
                R.warn{ia, ib, is}    = local_warn_line(txt);
                R.window              = w;
                if ib == 3 && is == 1     % anchor cell: keep the exact ctrl_const
                    R.cfg = out.cfg;
                    R.ov{ia} = r.meta.ctrl_const_override;
                end
                R.done(ia, ib, is) = true;
                n_new = n_new + 1;
                save(cache_f, 'R');
                fprintf('%-5s b_plant %.4f seed %5d | ws_hold %+.5f  b_hold %.5f  hold e_a %+6.2f%%  %s\n', ...
                        ARMS{ia}, B_GRID(ib), SEEDS(is), R.ws_hold(ia, ib, is), ...
                        R.b_hold(ia, ib, is), R.hold_mu(ia, ib, is), R.warn{ia, ib, is});
            end
        end
    end

    % ---------------- posterior correlation rho(b, w_s), one seed per arm ----------------
    if ~isfield(R, 'rho_hold') || any(isnan(R.rho_hold))
        R.rho_hold = nan(1, na); R.rho_end = nan(1, na);
        for ia = 1:na
            law = struct('b', B_ANCHOR, 'p', P_TRUE, 'ws', WS_TRUE);
            topts = struct('seed', SEEDS(1), 'verbose', false, ...
                           'ctrl_const_override', R.ov{ia}, 'log_P_full', true, ...
                           'plant_cperp', @(hb) calc_formB_cperp(hb, law));
            s = run_formB_ws(R.cfg, topts);
            Pz = squeeze(s.P_full_out(:, :, :, AX_Z));       % N x n x n
            pbb = Pz(:, IX_B,  IX_B);
            pww = Pz(:, IX_WS, IX_WS);
            pbw = Pz(:, IX_B,  IX_WS);
            rho = pbw ./ sqrt(max(pbb .* pww, eps));
            mh  = s.tout(:) > R.window(1);
            R.rho_hold(ia) = mean(rho(mh));
            R.rho_end(ia)  = rho(end);
        end
        save(cache_f, 'R');
    end

    % ---------------- statistics ----------------
    fprintf('\n================ Menq exchange-rate experiment ================\n');
    fprintf('plant: Form B z curve, true w_s = %.3f, true p = %.3f, b_plant swept %s\n', ...
            WS_TRUE, P_TRUE, mat2str(B_GRID));
    fprintf('filter: tier t2 (w_s free), p LOCKED, b free; %d seeds %s\n', ns, mat2str(SEEDS));
    fprintf('readout window: final hold t in (%.2f, %.2f] s, mean over window\n', R.window(1), R.window(2));
    fprintf('mean true gap g = w_bar - 1 in that window: %.4f  ->  ridge scalar g/b_anchor = %.4f\n', ...
            mean(R.g_hold(:), 'omitnan'), mean(R.g_hold(:), 'omitnan') / B_ANCHOR);

    R.slope = struct();
    for ia = 1:na
        fprintf('\n---- arm %s ----\n', ARMS{ia});
        fprintf('prior widths (z): sqrt P_bb[0] = %.4f   sqrt P_wsws[0] = %.4f\n', ...
                local_prior_b(ARMS{ia}, PF_B_LOOSE, R), local_prior_ws(ARMS{ia}, PF_WS_PROD, PF_WS_LOOSE));
        fprintf('%9s | %12s %12s | %12s %12s | %9s %9s\n', 'b_plant', ...
                'ws_hat mean', '(sd)', 'b_hat mean', '(sd)', 'desc pk%', 'hold e_a%');
        for ib = 1:nb
            fprintf('%9.4f | %12.5f %12.5f | %12.5f %12.5f | %9.3f %+9.3f\n', B_GRID(ib), ...
                    mean(R.ws_hold(ia, ib, :)), std(R.ws_hold(ia, ib, :), 0, 3), ...
                    mean(R.b_hold(ia, ib, :)),  std(R.b_hold(ia, ib, :), 0, 3), ...
                    mean(R.desc_pk(ia, ib, :)), mean(R.hold_mu(ia, ib, :)));
        end

        [sw_mu, sw_ci, sw_i] = local_perseed_slope(B_GRID, squeeze(R.ws_hold(ia, :, :)), T_CRIT_5);
        [sb_mu, sb_ci, sb_i] = local_perseed_slope(B_GRID, squeeze(R.b_hold(ia, :, :)),  T_CRIT_5);
        [pw_mu, pw_ci] = local_pooled_slope(B_GRID, squeeze(R.ws_hold(ia, :, :)));
        [pb_mu, pb_ci] = local_pooled_slope(B_GRID, squeeze(R.b_hold(ia, :, :)));
        gb = mean(reshape(R.g_hold(ia, :, :), 1, []), 'omitnan') / B_ANCHOR;
        pred_w = gb * (1 - sb_mu);

        fprintf('EXCHANGE RATE (per-seed slopes, mean +- 95%% CI over %d seeds):\n', ns);
        fprintf('   d ws_hat / d b_plant = %+.4f  [%+.4f, %+.4f]   (pooled OLS %+.4f [%+.4f, %+.4f])\n', ...
                sw_mu, sw_ci(1), sw_ci(2), pw_mu, pw_ci(1), pw_ci(2));
        fprintf('   d b_hat  / d b_plant = %+.4f  [%+.4f, %+.4f]   (pooled OLS %+.4f [%+.4f, %+.4f])\n', ...
                sb_mu, sb_ci(1), sb_ci(2), pb_mu, pb_ci(1), pb_ci(2));
        fprintf('   gain-matching prediction (g/b)(1 - db/db) = %+.4f  -> filter delivers %.0f%% of it\n', ...
                pred_w, 100 * sw_mu / max(pred_w, eps));
        fprintf('   posterior corr rho(b, w_s) at hold = %+.4f (end %+.4f)  [|rho|->1 = ridge]\n', ...
                R.rho_hold(ia), R.rho_end(ia));
        sb0 = local_prior_b(ARMS{ia}, PF_B_LOOSE, R);
        sw0 = local_prior_ws(ARMS{ia}, PF_WS_PROD, PF_WS_LOOSE);
        fprintf('   leak in PRIOR units: a 1-sigma_b curve error moves ws_hat by %.3f sigma_ws\n', ...
                sw_mu * sb0 / sw0);
        fprintf('   per-seed d ws/d b_plant: %s\n', mat2str(round(sw_i, 4)));
        fprintf('   per-seed d b /d b_plant: %s\n', mat2str(round(sb_i, 4)));
        R.slope.(ARMS{ia}) = struct('ws', sw_mu, 'ws_ci', sw_ci, 'b', sb_mu, 'b_ci', sb_ci, ...
                                    'ws_pooled', pw_mu, 'b_pooled', pb_mu, 'pred_ws', pred_w, ...
                                    'ws_perseed', sw_i, 'b_perseed', sb_i);
    end
    save(cache_f, 'R');

    % ---------------- figure ----------------
    local_make_figure(R, B_GRID, ARMS, WS_TRUE, fig_f);
    fprintf('\nfigure: %s\ncache : %s\n', fig_f, cache_f);
end


%% =================== Local Helpers ===================

function o = local_arm_opts(arm, b_plant, ws_true, p_true, seed, pf_b_prod, pf_b_loose, pf_ws_prod, pf_ws_loose)
%LOCAL_ARM_OPTS  run_formB_ws options for one (arm, b_plant, seed) cell.
%   ctrl_const_override is applied AFTER the driver's arm flags, so every lock
%   the arm needs is restated here (documented run_formB_ws trap).
    ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false);
    switch arm
        case 'prod'
            ov.Pf_ws_std = pf_ws_prod;
            if ~isempty(pf_b_prod); ov.Pf_b_std = pf_b_prod; end
        case 'loose'
            ov.Pf_ws_std = pf_ws_loose;
            ov.Pf_b_std  = pf_b_loose;
        otherwise
            error('run_menq_exchange_rate:badArm', 'unknown arm %s', arm);
    end
    o = struct();
    o.tier                = 't2';
    o.seeds               = seed;
    o.verbose             = false;
    o.plant_gain_law      = struct('b', b_plant, 'p', p_true, 'ws', ws_true);
    o.ctrl_const_override = ov;
end


function [out, txt] = local_quiet_run(o)   %#ok<INUSD>
%LOCAL_QUIET_RUN  One run_formB_ws call with its console banner captured.
    out = [];
    txt = evalc('out = run_formB_ws(o);');
end


function s = local_warn_line(txt)
%LOCAL_WARN_LINE  Extract the driver's WARNINGS line unless it is 'none'.
    s = '';
    k = strfind(txt, 'WARNINGS: ');
    if isempty(k); return; end
    tail = txt(k(end):end);
    nl = find(tail == newline, 1);
    if isempty(nl); nl = numel(tail) + 1; end
    line = strtrim(tail(1:nl-1));
    if ~strcmp(line, 'WARNINGS: none'); s = line; end
end


function [mu, ci, sl] = local_perseed_slope(x, Y, tcrit)
%LOCAL_PERSEED_SLOPE  OLS slope of Y(:,s) on x per seed, then mean +- 95% CI.
%   Seeds are the replicate unit (b_plant is fully crossed with seed), so the
%   across-seed t interval is the honest one; it needs no residual model.
    x = x(:); ns = size(Y, 2);
    sl = zeros(1, ns);
    for s = 1:ns
        y = Y(:, s);
        sl(s) = sum((x - mean(x)) .* (y - mean(y))) / sum((x - mean(x)).^2);
    end
    mu = mean(sl);
    se = std(sl) / sqrt(ns);
    ci = [mu - tcrit * se, mu + tcrit * se];
end


function [mu, ci] = local_pooled_slope(x, Y)
%LOCAL_POOLED_SLOPE  OLS on all (b_plant, seed) points; residual-based 95% CI.
%   Reported alongside the per-seed interval because it ignores seed blocking
%   and therefore prices the seed scatter into the slope error.
    x = x(:); [nb, ns] = size(Y);
    xv = repmat(x, ns, 1); yv = Y(:);
    ok = isfinite(xv) & isfinite(yv); xv = xv(ok); yv = yv(ok);
    n  = numel(xv);
    Sxx = sum((xv - mean(xv)).^2);
    mu  = sum((xv - mean(xv)) .* (yv - mean(yv))) / Sxx;
    b0  = mean(yv) - mu * mean(xv);
    res = yv - (b0 + mu * xv);
    s2  = sum(res.^2) / (n - 2);
    se  = sqrt(s2 / Sxx);
    tc  = local_tinv975(n - 2);
    ci  = [mu - tc * se, mu + tc * se];
    if nb < 2; ci = [NaN NaN]; end
end


function t = local_tinv975(df)
%LOCAL_TINV975  t_{0.975, df} without the Statistics Toolbox (small-df table +
%   Cornish-Fisher expansion beyond it; both standard, nothing tuned).
    tbl = [12.706 4.3027 3.1824 2.7764 2.5706 2.4469 2.3646 2.3060 2.2622 2.2281 ...
           2.2010 2.1788 2.1604 2.1448 2.1314 2.1199 2.1098 2.1009 2.0930 2.0860 ...
           2.0796 2.0739 2.0687 2.0639 2.0595 2.0555 2.0518 2.0484 2.0452 2.0423];
    if df >= 1 && df <= numel(tbl)
        t = tbl(df);
    else
        z = 1.959963984540054;
        t = z + (z^3 + z) / (4 * df) + (5 * z^5 + 16 * z^3 + 3 * z) / (96 * df^2);
    end
end


function w = local_prior_ws(arm, pf_ws_prod, pf_ws_loose)
    if strcmp(arm, 'prod'); w = pf_ws_prod; else; w = pf_ws_loose; end
end


function w = local_prior_b(arm, pf_b_loose, R)
%LOCAL_PRIOR_B  b prior width actually handed to the controller for this arm.
    if strcmp(arm, 'loose')
        w = pf_b_loose;
    else
        w = R.ov{1}.Pf_b_std;     % driver's envelope prior, as recorded
    end
end


function local_make_figure(R, b_grid, arms, ws_true, fig_f)
%LOCAL_MAKE_FIGURE  Two-panel exchange-rate figure (.claude/rules/figure-style.md).
%   Top: ws_hat vs b_plant against the true wall (red). Bottom: b_hat vs
%   b_plant against perfect tracking (red diagonal). Band = across-seed sigma.
    COL_TRUE  = [0.8 0 0];
    COL_HAT   = [0 0.2 0.9];
    COL_HAT2  = [0.35 0.60 0.90];
    BANDC     = [0.45 0.55 0.95];
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

    x = b_grid(:);
    fig = figure('Position', [80 60 1000 780], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    src = {R.ws_hold, R.b_hold};
    for row = 1:2
        nexttile; hold on;
        D = src{row};
        mu = zeros(numel(arms), numel(x)); sd = mu;
        for ia = 1:numel(arms)
            mu(ia, :) = mean(squeeze(D(ia, :, :)), 2).';
            sd(ia, :) = std(squeeze(D(ia, :, :)), 0, 2).';
        end
        hb = fill([x; flipud(x)], [mu(1, :).' - sd(1, :).'; flipud(mu(1, :).' + sd(1, :).')], ...
                  BANDC, 'FaceAlpha', 0.30, 'EdgeColor', 'none', ...
                  'DisplayName', 'prod \pm \sigma_{seed}');
        if row == 1
            ht = plot(x, ws_true * ones(size(x)), '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
                      'DisplayName', 'true w_s');
            ylabel('$\hat{w}_s$  [--]', 'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
        else
            ht = plot(x, x, '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
                      'DisplayName', 'true b (perfect tracking)');
            ylabel('$\hat{b}$  [--]', 'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
        end
        h1 = plot(x, mu(1, :), '-o', 'Color', COL_HAT, 'LineWidth', LW, 'MarkerSize', 8, ...
                  'MarkerFaceColor', COL_HAT, 'DisplayName', 'prod priors');
        h2 = plot(x, mu(2, :), '--s', 'Color', COL_HAT2, 'LineWidth', LW, 'MarkerSize', 8, ...
                  'DisplayName', 'loose priors');
        errorbar(x, mu(2, :), sd(2, :), 'LineStyle', 'none', 'Color', COL_HAT2, ...
                 'LineWidth', 1.2, 'CapSize', 10, 'HandleVisibility', 'off');
        xlim([min(x) - 0.02, max(x) + 0.02]);
        if row == 1
            legend([ht h1 h2 hb], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        else
            % The red reference means something different per panel, so the
            % lower panel carries its own one-entry legend.
            legend(ht, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        end
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    xlabel('$b_{\rm plant}$  [--]', 'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
    exportgraphics(fig, fig_f, 'Resolution', 150);
    close(fig);
end
