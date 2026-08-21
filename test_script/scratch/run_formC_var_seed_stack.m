function S = run_formC_var_seed_stack(seeds, S, opts)
%RUN_FORMC_VAR_SEED_STACK  Collect the per-seed channel stack for the variance
%   identity check, in chunks, inside ONE MATLAB process.
%
%   S = run_formC_var_seed_stack(1:50);        % first chunk
%   S = run_formC_var_seed_stack(51:100, S);   % append
%
% STATUS: ACTIVE | collector for check_formC_var_identity
%
% WHY CHUNKED. run_formC_b.m:505 saves the whole out struct unconditionally,
% and out is ~6.5 MB per seed, so one 400-seed call would build a 2.6 GB struct
% and then write it. Chunking keeps the peak near 325 MB and lets a long run be
% driven from several short calls of the SAME process -- separate MATLAB -batch
% launches each take a licence seat and queue up behind each other.
%
% ARM. The P0 (hard plane) column of the formC comparison pages:
%     run_formC_b(struct('arm','best', 'ap_src','post'))
% no plant_law_b, no ctrl_const_override. Confirmed against the session that
% produced those pages, 2026-08-20.

    if nargin < 2; S = []; end
    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'arm');    opts.arm    = 'best'; end
    if ~isfield(opts, 'ap_src'); opts.ap_src = 'post'; end
    if ~isfield(opts, 'a_ctrl_override'); opts.a_ctrl_override = []; end
    % Anything else the driver takes, passed straight through (a_cov_scale,
    % plant_law_b, plant_cperp, ...). Kept as a struct so the arm's identity
    % is recorded in S.opts alongside the data rather than living only in
    % whatever command line produced it.
    if ~isfield(opts, 'driver_extra'); opts.driver_extra = struct(); end
    if ~isfield(opts, 'chunk');  opts.chunk  = 25;     end
    % Total seeds this stack will ever hold. Preallocated ONCE -- with 13
    % three-column channels each seed costs ~2.5 MB, so a generous default
    % would quietly allocate gigabytes. Set it on the first call when the
    % collection is going to be driven in several calls.
    if ~isfield(opts, 'nmax');   opts.nmax   = numel(seeds); end
    % ALL THREE y2 GUARDS ARE RECONSTRUCTIBLE OFFLINE from this list, so the
    % driver needs no new log line (it currently keeps only their OR,
    % run_formC_b.m:1141, while the controller has them separately at
    % motion_control_law_formC_b.m:1293):
    %   G1 = t < t_warmup_kf                      -> from S.t and ctrl_const
    %   G2 = sigma2_dwr_hat - C_n*sigma2_nw <= 0  -> exactly a_xm_out <= 0,
    %        because a_bar_wm IS that bracket divided by C_dpmr*kappa_T
    %   G3 = h_bar < h_bar_safe                   -> from h_bar_out (the
    %        CONTROLLER's h_bar, which is the one the gate tests -- not
    %        h_bar_true_out and not h_bar_d_out)
    % 14 three-column channels + 3 single-column ones. p_true/dx_r/a_true are
    % what the two identities need; the rest is the R22 / a_m chain's list
    % (session "R22 am", 2026-08-20) so that one 25-minute run serves both.
    if ~isfield(opts, 'keep')
        opts.keep = {'p_true_out', 'dx_r_out', 'a_true_out', 'a_bar_hat_out', ...
                     'a_xm_out', 'R2_out', 'gate_out', 'dh_m_out', ...
                     'Q33_out', 'a_prime_out', 'b_hat_out', 'innov_y2_out', ...
                     'P_a_out', 'h_bar_true_out', 'h_bar_d_out', 'h_bar_out'};
    end

    seeds = seeds(:).';
    nchunk = ceil(numel(seeds) / opts.chunk);
    t_run = tic;

    for c = 1:nchunk
        i0 = (c-1)*opts.chunk + 1;
        i1 = min(c*opts.chunk, numel(seeds));
        dopts = struct('arm', opts.arm, 'ap_src', opts.ap_src, ...
                       'seeds', seeds(i0:i1), 'verbose', false);
        if ~isempty(opts.a_ctrl_override)
            dopts.a_ctrl_override = opts.a_ctrl_override;
        end
        fx = fieldnames(opts.driver_extra);
        for ii = 1:numel(fx)
            dopts.(fx{ii}) = opts.driver_extra.(fx{ii});
        end
        oc = run_formC_b(dopts);

        if isempty(S)
            S = local_init(oc.runs{1}, opts);
        end
        assert(numel(S.seeds) + numel(oc.runs) <= size(S.p_true_out, 3), ...
               ['stack capacity %d exhausted; pass opts.nmax = <total seeds> ' ...
                'on the FIRST call'], size(S.p_true_out, 3));
        for q = 1:numel(oc.runs)
            r = oc.runs{q};
            local_assert_run(r, S);
            n0 = numel(S.seeds) + 1;
            for f = 1:numel(opts.keep)
                S.(opts.keep{f})(:, :, n0) = r.(opts.keep{f});
            end
            S.seeds(n0) = r.meta.seed;
        end
        clear oc r;
        fprintf('[stack] chunk %d/%d  seeds %d..%d  total %d  %.1f s\n', ...
                c, nchunk, seeds(i0), seeds(i1), numel(S.seeds), toc(t_run));
    end
    fprintf('[stack] %d seeds held, %.2f s/seed this call\n', ...
            numel(S.seeds), toc(t_run)/numel(seeds));
end

% ----------------------------------------------------------------------
function S = local_init(r, opts)
    NMAX = opts.nmax;
    S = struct();
    S.keep = opts.keep;
    S.opts = opts;
    S.seeds = [];
    for f = 1:numel(opts.keep)
        v = r.(opts.keep{f});
        S.(opts.keep{f}) = zeros(size(v,1), size(v,2), NMAX, class(v));
    end
    S.p_d = r.p_d_out;                 % identical across seeds, stored once
    S.t   = r.tout;
    S.K   = local_constants(r);

    % SANITY, written before the run and checked on every run (rule 13).
    % 1) row 1 of every driver log is the controller's init-only call, which
    %    returns an all-zero diag. If it ever stops being zero the log layout
    %    changed and every statistic below is on different rows.
    assert(all(r.a_xm_out(1,:) == 0) && r.h_bar_out(1) == 0, ...
           'row 1 is not the init-only all-zero row; log layout changed');
    % 2) arm 'best' and 'b98' seed b at 8/9 since 2026-08-18. Asserted rather
    %    than believed, because the two names diverging silently is exactly the
    %    class of bug that still runs and still prints plausible numbers.
    assert(abs(S.K.b_init - 8/9) < 1e-12, ...
           'b_init = %.6f, expected 8/9 -- arm naming changed', S.K.b_init);
    % 3) the deep band. A shallow run would give numbers 5.4x apart in percent
    %    and they must never be mixed.
    assert(abs(S.K.h_bottom / S.K.R - 1.10) < 1e-6, ...
           'trough w_bar = %.4f, expected 1.10 (deep)', S.K.h_bottom / S.K.R);
    assert(abs(S.K.h_bar_safe - 1.0) < 1e-12, ...
           'h_bar_safe = %.3f, expected 1.0 on the deep band', S.K.h_bar_safe);
end

% ----------------------------------------------------------------------
function local_assert_run(r, S)
    % Same trajectory for every seed -- the ensemble estimator depends on it.
    assert(isequal(size(r.p_d_out), size(S.p_d)) && ...
           max(abs(r.p_d_out(:) - S.p_d(:))) < 1e-12, ...
           'command trajectory differs between seeds; ensemble var is invalid');
    assert(r.ctrl_const.C_dpmr == S.K.C_dpmr && r.ctrl_const.C_n == S.K.C_n, ...
           'constants changed mid-collection');
end

% ----------------------------------------------------------------------
function K = local_constants(r)
%LOCAL_CONSTANTS  Every constant read OUT OF THE RUN. Nothing rebuilt from
%   user_config() or physical_constants(): the canonical scenario overrides
%   several of them, and on 2026-08-20 rebuilding sigma_n from the config
%   default (10 nm, vs the scenario's 3.31 nm) turned a healthy ratio into a
%   0.485 phantom.
    cc = r.ctrl_const;  cfg = r.meta.config;  pv = r.meta.params_value;
    % Ts is not in meta.config -- it lives in params_value.thermal. Take it from
    % there and cross-check against the log's own time base, which is a free
    % instrument check on the whole time axis.
    Ts = pv.thermal.Ts;
    assert(abs(Ts - (r.tout(2) - r.tout(1))) < 1e-12, ...
           'Ts %.8g disagrees with the logged time base %.8g', Ts, r.tout(2)-r.tout(1));
    % kBT reached the controller by a different path from the plant's thermal
    % generator; if they ever disagree the identity is meaningless.
    assert(abs(cc.kBT - pv.thermal.k_B * pv.thermal.T) < 1e-12 * cc.kBT, ...
           'controller kBT %.6g != plant k_B*T %.6g', cc.kBT, pv.thermal.k_B*pv.thermal.T);
    % a_nom = a_o*R = Ts/gamma_N is the far-field gain both sides must share.
    assert(abs(r.a_nom - Ts / pv.common.gamma_N) < 1e-12 * r.a_nom, ...
           'a_nom %.6g != Ts/gamma_N %.6g', r.a_nom, Ts / pv.common.gamma_N);
    K = struct('lambda_c', cc.lambda_c, 'd', cc.d, 'a_pd', cc.a_pd, ...
               'a_cov', cc.a_cov, 'kBT', cc.kBT, ...
               'sigma2_n_s', cc.sigma2_n_s(:).', 'C_dpmr', cc.C_dpmr, ...
               'C_n', cc.C_n, 'K_var', cc.K_var, 'IF_abc', cc.IF_abc, ...
               'xi_per_axis', cc.xi_per_axis(:).', ...
               'R22_prefactor', cc.R22_prefactor, 'h_bar_safe', cc.h_bar_safe, ...
               'b_init', cc.b_init, 'lock_b', cc.lock_b, 'ap_src', cc.ap_src, ...
               'R', r.R, 'a_nom', r.a_nom, 'a_o', r.a_nom / r.R, ...
               'Ts', Ts, 'gamma_N', pv.common.gamma_N, ...
               'meas_noise_std', cfg.meas_noise_std(:).', ...
               'h_bottom', cfg.h_bottom, 'h_min', cfg.h_min, ...
               'frequency', cfg.frequency, 'amplitude', cfg.amplitude, ...
               'T_sim', cfg.T_sim, 'driver', r.meta.driver);
end
