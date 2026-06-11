function analysis = analyze_gain_oracle_6state(freqs, opts)
%ANALYZE_GAIN_ORACLE_6STATE det/ram analysis of compare_gain_oracle_6state
%   output (design doc §6-§8). Loads runs.mat per frequency, computes det
%   metrics, ram window statistics, paired A/B ratios, p_m cross-check,
%   theory anchor, a_hat decomposition; writes summary.md (+ figures,
%   make_figs). Re-runnable without re-simulating.
%
%   analysis = analyze_gain_oracle_6state()             % freqs = [1 2 5]
%   analysis = analyze_gain_oracle_6state(freqs, opts)
%
%   opts: out_root (test_results/gain_oracle_ab), save_fig (true),
%         verbose (true).
%
%   See also: compare_gain_oracle_6state

    if nargin < 1 || isempty(freqs); freqs = [1 2 5]; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'verbose');  opts.verbose = true;  end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    if ~isfield(opts, 'out_root')
        opts.out_root = fullfile(project_root, 'test_results', 'gain_oracle_ab');
    end

    analysis = struct('freq', {}, 'det', {}, 'ram', {}, 'anchor', {}, ...
                      'ahat', {}, 'flags', {});
    for fi = 1:numel(freqs)
        f = freqs(fi);
        out_dir = fullfile(opts.out_root, sprintf('f%gHz', f));
        try
            S = load(fullfile(out_dir, 'runs.mat'));   % runs, cfg, opts, layer0, ...

            A = per_freq_analysis(S.runs, S.cfg, f);
            write_summary_md(fullfile(out_dir, 'summary.md'), f, S, A);
            save(fullfile(out_dir, 'analysis.mat'), 'A');
            if opts.save_fig
                make_figs(S, A, f, out_dir);
            end
            analysis(end+1) = struct('freq', f, 'det', A.det, 'ram', A.ram, ...
                                     'anchor', A.anchor, 'ahat', A.ahat, ...
                                     'flags', A.flags); %#ok<AGROW>
            if opts.verbose; fprintf('[analyze:%gHz] done -> %s\n', f, out_dir); end
        catch err
            % per-frequency isolation: report and continue (design §9.1 —
            % near-wall divergence is a plausible result, not an abort)
            if isempty(err.stack)
                loc = '(no stack)';
            else
                s1 = err.stack(1);
                loc = sprintf('%s > %s (line %d)', s1.file, s1.name, s1.line);
            end
            fprintf('[analyze:%gHz] FAILED [%s]: %s\n  at: %s\n', ...
                    f, err.identifier, err.message, loc);
            % NOTE: failed entries carry a reduced flags schema ({failed, err}
            % only) and [] det/ram/anchor/ahat — consumers must branch on
            % flags.failed BEFORE touching any other field.
            analysis(end+1) = struct('freq', f, 'det', [], 'ram', [], ...
                                     'anchor', [], 'ahat', [], ...
                                     'flags', struct('failed', true, ...
                                                     'err', err.message)); %#ok<AGROW>
        end
    end
    if numel(freqs) > 1 && opts.save_fig
        make_overview_fig(analysis, fullfile(opts.out_root, 'overview'));
    end
end


% ====================================================================
function A = per_freq_analysis(runs, cfg, f)
    % --- C3: a diverged det run invalidates the whole arm's det/ram
    %     decomposition — fail the frequency loudly (caught per-freq) ---
    for arm = 'AB'
        assert(~runs.(arm).det.diverged && ~isempty(runs.(arm).det.simOut), ...
               'arm %c det run diverged/empty: %s', ...
               arm, runs.(arm).det.diverge_reason);
    end

    % --- adaptation (a): extract physical constants from saved params ---
    P      = runs.A.det.simOut.meta.params_value;
    Ts     = P.common.Ts;
    R_phys = P.common.R;
    kBT    = P.ctrl.k_B * P.ctrl.T;
    w_hat  = P.wall.w_hat;
    pz     = P.wall.pz;

    sigma2_n = cfg.meas_noise_std(:).^2;            % [um^2], 3x1

    % --- adaptation (b): windows derived from cfg ---
    t_osc0    = cfg.t_hold + cfg.t_descend_override;
    t_osc1    = t_osc0 + cfg.n_cycles / cfg.frequency;
    t_discard = 1.0;   % drop first 1 s = first f cycles (design §6.0)

    % --- adaptation (c): gate-mask threshold ---
    H_BAR_GATE = 1.5;  % matches ctrl_const.h_bar_safe (design §3)

    % --- aligned error signals (design §6.0: e[k] = p_d[k+1] - p_true[k]) ---
    get_e  = @(so) so.p_d_out(2:end, :) - so.p_true_out(1:end-1, :);
    get_em = @(so) so.p_d_out(2:end, :) - so.p_m_out(1:end-1, :);
    so_ref = runs.A.det.simOut;
    t_e    = so_ref.tout(2:end);                    % time of aligned samples
    pd_al  = so_ref.p_d_out(2:end, :);

    % --- windows ---
    W.desc  = (t_e >= cfg.t_hold & t_e < t_osc0);
    W.osc   = (t_e >= t_osc0 + t_discard & t_e < t_osc1);
    W.tail  = (t_e >= t_osc1);
    h_bar_d = (pd_al * w_hat - pz) / R_phys;       % [N-1 x 1], deterministic
    W.gon   = W.osc & (h_bar_d < H_BAR_GATE);
    W.goff  = W.osc & (h_bar_d >= H_BAR_GATE);
    W.full  = true(size(W.osc));                   % full displayed span

    % --- det metrics per arm (design §6.1) ---
    D = struct();
    for arm = 'AB'
        e_det      = get_e(runs.(arm).det.simOut);
        D.(arm)    = det_metrics(e_det, t_e, W, f, t_osc0, t_osc1, Ts);
        A.e_det.(arm) = e_det;                      % stored for figures
    end
    A.det = D;

    % --- ram metrics per arm (design §6.2) + p_m cross-check (§6.3) ---
    wins = {'desc', 'osc', 'gon', 'goff'};
    R = struct();
    for arm = 'AB'
        em_det = get_em(runs.(arm).det.simOut);     % = e_det (noise-free)
        R.(arm) = ram_window_stats(runs.(arm).noisy, A.e_det.(arm), em_det, ...
                                   get_e, get_em, W, wins, sigma2_n);
    end
    % paired ratios (same-seed, non-diverged pairs only); C1: if no valid
    % pair exists, ratios stay NaN (no empty-assignment crash)
    ok = ~R.A.diverged & ~R.B.diverged;
    R.ratio = paired_ratio_stats(R.A, R.B, numel(wins));
    % M4: p_m cross-check (design §6.3) — osc window, z axis, both arms
    w_osc = find(strcmp(wins, 'osc'));
    rd = zeros(1, 0);
    for arm = 'AB'
        % reshape (not squeeze): squeeze on [1 x ns x 1] yields a COLUMN for
        % ns > 1; AND-ing it with the ROW .diverged would implicitly expand
        % to [ns x ns] and break the logical indexing below.
        sd_z  = reshape(R.(arm).sd(w_osc, :, 3), 1, []);
        sdp_z = reshape(R.(arm).sd_pm(w_osc, :, 3), 1, []);
        okv   = ~R.(arm).diverged & ~isnan(sd_z) & sd_z > 0;
        rd    = [rd, abs(sdp_z(okv) - sd_z(okv)) ./ sd_z(okv)]; %#ok<AGROW>
    end
    if isempty(rd)
        R.pm_rel_diff_z = NaN;
    else
        R.pm_rel_diff_z = max(rd);
    end
    R.wins = wins;
    A.ram = R;

    % --- noisy-ensemble det extraction (methodology update 2026-06-11) ---
    % The noise-free det run is NOT a valid det reference for the estimated
    % arm: its EKF runs in a no-noise regime (Guard 2 latched, y2 off); the
    % saved mu columns quantify the discrepancy (-2.7 to -12.7 nm on z).
    % det := ensemble mean of p_true over non-diverged noisy seeds.
    % Feeds the trajectory-space figures and the ram v2 statistics below;
    % the v1 tables (noise-free det reference) are kept for comparison.
    nd_pair_idx = find(ok, 1);                      % first paired seed
    for arm = 'AB'
        nd = noisy_det_extract(runs.(arm));
        fprintf(['[noisy-det:%gHz arm %c] crossval vs noise-free run ', ...
                 '[x y z] nm: max [%.2f %.2f %.2f] / rms [%.2f %.2f %.2f]\n'], ...
                f, arm, nd.crossval * 1e3, nd.crossval_rms * 1e3);
        % keep only what the figures need (limit analysis.mat size)
        keep = struct('det_traj', nd.det_traj, 'crossval', nd.crossval, ...
                      'crossval_rms', nd.crossval_rms, ...
                      'n_seeds', nd.n_seeds, 'seeds_used', nd.seeds_used);
        if isempty(nd_pair_idx)
            keep.ram_traj = nan(size(nd.det_traj));
        else
            si = find(nd.seeds_used == nd_pair_idx, 1);
            keep.ram_traj = nd.ram_traj(:, :, si);  % [N-1 x 3], first paired seed
        end
        A.noisy_det.(arm) = keep;
    end

    % --- ram v2: ensemble-det reference (same windows/stats as v1) ---
    % det_e_v2 = p_d_al - det_traj (error-space ensemble det); ram_v2_s =
    % e_s - det_e_v2, identical sign convention to v1. Notes: (a) seed-mean
    % of ram_v2 at every t is zero BY CONSTRUCTION (sample-mean centering) —
    % rectification bias lives inside det_traj; (b) sd_v2 carries the
    % self-subtraction deflation sqrt(1-1/Ns). No seed count is hardcoded.
    R2 = struct();
    det_e_v2 = struct();
    for arm = 'AB'
        nz_a = runs.(arm).noisy;
        ok_a = find(~[nz_a.diverged]);
        acc  = 0;
        for s = ok_a
            acc = acc + nz_a(s).simOut.p_m_out(1:end-1, :);
        end
        det_e_v2.(arm)  = pd_al - A.noisy_det.(arm).det_traj;
        det_em_v2       = pd_al - acc / numel(ok_a);   % measured-space ensemble det
        R2.(arm) = ram_window_stats(nz_a, det_e_v2.(arm), det_em_v2, ...
                                    get_e, get_em, W, wins, sigma2_n);
    end
    R2.ratio = paired_ratio_stats(R2.A, R2.B, numel(wins));
    R2.wins  = wins;
    A.ram_v2 = R2;

    % --- thermal-theory validation of ram_v2 (V1 closed form, h50-validated) ---
    A.thermal = thermal_theory_check(runs, det_e_v2, get_e, W, cfg, kBT, f);

    % --- tail-window observation (M3): hold at h_bar bottom, z axis ---
    tail = struct();
    for arm = 'AB'
        ed = A.e_det.(arm);
        tail.(arm).e_mean_z = mean(ed(W.tail, 3));
        tail.(arm).e_std_z  = std(ed(W.tail, 3));
        nzt = runs.(arm).noisy;
        sdt = nan(1, numel(nzt));
        for s = 1:numel(nzt)
            if nzt(s).diverged; continue; end
            ram_t  = get_e(nzt(s).simOut) - ed;
            sdt(s) = std(ram_t(W.tail, 3));
        end
        tail.(arm).ram_sd_z = mean(sdt, 'omitnan');
    end
    A.tail = tail;

    % --- stationarity per cycle (design §6.2 / Layer 2) ---
    A.stationarity = cycle_stationarity(runs, A.e_det, get_e, W, f, Ts);

    % --- theory anchor on arm A (design §7.4-7.5) ---
    A.anchor = theory_anchor(runs.A, A.e_det.A, get_e, W, cfg, kBT);
    % v2: same anchor with the ensemble-det reference (only norm_std kept;
    % the det-anchor field of this call is noise-dominated and discarded)
    anc_v2 = theory_anchor(runs.A, det_e_v2.A, get_e, W, cfg, kBT);
    A.anchor.norm_std_v2 = anc_v2.norm_std;

    % --- a_hat decomposition, arm B (design §6.4) ---
    A.ahat = ahat_analysis(runs.B, W);

    % --- flags ---
    A.flags = collect_flags(A);
end


function D = det_metrics(e_det, t_e, W, f, t_osc0, t_osc1, Ts)
%DET_METRICS design §6.1 (per axis; z = col 3 is primary).
    D = struct('desc_peak', zeros(1,3), 'desc_peak_t', zeros(1,3), ...
               'c0', zeros(1,3), 'A_e', zeros(1,3), 'phi_deg', zeros(1,3), ...
               'rms_res', zeros(1,3), 'trough_bias', zeros(1,3), ...
               'cycle_wave', []);
    for ax = 1:3
        % descent peak
        ew_desc = e_det(W.desc, ax);
        td      = t_e(W.desc);
        if isempty(ew_desc)
            D.desc_peak(ax) = NaN; D.desc_peak_t(ax) = NaN;
        else
            [pk, ipk] = max(abs(ew_desc));
            D.desc_peak(ax) = pk; D.desc_peak_t(ax) = td(ipk);
        end
        % sine fit on W.osc: e ~ c0 + a1 cos + b1 sin
        tw = t_e(W.osc); ew = e_det(W.osc, ax);
        X  = [ones(numel(tw), 1), cos(2*pi*f*tw), sin(2*pi*f*tw)];
        c  = X \ ew;
        D.c0(ax)      = c(1);
        D.A_e(ax)     = hypot(c(2), c(3));
        D.phi_deg(ax) = atan2d(c(3), c(2));
        D.rms_res(ax) = rms(ew - X * c);
        % trough bias: troughs near t = t_osc0 + j/f inside W.osc, +-5 samples
        tb = zeros(1, 0);
        for tj = (t_osc0 + 1/f):(1/f):(t_osc1 - 0.01/f)
            if tj < t_osc0 + 1.0; continue; end   % inside discard zone
            [~, k0] = min(abs(t_e - tj));
            tb(end+1) = mean(e_det(max(1,k0-5):min(numel(t_e),k0+5), ax)); %#ok<AGROW>
        end
        if isempty(tb)
            D.trough_bias(ax) = NaN;
        else
            D.trough_bias(ax) = mean(tb);
        end
        % cycle-averaged waveform (z only; stored for figures)
        if ax == 3
            npc = round(1 / (f * Ts));
            assert(abs(npc * f * Ts - 1) < 1e-9, ...
                   'det_metrics: non-integer samples/cycle (npc*f*Ts = %.12g)', ...
                   npc * f * Ts);
            io  = find(W.osc);
            nc  = floor(numel(io) / npc);
            if nc > 0
                D.cycle_wave = mean(reshape(e_det(io(1:nc*npc), 3), npc, nc), 2);
            end
        end
    end
end


function st = cycle_stationarity(runs, e_det_all, get_e, W, f, Ts)
%CYCLE_STATIONARITY per-cycle std of z-axis ram, first vs second half.
    st = struct();
    for arm = 'AB'
        nz  = runs.(arm).noisy;
        npc = round(1 / (f * Ts));
        assert(abs(npc * f * Ts - 1) < 1e-9, ...
               'cycle_stationarity: non-integer samples/cycle (npc*f*Ts = %.12g)', ...
               npc * f * Ts);
        io  = find(W.osc);
        nc  = floor(numel(io) / npc);
        sd_c = nan(nc, numel(nz));
        for s = 1:numel(nz)
            if nz(s).diverged; continue; end
            ram_z = get_e(nz(s).simOut) - e_det_all.(arm);
            ram_z = ram_z(:, 3);
            for c = 1:nc
                sd_c(c, s) = std(ram_z(io((c-1)*npc+1 : c*npc)));
            end
        end
        m  = mean(sd_c, 2, 'omitnan');
        h1 = mean(m(1:floor(nc/2)));
        h2 = mean(m(floor(nc/2)+1:end));
        st.(arm).per_cycle_sd  = m;
        st.(arm).half_rel_diff = abs(h2 - h1) / max(h1, eps);
        st.(arm).stationary    = st.(arm).half_rel_diff < 0.20;
    end
end


function anc = theory_anchor(runsA, e_detA, get_e, W, cfg, kBT)
%THEORY_ANCHOR design §7.4-7.5: arm A det ~ 0 and normalized ram ~ 1.
    % --- adaptation (d): theory-anchor constants from cfg ---
    lc         = cfg.lambda_c;
    C_dx       = 2 + 1 / (1 - lc^2);              % V1 closed form (3.9608 at lc=0.7)
    sigma2_nz  = cfg.meas_noise_std(3)^2;

    % I2: an empty osc window would make the anchors vacuous
    assert(any(W.osc), ...
           'theory_anchor: W.osc is empty — check cfg timing fields (t_hold/t_descend_override/n_cycles/frequency)');

    % envelope from arm A det run's a_true (z axis), aligned to e[k]:
    % a_true_out[k] is at t_k (pre-integration); e[k] is at t_k+Ts -> shift one.
    a_true_z = runsA.det.simOut.a_true_out(2:end, 3);
    sigma_th  = sqrt(C_dx * 4 * kBT * a_true_z + (1-lc)/(1+lc) * sigma2_nz);
    anc.sigma_th = sigma_th;
    % det anchor (soft gate < 1 nm = 1e-3 um in W.osc)
    anc.det_max_osc_um = max(abs(e_detA(W.osc, 3)));
    anc.det_pass       = anc.det_max_osc_um < 1e-3;
    % normalized ram per seed
    nz = runsA.noisy;
    zs = nan(1, numel(nz));
    for s = 1:numel(nz)
        if nz(s).diverged; continue; end
        ram_z = get_e(nz(s).simOut) - e_detA;
        ram_z = ram_z(:, 3);
        zn    = ram_z(W.osc) ./ sigma_th(W.osc);
        zs(s) = std(zn);
    end
    % I2: all-diverged arm A must not pass vacuously (all([]) == true)
    valid = ~isnan(zs);
    anc.norm_std  = zs;
    anc.norm_pass = any(valid) && all(abs(zs(valid) - 1) < 0.15);
end


function ah = ahat_analysis(runsB, W)
%AHAT_ANALYSIS design §6.4: ensemble-mean systematic + per-seed random.
    nz = runsB.noisy;
    ok = find(~[nz.diverged]);
    a_true = runsB.det.simOut.a_true_out(2:end, :);
    ah.a_true  = a_true;
    ah.n_seeds = numel(ok);
    if isempty(ok)
        % C2: all arm-B noisy seeds diverged — NaN-filled, same fields
        ah.ens_mean      = nan(size(a_true));
        ah.rel_err_osc   = nan(1, 3);
        ah.rel_err_gon   = nan(1, 3);
        ah.rel_err_goff  = nan(1, 3);
        ah.ram_std_osc   = nan(3, 1);
        ah.gate_duty_osc = nan(3, 1);
        return;
    end
    a_stack = [];
    for s = ok
        a_stack = cat(3, a_stack, nz(s).simOut.diag.a_hat(2:end, :));
    end
    ah.ens_mean = mean(a_stack, 3);                 % [N-1 x 3]
    rel = (ah.ens_mean - a_true) ./ a_true * 100;   % [%], elementwise
    ah.rel_err_osc  = mean(rel(W.osc,  :), 1);      % per axis [1x3]
    ah.rel_err_gon  = mean(rel(W.gon,  :), 1);      % gate-on (trough region)
    ah.rel_err_goff = mean(rel(W.goff, :), 1);      % gate-off (peak region)
    dev = a_stack - ah.ens_mean;
    sd3 = std(dev(W.osc, :, :), 0, 1);              % [1 x 3 x Ns]
    ah.ram_std_osc = reshape(sd3, 3, []);           % [3 x Ns] (M5)
    % gate duty cycle in W.osc (per axis, mean over seeds)
    gd = [];
    for s = ok
        gd = cat(3, gd, double(nz(s).simOut.diag.gate_active(2:end, :)));
    end
    ah.gate_duty_osc = squeeze(mean(mean(gd(W.osc, :, :), 1), 3)).';
end


function nd = noisy_det_extract(runs_arm)
%NOISY_DET_EXTRACT det trajectory as ensemble mean over noisy seeds.
%   The noise-free det run is invalid as a det reference for the estimated
%   arm (EKF in a no-noise regime: Guard 2 latched, y2 off), so det is
%   extracted as the ensemble mean of p_true over NON-diverged noisy seeds,
%   on the aligned [N-1 x 3] grid (t_e = tout(2:end), p_true_out(1:end-1,:)).
%   crossval / crossval_rms compare against the noise-free det run's
%   trajectory: for arm A this validates the extraction (ensemble residual
%   noise); for arm B it measures the Guard-2 discrepancy.
    nz = runs_arm.noisy;
    ok = find(~[nz.diverged]);
    assert(~isempty(ok), 'noisy_det_extract: all noisy seeds diverged');
    n  = numel(ok);
    p1 = nz(ok(1)).simOut.p_true_out(1:end-1, :);
    stack = zeros([size(p1), n]);
    stack(:, :, 1) = p1;
    for i = 2:n
        stack(:, :, i) = nz(ok(i)).simOut.p_true_out(1:end-1, :);
    end
    nd.det_traj   = mean(stack, 3);                 % [N-1 x 3]
    nd.ram_traj   = stack - nd.det_traj;            % [N-1 x 3 x n] per seed
    nd.seeds_used = ok;
    nd.n_seeds    = n;
    p_det = runs_arm.det.simOut.p_true_out(1:end-1, :);
    nd.crossval     = max(abs(nd.det_traj - p_det), [], 1).';   % [3x1]
    nd.crossval_rms = rms(nd.det_traj - p_det, 1).';            % [3x1]
end


function Rarm = ram_window_stats(nz, e_ref, em_ref, get_e, get_em, W, wins, sigma2_n)
%RAM_WINDOW_STATS per-arm per-window ram statistics against a det reference.
%   Shared by v1 (noise-free det reference) and v2 (ensemble det reference).
%   Diverged seeds excluded (NaN). Works for any number of seeds.
    ns  = numel(nz);
    div = [nz.diverged];
    Rarm.mu    = nan(numel(wins), ns, 3);
    Rarm.sd    = nan(numel(wins), ns, 3);
    Rarm.sd_pm = nan(numel(wins), ns, 3);
    for s = 1:ns
        if div(s); continue; end                    % diverged: excluded (NaN)
        ram  = get_e(nz(s).simOut)  - e_ref;
        ramm = get_em(nz(s).simOut) - em_ref;
        for w = 1:numel(wins)
            idx = W.(wins{w});
            for ax = 1:3
                Rarm.mu(w, s, ax) = mean(ram(idx, ax));
                Rarm.sd(w, s, ax) = std(ram(idx, ax));
                sd_m = std(ramm(idx, ax));
                Rarm.sd_pm(w, s, ax) = sqrt(max(sd_m^2 - sigma2_n(ax), 0));
            end
        end
    end
    Rarm.diverged = div;
end


function ratio = paired_ratio_stats(RA, RB, nwins)
%PAIRED_RATIO_STATS same-seed B/A sd ratios (non-diverged pairs only).
%   C1: if no valid pair exists, ratios stay NaN (no empty-assignment crash).
    okp = ~RA.diverged & ~RB.diverged;
    ratio.mean = nan(nwins, 3);
    ratio.rng  = nan(nwins, 3, 2);
    if any(okp)
        for w = 1:nwins
            for ax = 1:3
                r = squeeze(RB.sd(w, okp, ax)) ./ squeeze(RA.sd(w, okp, ax));
                ratio.mean(w, ax)   = mean(r);
                ratio.rng(w, ax, 1) = min(r);
                ratio.rng(w, ax, 2) = max(r);
            end
        end
    end
end


function th = thermal_theory_check(runs, det_e_v2, get_e, W, cfg, kBT, f)
%THERMAL_THEORY_CHECK V1 closed-form thermal baseline vs ram_v2 (B1/B2).
%   sigma2_th_i(t) = C_dx*4*kBT*a_true_i(t) + C_n_fb*sigma2_n_i, with
%   C_dx = 2 + 1/(1-lc^2), C_n_fb = (1-lc)/(1+lc) (V1 closed form, validated
%   at h50). a_true_i(t) is the arm's det-run deterministic skeleton.
%   Measured window variances are corrected for the ensemble-det
%   self-subtraction by dividing by (1 - 1/Ns).
    lc       = cfg.lambda_c;
    C_dx     = 2 + 1 / (1 - lc^2);
    C_n_fb   = (1 - lc) / (1 + lc);
    sigma2_n = cfg.meas_noise_std(:).^2;            % [um^2], 3x1

    th.C_dx = C_dx; th.C_n_fb = C_n_fb; th.lc = lc;
    th.wx = {'desc', 'osc', 'gon', 'goff', 'full'}; % B1 x-axis windows
    th.wz = {'osc', 'gon', 'goff'};                 % B2 z-axis windows
    for arm = 'AB'
        nz = runs.(arm).noisy;
        ok = find(~[nz.diverged]);
        Ns = numel(ok);
        if Ns > 1
            corr = 1 - 1/Ns;                        % self-subtraction correction
        else
            corr = NaN;                             % degenerate: stats undefined
        end
        a_true = runs.(arm).det.simOut.a_true_out(2:end, :);
        s2x = C_dx * 4 * kBT * a_true(:, 1) + C_n_fb * sigma2_n(1);  % [um^2]
        s2z = C_dx * 4 * kBT * a_true(:, 3) + C_n_fb * sigma2_n(3);
        vx = nan(numel(th.wx), Ns);
        vz = nan(numel(th.wz), Ns);
        for si = 1:Ns
            ram = get_e(nz(ok(si)).simOut) - det_e_v2.(arm);
            zn  = ram(:, 3) ./ sqrt(s2z);
            for k = 1:numel(th.wx)
                vx(k, si) = var(ram(W.(th.wx{k}), 1));
            end
            for k = 1:numel(th.wz)
                vz(k, si) = var(zn(W.(th.wz{k})));
            end
        end
        th.(arm).Ns = Ns;
        th.(arm).x_meas_nm2   = mean(vx, 2).' / corr * 1e6;   % um^2 -> nm^2
        th.(arm).x_theory_nm2 = cellfun(@(w) mean(s2x(W.(w))), th.wx) * 1e6;
        th.(arm).x_ratio      = th.(arm).x_meas_nm2 ./ th.(arm).x_theory_nm2;
        th.(arm).z_normvar    = mean(vz, 2).' / corr;
        fprintf(['[thermal:%gHz arm %c] x full-span meas/theory = %.3f, ', ...
                 'z osc norm-var = %.3f\n'], f, arm, ...
                th.(arm).x_ratio(strcmp(th.wx, 'full')), ...
                th.(arm).z_normvar(strcmp(th.wz, 'osc')));
    end
end


function flags = collect_flags(A)
    flags.failed        = false;
    flags.anchor_det    = A.anchor.det_pass;
    flags.anchor_norm   = A.anchor.norm_pass;
    flags.stationary_A  = A.stationarity.A.stationary;
    flags.stationary_B  = A.stationarity.B.stationary;
    flags.pm_crosscheck = A.ram.pm_rel_diff_z < 0.02;   % M4 (NaN -> false)
end


function write_summary_md(path, f, S, A)
    fid = fopen(path, 'w');
    if fid == -1
        error('write_summary_md: cannot open %s for writing', path);
    end
    fprintf(fid, '# gain_oracle_ab : %g Hz\n\n', f);
    R_phys_hdr = S.runs.A.det.simOut.meta.params_value.common.R;
    h_bar_min_hdr = S.cfg.h_bottom / R_phys_hdr;
    fprintf(fid, 'osc_aggr (h %g -> %g um, h_bar_min=%.2f, A=%g um), %d seeds x %.1fs, suppress_xD both arms.\n\n', ...
            S.cfg.h_init, S.cfg.h_bottom, h_bar_min_hdr, S.cfg.amplitude, numel(S.opts.seeds), S.cfg.T_sim);
    fprintf(fid, '## det (e_det = p_d - p_true, noise-free run)\n\n');
    fprintf(fid, '| metric | arm | x | y | z |\n|---|---|---|---|---|\n');
    for arm = 'AB'
        D = A.det.(arm);
        fprintf(fid, '| descent peak [nm] | %c | %.1f | %.1f | %.1f |\n', arm, D.desc_peak*1e3);
        fprintf(fid, '| osc A_e [nm] | %c | %.2f | %.2f | %.2f |\n',      arm, D.A_e*1e3);
        fprintf(fid, '| osc phase [deg] | %c | %.2f | %.2f | %.2f |\n',   arm, D.phi_deg);
        fprintf(fid, '| osc rms_res [nm] | %c | %.2f | %.2f | %.2f |\n',  arm, D.rms_res*1e3);
        fprintf(fid, '| trough bias [nm] | %c | %.2f | %.2f | %.2f |\n',  arm, D.trough_bias*1e3);
    end
    fprintf(fid, '\n## ram (noise-free det reference — superseded, kept for comparison)\n\n');
    ax_name = 'xyz';
    write_ram_table(fid, A.ram, ax_name);
    fprintf(fid, '\n## ram v2 (ensemble det reference)\n\n');
    defl = @(n) (1 - sqrt(max(1 - 1/n, 0))) * 100;  % self-subtraction deflation [%]
    fprintf(fid, ['Seed-mean of ram_v2 at every t is zero by construction ', ...
                  '(sample-mean centering); rectification bias lives inside ', ...
                  'det_traj. sd carries the self-subtraction deflation ', ...
                  'sqrt(1-1/Ns): arm A -%.1f%% (Ns=%d), arm B -%.1f%% (Ns=%d).\n\n'], ...
            defl(A.noisy_det.A.n_seeds), A.noisy_det.A.n_seeds, ...
            defl(A.noisy_det.B.n_seeds), A.noisy_det.B.n_seeds);
    write_ram_table(fid, A.ram_v2, ax_name);
    wsel = {'osc', 'gon', 'goff'};
    cmp = cell(1, numel(wsel));
    for k = 1:numel(wsel)
        wi = find(strcmp(A.ram.wins, wsel{k}));
        cmp{k} = sprintf('%s %.2f -> %.2f', wsel{k}, ...
                         A.ram.ratio.mean(wi, 3), A.ram_v2.ratio.mean(wi, 3));
    end
    fprintf(fid, '\n- headline z ratio (old -> v2): %s\n', strjoin(cmp, ', '));
    fprintf(fid, '\n## thermal-theory validation (ram v2)\n\n');
    T = A.thermal;
    fprintf(fid, ['sigma2_th_i(t) = C_dx*4kBT*a_true_i(t) + C_n_fb*sigma2_n_i, ', ...
                  'C_dx = %.3f, C_n_fb = %.3f (lc = %g). Measured variances ', ...
                  'corrected by /(1-1/Ns).\n\n'], T.C_dx, T.C_n_fb, T.lc);
    fprintf(fid, '### x-axis windowed variance\n\n');
    fprintf(fid, '| window | arm | measured [nm^2] | theory [nm^2] | ratio |\n|---|---|---|---|---|\n');
    for k = 1:numel(T.wx)
        for arm = 'AB'
            fprintf(fid, '| %s | %c | %.1f | %.1f | %.3f |\n', T.wx{k}, arm, ...
                    T.(arm).x_meas_nm2(k), T.(arm).x_theory_nm2(k), T.(arm).x_ratio(k));
        end
    end
    fprintf(fid, '\n### z-axis normalized variance (target 1)\n\n');
    fprintf(fid, '| window | arm | norm var |\n|---|---|---|\n');
    for k = 1:numel(T.wz)
        for arm = 'AB'
            fprintf(fid, '| %s | %c | %.3f |\n', T.wz{k}, arm, T.(arm).z_normvar(k));
        end
    end
    fprintf(fid, ['\n_theory describes the thermal+noise baseline; arm-â excess ', ...
                  'above 1 is estimation-induced (cf. paired ratios); quasi-static ', ...
                  'approximation weakest at gate-on @5 Hz_\n']);
    fprintf(fid, '\n## ram rectification bias mu (seed mean over window)\n\n');
    fprintf(fid, '| window | axis | mu_A [nm] | mu_B [nm] |\n|---|---|---|---|\n');
    for w = 1:numel(A.ram.wins)
        for ax = 1:3
            muA = mean(squeeze(A.ram.A.mu(w, :, ax)), 'omitnan') * 1e3;
            muB = mean(squeeze(A.ram.B.mu(w, :, ax)), 'omitnan') * 1e3;
            fprintf(fid, '| %s | %c | %.3f | %.3f |\n', ...
                    A.ram.wins{w}, ax_name(ax), muA, muB);
        end
    end
    fprintf(fid, '\n## tail window (hold at h_bar bottom; observation only, z axis)\n\n');
    fprintf(fid, '| arm | det e mean [nm] | det e std [nm] | ram sd [nm] (seed mean) |\n|---|---|---|---|\n');
    for arm = 'AB'
        fprintf(fid, '| %c | %.2f | %.2f | %.2f |\n', arm, ...
                A.tail.(arm).e_mean_z*1e3, A.tail.(arm).e_std_z*1e3, ...
                A.tail.(arm).ram_sd_z*1e3);
    end
    fprintf(fid, '\n## validation\n\n');
    fprintf(fid, '- arm A det anchor: max|e_det| (osc, z) = %.3f nm -> %s (soft gate < 1 nm)\n', ...
            A.anchor.det_max_osc_um*1e3, passstr(A.anchor.det_pass));
    fprintf(fid, '- arm A normalized ram std (per seed): %s -> %s (soft gate 1 +- 0.15)\n', ...
            mat2str(round(A.anchor.norm_std, 3)), passstr(A.anchor.norm_pass));
    fprintf(fid, '- arm A normalized ram std v2 (ensemble det, per seed): %s\n', ...
            mat2str(round(A.anchor.norm_std_v2, 3)));
    idx_pair = find(~A.ram.A.diverged & ~A.ram.B.diverged, 1);
    if isempty(idx_pair)
        fprintf(fid, '- traj-figure seed: none (no paired non-diverged seed)\n');
    else
        if isfield(S.runs.B.noisy, 'seed')
            seed_no = S.runs.B.noisy(idx_pair).seed;
        else
            seed_no = idx_pair;
        end
        fprintf(fid, '- traj-figure seed: %d\n', seed_no);
    end
    fprintf(fid, '- stationarity (half-diff): A %.1f%% / B %.1f%%\n', ...
            A.stationarity.A.half_rel_diff*100, A.stationarity.B.half_rel_diff*100);
    fprintf(fid, '- p_m cross-check (osc, z): max rel-diff = %.3f%% -> %s (soft gate < 2%%)\n', ...
            A.ram.pm_rel_diff_z*100, passstr(A.flags.pm_crosscheck));
    fprintf(fid, '- diverged runs: A %s / B %s\n', ...
            mat2str(A.ram.A.diverged), mat2str(A.ram.B.diverged));
    fprintf(fid, '\n## arm B gain estimation\n\n');
    fprintf(fid, '- a_hat ensemble-mean rel-err (osc) [%%]: %s\n', ...
            mat2str(round(A.ahat.rel_err_osc, 2)));
    fprintf(fid, '- a_hat ensemble-mean rel-err (gate-on, trough) [%%]: %s\n', ...
            mat2str(round(A.ahat.rel_err_gon, 2)));
    fprintf(fid, '- a_hat ensemble-mean rel-err (gate-off, peak) [%%]: %s\n', ...
            mat2str(round(A.ahat.rel_err_goff, 2)));
    rs = mean(A.ahat.ram_std_osc, 2, 'omitnan');    % [3x1], seed mean
    fprintf(fid, '- a_hat ram std (osc, seed mean) [um/pN]: [%.3g %.3g %.3g]\n', rs);
    fprintf(fid, '- gate duty cycle (osc): %s\n', mat2str(round(A.ahat.gate_duty_osc, 3)));
    if A.ahat.n_seeds > 0
        defl_ahat = (1 - sqrt(1 - 1/A.ahat.n_seeds)) * 100;
    else
        defl_ahat = NaN;
    end
    fprintf(fid, ['\n_Note: a_hat ram std (and ram v2 sd above) are measured ', ...
                  'against the %d-seed ensemble mean; self-subtraction deflates ', ...
                  'std by sqrt(1-1/Ns) = -%.1f%% at Ns=%d._\n'], ...
            A.ahat.n_seeds, defl_ahat, A.ahat.n_seeds);
    fclose(fid);
end


function write_ram_table(fid, R, ax_name)
%WRITE_RAM_TABLE std-over-window table (seed mean [min, max]; paired B/A ratio).
    fprintf(fid, '| window | axis | A sd [nm] (rng) | B sd [nm] (rng) | ratio (rng) |\n|---|---|---|---|---|\n');
    for w = 1:numel(R.wins)
        for ax = 1:3
            sdA = reshape(R.A.sd(w, :, ax), 1, []) * 1e3;
            sdB = reshape(R.B.sd(w, :, ax), 1, []) * 1e3;
            fprintf(fid, '| %s | %c | %.2f [%.2f, %.2f] | %.2f [%.2f, %.2f] | %.2f [%.2f, %.2f] |\n', ...
                    R.wins{w}, ax_name(ax), ...
                    mean(sdA, 'omitnan'), min(sdA), max(sdA), ...
                    mean(sdB, 'omitnan'), min(sdB), max(sdB), ...
                    R.ratio.mean(w, ax), ...
                    R.ratio.rng(w, ax, 1), R.ratio.rng(w, ax, 2));
        end
    end
end


function s = passstr(b)
    if b; s = 'PASS'; else; s = 'FLAG'; end
end


function make_figs(S, A, f, out_dir)
%MAKE_FIGS per-frequency figures, EXP style (user presentation spec).
%   No titles; bold large fonts; 3 y-ticks; FIXED axis ranges across all
%   frequencies (except fig_det_err: auto bound this round, reported per
%   freq so a common lock can be chosen). The single-seed ram comes from
%   A.noisy_det.(arm).ram_traj (first paired non-diverged seed; seed number
%   recorded in summary.md).
%   fig_traj_det : desired vs noisy-ensemble det trajectory, z only
%   fig_det_err  : det error overlay, z only (asymmetric det reference)
%   fig_traj_ram : trajectory-space ram (vs noisy-ensemble det), x/z
    COL_TRUE = [0 0.6 0]; COL_B = [0.8 0 0]; COL_A = [0.45 0.30 0.75];
    FS_LAB = 24; FS_AX = 20; FS_LEG = 18; LW = 2.0;
    % cross-frequency comparability locks: identical axis ranges/ticks at
    % every frequency so the figures can be compared side by side
    XLIM_T     = [0 7];      XTICK_T     = 0:1:7;          % time axis [s]
    DET_Z_YLIM = [0 55];     DET_Z_YTICK = [0 25 50];      % [um]
    RAM_YLIM   = [-150 150]; RAM_YTICK   = [-100 0 100];   % [nm]
    so_ref = S.runs.A.det.simOut;
    t_e   = so_ref.tout(2:end);
    pd_al = so_ref.p_d_out(2:end, :);
    cols = [1 3]; axl = 'xz';                       % ram fig: top = x, bottom = z

    % ---- fig_traj_det: desired vs noisy-ensemble det trajectory, z only ----
    ft = figure('Position', [80 80 1100 460], 'Color', 'w', 'NumberTitle', 'off', ...
                'Visible', 'off');
    hold on;
    plot(t_e, pd_al(:, 3), '-', 'Color', COL_TRUE, 'LineWidth', 3, ...
         'DisplayName', 'desired');
    plot(t_e, A.noisy_det.A.det_traj(:, 3), '-', 'Color', COL_A, 'LineWidth', LW, ...
         'DisplayName', 'a = a_{true}');
    plot(t_e, A.noisy_det.B.det_traj(:, 3), '-', 'Color', COL_B, 'LineWidth', LW, ...
         'DisplayName', 'a = â');
    xlim(XLIM_T); ylim(DET_Z_YLIM);
    set(gca, 'XTick', XTICK_T, 'YTick', DET_Z_YTICK, ...
             'FontSize', FS_AX, 'FontWeight', 'bold');
    ylabel('z  (\mum)', 'FontSize', FS_LAB, 'FontWeight', 'bold');
    xlabel('t (s)', 'FontSize', FS_LAB, 'FontWeight', 'bold');
    grid off;
    lg = legend('Location', 'northoutside', 'Orientation', 'horizontal');
    lg.FontSize = FS_LEG; lg.FontWeight = 'bold';
    % NOTE: figures leak only if exportgraphics throws (close is skipped);
    % acceptable for the -batch workflow (make_eq17_6state_figures precedent).
    exportgraphics(ft, fullfile(out_dir, 'fig_traj_det.png'), 'Resolution', 150);
    close(ft);

    % ---- fig_det_err: det error overlay, z only (asymmetric det reference) ----
    % User-approved asymmetric references: the a_true arm uses the NOISE-FREE
    % det run (valid for arm A — crossval-validated, exact, zero extraction
    % residual), while the â arm uses the 20-seed ENSEMBLE det (the noise-free
    % run is invalid for arm B: Guard 2 latched, y2 off). The red curve
    % therefore carries the ensemble extraction residual (~+-5.6 nm).
    errA = A.e_det.A(:, 3) * 1e3;                                % [nm]
    errB = (pd_al(:, 3) - A.noisy_det.B.det_traj(:, 3)) * 1e3;   % [nm]
    % auto symmetric nice bound this round (not locked); reported per freq so
    % the coordinator can lock a common value next round
    b_nm = ceil(max(abs(errB)) * 1.15 / 50) * 50;
    fprintf('[fig_det_err:%gHz] auto y-bound b = %g nm\n', f, b_nm);
    fde = figure('Position', [80 80 1100 460], 'Color', 'w', 'NumberTitle', 'off', ...
                 'Visible', 'off');
    hold on;
    plot(t_e, errA, '-', 'Color', COL_A, 'LineWidth', LW, 'DisplayName', 'a = a_{true}');
    plot(t_e, errB, '-', 'Color', COL_B, 'LineWidth', LW, 'DisplayName', 'a = â');
    xlim(XLIM_T); ylim([-b_nm b_nm]);
    set(gca, 'XTick', XTICK_T, 'YTick', [-b_nm 0 b_nm], ...
             'FontSize', FS_AX, 'FontWeight', 'bold');
    ylabel('\deltaz_{det}  (nm)', 'FontSize', FS_LAB, 'FontWeight', 'bold');
    xlabel('t (s)', 'FontSize', FS_LAB, 'FontWeight', 'bold');
    grid off;
    lg = legend('Location', 'northoutside', 'Orientation', 'horizontal');
    lg.FontSize = FS_LEG; lg.FontWeight = 'bold';
    exportgraphics(fde, fullfile(out_dir, 'fig_det_err.png'), 'Resolution', 150);
    close(fde);

    % ---- fig_traj_ram: trajectory-space ram vs noisy-ensemble det, x/z ----
    % first-paired-seed ram stored by per_freq_analysis (A.noisy_det.(arm).ram_traj);
    % per-tile legends: x row carries each curve's full-span variance, z row plain
    fm = figure('Position', [80 80 1100 720], 'Color', 'w', 'NumberTitle', 'off', ...
                'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for r = 1:2
        c = cols(r); nexttile; hold on;
        sigA = A.noisy_det.A.ram_traj(:, c) * 1e3;  % [nm]
        sigB = A.noisy_det.B.ram_traj(:, c) * 1e3;  % [nm]
        if r == 1                                   % x row: legend with var
            dnA = sprintf('a = a_{true}  (var %.0f nm^2)', var(sigA));
            dnB = sprintf('a = â  (var %.0f nm^2)', var(sigB));
        else                                        % z row: plain legend
            dnA = 'a = a_{true}';
            dnB = 'a = â';
        end
        plot(t_e, sigA, '-', 'Color', COL_A, 'LineWidth', LW, 'DisplayName', dnA);
        plot(t_e, sigB, '-', 'Color', COL_B, 'LineWidth', LW, 'DisplayName', dnB);
        xlim(XLIM_T); ylim(RAM_YLIM);
        set(gca, 'XTick', XTICK_T, 'YTick', RAM_YTICK, ...
                 'FontSize', FS_AX, 'FontWeight', 'bold');
        ylabel(sprintf('%c_{ram}  (nm)', axl(r)), 'FontSize', FS_LAB, 'FontWeight', 'bold');
        grid off;
        lg = legend('Location', 'northoutside', 'Orientation', 'horizontal');
        lg.FontSize = FS_LEG; lg.FontWeight = 'bold';
    end
    xlabel('t (s)', 'FontSize', FS_LAB, 'FontWeight', 'bold');
    exportgraphics(fm, fullfile(out_dir, 'fig_traj_ram.png'), 'Resolution', 150);
    close(fm);
end


function make_overview_fig(analysis, out_dir)
%MAKE_OVERVIEW_FIG fig5: paired B/A ratio vs frequency (z), per window.
    % DISABLED: stats based on superseded det reference; reinstate after
    % det/ram stats redo.
    fig5_enabled = false;
    if ~fig5_enabled
        return;
    end
    ok_mask = arrayfun(@(a) ~a.flags.failed, analysis);   %#ok<UNRCH> skip failed freqs
    ok = analysis(ok_mask);
    if isempty(ok); return; end
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    sweep = [0.10 0.30 0.85; 0.95 0.55 0.10; 0.55 0.20 0.65; 0.85 0.25 0.10];
    FS = 18; LFS = 14;
    f5 = figure('Position', [80 80 900 500], 'Color', 'w', 'NumberTitle', 'off', ...
                'Visible', 'off');
    hold on;
    wins = ok(1).ram.wins;
    freqs = [ok.freq];
    mk = {'o-', 's-', 'd-', '^-'};
    for w = 1:numel(wins)
        r = arrayfun(@(a) a.ram.ratio.mean(w, 3), ok);
        plot(freqs, r, mk{w}, 'Color', sweep(min(w, size(sweep, 1)), :), 'LineWidth', 2, ...
             'MarkerSize', 9, 'DisplayName', wins{w});
    end
    yline(1.0, '--', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XTick', freqs, 'FontSize', LFS);
    xlabel('frequency (Hz)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('ram std ratio  B/A  (z)', 'FontSize', FS, 'FontWeight', 'bold');
    title('Cost of estimated gain vs frequency', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');
    grid off;
    exportgraphics(f5, fullfile(out_dir, 'fig5_freq_overview.png'), 'Resolution', 150);
    close(f5);
end
