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
        S = load(fullfile(out_dir, 'runs.mat'));   % runs, cfg, opts, layer0, ...

        A = per_freq_analysis(S.runs, S.cfg, f);
        write_summary_md(fullfile(out_dir, 'summary.md'), f, S, A);
        save(fullfile(out_dir, 'analysis.mat'), 'A');
        if opts.save_fig
            make_figs(S, A, f, out_dir);           % Task 6 (stub for now)
        end
        analysis(end+1) = struct('freq', f, 'det', A.det, 'ram', A.ram, ...
                                 'anchor', A.anchor, 'ahat', A.ahat, ...
                                 'flags', A.flags); %#ok<AGROW>
        if opts.verbose; fprintf('[analyze:%gHz] done -> %s\n', f, out_dir); end
    end
    if numel(freqs) > 1 && opts.save_fig
        make_overview_fig(analysis, fullfile(opts.out_root, 'overview'));
    end
end


% ====================================================================
function A = per_freq_analysis(runs, cfg, f)
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
        e_det  = A.e_det.(arm);
        em_det = get_em(runs.(arm).det.simOut);     % = e_det (noise-free)
        nz  = runs.(arm).noisy;
        ns  = numel(nz);
        div = [nz.diverged];
        R.(arm).mu    = nan(numel(wins), ns, 3);
        R.(arm).sd    = nan(numel(wins), ns, 3);
        R.(arm).sd_pm = nan(numel(wins), ns, 3);
        for w = 1:numel(wins)
            idx = W.(wins{w});
            for s = 1:ns
                if div(s); continue; end            % diverged: excluded (NaN)
                ram  = get_e(nz(s).simOut)  - e_det;
                ramm = get_em(nz(s).simOut) - em_det;
                for ax = 1:3
                    R.(arm).mu(w, s, ax)  = mean(ram(idx, ax));
                    R.(arm).sd(w, s, ax)  = std(ram(idx, ax));
                    sd_m = std(ramm(idx, ax));
                    R.(arm).sd_pm(w, s, ax) = sqrt(max(sd_m^2 - sigma2_n(ax), 0));
                end
            end
        end
        R.(arm).diverged = div;
    end
    % paired ratios (same-seed, non-diverged pairs only)
    ok = ~R.A.diverged & ~R.B.diverged;
    for w = 1:numel(wins)
        for ax = 1:3
            r = squeeze(R.B.sd(w, ok, ax)) ./ squeeze(R.A.sd(w, ok, ax));
            R.ratio.mean(w, ax)    = mean(r);
            R.ratio.rng(w, ax, 1) = min(r);
            R.ratio.rng(w, ax, 2) = max(r);
        end
    end
    R.wins = wins;
    A.ram = R;

    % --- stationarity per cycle (design §6.2 / Layer 2) ---
    A.stationarity = cycle_stationarity(runs, A.e_det, get_e, W, f, Ts);

    % --- theory anchor on arm A (design §7.4-7.5) ---
    A.anchor = theory_anchor(runs.A, A.e_det.A, get_e, W, cfg, kBT);

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
    anc.norm_std  = zs;
    anc.norm_pass = all(abs(zs(~isnan(zs)) - 1) < 0.15);
end


function ah = ahat_analysis(runsB, W)
%AHAT_ANALYSIS design §6.4: ensemble-mean systematic + per-seed random.
    nz = runsB.noisy;
    ok = find(~[nz.diverged]);
    a_stack = [];
    for s = ok
        a_stack = cat(3, a_stack, nz(s).simOut.diag.a_hat(2:end, :));
    end
    ah.ens_mean = mean(a_stack, 3);                 % [N-1 x 3]
    a_true = runsB.det.simOut.a_true_out(2:end, :);
    ah.rel_err_osc = mean((ah.ens_mean(W.osc, :) - a_true(W.osc, :)) ...
                          ./ a_true(W.osc, :), 1) * 100;  % [%], per axis
    dev = a_stack - ah.ens_mean;
    ah.ram_std_osc = squeeze(std(reshape(dev(W.osc, :, :), [], 3, size(dev,3)), 0, 1));
    ah.a_true = a_true;
    % gate duty cycle in W.osc (per axis, mean over seeds)
    gd = [];
    for s = ok
        gd = cat(3, gd, double(nz(s).simOut.diag.gate_active(2:end, :)));
    end
    ah.gate_duty_osc = squeeze(mean(mean(gd(W.osc, :, :), 1), 3)).';
end


function flags = collect_flags(A)
    flags.anchor_det   = A.anchor.det_pass;
    flags.anchor_norm  = A.anchor.norm_pass;
    flags.stationary_A = A.stationarity.A.stationary;
    flags.stationary_B = A.stationarity.B.stationary;
end


function write_summary_md(path, f, S, A)
    fid = fopen(path, 'w');
    fprintf(fid, '# gain_oracle_ab : %g Hz\n\n', f);
    fprintf(fid, 'osc_aggr (h %g -> %g um, A=%g um), %d seeds x %.1fs, suppress_xD both arms.\n\n', ...
            S.cfg.h_init, S.cfg.h_bottom, S.cfg.amplitude, numel(S.opts.seeds), S.cfg.T_sim);
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
    fprintf(fid, '\n## ram (std over window, seed mean [min, max]; paired B/A ratio)\n\n');
    ax_name = 'xyz';
    fprintf(fid, '| window | axis | A sd [nm] (rng) | B sd [nm] (rng) | ratio (rng) |\n|---|---|---|---|---|\n');
    for w = 1:numel(A.ram.wins)
        for ax = 1:3
            sdA = squeeze(A.ram.A.sd(w, :, ax)) * 1e3;
            sdB = squeeze(A.ram.B.sd(w, :, ax)) * 1e3;
            fprintf(fid, '| %s | %c | %.2f [%.2f, %.2f] | %.2f [%.2f, %.2f] | %.2f [%.2f, %.2f] |\n', ...
                    A.ram.wins{w}, ax_name(ax), ...
                    mean(sdA, 'omitnan'), min(sdA), max(sdA), ...
                    mean(sdB, 'omitnan'), min(sdB), max(sdB), ...
                    A.ram.ratio.mean(w, ax), ...
                    A.ram.ratio.rng(w, ax, 1), A.ram.ratio.rng(w, ax, 2));
        end
    end
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
    fprintf(fid, '\n## validation\n\n');
    fprintf(fid, '- arm A det anchor: max|e_det| (osc, z) = %.3f nm -> %s (soft gate < 1 nm)\n', ...
            A.anchor.det_max_osc_um*1e3, passstr(A.anchor.det_pass));
    fprintf(fid, '- arm A normalized ram std (per seed): %s -> %s (soft gate 1 +- 0.15)\n', ...
            mat2str(round(A.anchor.norm_std, 3)), passstr(A.anchor.norm_pass));
    fprintf(fid, '- stationarity (half-diff): A %.1f%% / B %.1f%%\n', ...
            A.stationarity.A.half_rel_diff*100, A.stationarity.B.half_rel_diff*100);
    fprintf(fid, '- diverged runs: A %s / B %s\n', ...
            mat2str(A.ram.A.diverged), mat2str(A.ram.B.diverged));
    fprintf(fid, '\n## arm B gain estimation\n\n');
    fprintf(fid, '- a_hat ensemble-mean rel-err (osc) [%%]: %s\n', ...
            mat2str(round(A.ahat.rel_err_osc, 2)));
    fprintf(fid, '- gate duty cycle (osc): %s\n', mat2str(round(A.ahat.gate_duty_osc, 3)));
    fclose(fid);
end


function s = passstr(b)
    if b; s = 'PASS'; else; s = 'FLAG'; end
end


function make_figs(varargin)          %#ok<VANUS>  % Task 6 replaces this stub
end
function make_overview_fig(varargin)  %#ok<VANUS>  % Task 6 replaces this stub
end
