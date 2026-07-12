function summary = analyze_aprime_4state(opts)
%ANALYZE_APRIME_4STATE Metrics + figures for the six-arm taylor-gain study.
%
%   summary = analyze_aprime_4state()
%   summary = analyze_aprime_4state(opts)
%
%   Reads <data_root>/f1Hz[-smoke]/runs.mat (from compare_aprime_4state) and
%   writes into the same folder:
%     fig_aprime_timeseries.png  z-axis a_hat vs a_true, first non-diverged
%                                seed, one tile per arm
%     fig_aprime_binned.png      z-axis a_hat rel bias vs h_bar_d (osc window,
%                                binned, mean +/- cross-seed SEM), all arms
%     fig_aprime_p44.png         P44 honesty: cross-seed Var(e_ax) vs mean
%                                KF P44 over time (z), one tile per arm
%     summary.txt                per-arm metric table
%
%   Metrics (z axis, osc window, aligned same-row a_hat(k) vs a_true_out(k)):
%     bias    = pooled mean of a_hat/a_true - 1, +/- cross-seed SEM
%     a_std   = pooled std of a_hat/a_true - 1
%     track   = std of tracking error e_z (osc window), nm
%     P44/Var = median over osc window of mean-seed P44 / cross-seed Var(e_ax).
%               ratio ~ 1 = honest RANDOM spread; ratio < 1 = P44 UNDER-states
%               the error (overconfident); ratio > 1 = conservative. NOTE:
%               cross-seed var subtracts the ensemble mean, so any common-mode
%               bias/lag of a_hat is INVISIBLE here (spread honesty only).
%     div     = diverged runs / total
%
%   opts fields (all optional):
%     data_root (test_results/aprime_4state) | smoke (false) | save_fig (true)
%     n_bin (12) h_bar bins for the binned figure
%
%   See also: compare_aprime_4state

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'smoke');    opts.smoke = false;   end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'n_bin');    opts.n_bin = 12;      end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    if ~isfield(opts, 'data_root')
        opts.data_root = fullfile(project_root, 'test_results', 'aprime_4state');
    end
    if opts.smoke
        out_dir = fullfile(opts.data_root, 'f1Hz-smoke');
    else
        out_dir = fullfile(opts.data_root, 'f1Hz');
    end
    S = load(fullfile(out_dir, 'runs.mat'));
    runs = S.runs; cfg = S.cfg;

    arm_names = {'REF', 'A0', 'A1', 'A2', 'A3', 'A4'};
    arm_label = {'REF a=a\_true', 'A0 AR(1)', 'A1 RW+FF', ...
                 'A2 taylor known', 'A3 taylor ahat', 'A4 taylor diff'};
    n_arm = numel(arm_names);

    % --- time grid / windows (any non-diverged run gives the shared p_d) ---
    ref0 = first_ok(runs, arm_names);
    t    = ref0.simOut.tout;
    P    = ref0.simOut.meta.params_value;
    w_hat = P.wall.w_hat; pz = P.wall.pz; R = P.common.R;
    h_bar_d = (ref0.simOut.p_d_out * w_hat - pz) / R;
    t_osc0 = cfg.t_hold + cfg.t_descend_override;
    osc_dur = cfg.n_cycles / cfg.frequency;
    t_disc = min(1 / cfg.frequency, 0.5 * osc_dur);
    osc = (t >= t_osc0 + t_disc) & (t < t_osc0 + osc_dur);

    % --- per-arm metrics ---
    summary = struct();
    stacks  = struct();     % per-arm stacked arrays for figures
    for ai = 1:n_arm
        nm = arm_names{ai};
        nz = runs.(nm).noisy;
        ok = find(~[nz.diverged]);
        n_tot = numel(nz) + 1;
        n_div = (n_tot - 1 - numel(ok)) + runs.(nm).det.diverged;
        Ns = numel(ok);
        assert(Ns > 0, 'analyze_aprime_4state: all %s noisy seeds diverged', nm);

        N_t = numel(t);
        ah  = zeros(N_t, Ns); at = zeros(N_t, Ns);
        Pa  = zeros(N_t, Ns); ez = zeros(N_t - 1, Ns);
        ap  = zeros(N_t, Ns);
        for si = 1:Ns
            so = nz(ok(si)).simOut;
            ah(:, si) = so.diag.a_hat(:, 3);
            at(:, si) = so.a_true_out(:, 3);
            Pa(:, si) = so.diag.P_a(:, 3);
            ap(:, si) = so.diag.a_prime_used(:, 3);
            e = so.p_d_out(2:end, :) - so.p_true_out(1:end-1, :);
            ez(:, si) = e(:, 3);
        end
        rel = ah(osc, :) ./ at(osc, :) - 1;                  % [Nosc x Ns]
        per_seed_bias = mean(rel, 1);
        bias  = mean(per_seed_bias);
        bias_sem = std(per_seed_bias) / sqrt(Ns);
        a_std = std(rel(:));
        osc_e = osc(2:end);
        track_nm = 1e3 * std(reshape(ez(osc_e, :), [], 1));

        % P44 honesty (needs >= 2 seeds for a cross-seed variance)
        if Ns >= 2
            e_ax = at - ah;                                  % same-row error
            var_emp = var(e_ax, 0, 2);                       % [N_t x 1]
            p44_mean = mean(Pa, 2);
            r = p44_mean(osc) ./ var_emp(osc);
            p44_ratio = median(r);   % median: far-field tiny var_emp spikes the mean
        else
            p44_ratio = NaN;
        end

        summary.(nm) = struct('Ns', Ns, 'n_div', n_div, 'bias', bias, ...
                              'bias_sem', bias_sem, 'a_std', a_std, ...
                              'track_nm', track_nm, 'p44_ratio', p44_ratio);
        stacks.(nm) = struct('ah', ah, 'at', at, 'Pa', Pa, 'ap', ap, 'ok', ok);
    end

    % --- console + summary.txt ---
    fid_list = 1;
    if opts.save_fig
        fid = fopen(fullfile(out_dir, 'summary.txt'), 'w');
        fid_list = [1, fid];
    end
    for fo = fid_list
        fprintf(fo, 'aprime_4state six-arm study  (z axis, osc window %.1f-%.1f s, Ns per arm below)\n', ...
                t_osc0 + t_disc, t_osc0 + osc_dur);
        fprintf(fo, '%-4s %-4s %-5s %12s %9s %10s %12s\n', ...
                'arm', 'Ns', 'div', 'bias[%]', 'std[%]', 'track[nm]', 'P44/Var(e)');
        for ai = 1:n_arm
            nm = arm_names{ai}; m = summary.(nm);
            fprintf(fo, '%-4s %-4d %-5d %+7.2f+-%.2f %9.2f %10.1f %12.3f\n', ...
                    nm, m.Ns, m.n_div, 100*m.bias, 100*m.bias_sem, ...
                    100*m.a_std, m.track_nm, m.p44_ratio);
        end
    end
    if opts.save_fig; fclose(fid); end
    if ~opts.save_fig; return; end

    % ================= figures (EXP style: no grid/title, legend northoutside)
    COL = lines(n_arm);
    FS = 13; LFS = 10;

    % --- FIG 1: timeseries, one tile per arm ---
    f1 = figure('Position', [60 60 1250 700], 'Color', 'w', 'Visible', 'off');
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ai = 1:n_arm
        nm = arm_names{ai}; st = stacks.(nm);
        nexttile; hold on;
        plot(t, st.at(:, 1), 'k-', 'LineWidth', 1.4);
        plot(t, st.ah(:, 1), '-', 'Color', COL(ai, :), 'LineWidth', 1.0);
        xlabel('t [sec]'); ylabel('a_z [\mum/pN]');
        set(gca, 'FontSize', FS, 'LineWidth', 1.2, 'Box', 'on');
        legend({'a\_true', arm_label{ai}}, 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS);
    end
    exportgraphics(f1, fullfile(out_dir, 'fig_aprime_timeseries.png'), 'Resolution', 150);
    close(f1);

    % --- FIG 2: binned rel bias vs h_bar_d (osc window), all arms ---
    f2 = figure('Position', [80 80 900 620], 'Color', 'w', 'Visible', 'off');
    hold on;
    idx_osc = find(osc);
    hb_s = h_bar_d(idx_osc);
    nb = opts.n_bin;
    edges = linspace(min(hb_s), max(hb_s), nb + 1);
    hplots = gobjects(n_arm, 1);
    for ai = 1:n_arm
        nm = arm_names{ai}; st = stacks.(nm);
        rel_t = st.ah(idx_osc, :) ./ st.at(idx_osc, :) - 1;   % [Nosc x Ns]
        Ns = size(rel_t, 2);
        [bx, bm, bs] = deal(nan(nb, 1));
        for b = 1:nb
            hi = edges(b + 1) + (b == nb) * 1e-9;
            sel = hb_s >= edges(b) & hb_s < hi;
            if nnz(sel) < 3; continue; end
            per_seed = mean(rel_t(sel, :), 1);               % [1 x Ns]
            bx(b) = mean(hb_s(sel));
            bm(b) = mean(per_seed);
            bs(b) = std(per_seed) / sqrt(Ns);
        end
        hplots(ai) = errorbar(bx, 100*bm, 100*bs, '-o', 'Color', COL(ai, :), ...
                              'MarkerSize', 4, 'LineWidth', 1.2, 'CapSize', 4);
    end
    yline(0, 'k-');
    set(gca, 'XScale', 'log', 'FontSize', FS, 'LineWidth', 1.2, 'Box', 'on');
    xlabel('desired h\_bar'); ylabel('a\_hat_z / a\_true_z - 1 [%]');
    legend(hplots, arm_label, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', LFS);
    exportgraphics(f2, fullfile(out_dir, 'fig_aprime_binned.png'), 'Resolution', 150);
    close(f2);

    % --- FIG 3: P44 honesty, one tile per arm ---
    f3 = figure('Position', [100 100 1250 700], 'Color', 'w', 'Visible', 'off');
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ai = 1:n_arm
        nm = arm_names{ai}; st = stacks.(nm);
        nexttile; hold on;
        if size(st.ah, 2) >= 2
            var_emp = var(st.at - st.ah, 0, 2);
            plot(t, var_emp, 'k-', 'LineWidth', 1.2);
            plot(t, mean(st.Pa, 2), '-', 'Color', COL(ai, :), 'LineWidth', 1.2);
            legend({'Var(e_{ax}) emp', sprintf('%s P44', arm_label{ai})}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS);
        end
        set(gca, 'YScale', 'log', 'FontSize', FS, 'LineWidth', 1.2, 'Box', 'on');
        xlabel('t [sec]'); ylabel('[\mum/pN]^2');
    end
    exportgraphics(f3, fullfile(out_dir, 'fig_aprime_p44.png'), 'Resolution', 150);
    close(f3);

    fprintf('[analyze_aprime_4state] figures + summary.txt -> %s\n', out_dir);
end


function rec = first_ok(runs, arm_names)
    for ai = 1:numel(arm_names)
        recs = [runs.(arm_names{ai}).det, runs.(arm_names{ai}).noisy];
        for r = recs
            if ~isempty(r.simOut) && ~r.diverged
                rec = r; return;
            end
        end
    end
    error('analyze_aprime_4state: no non-diverged run found');
end
