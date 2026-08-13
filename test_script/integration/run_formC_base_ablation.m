% STATUS: ACTIVE (diagnostic, four-arm feature ablation on the Form C BASELINE
%          arm) -- controller model/controller/motion_control_law_formC_dist.m,
%          driver test_script/integration/run_formC_dist.m (arm 'base').
%          Read-only w.r.t. both: every arm is expressed as a
%          ctrl_const_override, nothing is edited.
% PURPOSE: what are the two inherited features worth on the parameter-free
%          4-state baseline?
%            A0 nominal | A1 y2 OFF | A2 fdet OFF | A3 both OFF | A4 S echo OFF
%          Two seeds (7 = best-behaved, 11 = worst of the house set) run on
%          IDENTICAL random draws, so the per-seed difference is the signal.
% EXPIRES: when the baseline / disturbance adjudication closes.
function out = run_formC_base_ablation(opts)
%RUN_FORMC_BASE_ABLATION  4-arm x 2-seed paired ablation on the Form C baseline.
%
%   out = run_formC_base_ablation()
%   out = run_formC_base_ablation(struct('seeds', [7 11], 'plot', true))
%
%   THE TWO FEATURES (both read at controller init from ctrl_const):
%     y2_off  (default false)  true  -> the gain-readout measurement update
%                                       (block [3](b)) is skipped every step.
%                                       The gain state is then corrected ONLY
%                                       through the y1 = dw_m update via the
%                                       cross-covariance P(4,1), plus its own
%                                       predict; y2 is the only channel that
%                                       MEASURES the gain.
%     use_fdet (default true)  false -> F_dw, the exogenous regressor that
%                                       enters F_e(3,4) = -F_dw and
%                                       F_e(4,4) = 1 + a_bar'*F_dw + A_a*M,
%                                       is built from the REALISED force
%                                       (F_dw_raw, which contains the measured
%                                       feedback term (1-lc)*dw_m) instead of
%                                       the deterministic mirror (F_dw_det,
%                                       dw_m := 0 and its own force history).
%                                       That makes the regressor correlated
%                                       with the state error -> rectification.
%
%   Metrics per arm/seed on z (e_a = 100*(a_hat - a_true)/a_true), windows as
%   run_formC_dist: desc peak / osc RMS / hold mean / RMS over the active run.
%   Gate fractions are RECONSTRUCTED EXACTLY from the logs (the driver stores
%   only the OR): G1 = t < t_warmup_kf, G2 <=> a_bar_wm <= 0 (a_xm_out has the
%   same sign, C_dpmr*kappa_T > 0), G3 = h_bar_hat < h_bar_safe. The
%   reconstruction is asserted against the logged OR every run.
%
%   Saves test_results/formC_base_ablation.mat (gitignored) and, with
%   opts.plot, writes the two figure pages via plot_formC_base_ablation.
%
%   See also: run_formC_dist, motion_control_law_formC_dist,
%             plot_formC_base_ablation

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds = [7 11]; end   % best / worst of the house set
    if ~isfield(opts, 'plot');    opts.plot  = true;   end
    if ~isfield(opts, 'verbose'); opts.verbose = false; end

    here = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(here));

    AX_Z = 3;
    seeds = opts.seeds(:).';
    n_seeds = numel(seeds);

    % --- arm table: name | y2_off | use_fdet ---------------------------------
    arms = struct( ...
        'tag',  {'A0', 'A1', 'A2', 'A3', 'A4'}, ...
        'name', {'nominal', 'y2 OFF', 'fdet OFF', 'both OFF', 'S echo OFF'}, ...
        'y2_off',   {false, true,  false, true,  false}, ...
        'use_fdet', {true,  true,  false, false, true}, ...
        'echo',     {true,  true,  true,  true,  false});
    n_arms = numel(arms);

    res = cell(n_arms, 1);
    for a = 1:n_arms
        ov = struct('y2_off', arms(a).y2_off, 'use_fdet', arms(a).use_fdet, ...
                    'y2_echo_corr', arms(a).echo);
        dopts = struct('arm', 'base', 'seeds', seeds, 'verbose', opts.verbose, ...
                       'ctrl_const_override', ov, ...
                       'y2_on', ~arms(a).y2_off);   % cosmetic only (see NOTE below)
        fprintf('\n########## ARM %s (%s): y2_off=%d use_fdet=%d echo=%d ##########\n', ...
                arms(a).tag, arms(a).name, arms(a).y2_off, arms(a).use_fdet, arms(a).echo);
        res{a} = run_formC_dist(dopts);
    end

    % ------------------------------------------------------------------
    % Gate fractions, clamp saturation, NaN sweep (exact reconstruction)
    % ------------------------------------------------------------------
    G = zeros(n_arms, n_seeds, 4);   % G1 | G2 | G3 | y2-update-ran, fractions
    sat = zeros(n_arms, n_seeds, 3); % at a_bar_floor | at a_bar_ceil | NaN flag
    for a = 1:n_arms
        for q = 1:n_seeds
            s  = res{a}.runs{q};
            cc = s.ctrl_const;
            t  = s.tout(:);
            % Row 1 is the controller's INIT call, which returns the prior
            % diag (h_bar = 0, a_xm = 0) and no gate decision at all; the
            % filter loop starts at row 2. Everything below is over the loop.
            kk = 2:numel(t);
            g1 = t(kk) < cc.t_warmup_kf;
            g2 = s.a_xm_out(kk, AX_Z) <= 0;
            g3 = s.h_bar_out(kk) < cc.h_bar_safe;
            assert(isequal(g1 | g2 | g3, s.gate_out(kk, AX_Z)), ...
                   'run_formC_base_ablation:gateRecon', ...
                   'gate reconstruction disagrees with the logged OR (arm %s, seed %d).', ...
                   arms(a).tag, seeds(q));
            N = numel(kk);
            G(a, q, 1) = sum(g1) / N;
            G(a, q, 2) = sum(g2) / N;
            G(a, q, 3) = sum(g3) / N;
            G(a, q, 4) = sum(~(g1 | g2 | g3) & ~arms(a).y2_off) / N;

            a_bar_floor = local_field(cc, 'a_bar_floor', 0.05);
            a_bar_ceil  = local_field(cc, 'a_bar_ceil', 1 - 1e-4);
            ab = s.a_bar_hat_out(kk, AX_Z);
            sat(a, q, 1) = sum(abs(ab - a_bar_floor) <= 1e-12) / N;
            sat(a, q, 2) = sum(abs(ab - a_bar_ceil)  <= 1e-12) / N;
            sat(a, q, 3) = res{a}.metrics.rows(q, 7);
        end
    end

    % ------------------------------------------------------------------
    % Tables (console only; nothing goes on the figures)
    % ------------------------------------------------------------------
    % desc pk | osc RMS | hold mean | rms all | final-hold drift %/s
    M = zeros(n_arms, n_seeds, 5);
    for a = 1:n_arms
        M(a, :, 1:4) = res{a}.metrics.rows(:, 1:4);
        M(a, :, 5)   = res{a}.metrics.da_rows(:, 8);
    end

    out_windows = res{1}.metrics.windows;
    mnames = {'desc peak %', 'osc RMS %', 'hold mean %', 'rel-err RMS %', ...
              'hold drift %/s'};
    n_met  = numel(mnames);
    fprintf('\n================ 4x2 METRIC TABLE (z axis, arm x seed) ================\n');
    for mi = 1:n_met
        fprintf('\n-- %s --\n%4s %-10s', mnames{mi}, 'arm', 'feature');
        fprintf('%12s', string(seeds) + " (seed)");
        fprintf('\n');
        for a = 1:n_arms
            fprintf('%4s %-10s', arms(a).tag, arms(a).name);
            fprintf('%12.3f', squeeze(M(a, :, mi)));
            fprintf('\n');
        end
    end

    fprintf('\n================ PAIRED DELTAS vs A0 (same seed, same draws) ================\n');
    for mi = 1:n_met
        fprintf('\n-- delta %s (positive = worse, except hold mean which is signed) --\n%4s', ...
                mnames{mi}, 'arm');
        fprintf('%12s', string(seeds) + " (seed)");
        fprintf('\n');
        for a = 2:n_arms
            fprintf('%4s', arms(a).tag);
            fprintf('%+12.3f', squeeze(M(a, :, mi) - M(1, :, mi)));
            fprintf('\n');
        end
        % additivity check: is A3 - A0 close to (A1 - A0) + (A2 - A0)?
        d1 = squeeze(M(2, :, mi) - M(1, :, mi));
        d2 = squeeze(M(3, :, mi) - M(1, :, mi));
        d3 = squeeze(M(4, :, mi) - M(1, :, mi));
        fprintf('%4s', 'sum');
        fprintf('%+12.3f', d1 + d2);
        fprintf('   <- (A1-A0)+(A2-A0)\n');
        fprintf('%4s', 'int');
        fprintf('%+12.3f', d3 - (d1 + d2));
        fprintf('   <- interaction A3-(A1+A2-A0)\n');
    end

    fprintf('\n================ GATE FRACTIONS (z axis, of the whole run) ================\n');
    fprintf('%4s %-10s %6s %8s %8s %8s %10s\n', 'arm', 'feature', 'seed', ...
            'G1 warm', 'G2 inval', 'G3 wall', 'y2 ran');
    for a = 1:n_arms
        for q = 1:n_seeds
            fprintf('%4s %-10s %6d %8.4f %8.4f %8.4f %10.4f\n', arms(a).tag, ...
                    arms(a).name, seeds(q), G(a, q, 1), G(a, q, 2), G(a, q, 3), G(a, q, 4));
        end
    end

    % --- who moves the gain in the final hold? ------------------------
    % The truth is constant there, so any net motion of a_bar_hat is spurious.
    % The y2 share is exact: sum over the hold of K2(4)*innov2 is the total
    % gain increment the readout update contributed (both are logged).
    fprintf('\n================ FINAL-HOLD GAIN MOTION (truth is constant) ================\n');
    fprintf('%4s %6s %14s %14s %14s %10s\n', 'arm', 'seed', 'd a_bar hold', ...
            'y2 share', 'y2 sum, run', 'y2 frac');
    hold_share = zeros(n_arms, n_seeds, 3);
    for a = 1:n_arms
        for q = 1:n_seeds
            s  = res{a}.runs{q};
            t  = s.tout(:);
            hm = t > out_windows.hold(1);
            ih = find(hm);
            k2i2 = s.K_a_y2_out(:, AX_Z) .* s.innov_y2_out(:, AX_Z);
            d_ab = s.a_bar_hat_out(ih(end), AX_Z) - s.a_bar_hat_out(ih(1), AX_Z);
            hold_share(a, q, :) = [d_ab, sum(k2i2(hm)), sum(k2i2)];
            fprintf('%4s %6d %+14.4e %+14.4e %+14.4e %10.3f\n', arms(a).tag, seeds(q), ...
                    d_ab, sum(k2i2(hm)), sum(k2i2), sum(k2i2(hm)) / d_ab);
        end
    end

    fprintf('\n================ CLAMPS / NaN ================\n');
    fprintf('%4s %6s %14s %14s %6s\n', 'arm', 'seed', 'frac@a_bar_flr', 'frac@a_bar_ceil', 'NaN');
    for a = 1:n_arms
        for q = 1:n_seeds
            fprintf('%4s %6d %14.4f %14.4f %6d\n', arms(a).tag, seeds(q), ...
                    sat(a, q, 1), sat(a, q, 2), sat(a, q, 3));
        end
    end

    % ------------------------------------------------------------------
    out = struct('arms', arms, 'seeds', seeds, 'res', {res}, 'M', M, 'G', G, ...
                 'sat', sat, 'hold_share', hold_share, ...
                 'metric_names', {mnames}, 'axis', AX_Z, 'windows', out_windows);

    out_dir = fullfile(project_root, 'test_results');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    out_file = fullfile(out_dir, 'formC_base_ablation.mat');
    save(out_file, 'out', '-v7.3');
    fprintf('\nsaved: %s\n', out_file);

    if opts.plot
        plot_formC_base_ablation(out);
    end
end


function v = local_field(s, f, dflt)
    if isfield(s, f) && ~isempty(s.(f)); v = s.(f); else; v = dflt; end
end
