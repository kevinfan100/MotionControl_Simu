% FORK OF test_script/scratch/run_formC_dist_meng_pair.m @ 10e51db | PURPOSE:
%   on the Meng 10 s monotone ramp, run formC_b arm 'best' as a PAIRED
%   comparison of two (a_pd, a_cov) settings, and measure whether shrinking
%   var(a_m) buys the filter any y2 weight | EXPIRES: when the y2-weight
%   question is adjudicated | production changes do NOT follow.
%
% THE QUESTION (Kevin, 08-25): can (a_pd, a_cov) be chosen so var(a_m) drops
%   and the KF therefore leans harder on y2?
%
% THE STRUCTURAL OBJECTION, registered BEFORE the run. Production whitens:
%       y2 = a_bar_wm[k] - (1-a_cov)*a_bar_wm[k-1] = a_cov*u[k],  H2 ~ a_cov
%       R2 = a_cov*(2-a_cov) * K_var * amlpf*IF*(a_bar+xi)^2 ,  K_var = 2a_cov/(2-a_cov)
%          = 2*a_cov^2 * amlpf*IF*(a_bar+xi)^2
%   so H2^2/R2 has a_cov cancel EXACTLY. Lowering a_cov lowers var(a_m)
%   (~K_var ~ a_cov) and lowers var(y2) (~a_cov^2) but should NOT move the
%   weight y2 carries. The one residual a_cov dependence is IF_eff, whose
%   geometric sums run on s = 1-a_cov, so smaller a_cov means LARGER IF,
%   i.e. slightly LESS y2 weight -- the opposite of the intuition.
%
% PRE-REGISTERED PREDICTION (arm B vs arm A):
%   sd(a_m)/a_bar   41.7 % -> ~22 %        (falls, as intended)
%   var(y2)         falls ~16x             (a_cov^2)
%   w_y2 share      FLAT, or slightly down (the objection)
%   e_a metrics     FLAT
%   Prediction wrong -> the cancellation argument is wrong and the sweep is
%   worth running in full. Prediction right -> the lever is amlpf/IF, not a_cov.
%
% LIVENESS FIRST (project rule 13, three dead flags found on 08-24):
%   read the a_pd/a_cov echo, the y2 gate duty, the a_bar clamp fraction and
%   the NaN count BEFORE any bias or RMS number.
function out = pair_apd_acov_meng(seeds, armB)
%PAIR_APD_ACOV_MENG  One paired (a_pd, a_cov) test on the Meng 10 s ramp.
%
%   out = pair_apd_acov_meng()                          % default vs a_cov x0.25
%   out = pair_apd_acov_meng(1:12)                      % more seeds
%   out = pair_apd_acov_meng([], struct('a_pd',0.2))    % probe a_pd instead

    if nargin < 1 || isempty(seeds); seeds = [7 11 23 42 101 777]; end
    if nargin < 2 || isempty(armB);  armB  = struct('a_pd', 0.05, 'a_cov', 0.0125); end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(fullfile(root, 'test_script', 'integration'));

    AX = 3;                                   % z, the wall-normal axis
    A_COV_BASE = 0.05;                        % run_formC_b's base, mirrored
    armA = struct('a_pd', 0.05, 'a_cov', A_COV_BASE);

    ov = local_meng_override();
    ARMS = {armA, armB};
    O = cell(1, 2);
    for c = 1:2
        ovc = ov;  ovc.a_pd = ARMS{c}.a_pd;
        clear run_formC_b motion_control_law_formC_b;
        fprintf('\n######## arm %c : a_pd = %g , a_cov = %g ########\n', ...
                'A' + c - 1, ARMS{c}.a_pd, ARMS{c}.a_cov);
        O{c} = run_formC_b(struct('arm', 'best', 'seeds', seeds, ...
                                  'a_cov_scale', ARMS{c}.a_cov / A_COV_BASE, ...
                                  'config_override', ovc));
    end

    % ---------------- LIVENESS (read this before anything else) ----------
    fprintf('\n=============== LIVENESS (read first) ===============\n');
    fprintf('%-4s %8s %8s | %8s %9s %9s %8s %7s\n', 'arm', 'a_pd', 'a_cov', ...
            'y2gate', 'aBarClmp', 'min aBar', 'hbClamp', 'NaN');
    for c = 1:2
        cc = O{c}.runs{1}.ctrl_const;
        g = 0; cl = 0; hb = 0; nn = 0; mab = Inf; nt = 0;
        for q = 1:numel(seeds)
            r  = O{c}.runs{q};
            fl = local_field(r.ctrl_const, 'a_bar_floor', 0.05);
            ce = local_field(r.ctrl_const, 'a_bar_ceil',  1 - 1e-4);
            ab = r.a_bar_hat_out(2:end, AX);
            hbr = r.p_true_out(2:end, AX) / r.R;
            hfl = r.meta.params_value.wall.h_bar_min;
            g  = g  + mean(r.gate_out(2:end, AX));
            cl = cl + mean(ab <= fl + 1e-12 | ab >= ce - 1e-12);
            hb = hb + mean(hbr <= hfl + 1e-12);
            nn = nn + sum(~isfinite(ab));
            mab = min(mab, min(ab));  nt = nt + numel(ab);
        end
        ns = numel(seeds);
        fprintf('%-4c %8.4f %8.4f | %8.4f %9.4f %9.4f %8.4f %7d\n', ...
                'A' + c - 1, cc.a_pd, cc.a_cov, g/ns, cl/ns, mab, hb/ns, nn);
    end
    fprintf('  expected: a_pd/a_cov echo the setting; y2gate = 0 everywhere\n');
    fprintf('  (gate_out = 1 means y2 SUPPRESSED; h_bar_safe = %.2f < Meng trough\n', ...
            local_field(O{1}.runs{1}.ctrl_const, 'h_bar_safe', NaN));
    fprintf('   h_bar 1.111, so y2 is live for the WHOLE run.)\n');
    fprintf('  NaN = 0. Anything else invalidates the numbers below.\n');

    % ---------------- METRICS -------------------------------------------
    M = cell(1, 2);
    for c = 1:2
        M{c} = local_metrics(O{c}, AX, ARMS{c}.a_cov, seeds);
    end
    local_report(M, ARMS, seeds);

    out = struct('O', {O}, 'M', {M}, 'ARMS', {ARMS}, 'seeds', seeds, ...
                 'ov', ov, 'AX', AX);
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if ~exist(od, 'dir'); mkdir(od); end
    % Arm-specific name: a bare 'pair_apd_acov_meng.mat' let the second call
    % of the session silently overwrite the first arm's result.
    fn = sprintf('pair_apd%g_acov%g.mat', armB.a_pd, armB.a_cov);
    save(fullfile(od, fn), 'out', '-v7.3');
    fprintf('\nsaved %s\n', fullfile(od, fn));
end

% =====================================================================
function ov = local_meng_override()
%LOCAL_MENG_OVERRIDE  Fei Long dissertation 4.3.2 (Meng-group experiment):
%   z ramps from h = 15 um (h_bar 6.667) down to 2.5 um (h_bar 1.111) over
%   10 s, NO wall-normal oscillation. Copied verbatim from
%   run_formC_dist_meng_pair.m's 'meng' branch @ 10e51db; the phase structure
%   is kept so every metric window keeps the canonical definition, and
%   amplitude = 0 collapses phase 3 to a flat 1 s at the trough.
    pc = physical_constants();
    ov = struct();
    ov.trajectory_type = 'osc';
    ov.h_init    = 15.0;            % [um] h_bar 6.6667 (R = 2.25)
    ov.h_bottom  = 2.5;             % [um] h_bar 1.1111
    ov.amplitude = 0;               % [um] no oscillation
    ov.frequency = 1;               % [Hz] only sets the phase-3 length
    ov.n_cycles  = 1;               % flat 1 s at the trough
    ov.t_hold    = 0.5;             % [s] initial hold
    ov.t_descend_override = 10.0;   % [s] the 12.5 um ramp
    ov.T_sim     = 12.5;            % [s] leaves a 1.0 s final hold
    ov.h_min     = 1.1 * pc.R;      % [um] truth-curve validity floor
end

% ---------------------------------------------------------------------
function m = local_metrics(O, ax, a_cov, seeds)
%LOCAL_METRICS  Per-seed, per-segment. The y2-weight quantities come first
%   because they are the judgment; var(a_m) is recorded, not judged.
    ns = numel(seeds);
    r0 = O.runs{1};
    t  = r0.tout(2:end);
    hb = r0.h_bar_d_out(2:end);               % commanded height (Nx1), seed-independent
    % Segments: initial hold / far descent / near descent / trough+final hold
    seg = struct('name', {'hold0', 'far', 'near', 'trough'}, ...
                 'idx',  {t < 0.5, ...
                          t >= 0.5 & hb > 3.0, ...
                          t >= 0.5 & hb <= 3.0 & t < 10.5, ...
                          t >= 10.5});
    nseg = numel(seg);
    Z = @() zeros(ns, nseg);
    m = struct('t', t, 'hb', hb, 'seg', seg, 'a_cov', a_cov, ...
               'sd_am_rel', Z(), 'var_y2', Z(), 'R2', Z(), 'ratio_y2R2', Z(), ...
               'w_y2', Z(), 'w_y2_a', Z(), 'info_y2', Z(), 'e_a', Z(), 'am_bias', Z());
    m.b_end = zeros(1, ns);  m.e_rms_all = zeros(1, ns);
    for q = 1:ns
        r  = O.runs{q};
        ad = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);   % um/pN per unit a_bar
        aM = r.a_xm_out(2:end, ax) / ad;                    % a_bar_wm (pre-whitening)
        aH = r.a_bar_hat_out(2:end, ax);
        aT = r.a_true_out(2:end, ax) / ad;
        % whitened increment, reconstructed exactly as the controller forms it
        y2 = aM - (1 - a_cov) * [aM(1); aM(1:end-1)];
        R2 = r.R2_out(2:end, ax);
        % SHARE must be built from the CORRECTIONS, not the gains: K(.,y1) has
        % units [state per length] and K(.,y2) [state per gain unit], so
        % |K1| + |K2| is dimensionally meaningless. K*innovation is in state
        % units for both legs, and is literally "how much of this step's
        % update came from that channel".
        i1 = r.innov_y1_out(2:end, ax);
        i2 = r.innov_y2_out(2:end, ax);
        K1 = abs(r.K_b_y1_out(2:end, ax) .* i1);      % b     <- y1 correction
        K2 = abs(r.K_b_y2_out(2:end, ax) .* i2);      % b     <- y2 correction
        A1 = abs(r.K_a_y1_out(2:end, ax) .* i1);      % a_bar <- y1 correction
        A2 = abs(r.K_a_y2_out(2:end, ax) .* i2);      % a_bar <- y2 correction
        ea = 100 * (aH - aT) ./ aT;
        m.e_rms_all(q) = sqrt(mean(ea(m.seg(2).idx | m.seg(3).idx).^2));
        m.b_end(q)     = r.b_hat_out(end, ax);
        for s = 1:nseg
            i = seg(s).idx;
            m.sd_am_rel(q, s)  = std(aM(i)) / mean(aT(i));
            m.var_y2(q, s)     = var(y2(i));
            m.R2(q, s)         = median(R2(i));
            m.ratio_y2R2(q, s) = var(y2(i)) / median(R2(i));
            m.w_y2(q, s)       = median(K2(i) ./ max(K1(i) + K2(i), realmin));
            m.w_y2_a(q, s)     = median(A2(i) ./ max(A1(i) + A2(i), realmin));
            m.info_y2(q, s)    = a_cov^2 / median(R2(i));     % H2^2/R2, H2 ~ a_cov
            m.e_a(q, s)        = mean(ea(i));
            m.am_bias(q, s)    = 100 * mean((aM(i) - aT(i)) ./ aT(i));
        end
    end
end

% ---------------------------------------------------------------------
function local_report(M, ARMS, seeds)
    ns = numel(seeds);
    names = {M{1}.seg.name};
    rows = {'sd_am_rel',  'sd(a_m)/a_true      ', '%9.4f'; ...
            'var_y2',     'var(y2) measured    ', '%9.3e'; ...
            'R2',         'R2 (median)         ', '%9.3e'; ...
            'ratio_y2R2', 'var(y2)/R2   <- 3.3x', '%9.4f'; ...
            'w_y2',       'y2 SHARE of b update', '%9.4f'; ...
            'w_y2_a',     'y2 SHARE of a update', '%9.4f'; ...
            'info_y2',    'H2^2/R2 (info rate) ', '%9.3e'; ...
            'am_bias',    'a_m bias [%]        ', '%9.3f'; ...
            'e_a',        'a-hat error [%]     ', '%9.3f'};
    fprintf('\n=============== PAIRED METRICS (z axis, %d seeds) ===============\n', ns);
    fprintf('  arm A: a_pd %g  a_cov %g   |   arm B: a_pd %g  a_cov %g\n', ...
            ARMS{1}.a_pd, ARMS{1}.a_cov, ARMS{2}.a_pd, ARMS{2}.a_cov);
    for k = 1:size(rows, 1)
        f = rows{k, 1};
        fprintf('\n  %s\n', rows{k, 2});
        fprintf('  %-6s %11s %11s %11s   %s\n', 'seg', 'arm A', 'arm B', 'B/A', 'paired B-A (sem)');
        for s = 1:numel(names)
            a = M{1}.(f)(:, s);  b = M{2}.(f)(:, s);
            d = b - a;
            fprintf('  %-6s ', names{s});
            fprintf([rows{k,3} ' '], mean(a));
            fprintf([rows{k,3} ' '], mean(b));
            fprintf('%11.4f   %+.4g (%.2g)\n', mean(b)/mean(a), mean(d), std(d)/sqrt(ns));
        end
    end
    fprintf('\n  a-hat error RMS over the ramp [%%] : A %.3f   B %.3f   paired B-A %+.3f (sem %.3f)\n', ...
            mean(M{1}.e_rms_all), mean(M{2}.e_rms_all), ...
            mean(M{2}.e_rms_all - M{1}.e_rms_all), ...
            std(M{2}.e_rms_all - M{1}.e_rms_all)/sqrt(ns));
    fprintf('  b_hat[end]                        : A %.5f   B %.5f\n', ...
            mean(M{1}.b_end), mean(M{2}.b_end));
    fprintf('\n  READ: "y2 SHARE" flat across arms => the a_cov cancellation holds,\n');
    fprintf('        var(a_m) is not the lever. Moving => the objection is wrong.\n');
end

% ---------------------------------------------------------------------
function v = local_field(s, f, d)
    if isstruct(s) && isfield(s, f) && ~isempty(s.(f)); v = s.(f); else; v = d; end
end
