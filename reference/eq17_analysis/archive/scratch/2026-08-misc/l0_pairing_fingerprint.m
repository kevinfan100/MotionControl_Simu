% L0_PAIRING_FINGERPRINT  Off-by-one delta pairing check on one canonical seed.
%
% STATUS: ACTIVE (scratch, L0 instrument) | PURPOSE: prove which pairing of
%   command and position the logs support before any tracking-error number
%   enters a criterion. A past driver paired p_d[k] with the POST-update
%   p[k+1] and manufactured a quadrature 1 Hz line whose amplitude was the
%   per-step command increment and whose phase sat at -90 deg relative to
%   the command on every seed. | EXPIRES: never (re-run after any driver
%   change to the log ordering).
%
% LOG-INDEX SEMANTICS (run_formC_b local_run_once, read from the code):
%   p_d_out(k)    = pd_k, the command handed to the controller at t_k
%   p_true_out(k) = p_curr AFTER step_dynamics of step k  = p(t_{k+1})
%   p_m_out(k)    = p_true_out(k) + noise
%   dh_m_out(k)   = R * delta_w_m = pd_km2 - p_m_delayed
%                 = p_d_out(k-2) - p_m_out(k-3)          (d = 2)
%   so in the lead's notation p[k+1] == p_true_out(k) and
%     ALIGNED  d_al[k] = p_d[k+1] - p[k+1] = p_d_out(k+1) - p_true_out(k)
%     SKEWED   d_sk[k] = p_d[k]   - p[k+1] = p_d_out(k)   - p_true_out(k)
%     d_sk = d_al - Delta_d[k],  Delta_d[k] = p_d_out(k+1) - p_d_out(k)
%   Both are stamped at the position's own time t(k+1).
%
% OUTPUT: table on the console and in
%   /Users/kevin/.claude/jobs/8581427c/tmp/l0a/pairing_fingerprint.txt
%
% NOTE: the multi-seed driver form writes test_results/run_formC_b_best_y2on.mat
%   (gitignored) as a side effect. It is used here because the single-seed
%   test-ladder form bypasses the arm overrides (b_init 8/9, lock_b, floors,
%   parallel package) and would not be the canonical estimator.

cd('/Users/kevin/code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));
clear run_formC_b motion_control_law_formC_b trajectory_generator calc_thermal_force;

SEED      = 7;
AXL       = 'xyz';
LAG_SCAN  = -4:4;          % pairing-lag scan of delta against the command increment
DH_SCAN   = 0:6;           % lag scan of the driver's own dh_m against the two pairings
PASS_AMP_NM   = 5;         % aligned z 1 Hz line must be below this
LOCK_PHASE_DEG = -90;      % the skew signature (relative to the command)
LOCK_TOL_DEG   = 15;
OUT_TXT = '/Users/kevin/.claude/jobs/8581427c/tmp/l0a/pairing_fingerprint.txt';

out = run_formC_b(struct('seeds', SEED, 'verbose', false));
s   = out.runs{1};
cfg = out.cfg;
f   = cfg.frequency;
w   = out.metrics.windows;       % driver's own windows (osc = [t2 + 0.1, t3])

t   = s.tout(:);
Ts  = t(2) - t(1);
N   = numel(t);
pd  = s.p_d_out;                 % N x 3 [um]
p   = s.p_true_out;
pm  = s.p_m_out;
dh  = s.dh_m_out;

k   = (1:N-1).';
d_al = pd(k+1, :) - p(k, :);     % aligned
d_sk = pd(k,   :) - p(k, :);     % skewed
Dd   = pd(k+1, :) - pd(k, :);    % command increment handed to the controller at step k
t_pos = t(k+1);                  % time of p_true_out(k)

% the driver's own delta: identity check first (exact, by construction)
kk = (4:N).';
dh_rebuilt = pd(kk-2, :) - pm(kk-3, :);
dh_ident_max = max(abs(dh(kk, :) - dh_rebuilt), [], 1);

in = t_pos > w.osc(1) & t_pos <= w.osc(2);
tw = t_pos(in);

fid = fopen(OUT_TXT, 'w');
F = [1 fid];
pr = @(varargin) local_pr(F, varargin{:});

pr('L0 PAIRING FINGERPRINT | seed %d | arm %s | band trough w_bar %.2f | Ts %.6g s | f %g Hz\n', ...
   SEED, out.arm_tag, cfg.h_bottom / s.R, Ts, f);
pr('window (driver osc): t in (%.2f, %.2f] s, %d samples\n', w.osc(1), w.osc(2), sum(in));
pr('per-step command increment at 1 Hz (analytic A*2*pi*f*Ts): %.2f nm; peak descent step %.1f nm\n', ...
   1e3 * cfg.amplitude * 2 * pi * f * Ts, 1e3 * max(abs(Dd(:, 3))));
pr('dh_m_out identity  dh_m(k) == p_d_out(k-2) - p_m_out(k-3):  max|diff| x/y/z = %.3g / %.3g / %.3g um\n', ...
   dh_ident_max);

% command 1 Hz phasor (z only; x/y commands are constant)
[~, ph_cmd, ~] = local_phasor(tw, pd(k(in)+1, 3), f);
pr('command z 1 Hz phasor phase re sin(2*pi*f*t): %+7.2f deg (trajectory is h_bottom + A - A cos(2*pi*f*(t-t2)))\n', ph_cmd);

pr('\n%-8s %-4s | %9s %11s %11s | %9s %9s | %5s %8s %8s\n', 'pairing', 'ax', ...
   'amp1Hz nm', 'ph re sin', 'ph re cmd', 'mean nm', 'std nm', 'lag*', 'corr*', 'slope*');
res = struct();
for pp = 1:2
    if pp == 1; D = d_al; nm = 'ALIGNED'; else; D = d_sk; nm = 'SKEWED'; end
    for ax = 1:3
        x = D(in, ax);
        [amp, ph, ~] = local_phasor(tw, x, f);
        ph_rel = local_wrap(ph - ph_cmd);
        mu = mean(x); sd = std(x);
        % pairing lag: regress delta[k] on Delta_d[k+L]
        cr = zeros(size(LAG_SCAN)); sl = zeros(size(LAG_SCAN));
        for j = 1:numel(LAG_SCAN)
            L = LAG_SCAN(j);
            idx = find(in);
            idx = idx(idx + L >= 1 & idx + L <= N-1);
            a = D(idx, ax) - mean(D(idx, ax));
            b = Dd(idx + L, ax) - mean(Dd(idx + L, ax));
            if all(b == 0); cr(j) = 0; sl(j) = 0; continue; end
            cr(j) = (a.' * b) / sqrt((a.' * a) * (b.' * b));
            sl(j) = (a.' * b) / (b.' * b);
        end
        [~, jm] = max(abs(cr));
        pr('%-8s %-4s | %9.3f %+11.2f %+11.2f | %+9.3f %9.3f | %+5d %+8.4f %+8.4f\n', ...
           nm, AXL(ax), 1e3 * amp, ph, ph_rel, 1e3 * mu, 1e3 * sd, LAG_SCAN(jm), cr(jm), sl(jm));
        res(pp, ax).amp_nm = 1e3 * amp; res(pp, ax).ph = ph; res(pp, ax).ph_rel = ph_rel;
        res(pp, ax).mean_nm = 1e3 * mu; res(pp, ax).std_nm = 1e3 * sd;
        res(pp, ax).lag = LAG_SCAN(jm); res(pp, ax).corr = cr(jm); res(pp, ax).slope = sl(jm);
        res(pp, ax).lag_scan = [LAG_SCAN; cr; sl];
    end
end
pr('(x/y commands are constant, so "ph re cmd" on x/y is relative to the z command; lag* = argmax |corr| of delta vs Delta_d[k+lag])\n');
% The lag scan cannot resolve the pairing by itself: the increment is a
% smooth 1 Hz sinusoid, so a 4-step shift moves its phase by only 0.9 deg
% and the correlation is flat within noise. The DISCRIMINANT is the lag-0
% regression slope: d_sk = d_al - Delta_d exactly, so slope(0) ~ -1 for the
% skewed pairing and ~ 0 (the loop's response to the increment) for the
% aligned one. Full z scan printed so the flatness is visible.
pr('\nz lag scan (lag : corr / slope), ALIGNED then SKEWED:\n');
for pp = 1:2
    sc = res(pp, 3).lag_scan;
    if pp == 1; pr('  ALIGNED'); else; pr('  SKEWED '); end
    for j = 1:size(sc, 2); pr('  %+d:%+.4f/%+.4f', sc(1, j), sc(2, j), sc(3, j)); end
    pr('\n');
end
j0 = find(LAG_SCAN == 0);
pr('lag-0 slope z: ALIGNED %+.4f  SKEWED %+.4f   (skewed pairing predicts -1 exactly on the increment)\n', ...
   res(1, 3).lag_scan(3, j0), res(2, 3).lag_scan(3, j0));

% which pairing does the driver's own logged delta match?
pr('\ndriver dh_m_out vs the two pairings (z), corr of dh_m[k] with d[k-L]:\n');
pr('%5s %12s %12s\n', 'L', 'ALIGNED', 'SKEWED');
best = [0 -1; 0 -1];
for L = DH_SCAN
    idx = find(in); idx = idx(idx - L >= 1);
    a = dh(idx + 1, 3);            % dh_m_out row k+1 carries the same time stamp t(k+1) as d[k]
    c_al = corr(a, d_al(idx - L, 3));
    c_sk = corr(a, d_sk(idx - L, 3));
    pr('%5d %12.5f %12.5f\n', L, c_al, c_sk);
    if abs(c_al) > best(1, 2); best(1, :) = [L, abs(c_al)]; end
    if abs(c_sk) > best(2, 2); best(2, :) = [L, abs(c_sk)]; end
end
if best(1, 2) >= best(2, 2)
    match = sprintf('ALIGNED at L = %d (corr %.5f)', best(1, 1), best(1, 2));
else
    match = sprintf('SKEWED at L = %d (corr %.5f)', best(2, 1), best(2, 2));
end
pr('driver dh_m_out matches: %s  [expected ALIGNED at L = 2 (d = 2 delay; both stamped at the position time), residual = 3.3 nm meas noise]\n', match);

% verdict
amp_z = res(1, 3).amp_nm;  ph_z = res(1, 3).ph_rel;
locked = abs(local_wrap(ph_z - LOCK_PHASE_DEG)) < LOCK_TOL_DEG;
if amp_z < PASS_AMP_NM && ~locked
    verdict = 'PASS';
else
    verdict = 'FAIL';
end
pr('\nVERDICT %s: aligned z 1 Hz line %.3f nm (limit %g), phase re command %+.1f deg (lock signature %+d +- %d deg: %s)\n', ...
   verdict, amp_z, PASS_AMP_NM, ph_z, LOCK_PHASE_DEG, LOCK_TOL_DEG, local_yn(locked));
pr('skewed z for reference: %.3f nm at %+.1f deg re command (analytic skew line %.2f nm at -90 deg)\n', ...
   res(2, 3).amp_nm, res(2, 3).ph_rel, 1e3 * cfg.amplitude * 2 * pi * f * Ts);
fclose(fid);
fprintf('saved: %s\n', OUT_TXT);

% ----------------------------------------------------------------------
function [amp, ph_deg, c] = local_phasor(t, x, f)
%LOCAL_PHASOR  LS fit x ~ c0 + c1 (t - mean t) + A sin(2 pi f t) + B cos(2 pi f t).
%   amp = sqrt(A^2 + B^2); phase such that x ~ amp * sin(2 pi f t + ph).
    t = t(:); x = x(:);
    M = [ones(size(t)), t - mean(t), sin(2*pi*f*t), cos(2*pi*f*t)];
    c = M \ x;
    A = c(3); B = c(4);
    amp = hypot(A, B);
    ph_deg = atan2d(B, A);
end

function y = local_wrap(x)
    y = mod(x + 180, 360) - 180;
end

function local_pr(F, varargin)
    for i = 1:numel(F); fprintf(F(i), varargin{:}); end
end

function s = local_yn(b)
    if b; s = 'PRESENT'; else; s = 'absent'; end
end
