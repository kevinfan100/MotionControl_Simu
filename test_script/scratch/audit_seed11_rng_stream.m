% STATUS: ACTIVE | seed-11 outlier adjudication (RNG draw vs arm sensitivity)
%   EXPIRES: when the seed-11 verdict is on record. Zero production changes:
%   production is called only through run_formB_ws(opts) / run_formC_state(opts).
% FORK OF nothing (new diagnostic) | PURPOSE: (1) verify the per-seed random
%   stream is generated and consumed unconditionally and identically across the
%   two arms, (2) characterise seed 11's realised draws against the other
%   seeds, (3) run BOTH arms on 8 seeds so the outlier can be attributed.
%   產線改動不會自動跟上
%
% STREAM MODEL BEING TESTED (read off the code, then checked bit-exactly):
%   local_run_once:  clear <controller> trajectory_generator calc_thermal_force
%                    rng(seed)
%                    calc_simulation_params  -> calc_ctrl_params    draws randi
%                                            -> calc_thermal_params draws randi
%                                               = params.thermal.seed
%   step k:          calc_thermal_force  (FIRST call: rng(thermal.seed), then
%                                         randn(3,1) every call)
%                    randn(3,1)          measurement noise
%   => after the first thermal call the global stream is exactly
%      rng(thermal.seed) followed by 6 normals per step, thermal first.
%      This script regenerates that stream and demands bit equality.

clear cd    % MATLAB cd-shadowing hang, recorded in memory

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

SEEDS8 = [7 11 23 42 101 777 27 31];
AX_NAME = {'x', 'y', 'z'};

% ======================================================================
% STEP 3/4: production driver, 8 seeds, default arm
% ======================================================================
fprintf('\n############ PRODUCTION run_formB_ws, 8 seeds ############\n');
outB = run_formB_ws(struct('seeds', SEEDS8));

fprintf('\n############ NEW ARM run_formC_state, 8 seeds ############\n');
outC = run_formC_state(struct('seeds', SEEDS8));

% ======================================================================
% STEP 1/2: stream audit on every seed (done on the production runs; the
% cross-arm identity check below proves the same draws reach the new arm)
% ======================================================================
fprintf('\n############ STREAM AUDIT ############\n');
ns = numel(SEEDS8);
Zall = cell(ns, 1);          % whitened draws, N x 6 [thermal xyz, meas xyz]
bit_ok = false(ns, 1);
cross_ok = false(ns, 1);
max_stream_err = zeros(ns, 1);
max_cross_err  = zeros(ns, 2);

for q = 1:ns
    sB = outB.runs{q};
    sC = outC.runs{q};
    Zall{q} = local_whiten(sB);
    Zc      = local_whiten(sC);

    % --- (a) bit-exact regeneration of the whole stream from thermal.seed ---
    N  = size(Zall{q}, 1);
    rng(sB.meta.params_value.thermal.seed);
    Zref = randn(6, N).';                 % column-major fill = draw order
    err  = max(abs(Zall{q} - Zref), [], 'all');
    max_stream_err(q) = err;
    bit_ok(q) = err < 1e-9;

    % --- (b) cross-arm identity: same seed must give the same draws ---
    e_th = max(abs(Zall{q}(:, 1:3) - Zc(:, 1:3)), [], 'all');
    e_me = max(abs(Zall{q}(:, 4:6) - Zc(:, 4:6)), [], 'all');
    max_cross_err(q, :) = [e_th, e_me];
    cross_ok(q) = (e_th < 1e-9) && (e_me < 1e-9);

    fprintf('seed %4d: thermal.seed %10d  N %5d | stream bit-exact %d (err %.2e) | cross-arm identical %d (%.2e / %.2e)\n', ...
            SEEDS8(q), sB.meta.params_value.thermal.seed, N, bit_ok(q), err, ...
            cross_ok(q), e_th, e_me);
end

% ======================================================================
% Per-seed noise statistics on the whitened draws
% ======================================================================
CH = {'th_x', 'th_y', 'th_z', 'me_x', 'me_y', 'me_z'};
N  = size(Zall{1}, 1);
fprintf('\n--- whitened draws: mean (x sqrt(N)) and std (should be N(0,1) / 1) ---\n');
fprintf('%6s |', 'seed');  fprintf(' %14s', CH{:});  fprintf('\n');
Mz = zeros(ns, 6);  Sz = zeros(ns, 6);
for q = 1:ns
    Z = Zall{q};
    Mz(q, :) = mean(Z, 1) * sqrt(N);                 % z-score of the mean
    Sz(q, :) = std(Z, 0, 1);
    fprintf('%6d |', SEEDS8(q));
    for c = 1:6; fprintf('  %+6.2f/%5.3f', Mz(q, c), Sz(q, c)); end
    fprintf('\n');
end

fprintf('\n--- autocorrelation z-scores (sqrt(N)*rho) at lags 1,2,3 ---\n');
fprintf('%6s |', 'seed');  fprintf(' %20s', CH{:});  fprintf('\n');
Az = zeros(ns, 6, 3);
for q = 1:ns
    Z = Zall{q};
    fprintf('%6d |', SEEDS8(q));
    for c = 1:6
        z = Z(:, c) - mean(Z(:, c));
        v = sum(z.^2);
        for L = 1:3
            Az(q, c, L) = sqrt(N) * sum(z(1:end-L) .* z(1+L:end)) / v;
        end
        fprintf('  %+5.2f %+5.2f %+5.2f', Az(q, c, 1), Az(q, c, 2), Az(q, c, 3));
    end
    fprintf('\n');
end

fprintf('\n--- extremes and window means (E[max|z|] ~ %.2f for N = %d) ---\n', ...
        sqrt(2*log(2*N)), N);
fprintf('%6s | %8s | %s\n', 'seed', 'max|z|', 'hold-window mean z-score per channel (t > 3.8 s)');
tB = outB.runs{1}.tout;
w_hold = tB > 3.8;   nh = sum(w_hold);
w_osc  = tB > 1.6 & tB <= 3.5;  no = sum(w_osc);
Hz = zeros(ns, 6);  Oz = zeros(ns, 6);
for q = 1:ns
    Z = Zall{q};
    Hz(q, :) = mean(Z(w_hold, :), 1) * sqrt(nh);
    Oz(q, :) = mean(Z(w_osc,  :), 1) * sqrt(no);
    fprintf('%6d | %8.2f |', SEEDS8(q), max(abs(Z), [], 'all'));
    fprintf(' %+6.2f', Hz(q, :));  fprintf('\n');
end
fprintf('%6s | %8s |', '', 'osc win');
fprintf('\n');
for q = 1:ns
    fprintf('%6d | %8s |', SEEDS8(q), '');
    fprintf(' %+6.2f', Oz(q, :));  fprintf('\n');
end

% ======================================================================
% Side-by-side metric table
% ======================================================================
fprintf('\n############ ARM COMPARISON (z axis) ############\n');
fprintf('%6s | %19s | %19s | %19s\n', 'seed', 'desc pk %  (B / C)', 'osc RMS %  (B / C)', 'hold mean % (B / C)');
for q = 1:ns
    fprintf('%6d | %8.2f %8.2f  | %8.2f %8.2f  | %+8.2f %+8.2f\n', SEEDS8(q), ...
            outB.metrics.rows(q, 1), outC.metrics.rows(q, 1), ...
            outB.metrics.rows(q, 2), outC.metrics.rows(q, 2), ...
            outB.metrics.rows(q, 3), outC.metrics.rows(q, 3));
end

save(fullfile(proj, 'test_results', 'temp_audit_seed11_rng_stream.mat'), ...
     'SEEDS8', 'Mz', 'Sz', 'Az', 'Hz', 'Oz', 'bit_ok', 'cross_ok', ...
     'max_stream_err', 'max_cross_err', '-v7.3');
fprintf('\nsaved: test_results/temp_audit_seed11_rng_stream.mat\n');


% ---------------------------------------------------------------------
function Z = local_whiten(s)
%LOCAL_WHITEN  Recover the underlying standard-normal draws from one run.
%   Columns 1:3 = thermal (whitened by the position-dependent sqrt(Variance)
%   calc_thermal_force used at that step), 4:6 = measurement noise.
    P  = s.meta.params_value;
    cfg = s.meta.config;
    N  = size(s.F_th_out, 1);

    % Position the thermal draw at step k saw = position BEFORE step k.
    p_pre = [P.common.p0.'; s.p_true_out(1:end-1, :)];
    h_bar = (p_pre * P.wall.w_hat - P.wall.pz) / P.common.R;   % NOT clamped,
                                                               % same as calc_thermal_force
    c_para = zeros(N, 1);  c_perp = zeros(N, 1);
    for k = 1:N
        [c_para(k), c_perp(k)] = calc_correction_functions(h_bar(k));
    end
    if P.wall.enable_wall_effect > 0.5
        % same C as calc_thermal_force, in the wall frame's world components
        C = abs(c_para * (P.wall.u_hat + P.wall.v_hat).' + c_perp * P.wall.w_hat.');
    else
        C = abs(repmat((P.wall.u_hat + P.wall.v_hat + P.wall.w_hat).', N, 1));
    end
    sd_th = sqrt(P.thermal.variance_coeff * C);

    Z = zeros(N, 6);
    Z(:, 1:3) = s.F_th_out ./ sd_th;
    Z(:, 4:6) = (s.p_m_out - s.p_true_out) ./ cfg.meas_noise_std(:).';
end
