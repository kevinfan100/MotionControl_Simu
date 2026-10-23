% STATUS: SCRATCH -- 2026-10-16 (user): constant-velocity descent only (22.2 -> 1.10 R in 1 s, 0.2 s hold, lambda_c 0.4; the
%   run_mc_apslot_compare 'desc' case), offline. Compares the slope a-bar' from
%     state : a' as a state (ap_slot, apP002F400H110, the best descent setting, file apcmp_apP002F400H110_<wall>_desc.mat);
%     diff N: Layer-1 first-order a_hat (5state_noap production, eps 1e-5, no slope anywhere), (a_hat[k] - a_hat[k-N]) / (h_d[k] - h_d[k-N]);
%     lpf N : same a_hat, ratio of low-passed increments  num = (1-beta) num + beta Delta a_hat,  den = (1-beta) den + beta Delta h_d,
%             a' = num / den, beta = 2/(N+1) (same mean age as the N window).
%   Both difference forms keep the last value when the denominator is too small (|h_d[k] - h_d[k-N]| <= 1e-3 R; |den| <= 1e-3/N R).
%   Scores over the descent (t <= 1 s) by band of the true height: median of slope/true, % negative, mean |ln(slope/true)| over the
%   positive values. Cache test_results/temp_1009_ws/apdiff_desc_l1_<wall>_s<seed>.mat.
function probe_apdiff_desc_l1(seeds, NS)
    if nargin < 1 || isempty(seeds); seeds = 1:5; end
    if nargin < 2 || isempty(NS); NS = [10 50 100]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    WALLS = {'plane', 'cell'};  BANDS = [5 Inf; 2 5; 1 2];  BN = {'h > 5', '2-5', '< 2'};
    for iw = 1:2
        A = load(fullfile(od, sprintf('apcmp_apP002F400H110_%s_desc.mat', WALLS{iw})));
        L1 = cell(1, max(seeds));
        for s = seeds
            fn = fullfile(od, sprintf('apdiff_desc_l1_%s_s%d.mat', WALLS{iw}, s));
            if exist(fn, 'file'); L1{s} = load(fn); continue; end
            cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
            ov = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * pc.R, 't_hold', 0, ...
                        't_descend_override', 1.0, 'T_sim', 1.2, 'lambda_c', 0.4);
            fo = fieldnames(ov);  for q = 1:numel(fo); cfg.(fo{q}) = ov.(fo{q}); end
            cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', 1e-5);
            o = struct('seed', s, 'collect_diag', true, 'ctrl_const_override', cc);
            if strcmp(WALLS{iw}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
            S = run_pure_simulation(cfg, o);
            r = struct('t', S.tout(:), 'a_true', S.a_true_out(:, 3) / a_nom, 'a_hat', S.diag.a_hat(:, 3) / a_nom, ...
                       'h_d', S.p_d_out(:, 3) / pc.R, 'h', S.p_true_out(:, 3) / pc.R);
            save(fn, '-struct', 'r');  L1{s} = r;
        end
        r1 = L1{seeds(1)};
        fprintf('\n[%s] |a_true - a_hat| over the descent: Layer-1 %.3f, a''-state run %.3f (seed mean)\n', WALLS{iw}, ...
                mean(arrayfun(@(s) mean(abs(L1{s}.a_true(L1{s}.t <= 1) - L1{s}.a_hat(L1{s}.t <= 1))), seeds)), ...
                mean(arrayfun(@(q) mean(abs(A.a_true(A.t <= 1, q) - A.a_hat(A.t <= 1, q))), 1:numel(seeds))));
        fprintf('  slope     | band   | median slope/true | %% negative | mean |ln ratio|\n');
        ARMS = [{'state'}, arrayfun(@(N) sprintf('diff %d', N), NS, 'UniformOutput', false), arrayfun(@(N) sprintf('lpf %d', N), NS, 'UniformOutput', false)];
        for ia = 1:numel(ARMS)
            for ib = 1:3
                rat = [];  ng = 0;  nt = 0;
                for iq = 1:numel(seeds)
                    s = seeds(iq);
                    if ia == 1; h = max(A.h_true(:, iq), 1.001);  ap = -A.w_hat(:, iq);  t = A.t;
                    else
                        r = L1{s};  h = max(r.h, 1.001);  t = r.t;  N = NS(mod(ia - 2, numel(NS)) + 1);
                        if ia <= 1 + numel(NS); ap = apdiff_window(r.a_hat, r.h_d, N); else; ap = apdiff_lpf(r.a_hat, r.h_d, N); end
                    end
                    apt = local_ap(h, WALLS{iw});
                    Lb = t <= 1 & h > BANDS(ib, 1) & h <= BANDS(ib, 2) & isfinite(ap);
                    ng = ng + nnz(ap(Lb) <= 0);  nt = nt + nnz(Lb);  P = Lb & ap > 0;  rat = [rat; ap(P) ./ apt(P)]; %#ok<AGROW>
                end
                fprintf('  %-9s | %-6s | %8.3f          | %5.1f      | %5.2f\n', ARMS{ia}, BN{ib}, median(rat), 100 * ng / nt, mean(abs(log(rat))));
            end
        end
    end
end

function ap = apdiff_window(a, hd, N)
    n = numel(a);  ap = NaN(n, 1);  s = NaN;
    for k = N+1:n
        dh = hd(k) - hd(k-N);
        if abs(dh) > 1e-3; s = (a(k) - a(k-N)) / dh; end
        ap(k) = s;
    end
end

function ap = apdiff_lpf(a, hd, N)
    n = numel(a);  ap = NaN(n, 1);  b = 2 / (N + 1);  num = 0;  den = 0;  s = NaN;
    for k = 2:n
        num = (1 - b) * num + b * (a(k) - a(k-1));  den = (1 - b) * den + b * (hd(k) - hd(k-1));
        if k > N && abs(den) > 1e-3 / N; s = num / den; end
        ap(k) = s;
    end
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
