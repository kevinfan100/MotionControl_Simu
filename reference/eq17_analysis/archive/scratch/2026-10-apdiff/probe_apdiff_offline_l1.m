% STATUS: SCRATCH -- 2026-10-16 (user, Step 0 of the a'-difference line): offline difference slope of the Layer-1 first-order
%   estimator (5state_noap production, run_mc_s4_first_order form 'prod_tri10': 22.2 -> 1.10 R in 1 s, then triangle 1.10 <-> 10 R
%   at 2 Hz for 2 s, lambda_c 0.4, epsilon 1e-5). No feedback: the estimator never sees the slope.
%   a'-diff[k] = (a_hat[k] - a_hat[k-N]) / (h_d[k] - h_d[k-N]) (raw, no gate), compared with the true slope a'(h[k]) by height band.
function probe_apdiff_offline_l1(seeds, NS)
    if nargin < 1 || isempty(seeds); seeds = 1:5; end
    if nargin < 2 || isempty(NS); NS = [1 10 50 100]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    addpath(genpath(fullfile(root, 'model')));
    WALLS = {'plane', 'cell'};  BANDS = [5 Inf; 2 5; 1 2];  BN = {'h > 5', '2-5', '< 2'};
    R = run_mc_s4_first_order({'prod_tri10'}, seeds, WALLS);  D = R.prod_tri10;
    od = fullfile(root, 'test_results', 'temp_1009_ws');  save(fullfile(od, 'apdiff_offline_l1.mat'), 'D', 'seeds', 'NS');
    for iw = 1:2
        fprintf('\n[%s] Layer-1 a_hat itself: |a_true - a_hat| descent %.3f, triangle %.3f\n', WALLS{iw}, ...
                mean(arrayfun(@(s) mean(abs(D.(WALLS{iw})(s).a_true(D.(WALLS{iw})(s).t <= 1) - D.(WALLS{iw})(s).a_hat(D.(WALLS{iw})(s).t <= 1))), seeds)), ...
                mean(arrayfun(@(s) mean(abs(D.(WALLS{iw})(s).a_true(D.(WALLS{iw})(s).t > 1) - D.(WALLS{iw})(s).a_hat(D.(WALLS{iw})(s).t > 1))), seeds)));
        fprintf('  N   | band   | median diff/true | %% negative | mean |ln ratio| (positive only) | corr\n');
        for N = NS
            for ib = 1:3
                rat = [];  ng = 0;  nt = 0;  xd = [];  xt = [];
                for s = seeds
                    r = D.(WALLS{iw})(s);  [apd, apt, h] = local_pair(r, N, WALLS{iw});
                    L = h > BANDS(ib, 1) & h <= BANDS(ib, 2) & isfinite(apd);
                    ng = ng + nnz(apd(L) <= 0);  nt = nt + nnz(L);  P = L & apd > 0;  rat = [rat; apd(P) ./ apt(P)]; %#ok<AGROW>
                    xd = [xd; apd(L)];  xt = [xt; apt(L)]; %#ok<AGROW>
                end
                cc = corrcoef(xd, xt);
                fprintf('  %3d | %-6s | %8.3f         | %5.1f      | %6.2f                         | %+.2f\n', N, BN{ib}, median(rat), 100 * ng / nt, mean(abs(log(rat))), cc(1, 2));
            end
        end
    end
end

function [apd, apt, h] = local_pair(r, N, wall)
    n = numel(r.t);  apd = NaN(n, 1);  hd = r.h_d;  h = max(r.h, 1.001);
    dh = hd(N+1:end) - hd(1:end-N);  da = r.a_hat(N+1:end) - r.a_hat(1:end-N);
    v = abs(dh) > 1e-6;  tmp = NaN(size(dh));  tmp(v) = da(v) ./ dh(v);  apd(N+1:end) = tmp;
    apt = local_ap(h, wall);
end

function ap = local_ap(h, wall)
    d = 1e-4;  ap = (local_a(h + d, wall) - local_a(h - d, wall)) / (2 * d);
end

function a = local_a(h, wall)
    if strcmp(wall, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
