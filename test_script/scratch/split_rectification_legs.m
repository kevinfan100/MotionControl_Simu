function out = split_rectification_legs()
%SPLIT_RECTIFICATION_LEGS  Step 0 of the remaining-bias plan (2026-08-30).
%   Under the b_true oracle + exact law step (law right, no quadrature), the
%   residual +13.8 % needs noise. Split each leg of the oscillation into a
%   MEAN part and a COVARIANCE part, across seeds at each step:
%       sum_k K1 nu     = sum_k K1bar nubar  +  sum_k (K1 - K1bar)(nu - nubar)   (+ cross terms, zero in expectation)
%       sum_k a' dw     = sum_k a'bar dwbar  +  sum_k (a' - a'bar)(dw - dwbar)
%   route A = mean part of y1 (E[nu] != 0), route B = cov part of y1 (K1 co-moves
%   with nu), route C = cov part of the law leg (a'(a_hat) co-moves with the
%   estimated displacement).  The deterministic arm (no noise) gives the
%   noise-free baseline of each mean part.
%   Reads test_results/btrue_log_ledger/check_btrue_log_ledger.mat (8 seeds).
% STATUS: ACTIVE

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    S = load(fullfile(root, 'test_results', 'btrue_log_ledger', 'check_btrue_log_ledger.mat'));
    out = struct();
    PAIRS = {'noisyExact', 'detExact'; 'noisyEuler', 'detEuler'};
    for p = 1:2
        N = S.out.(PAIRS{p,1});  D = S.out.(PAIRS{p,2});  ns = N.ns;
        m = N.tt >= 1.5 & N.tt < 3.5;                       % both oscillation cycles
        aT = mean(N.at(2:end, :), 'all') * 0 + mean(N.at(find(N.tt > 3.7) + 1, :), 'all');   % trough a_true for %
        % --- y1 leg -------------------------------------------------------
        K1 = N.K1(2:end, :);  nu = N.n1(2:end, :);
        K1b = mean(K1, 2);  nub = mean(nu, 2);
        y1_mean = sum(K1b(m) .* nub(m));                                   % route A (incl. deterministic part)
        y1_cov  = sum(mean((K1(m,:) - K1b(m)) .* (nu(m,:) - nub(m)), 2));  % route B
        y1_tot  = mean(sum(K1(m,:) .* nu(m,:), 1));
        y1_det  = sum(D.K1(find(m)+1) .* D.n1(find(m)+1));                 % noise-free baseline of the mean part
        % --- law leg ------------------------------------------------------
        ap = N.bh(1:end-1, :) .* (1 - N.ah(1:end-1, :)).^2;  dw = N.dw;
        apb = mean(ap, 2);  dwb = mean(dw, 2);
        law_mean = sum(apb(m) .* dwb(m));
        law_cov  = sum(mean((ap(m,:) - apb(m)) .* (dw(m,:) - dwb(m)), 2));  % route C
        law_tot  = mean(sum(N.law(m,:), 1));
        law_det  = sum(D.law(m));
        y2_tot   = mean(sum(N.y2(m,:), 1));  res_tot = mean(sum(N.res(m,:), 1));
        dtrue    = mean(sum(N.dat(m,:), 1));  dobs = mean(sum(N.dah(m,:), 1));
        % --- seed-group error bars on the two cov terms (4 groups of 2) --------
        G = 4;  gy = zeros(G,1);  gl = zeros(G,1);
        for g = 1:G
            sel = (1:ns) > (g-1)*ns/G & (1:ns) <= g*ns/G;
            gy(g) = sum(mean((K1(m,sel) - K1b(m)) .* (nu(m,sel) - nub(m)), 2));
            gl(g) = sum(mean((ap(m,sel) - apb(m)) .* (dw(m,sel) - dwb(m)), 2));
        end
        fprintf('\n=== %s vs %s, oscillation 1.5-3.5 s, a_o units (%% of trough %.4f) ===\n', PAIRS{p,1}, PAIRS{p,2}, aT);
        fprintf('  observed  d a_hat %+.4f   d a_true %+.4f   excess %+.4f  (%+.1f %%)\n', dobs, dtrue, dobs - dtrue, 100*(dobs-dtrue)/aT);
        fprintf('  y1  leg  total %+.4f = mean part %+.4f (det baseline %+.4f -> noise-added %+.4f)  +  cov part %+.4f +- %.4f   [route A | route B]\n', ...
                y1_tot, y1_mean, y1_det, y1_mean - y1_det, y1_cov, std(gy)/sqrt(G));
        fprintf('  law leg  total %+.4f = mean part %+.4f (det baseline %+.4f -> noise-added %+.4f)  +  cov part %+.4f +- %.4f   [       | route C]\n', ...
                law_tot, law_mean, law_det, law_mean - law_det, law_cov, std(gl)/sqrt(G));
        fprintf('  y2  leg  %+.4f    resid (exact-step 2nd order) %+.4f\n', y2_tot, res_tot);
        fprintf('  noise-added total = (y1 mean - det) %+.4f + y1 cov %+.4f + (law mean - det) %+.4f + law cov %+.4f + y2 %+.4f + resid-det %+.4f\n', ...
                y1_mean - y1_det, y1_cov, law_mean - law_det, law_cov, y2_tot, res_tot - sum(D.res(m)));
        out.(PAIRS{p,1}) = struct('y1_mean', y1_mean, 'y1_cov', y1_cov, 'y1_det', y1_det, ...
                                  'law_mean', law_mean, 'law_cov', law_cov, 'law_det', law_det, 'y2', y2_tot, 'res', res_tot);
    end
    fprintf('SPLIT DONE\n');
end
