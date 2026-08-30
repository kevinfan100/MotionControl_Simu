function [mu, se, th] = l0_jackknife_se(X)
%L0_JACKKNIFE_SE  Delete-one-seed jackknife mean and standard error.
%
%   [mu, se] = l0_jackknife_se(X)     X = n_seeds x m per-seed statistics
%   l0_jackknife_se()                 self-test against the closed form
%
% STATUS: ACTIVE (scratch, L0 instrument) | PURPOSE: the ONE error bar this
%   project uses -- the seed is the only resampling unit (each row of X is
%   one seed's whole-trajectory statistic, so its internal time correlation
%   stays inside the row and is never discounted by a tau or a sqrt(n_k))
%   | EXPIRES: never.
%
%   mu  1 x m   mean over seeds of each column
%   se  1 x m   jackknife SE:  se = sqrt((n-1)/n * sum_i (th_i - mean(th))^2)
%               with th_i = the column mean with seed i left out
%   th  n x m   the leave-one-out replicates (for compound statistics, pass
%               the per-seed CONTRIBUTIONS and compute the estimator on th)
%
%   For a plain mean the jackknife SE equals std(X, 0, 1) / sqrt(n) exactly;
%   the self-test checks that identity to 1e-12.
%
%   NaN rows (a seed that produced no value) are dropped column-wise.

    if nargin == 0
        rng(1);
        Xt = randn(17, 4) * diag([1 3 0.2 10]) + 5;
        [mu_t, se_t] = l0_jackknife_se(Xt);
        se_cf = std(Xt, 0, 1) / sqrt(size(Xt, 1));
        err = max(abs(se_t - se_cf) ./ se_cf);
        assert(err < 1e-12 && max(abs(mu_t - mean(Xt, 1))) < 1e-12, ...
               'l0_jackknife_se:selfTest', 'self-test FAILED: rel err %.3e', err);
        fprintf('[l0_jackknife_se] self-test PASS: jackknife SE == std/sqrt(n) to %.1e (n=%d, m=%d)\n', ...
                err, size(Xt, 1), size(Xt, 2));
        if nargout > 0; mu = mu_t; se = se_t; th = []; end
        return;
    end

    assert(ismatrix(X) && isnumeric(X), 'l0_jackknife_se:input', 'X must be an n_seeds x m numeric matrix');
    [n, m] = size(X);
    assert(n >= 2, 'l0_jackknife_se:n', 'need at least 2 seeds (rows), got %d', n);
    mu = nan(1, m);  se = nan(1, m);  th = nan(n, m);
    for j = 1:m
        x = X(:, j);
        ok = isfinite(x);
        nj = sum(ok);
        if nj < 2; continue; end
        x = x(ok);
        S = sum(x);
        thj = (S - x) / (nj - 1);              % leave-one-out means
        mu(j) = S / nj;
        se(j) = sqrt((nj - 1) / nj * sum((thj - mean(thj)).^2));
        th(ok, j) = thj;
    end
end
