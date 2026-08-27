function out = check_hold_y1_gain(S, label)
%CHECK_HOLD_Y1_GAIN  In a hold, is the filter's y1 -> a_bar gain K1(4) the gain
%   the data actually support?
%
%   L = load('test_results/formC_cdpmr_var_check/raw_seeds_budget.mat'); S = L.(fieldnames(L){1});
%   out = check_hold_y1_gain(S, 'production');
%
% STATUS: ACTIVE | discriminator for the hold-segment spread (level-2 verdict 2026-08-27)
%
% THE QUESTION. With v_d = 0 the position measurement carries no information
% about a gain ERROR, yet production uses K1(4) = -0.49 there (the Q34 route:
% "a thermal kick moved the particle, so the gain moved with it"). That is a
% legitimate tracking of a_true's own thermal wander IF the correlation the
% filter assumes exists in the data. A Kalman gain is, by definition,
%       K_opt[k] = Cov(e[k], nu[k]) / Var(nu[k])
% with e = a_true - a_hat (prior) and nu the innovation. Both are in the log,
% so K_opt is MEASURABLE across seeds at every hold step and can be compared
% with the K1(4) the filter used. Reading:
%   K_opt ~ K1(4) (-0.49)  => the y1 leg is tracking real gain wander; the hold
%                             spread is the honest price of that tracking
%   K_opt ~ 0               => the y1 leg is injecting position noise into a_hat
%                             with a gain the data do not support
%   K_opt of opposite sign  => the Q34 attribution is inverted in the hold
% Also reported: how much of the true gain wander the y1 leg explains
% (regression of the per-block a_true change on the per-block y1 leg).

    if nargin < 2; label = ''; end
    ax = 3;  ns = numel(S.seeds);  t = S.t(:);  kk = 2:numel(t);  tt = t(kk);
    a_nom = S.K.a_nom;
    at = squeeze(S.a_true_out(kk, ax, 1:ns)) / a_nom;
    ah = squeeze(S.a_bar_hat_out(kk, ax, 1:ns));
    K1 = squeeze(S.K_a_y1_out(kk, ax, 1:ns));
    i1 = squeeze(S.innov_y1_out(kk, ax, 1:ns));
    hold_m = tt > 3.70;
    idx = find(hold_m);

    % prior error at step k: a_true[k] - a_hat[k-1] (the belief before this step's update)
    e_prior = at(idx, :) - ah(idx - 1, :);
    nu = i1(idx, :);
    % pooled across seeds and hold steps, mean removed per step (across seeds)
    e_c  = e_prior - mean(e_prior, 2);
    nu_c = nu - mean(nu, 2);
    K_opt = sum(e_c(:) .* nu_c(:)) / sum(nu_c(:).^2);
    K_used = mean(K1(idx, :), 'all');
    rho = corr(e_c(:), nu_c(:));
    % seed-group error bar on K_opt
    G = 10;  grp = mod((1:ns) - 1, G) + 1;  kg = zeros(G, 1);
    for g = 1:G
        sel = grp == g;  ec = e_c(:, sel);  nc = nu_c(:, sel);
        kg(g) = sum(ec(:) .* nc(:)) / sum(nc(:).^2);
    end
    fprintf('\n[%s] end hold, %d seeds x %d steps\n', label, ns, numel(idx));
    fprintf('  K_opt = Cov(e_prior, innov1)/Var(innov1) = %+.4f +- %.4f   (filter used K1(4) = %+.4f)\n', ...
            K_opt, std(kg)/sqrt(G), K_used);
    fprintf('  corr(e_prior, innov1) = %+.4f   ratio K_opt/K1 = %.3f\n', rho, K_opt / K_used);
    % is the PRIOR P44 the actual prior error variance? (spread part only)
    if isfield(S, 'P_a_out')
        sP = squeeze(S.P_a_out(kk, ax, 1:ns)) / a_nom;
        fprintf('  prior error sd across seeds %.4f  vs  filter sqrtP44 %.4f  (ratio %.2f);  bias %+.4f\n', ...
                mean(std(e_prior, 0, 2)), mean(sP(idx, :), 'all'), mean(std(e_prior, 0, 2)) / mean(sP(idx, :), 'all'), ...
                mean(e_prior, 'all'));
    end

    % does the y1 leg follow the true gain wander? block sums over 50 ms
    B = 80;  nb = floor(numel(idx) / B);
    U = zeros(nb, ns);  D = zeros(nb, ns);
    for b = 1:nb
        j = idx((b-1)*B + (1:B));
        U(b, :) = sum(K1(j, :) .* i1(j, :), 1);          % y1 leg over the block
        D(b, :) = at(j(end), :) - at(j(1), :);            % true gain change over the block
    end
    Uc = U - mean(U, 2);  Dc = D - mean(D, 2);
    slope = sum(Uc(:) .* Dc(:)) / sum(Dc(:).^2);
    rho2 = corr(Uc(:), Dc(:));
    fprintf('  y1 leg vs true gain change, %d ms blocks: slope %.3f (1 = tracking), corr %+.3f\n', ...
            round(B * (t(2)-t(1)) * 1000), slope, rho2);
    fprintf('  sd per block: y1 leg %.2e   true gain change %.2e   ratio %.1f\n', ...
            std(Uc(:)), std(Dc(:)), std(Uc(:))/std(Dc(:)));
    out = struct('K_opt', K_opt, 'K_used', K_used, 'rho', rho, 'slope', slope, 'rho2', rho2, ...
                 'se', std(kg)/sqrt(G), 'label', label);
end
