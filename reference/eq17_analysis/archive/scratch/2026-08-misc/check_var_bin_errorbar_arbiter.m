function T = check_var_bin_errorbar_arbiter(S, opts)
%CHECK_VAR_BIN_ERRORBAR_ARBITER  Which error bar is right for a binned
%   cross-seed variance: tau_eff-corrected, delete-one-seed jackknife, or
%   neither?
%
% STATUS: ACTIVE | one-off arbiter, arising from a disagreement with the
%   R22/a_m session 2026-08-20.
%
% THE DISAGREEMENT
%   Both bars claim to be the standard error of a bin value, where a bin
%   pools many CONSECUTIVE k (heavily correlated in time) across N seeds
%   (independent).
%     mine       std inside the bin / sqrt(n_k / tau_eff), tau_eff measured
%                by Geyer's initial-positive rule -- treats k as the
%                resampling unit and discounts it for autocorrelation.
%     theirs     delete-one-SEED jackknife -- the seed is the resampling
%                unit and each replicate keeps a whole trajectory intact, so
%                no time-correlation factor is needed by construction.
%   Their argument is that block resampling on the independent unit already
%   contains what my tau_eff is trying to put back by hand.
%
% THE ARBITER (their proposal, and it is assumption-free)
%   Split the seeds into G disjoint groups, compute the bin value in each,
%   and take std(group values)/sqrt(G). No tau, no jackknife algebra, no
%   model of the correlation -- just the empirical spread of independent
%   replicas of the whole measurement. Reported at two G so the estimate of
%   the estimate is visible: G = 4 has 3 dof (+-40 % on the bar itself),
%   G = 20 has 19 dof (+-16 %).

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'ax');       opts.ax = 3;      end
    if ~isfield(opts, 'n_bin');    opts.n_bin = 28;  end
    if ~isfield(opts, 'warmup_s'); opts.warmup_s = 0.05; end
    if ~isfield(opts, 'chan');     opts.chan = 'p_true_out'; end

    K = S.K; ax = opts.ax; ns = numel(S.seeds);
    X = squeeze(S.(opts.chan)(:, ax, 1:ns));            % [N x ns]
    N = size(X, 1);
    kk = (round(opts.warmup_s / K.Ts) + 1 : N).';
    X = X(kk, :);
    ab = squeeze(S.a_true_out(kk, ax, 1)) / K.a_nom;

    % pointwise cross-seed variance, all seeds
    V = var(X, 0, 2);

    % --- bins along a --------------------------------------------------
    [xs, isort] = sort(ab);  Vs = V(isort);  Xs = X(isort, :);
    edges = round(linspace(1, numel(xs)+1, opts.n_bin+1));

    % --- tau_eff on the bin-value series (my bar's ingredient) ----------
    tau = local_tau(V ./ mean(V));

    nb = opts.n_bin;
    T = table('Size', [nb 6], 'VariableTypes', repmat({'double'},1,6), ...
              'VariableNames', {'a_bar','value','se_tau','se_jack','se_g4','se_g20'});
    % leave-one-seed-out pointwise variance, vectorised:
    %   V_(-j)[k] = ((S2 - x_j^2) - (S1 - x_j)^2/(n-1)) / (n-2)
    S1 = sum(Xs, 2);  S2 = sum(Xs.^2, 2);
    for b = 1:nb
        id = edges(b):edges(b+1)-1;
        T.a_bar(b) = mean(xs(id));
        T.value(b) = mean(Vs(id));
        % (1) mine
        T.se_tau(b) = std(Vs(id)) / sqrt(numel(id) / tau);
        % (2) delete-one-seed jackknife
        s1 = S1(id); s2 = S2(id); xj = Xs(id, :);
        Vloo = ((s2 - xj.^2) - (s1 - xj).^2 / (ns-1)) / (ns-2);   % [nb_k x ns]
        th = mean(Vloo, 1);                                        % [1 x ns]
        T.se_jack(b) = sqrt((ns-1)/ns * sum((th - mean(th)).^2));
        % (3) arbiter: disjoint groups
        T.se_g4(b)  = local_group(xj, 4);
        T.se_g20(b) = local_group(xj, 20);
    end

    fprintf('\n=== bin error bar arbiter | %s | %d seeds | tau_eff %.0f ===\n', ...
            opts.chan, ns, tau);
    fprintf('%7s %11s %9s %9s %9s %9s | %7s %7s\n', 'a/a_o', 'value', ...
            'se_tau', 'se_jack', 'se_G=4', 'se_G=20', 'tau/G20', 'jck/G20');
    for b = 1:nb
        fprintf('%7.4f %11.4e %9.2e %9.2e %9.2e %9.2e | %7.2f %7.2f\n', ...
                T.a_bar(b), T.value(b), T.se_tau(b), T.se_jack(b), ...
                T.se_g4(b), T.se_g20(b), ...
                T.se_tau(b)/T.se_g20(b), T.se_jack(b)/T.se_g20(b));
    end
    fprintf('median over bins:  se_tau/se_G20 = %.2f   se_jack/se_G20 = %.2f   se_G4/se_G20 = %.2f\n', ...
            median(T.se_tau./T.se_g20), median(T.se_jack./T.se_g20), ...
            median(T.se_g4./T.se_g20));
end

% ----------------------------------------------------------------------
function se = local_group(xj, G)
%LOCAL_GROUP  Disjoint-group standard error. Each group is an independent
%   replica of the WHOLE measurement (its own seeds, the same k), so the
%   spread of the group values needs no correction of any kind.
    ns = size(xj, 2);  per = floor(ns / G);
    gv = zeros(G, 1);
    for g = 1:G
        cols = (g-1)*per + (1:per);
        gv(g) = mean(var(xj(:, cols), 0, 2));
    end
    se = std(gv) / sqrt(G);
end

% ----------------------------------------------------------------------
function tau = local_tau(x)
    x = x(:) - mean(x);  c0 = mean(x.^2);  M = numel(x);
    maxlag = min(1000, floor(M/10));  c = zeros(maxlag,1); nk = 0;
    for L = 1:maxlag
        rho = mean(x(1:end-L).*x(1+L:end)) / c0;
        if rho <= 0; break; end
        nk = nk + 1; c(nk) = rho;
    end
    tau = max(1 + 2*sum(c(1:nk)), 1);
end
