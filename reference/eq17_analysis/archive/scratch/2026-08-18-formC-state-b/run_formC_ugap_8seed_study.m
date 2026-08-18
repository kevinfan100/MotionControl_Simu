% STATUS: ACTIVE (scratch) -- 8-seed adjudication of the u = 1/(1-a_bar)
%          writing against its a_bar-coordinate sibling.
%          Spec: reference/eq17_analysis/derivation/formC_ugap.tex (+ _ref).
% PURPOSE: run the pre-registered comparison of formC_ugap_ref and, because
%   the two writings CANNOT share a shape floor (constraint (i) forces the
%   local value on the u arm while the a_bar arm carries the envelope sup),
%   break the 2x2 so the coordinate effect and the prior-width effect are
%   separated rather than confounded.
%
%   ARMS (all on the identical canonical scenario and identical draws)
%     A_env   a_bar coordinate, envelope floor 0.03058  <- the 4.68% reference
%     A_loc   a_bar coordinate, local floor    0.00558
%     U_loc   u     coordinate, local floor    0.00558  <- the arm under test
%     U_env   u     coordinate, envelope floor 0.03058  (ill-posed by the ref's
%                                                        own table; measured)
%     U_dist  u coordinate, local floor, delta a FREE (derivation (b))
%     A_dist  a_bar coordinate, delta a free (the additive sibling, for the
%                                             row-3 panel of the figures)
% EXPIRES: when the adjudication is written up.
% Output: test_results/formC_ugap_8seed.mat + one figure page per seed.

clear; clc;
SEEDS = [7 11 23 42 101 777 27 31];
FLOOR_LOCAL = [];   % filled from the u driver's own derivation (printed there)

fprintf('\n################  ARM A_env : a_bar coordinate, envelope floor  ################\n');
A_env = run_formC_dist(struct('arm', 'base', 'seeds', SEEDS, 'verbose', false));

fprintf('\n################  ARM U_loc : u coordinate, local floor  ################\n');
U_loc = run_formC_ugap(struct('arm', 'base', 'seeds', SEEDS, 'verbose', false));
FLOOR_LOCAL = U_loc.floor.local;
FLOOR_ENV   = U_loc.floor.env;

fprintf('\n################  ARM A_loc : a_bar coordinate, LOCAL floor (floor control)  ################\n');
A_loc = run_formC_dist(struct('arm', 'base', 'seeds', SEEDS, 'verbose', false, ...
            'ctrl_const_override', struct('Pf_a_floor', FLOOR_LOCAL)));

fprintf('\n################  ARM U_env : u coordinate, envelope floor (ill-posed corner)  ################\n');
U_env = run_formC_ugap(struct('arm', 'base', 'seeds', SEEDS, 'verbose', false, ...
            'floor_mode', 'env'));

fprintf('\n################  ARM U_dist : u coordinate, delta a FREE  ################\n');
U_dist = run_formC_ugap(struct('arm', 'dist', 'seeds', SEEDS, 'verbose', false));

fprintf('\n################  ARM A_dist : a_bar coordinate, delta a free (figure companion)  ################\n');
A_dist = run_formC_dist(struct('arm', 'dist', 'seeds', SEEDS, 'verbose', false));

% ------------------------------------------------------------------------
% Summary table + the paired comparison against A_env (the 4.68% reference)
% ------------------------------------------------------------------------
arms = {A_env, A_loc, U_loc, U_env, U_dist, A_dist};
names = {'A_env (a-coord, env floor)', 'A_loc (a-coord, local floor)', ...
         'U_loc (u-coord, local floor)', 'U_env (u-coord, env floor)', ...
         'U_dist (u-coord, delta a free)', 'A_dist (a-coord, delta a free)'};

fprintf('\n\n================= SUMMARY over %d seeds =================\n', numel(SEEDS));
fprintf('%-32s %14s %14s %14s %14s\n', 'arm', 'desc pk %', 'osc RMS %', 'hold mean %', 'rms all %');
for i = 1:numel(arms)
    M = arms{i}.metrics.rows;
    fprintf('%-32s %7.3f+-%-6.3f %7.3f+-%-6.3f %+7.3f+-%-6.3f %7.3f+-%-6.3f\n', names{i}, ...
            mean(M(:,1)), std(M(:,1)), mean(M(:,2)), std(M(:,2)), ...
            mean(M(:,3)), std(M(:,3)), mean(M(:,4)), std(M(:,4)));
end

ref = A_env.metrics.rows(:, 4);
fprintf('\n----- PAIRED against A_env (same seeds, same draws), overall relative gain-error RMS -----\n');
fprintf('%-32s %9s %9s %9s %8s %8s\n', 'arm', 'mean d', 'sd d', 'SEM', 't', 'wins');
for i = 2:numel(arms)
    d = arms{i}.metrics.rows(:, 4) - ref;
    sem = std(d) / sqrt(numel(d));
    fprintf('%-32s %+9.3f %9.3f %9.3f %+8.2f %4d/%d\n', names{i}, ...
            mean(d), std(d), sem, mean(d)/max(sem, eps), sum(d < 0), numel(d));
end

% The A_env comparison confounds coordinate with prior width. A_loc is the
% SAME a_bar estimator carrying the u arm's local floor, so U_loc - A_loc is
% the coordinate alone and A_loc - A_env is the floor alone.
fprintf('\n----- PAIRED, matched floor (coordinate effect isolated) -----\n');
pairs = {'U_loc - A_loc  (coordinate only)', U_loc, A_loc, 4; ...
         'A_loc - A_env  (floor only)',      A_loc, A_env, 4; ...
         'U_dist - A_loc (coord + delta a)', U_dist, A_loc, 4; ...
         'U_dist - U_loc (delta a only)',    U_dist, U_loc, 4; ...
         'hold mean: U_loc - A_loc',         U_loc, A_loc, 3; ...
         'hold mean: U_dist - A_loc',        U_dist, A_loc, 3};
fprintf('%-34s %9s %9s %9s %8s %8s\n', 'contrast', 'mean d', 'sd d', 'SEM', 't', 'wins');
for i = 1:size(pairs, 1)
    d = pairs{i,2}.metrics.rows(:, pairs{i,4}) - pairs{i,3}.metrics.rows(:, pairs{i,4});
    sem = std(d) / sqrt(numel(d));
    fprintf('%-34s %+9.3f %9.3f %9.3f %+8.2f %4d/%d\n', pairs{i,1}, ...
            mean(d), std(d), sem, mean(d)/max(sem, eps), sum(d < 0), numel(d));
end

fprintf('\n----- per-seed rms all %% -----\n');
fprintf('%6s', 'seed');
for i = 1:numel(arms); fprintf(' %10s', extractBefore([names{i} ' '], ' ')); end
fprintf('\n');
for q = 1:numel(SEEDS)
    fprintf('%6d', SEEDS(q));
    for i = 1:numel(arms); fprintf(' %10.3f', arms{i}.metrics.rows(q,4)); end
    fprintf('\n');
end

fprintf('\n----- delta a diagnostics, arm U_dist -----\n');
fprintf('%6s %14s %14s %10s %10s\n', 'seed', 'da_hat[end]', 'sqrtP_dada[end]', 'clamp u z', 'clamp v z');
D = U_dist.metrics.da_rows;  M = U_dist.metrics.rows;
for q = 1:numel(SEEDS)
    fprintf('%6d %+14.5e %14.5e %10.4f %10.4f\n', SEEDS(q), D(q,1), D(q,2), M(q,8), M(q,9));
end
fprintf('anchor check: far-field reflection gives delta a = -1/9 = %+.5f ; recovered mean %+.5f, t = %+.2f\n', ...
        -1/9, mean(D(:,1)), (mean(D(:,1)) + 1/9) / (std(D(:,1))/sqrt(numel(SEEDS))));
fprintf('mean %+.5e +- %.5e ; sqrtP[0] %.5e -> [end] %.5e\n', ...
        mean(D(:,1)), std(D(:,1)), D(1,3), mean(D(:,2)));

fprintf('\n----- clamp audit (all u arms; MUST be zero) -----\n');
for i = [3 4 5]
    M = arms{i}.metrics.rows;
    nu = zeros(3,1); nv = zeros(3,1);
    for q = 1:numel(SEEDS)
        nu = nu + arms{i}.runs{q}.clamp_u_count(:);
        nv = nv + arms{i}.runs{q}.clamp_v_count(:);
    end
    fprintf(['%-32s z-axis max frac u %.5f / v %.5f ; worst-axis %.5f / %.5f ; ', ...
             'total per axis [x y z] u [%d %d %d] v [%d %d %d] ; any NaN %d\n'], names{i}, ...
            max(M(:,8)), max(M(:,9)), max(M(:,10)), max(M(:,11)), nu, nv, any(M(:,7)));
end

% ------------------------------------------------------------------------
% Save + figures
% ------------------------------------------------------------------------
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
out_file = fullfile(root, 'test_results', 'formC_ugap_8seed.mat');
save(out_file, 'A_env', 'A_loc', 'U_loc', 'U_env', 'U_dist', 'A_dist', ...
     'SEEDS', 'FLOOR_LOCAL', 'FLOOR_ENV', '-v7.3');
fprintf('\nsaved: %s\n', out_file);

plot_formC_dist_compare(A_env, U_loc, struct('suffix', '_ugap', ...
                                             'names', {{'a-coord', 'u-coord'}}));
