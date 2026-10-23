% STATUS: ACTIVE (scratch) | PURPOSE: 8-seed Config B (da free from 0) evidence
%   run for the state-writing arm -- console table + one panel per seed for the
%   "Simulation figures" section of formC_state_da / formB_ws.tex.
%   EXPIRES: T2/T3/T4 adoption decision on the state writing.
%   Config B ONLY by request (no Config A anywhere).
function run_formC_state_cfgB_8seed()

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    addpath(fullfile(root, 'test_script', 'integration'));

    SEEDS  = [7 11 23 42 101 777 27 31];
    AX_Z   = 3;                 % wall-normal axis
    DA_REF = -1/9;              % far-field reflection anchor, NOT fed to the filter

    oB = run_formC_state(struct('arm', 'B', 'seeds', SEEDS));

    % ---- per-seed table -------------------------------------------------
    n  = numel(SEEDS);
    T  = zeros(n, 7);   % desc pk | osc RMS | hold mean | relerr RMS | da_end | sP55_end | miss sigma
    for q = 1:n
        r  = oB.runs{q};
        a_disp = r.a_hat_out(1, AX_Z) / r.a_bar_hat_out(1, AX_Z);
        aT = r.a_true_out(:, AX_Z) / a_disp;          % a_bar true [-]
        aH = r.a_bar_hat_out(:, AX_Z);                % a_bar estimate [-]
        e  = 100 * (aH - aT) ./ aT;                   % relative gain error [%]

        da_end  = r.b_hat_out(end, AX_Z);             % slot 5 = da [-]
        sP_end  = r.P_b_out(end, AX_Z);               % sqrt(P55) [-] (already std)
        T(q, :) = [oB.metrics.rows(q, 1), oB.metrics.rows(q, 2), oB.metrics.rows(q, 3), ...
                   sqrt(mean(e.^2)), da_end, sP_end, abs(da_end - DA_REF) / sP_end];
    end

    fprintf('\n=== Config B (da free from 0), z axis, %d seeds ===\n', n);
    fprintf('%6s | %9s %9s %10s | %10s | %9s %9s %8s\n', 'seed', ...
            'desc pk %', 'osc RMS %', 'hold mean %', 'relerr RMS%', ...
            'da_end', 'sqrtP55', '|miss|s');
    for q = 1:n
        fprintf('%6d | %9.3f %9.3f %+10.3f | %10.3f | %+9.5f %9.5f %8.2f\n', ...
                SEEDS(q), T(q, 1), T(q, 2), T(q, 3), T(q, 4), T(q, 5), T(q, 6), T(q, 7));
    end
    mu = mean(T, 1);  sd = std(T, 0, 1);
    fprintf('%6s | %9s %9s %10s | %10s | %9s %9s %8s\n', repmat('-', 1, 6), ...
            repmat('-', 1, 9), repmat('-', 1, 9), repmat('-', 1, 10), ...
            repmat('-', 1, 10), repmat('-', 1, 9), repmat('-', 1, 9), repmat('-', 1, 8));
    fprintf('%6s | %4.3f+-%.3f %4.3f+-%.3f %+5.3f+-%.3f | %5.3f+-%.3f | %+.5f+-%.5f %.5f+-%.5f %.2f+-%.2f\n', ...
            'mean', mu(1), sd(1), mu(2), sd(2), mu(3), sd(3), mu(4), sd(4), ...
            mu(5), sd(5), mu(6), sd(6), mu(7), sd(7));

    % Ensemble miss of mean(da_hat) from -1/9, in units of the ENSEMBLE
    % standard error sd/sqrt(n) (spread across seeds, not the filter's P55).
    sem      = sd(5) / sqrt(n);
    ens_miss = (mu(5) - DA_REF) / sem;
    fprintf('ensemble: mean(da_end) %+.5f  target %+.5f  sd %.5f  sem %.5f  -> miss %+.2f sd/sqrt(%d)\n', ...
            mu(5), DA_REF, sd(5), sem, ens_miss, n);
    fprintf('mean(sqrt(P55)_end) %.5f  vs  across-seed sd(da_end) %.5f  (ratio %.2f)\n', ...
            mu(6), sd(5), sd(5) / mu(6));

    % ---- figures: four 3x2 pages, two seeds side by side, Config B only --
    % Pairing puts the two branches of the descent-peak split on their own
    % pages: p1 = the low branch (27, 31), p2 = the high branch incl. the
    % seed-11 extreme.
    plot_formC_state_panels(oB, struct('arm', 'B', ...
        'pairs', [27 31; 7 11; 23 42; 101 777], ...
        'names', {{'p1', 'p2', 'p3', 'p4'}}));

    save(fullfile(root, 'test_results', 'temp_formC_state_cfgB_8seed.mat'), 'oB', 'T', 'SEEDS');
end
