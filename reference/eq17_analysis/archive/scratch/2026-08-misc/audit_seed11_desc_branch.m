% STATUS: ACTIVE | seed-11 outlier adjudication, part 4 (why the new arm amplifies)
%   EXPIRES: with audit_seed11_rng_stream.m -- when the seed-11 verdict is on record.
% FORK OF nothing (new diagnostic) | PURPOSE: the new arm's descent peak is BIMODAL
%   across 20 seeds (a ~7-11.5 % cluster and a ~17-25.6 % cluster) while production is
%   unimodal at 1.0-2.7 %. Take one seed from each cluster plus the outlier and print
%   the z-axis state trajectories, so the amplification can be named. Single-seed
%   calls through the public test-ladder form; zero production changes.
%   產線改動不會自動跟上

clear cd
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

PROBE = [11 27 71];        % outlier | low-cluster | fresh-seed negative tail
AX = 3;

% Arm form (NOT the single-seed ladder form): only this path applies the arm's
% ctrl_const package -- envelope priors, da_init / lock_da, the parallel law --
% so it is the only call that reproduces the numbers under adjudication.
outB = run_formB_ws(struct('seeds', PROBE));
outC = run_formC_state(struct('seeds', PROBE));

for q = 1:numel(PROBE)
    sB = outB.runs{q};   sC = outC.runs{q};
    t  = sC.tout;
    eB = 100 * (sB.a_hat_out(:, AX) - sB.a_true_out(:, AX)) ./ sB.a_true_out(:, AX);
    eC = 100 * (sC.a_hat_out(:, AX) - sC.a_true_out(:, AX)) ./ sC.a_true_out(:, AX);
    da = sC.b_hat_out(:, AX);
    w_desc = t >= 0.5 & t <= 1.5;
    tsel = t(w_desc);
    [pk, ipk] = max(abs(eC(w_desc)));
    fprintf('\n--- seed %d ---\n', PROBE(q));
    fprintf('  formC desc peak %.2f %% at t = %.3f s (h_bar %.3f); formB peak %.2f %%\n', ...
            pk, tsel(ipk), sC.h_bar_true_out(find(t == tsel(ipk), 1)), ...
            max(abs(eB(w_desc))));
    fprintf('  %6s %8s %8s %9s %9s %9s\n', 't[s]', 'h_bar', 'eC %', 'da_hat', 'P_da', 'eB %');
    for tq = [0.50 0.75 1.00 1.25 1.50 2.00 2.50 3.00 3.50 4.00 4.50 4.79]
        k = find(t >= tq, 1);
        fprintf('  %6.2f %8.3f %8.2f %9.4f %9.4f %9.2f\n', t(k), sC.h_bar_true_out(k), ...
                eC(k), da(k), sC.P_b_out(k, AX), eB(k));
    end
end

