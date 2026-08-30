% STATUS: ACTIVE (scratch) | PURPOSE: is fdet still earning its place on the
%   CURRENT line? The -24.3% -> +0.0% evidence for the deterministic mirror is
%   4-state-era (4state_del_hd.tex); formB has never been measured with the
%   flag flipped, and the one measurement that exists on a descendant
%   (formC, 2026-08-12 four-arm ablation) points the OTHER way: fdet OFF
%   HALVED the spurious hold drift. Paired arms, same seeds, production
%   scenario. Judged on the seed-independent signature (hold drift rate) per
%   derivation-workflow rule 9, with the level metrics reported as paired
%   differences, never as arm means.
%   EXPIRES: when formB owns a fdet acceptance arm in test_script/integration/.
% Production files untouched; both arms go through ctrl_const_override, whose
% wiring was grepped first (run_formB_ws.m:956 copies it into ctrl_const, and
% local_run_once clears the controller persistents per run).

SEEDS = [7 11 23 42 101 777];
AX_Z  = 3;
W_DES = [0.5 1.5];    % descent
W_OSC = [1.6 3.5];    % oscillation, after the transition settle
W_HLD = [3.8 4.8];    % final hold, after the readout settle

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(genpath(fullfile(root, 'model')));
addpath(fullfile(root, 'test_script', 'integration'));

arms = struct('name', {'fdet ON (production)', 'fdet OFF'}, 'use_fdet', {true, false});
R = cell(2, 1);
for a = 1:2
    R{a} = run_formB_ws(struct('seeds', SEEDS, ...
                               'ctrl_const_override', struct('use_fdet', arms(a).use_fdet)));
end

n = numel(SEEDS);
E = cell(2, n);  T = [];
M = zeros(n, 4, 2);   % seed x [descPk oscRMS holdMean holdDrift] x arm
for a = 1:2
    for q = 1:n
        s = R{a}.runs{q};
        t = s.tout(:);
        err = 100 * (s.a_hat_out(:, AX_Z) - s.a_true_out(:, AX_Z)) ./ s.a_true_out(:, AX_Z);
        E{a, q} = err;  T = t;
        kd = t >= W_DES(1) & t <= W_DES(2);
        ko = t >= W_OSC(1) & t <= W_OSC(2);
        kh = t >= W_HLD(1) & t <= W_HLD(2);
        pf = polyfit(t(kh) - t(find(kh, 1)), err(kh), 1);
        M(q, :, a) = [max(abs(err(kd))), rms(err(ko)), mean(err(kh)), pf(1)];
    end
end

% ---- instrument check: the flag must actually change the run -------------
dmax = 0;
for q = 1:n; dmax = max(dmax, max(abs(E{1, q} - E{2, q}))); end
fprintf('\n[instrument] max |arm1 - arm2| gain error = %.3f %% (must be > 0)\n', dmax);
assert(dmax > 1e-9, 'use_fdet override is not wired -- arms are identical.');

lbl = {'desc peak %', 'osc RMS %', 'hold mean %', 'hold drift %/s'};
fprintf('\n==================== fdet paired arms, formB, 6 seeds ====================\n');
for m = 1:4
    d = M(:, m, 2) - M(:, m, 1);      % OFF - ON  (negative = OFF is better for |.|)
    fprintf('\n%-15s | %8s %8s | %9s\n', lbl{m}, 'ON', 'OFF', 'OFF-ON');
    for q = 1:n
        fprintf('%15d | %+8.3f %+8.3f | %+9.3f\n', SEEDS(q), M(q, m, 1), M(q, m, 2), d(q));
    end
    fprintf('%15s | %+8.3f %+8.3f | %+9.3f +- %.3f (SEM), t = %+.2f, %d/%d same sign\n', ...
            'mean', mean(M(:, m, 1)), mean(M(:, m, 2)), mean(d), std(d) / sqrt(n), ...
            mean(d) / (std(d) / sqrt(n)), max(sum(d > 0), sum(d < 0)), n);
end

% ---------------- figure: per-seed paired traces, shared y ----------------
ylim_all = [-1 1] * max(cellfun(@(e) max(abs(e)), E(:)));
fig = figure('Position', [60 60 1500 760]);
for q = 1:n
    subplot(2, 3, q); hold on; box on;
    plot(T, E{1, q}, 'b', 'LineWidth', 1.0);
    plot(T, E{2, q}, 'k', 'LineWidth', 1.0);
    yline(0, 'r--');
    ylim(ylim_all);
    xlabel('t [s]'); ylabel(sprintf('seed %d: gain error [%%]', SEEDS(q)));
    if q == 1
        legend({'fdet ON (production)', 'fdet OFF', 'truth'}, ...
               'Location', 'northoutside', 'Orientation', 'horizontal');
    end
end
png = fullfile(here, 'temp_fdet_arm_pair.png');
exportgraphics(fig, png, 'Resolution', 150);
fprintf('\nfigure: %s\n', png);
