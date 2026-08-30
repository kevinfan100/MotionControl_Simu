% STATUS: ACTIVE (scratch) | PURPOSE: the rival to audit_fdet_arm_pair.m.
%   That script found fdet ON vs OFF indistinguishable on formB, but its final
%   hold is only 1.0 s, whereas the ancestor's evidence for the mirror
%   (4state_del_hd.tex: -24.3% of a_h) was measured over 12 s of positioning
%   -- a rectification accumulates, so a 1 s window can hide it.
%   [hypothesis] formB no longer has the failure mode
%   [rival]      formB has it, the canonical scenario is just too short
%   [separator]  same paired arms with the final hold stretched to 12 s
%   Cost: 12 runs x ~14 s ~ 3 min.
%   EXPIRES: with audit_fdet_arm_pair.m.
% w_s stays locked (tier t1), so slot 7 cannot confound the gain drift.
% Production files untouched; both arms go through ctrl_const_override.

SEEDS  = [7 11 23 42 101 777];
AX_Z   = 3;
T_SIM  = 15.5;          % 0.5 hold + 1.0 descend + 2.0 osc + 12.0 final hold
W_HLD  = [3.8 15.5];    % the 12 s hold, after the readout settle
ANC    = 24.3 / 12;     % [%/s] the ancestor effect this test must be able to see

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(genpath(fullfile(root, 'model')));
addpath(fullfile(root, 'test_script', 'integration'));

R = cell(2, 1);
flags = [true false];
for a = 1:2
    R{a} = run_formB_ws(struct('seeds', SEEDS, 'config_override', struct('T_sim', T_SIM), ...
                               'ctrl_const_override', struct('use_fdet', flags(a))));
end

n = numel(SEEDS);
E = cell(2, n);  T = [];
M = zeros(n, 3, 2);   % seed x [holdMean holdDrift endErr] x arm
for a = 1:2
    for q = 1:n
        s = R{a}.runs{q};
        t = s.tout(:);
        err = 100 * (s.a_hat_out(:, AX_Z) - s.a_true_out(:, AX_Z)) ./ s.a_true_out(:, AX_Z);
        E{a, q} = err;  T = t;
        kh = t >= W_HLD(1) & t <= W_HLD(2);
        pf = polyfit(t(kh) - t(find(kh, 1)), err(kh), 1);
        M(q, :, a) = [mean(err(kh)), pf(1), mean(err(end-159:end))];
    end
end

dmax = 0;
for q = 1:n; dmax = max(dmax, max(abs(E{1, q} - E{2, q}))); end
fprintf('\n[instrument] max |arm1 - arm2| gain error = %.3f %% (must be > 0)\n', dmax);
assert(dmax > 1e-9, 'use_fdet override is not wired -- arms are identical.');

lbl = {'hold mean %', 'hold drift %/s', 'last 0.1 s err %'};
fprintf('\n=========== fdet paired arms, formB, 12 s hold, 6 seeds ===========\n');
for m = 1:3
    d = M(:, m, 2) - M(:, m, 1);
    se = std(d) / sqrt(n);
    fprintf('\n%-16s | %8s %8s | %9s\n', lbl{m}, 'ON', 'OFF', 'OFF-ON');
    for q = 1:n
        fprintf('%16d | %+8.3f %+8.3f | %+9.3f\n', SEEDS(q), M(q, m, 1), M(q, m, 2), d(q));
    end
    fprintf('%16s | %+8.3f %+8.3f | %+9.3f +- %.3f (SEM), t = %+.2f, %d/%d same sign\n', ...
            'mean', mean(M(:, m, 1)), mean(M(:, m, 2)), mean(d), se, mean(d) / se, ...
            max(sum(d > 0), sum(d < 0)), n);
    if m == 2
        fprintf('%16s | ancestor-scale effect %.2f %%/s would show at %.1f sigma\n', ...
                'power', ANC, ANC / se);
    end
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
png = fullfile(here, 'temp_fdet_arm_pair_long.png');
exportgraphics(fig, png, 'Resolution', 150);
fprintf('\nfigure: %s\n', png);
