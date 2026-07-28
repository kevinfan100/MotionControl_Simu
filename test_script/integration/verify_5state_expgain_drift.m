%VERIFY_5STATE_EXPGAIN_DRIFT  Acceptance V4: is the gain estimate stationary?
%
%   The power-law sibling's F_e endogeneity did not show up as a fixed bias but
%   as a monotone downward drift that never settled (a_hat/a_true 0.9025 ->
%   0.8655 -> 0.8375 at T = 2 / 5 / 12 s of PURE POSITIONING). Pure positioning
%   is the discriminating scenario because it removes the oscillation phase,
%   so a value that keeps moving with T_sim can only be drift.
%
%   Arms: use_fdet = true (default, deterministic exogenous F_e regressor)
%         use_fdet = false (realised force, i.e. the rectifying version)
%
%   Focus axis z. Scenario matches the power-law regression so the numbers are
%   directly comparable: h_bar = 22.2, 6 seeds.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

base = user_config();
base.h_min = 1.05 * pc.R;
base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
base.meas_noise_std = [0.00062; 0.00057; 0.00331];
base.h_bar_safe = 1.5;

pos = base;
pos.trajectory_type = 'positioning';
pos.h_init = 50;                       % h_bar = 22.2

lastwarn('');
fprintf('=== V4  pure positioning, h_bar = 22.2, 6 seeds, z axis ===\n');
fprintf('a_hat/a_true sampled at fixed wall-clock times; a value that MOVES\n');
fprintf('with T_sim is drift, not bias.\n\n');
fprintf('%-10s %6s | %8s %8s %8s %8s | %8s %8s\n', ...
        'arm', 'T[s]', 't=0', 't=1', 't=6', 't=12', 'a_xm/aT', 'trk nm');

for fd = [true false]
    for T = [2 5 12]
        c = pos; c.T_sim = T;
        ov = struct('use_fdet', fd);
        V = zeros(6, 4); axm = zeros(6, 1); tk = zeros(6, 1);
        for q = 1:6
            s = run_5state_expgain(c, struct('seed', seeds(q), 'verbose', false, ...
                                             'ctrl_const_override', ov));
            t = s.tout(:);
            r = s.a_hat_out(:, az) ./ s.a_true_out(:, az);
            tq = [0 1 6 12];
            for k = 1:4
                [~, i1] = min(abs(t - min(tq(k), T)));
                V(q, k) = r(i1);
            end
            ss = t > 1.0;
            axm(q) = mean(s.a_xm_out(ss, az)) / mean(s.a_true_out(ss, az));
            e = s.p_d_out(2:end, :) - s.p_true_out(1:end-1, :);
            te = t(2:end);
            tk(q) = 1e3 * std(e(te > 1.0, az));
        end
        if fd; nm = 'fdet ON'; else; nm = 'fdet OFF'; end
        fprintf('%-10s %6.0f | %8.4f %8.4f %8.4f %8.4f | %8.4f %8.2f\n', ...
                nm, T, mean(V(:,1)), mean(V(:,2)), mean(V(:,3)), mean(V(:,4)), ...
                mean(axm), mean(tk));
    end
    fprintf('\n');
end

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('WARNINGS: none\n');
else
    fprintf('WARNINGS: [%s] %s\n', wid, wmsg);
end
fprintf('PASS if the fdet-ON row is flat across T and the OFF row is not.\n');
