%% analyze_5state_frozen.m
% Frozen-f_d parametric analysis for the 5-state KF estimator (Plan B).
%
% For each constant f_d, the 5-state system is LTI:
%   Fe(f_d) = [0,1,0,0,0; 0,0,1,0,0; 0,0,1,-f_d,0; 0,0,0,1,1; 0,0,0,0,1]
%   H = [1,0,0,0,0]
%
% Computes: DARE solution, Kalman gains, observer poles, observability metrics.
%
% Usage: Run this script directly (no Simulink required).

clear; clc; close all;

%% 1. Physical constants and parameters
addpath('../model/config');
constants = physical_constants();

Ts       = constants.Ts;        % 1/1600
gamma_N  = constants.gamma_N;   % 0.0425
k_B      = constants.k_B;
T_temp   = constants.T;

a_nom         = Ts / gamma_N;                        % um/pN
sigma2_deltaXT = 4 * k_B * T_temp * Ts / gamma_N;   % um^2

lambda_c = 0.7;  % default

% Q/R parameters (from plan)
q3 = 1;
q4 = 1e-4;
q5 = 0;
r  = 1e-2;

Q = sigma2_deltaXT * diag([0, 0, q3, q4, q5]);
R = r * sigma2_deltaXT;
H = [1, 0, 0, 0, 0];

%% 2. f_d sweep
f_d_list = [0, 0.001, 0.01, 0.05, 0.1, 0.5, 1, 2, 5, 10, 20];
n_fd = length(f_d_list);

% Preallocate results
det_O      = zeros(n_fd, 1);
cond_O     = zeros(n_fd, 1);
Pf_ss_44   = zeros(n_fd, 1);   % gain estimation uncertainty
Pf_ss_55   = zeros(n_fd, 1);   % gain rate uncertainty
L_all      = zeros(n_fd, 5);   % Kalman gains
poles_all  = zeros(n_fd, 5);   % observer pole magnitudes
dare_ok    = true(n_fd, 1);    % DARE convergence flag

fprintf('=== 5-State Frozen-f_d Analysis ===\n');
fprintf('a_nom = %.5f um/pN, sigma2_deltaXT = %.4e um^2\n', a_nom, sigma2_deltaXT);
fprintf('Q diag scaling: [%g, %g, %g, %g, %g]\n', 0, 0, q3, q4, q5);
fprintf('R scaling: %g\n\n', r);

for idx = 1:n_fd
    f_d = f_d_list(idx);

    % Build Fe
    Fe = [0, 1, 0,    0, 0;
          0, 0, 1,    0, 0;
          0, 0, 1, -f_d, 0;
          0, 0, 0,    1, 1;
          0, 0, 0,    0, 1];

    % --- Observability matrix ---
    O = zeros(5, 5);
    HFe_power = H;
    for k = 1:5
        O(k, :) = HFe_power;
        HFe_power = HFe_power * Fe;
    end
    det_O(idx) = det(O);
    if abs(det_O(idx)) > 1e-20
        cond_O(idx) = cond(O);
    else
        cond_O(idx) = Inf;
    end

    % --- DARE solution ---
    % dare() solves: X = A'*X*A - A'*X*B*(B'*X*B + R_d)^{-1}*B'*X*A + Q_d
    % For our KF: Pf = Fe*P*Fe' + Q, P = Pf - Pf*H'*(H*Pf*H'+R)^{-1}*H*Pf
    % Equivalent dare call: dare(Fe', H', Q, R)
    try
        [Pf_ss, ~, ~] = dare(Fe', H', Q, R);
        Pf_ss_44(idx) = Pf_ss(4,4);
        Pf_ss_55(idx) = Pf_ss(5,5);

        % Kalman gain
        S = H * Pf_ss * H' + R;
        L = Pf_ss * H' / S;
        L_all(idx, :) = L';

        % Observer closed-loop poles: eig(Fe - L*H*Fe)
        % Actually: (I - L*H)*Fe is the observer error dynamics after update
        % But the standard KF error dynamics is: (Fe - Fe*L_post*H) or (I-L*H)*Fe
        % Let's compute eig of the closed-loop: Fe*(I - L*H)
        A_cl = Fe * (eye(5) - L * H);
        poles_all(idx, :) = abs(eig(A_cl))';

    catch ME
        fprintf('  f_d = %g: DARE failed (%s)\n', f_d, ME.message);
        dare_ok(idx) = false;

        % Fallback: iterative DARE (500 steps)
        Pf_iter = diag([0, 0, 1e-4, 10*a_nom^2, 0]);
        for iter = 1:500
            S_it = H * Pf_iter * H' + R;
            L_it = Pf_iter * H' / S_it;
            P_it = (eye(5) - L_it * H) * Pf_iter;
            P_it = 0.5 * (P_it + P_it');
            Pf_iter = Fe * P_it * Fe' + Q;
            Pf_iter = 0.5 * (Pf_iter + Pf_iter');
        end
        Pf_ss_44(idx) = Pf_iter(4,4);
        Pf_ss_55(idx) = Pf_iter(5,5);
        S_it = H * Pf_iter * H' + R;
        L_it = Pf_iter * H' / S_it;
        L_all(idx, :) = L_it';
        A_cl = Fe * (eye(5) - L_it * H);
        poles_all(idx, :) = abs(eig(A_cl))';
    end
end

%% 3. Results table
fprintf('\n%-8s | %-10s | %-10s | %-12s | %-12s | %-8s %-8s %-8s %-8s %-8s | %-30s\n', ...
    'f_d', 'det(O)', 'cond(O)', 'Pf_ss(4,4)', 'Pf_ss(5,5)', ...
    'L1', 'L2', 'L3', 'L4', 'L5', 'pole magnitudes');
fprintf('%s\n', repmat('-', 1, 130));

for idx = 1:n_fd
    f_d = f_d_list(idx);
    fprintf('%-8.3f | %-10.3e | %-10.2e | %-12.4e | %-12.4e | ', ...
        f_d, det_O(idx), cond_O(idx), Pf_ss_44(idx), Pf_ss_55(idx));
    fprintf('%-8.4f %-8.4f %-8.4f %-8.4f %-8.4f | ', L_all(idx,:));
    fprintf('[');
    fprintf('%.4f ', poles_all(idx,:));
    fprintf(']\n');
end

%% 4. Plots

% --- Figure 1: Observability ---
figure('Name', '5-State Observability vs f_d', 'Position', [100, 100, 900, 600]);

subplot(2,2,1);
loglog(f_d_list(2:end), abs(det_O(2:end)), 'bo-', 'LineWidth', 1.5);
hold on;
% Overlay f_d^2 reference
fd_ref = f_d_list(2:end);
loglog(fd_ref, fd_ref.^2 * abs(det_O(end))/fd_ref(end)^2, 'r--', 'LineWidth', 1);
xlabel('f_d [pN]');
ylabel('|det(O)|');
legend('det(O)', 'f_d^2 reference', 'Location', 'northwest');

subplot(2,2,2);
semilogy(f_d_list(2:end), cond_O(2:end), 'rs-', 'LineWidth', 1.5);
xlabel('f_d [pN]');
ylabel('cond(O)');

subplot(2,2,3);
semilogy(f_d_list, Pf_ss_44, 'ko-', 'LineWidth', 1.5);
hold on;
semilogy(f_d_list, Pf_ss_55, 'ms-', 'LineWidth', 1.5);
xlabel('f_d [pN]');
ylabel('Pf steady-state');
legend('Pf(4,4) gain', 'Pf(5,5) rate', 'Location', 'northeast');

subplot(2,2,4);
plot(f_d_list, L_all(:,4), 'bo-', 'LineWidth', 1.5);
hold on;
plot(f_d_list, L_all(:,5), 'rs-', 'LineWidth', 1.5);
xlabel('f_d [pN]');
ylabel('Kalman gain');
legend('L4 (gain)', 'L5 (rate)', 'Location', 'northwest');

% --- Figure 2: Observer poles ---
figure('Name', '5-State Observer Poles vs f_d', 'Position', [100, 100, 700, 400]);
hold on;
markers = {'o', 's', 'd', '^', 'v'};
for s = 1:5
    plot(f_d_list, poles_all(:,s), [markers{s} '-'], 'LineWidth', 1.2);
end
yline(1, 'r--', 'LineWidth', 1);
xlabel('f_d [pN]');
ylabel('|pole|');
legend('pole 1', 'pole 2', 'pole 3', 'pole 4', 'pole 5', 'stability', ...
       'Location', 'best');

%% 5. Typical f_d reference
fprintf('\n=== Typical f_d Magnitudes ===\n');
fprintf('Free-space osc (amp=5um, freq=1Hz):\n');
del_pd_peak = 2*pi*1*5*Ts;
f_d_free_peak = del_pd_peak / a_nom;
fprintf('  del_pd_peak = %.4f um, f_d_peak = %.2f pN\n', del_pd_peak, f_d_free_peak);

fprintf('Near-wall (h=2.5um, c_perp~3):\n');
a_wall = a_nom / 3;
f_d_wall_peak = del_pd_peak / a_wall;
fprintf('  a_x = %.5f, f_d_peak = %.2f pN\n', a_wall, f_d_wall_peak);

fprintf('Near-wall (h=h_min, c_perp~6):\n');
a_deep = a_nom / 6;
f_d_deep_peak = del_pd_peak / a_deep;
fprintf('  a_x = %.5f, f_d_peak = %.2f pN\n', a_deep, f_d_deep_peak);

%% 6. Save
save_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'test_results', 'verify');
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end
save(fullfile(save_dir, 'frozen_5state.mat'), ...
    'f_d_list', 'det_O', 'cond_O', 'Pf_ss_44', 'Pf_ss_55', ...
    'L_all', 'poles_all', 'dare_ok', ...
    'Q', 'R', 'lambda_c', 'a_nom', 'sigma2_deltaXT');

fprintf('\nResults saved to test_results/verify/frozen_5state.mat\n');
