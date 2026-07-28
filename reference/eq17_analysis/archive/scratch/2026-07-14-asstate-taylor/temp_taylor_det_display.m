% TEMP_TAYLOR_DET_DISPLAY  Display the det component of motion error injected
% by e_ax in the A2 taylor-known arm, via paired-seed (CRN) differencing
% against REF (control law fed a_true -> no e_ax injection).
%
%   Model (4state_del_hd.tex, boxed):
%       dx[k+1]     = lc*dx[k]     - F_dx[k]*e_ax[k] - eps[k]      (A2)
%       dx_REF[k+1] = lc*dx_REF[k]                   - eps[k]      (REF, same seed)
%   Paired difference D := dx_A2 - dx_REF (common eps cancels):
%       D[k+1] = lc*D[k] - F_dx[k]*e_ax[k]
%   Ensemble over seeds -> D_bar(t) vs one-pole prediction driven by the
%   measured ensemble means e_ax_bar(t), F_dx_bar(t).
%
%   Resumable: caches one small .mat per (arm, seed) under
%   test_results/aprime_4state/det_display/cache/; runs at most MAX_NEW new
%   sims per invocation (MCP timeout guard). Re-run until it prints DONE.
%
%   Temp script - delete after use.

seeds    = 1:100;
T_sim    = 4.0;
MAX_NEW  = 15;        % new sims per invocation
SMOOTH_N = 41;        % movmean window (~26 ms) for display

script_dir   = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
        fullfile(project_root, 'model', 'wall_effect'), ...
        fullfile(project_root, 'model', 'thermal_force'), ...
        fullfile(project_root, 'model', 'trajectory'), ...
        fullfile(project_root, 'model', 'controller'), ...
        fullfile(project_root, 'model', 'dual_track'), script_dir);

out_dir   = fullfile(project_root, 'test_results', 'aprime_4state', 'det_display');
cache_dir = fullfile(out_dir, 'cache');
if ~exist(cache_dir, 'dir'); mkdir(cache_dir); end

% --- scenario: identical to compare_aprime_4state/build_config ---
cfg = user_config();
cfg.eq17_variant    = '4state';
cfg.trajectory_type = 'osc';
cfg.h_init   = 50;
cfg.h_bottom = 2.7;
cfg.amplitude = 2.5;
cfg.frequency = 1;
cfg.n_cycles  = 2;
cfg.t_hold    = 0.5;
cfg.t_descend_override = 1.0;
cfg.T_sim     = T_sim;
pc = physical_constants();
cfg.h_min     = 1.05 * pc.R;
cfg.ctrl_enable = true;
cfg.thermal_enable = true;
cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;
cfg.a_pd  = 0.05;
cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.suppress_xD = true;
cfg.h_bar_safe = 1;
lc = cfg.lambda_c;

cfg_a2 = cfg;
cfg_a2.use_taylor_gain = true;
cfg_a2.aprime_source   = 'known';

arm_names = {'REF', 'A2'};

% --- stage 1: run missing (arm, seed) pairs, up to MAX_NEW this invocation ---
n_new = 0;
n_left = 0;
for ai = 1:2
    arm = arm_names{ai};
    if strcmp(arm, 'REF')
        cfg_arm = cfg;  use_tg = true;
    else
        cfg_arm = cfg_a2;  use_tg = false;
    end
    for s = seeds
        cache_f = fullfile(cache_dir, sprintf('%s_s%02d.mat', arm, s));
        if exist(cache_f, 'file'); continue; end
        if n_new >= MAX_NEW
            n_left = n_left + 1;
            continue;
        end
        ro = struct('seed', s, 'verbose', false, 'collect_diag', true, ...
                    'use_true_gain', use_tg);
        t_run = tic;
        simOut = run_pure_simulation(cfg_arm, ro);
        dx  = [0; simOut.p_d_out(2:end, 3) - simOut.p_true_out(1:end-1, 3)];  % dx[k]
        eax = simOut.a_true_out(:, 3) - simOut.diag.a_hat(:, 3);              % e_ax[k]
        fdz = simOut.f_d_out(:, 3);
        t   = simOut.tout;
        save(cache_f, 'dx', 'eax', 'fdz', 't');
        n_new = n_new + 1;
        fprintf('[det_display] %s seed %2d done (%.1f s)\n', arm, s, toc(t_run));
    end
end
if n_left > 0
    fprintf('PROGRESS: %d new this call, %d still missing - re-run script\n', n_new, n_left);
    return;
end

% --- stage 2: assemble ---
Ns = numel(seeds);
c0 = load(fullfile(cache_dir, sprintf('REF_s%02d.mat', seeds(1))));
N  = numel(c0.t);
t  = c0.t;
[D_all, eax_all, fdz_all] = deal(zeros(N, Ns));
for si = 1:Ns
    cR = load(fullfile(cache_dir, sprintf('REF_s%02d.mat', seeds(si))));
    cA = load(fullfile(cache_dir, sprintf('A2_s%02d.mat',  seeds(si))));
    D_all(:, si)   = cA.dx - cR.dx;
    eax_all(:, si) = cA.eax;
    fdz_all(:, si) = cA.fdz;
end
D_bar  = mean(D_all, 2);
e_bar  = mean(eax_all, 2);
fd_bar = mean(fdz_all, 2);

% F_dx[k] = f_d[k] + (1-lc)*(f_d[k-1] + f_d[k-2])
Fdx = fd_bar + (1 - lc) * ([0; fd_bar(1:end-1)] + [0; 0; fd_bar(1:end-2)]);

% one-pole model prediction driven by measured ensemble means
dx_pred = zeros(N, 1);
for k = 1:N-1
    dx_pred(k+1) = lc * dx_pred(k) - Fdx(k) * e_bar(k);
end

Ds = movmean(D_bar,   SMOOTH_N);
Ps = movmean(dx_pred, SMOOTH_N);
Es = movmean(e_bar,   SMOOTH_N);

idx = t >= 0.2;
cc  = corrcoef(Ps(idx), Ds(idx));
slope = (Ps(idx)' * Ds(idx)) / (Ps(idx)' * Ps(idx));
fprintf('[det_display] corr(model, measured) = %.3f, slope = %.3f (t >= 0.2 s)\n', ...
        cc(1, 2), slope);

% --- figure: EXP style (no grid/title, bold, legend northoutside) ---
COL_M = [0 0.35 0.8];   % measured D_bar
COL_P = [0 0.6 0];      % model prediction
COL_E = [0.8 0 0];      % e_ax_bar
FS = 18; LFS = 14; AXLW = 2.0;
T_END = ceil(t(end));

f = figure('Position', [80 80 1100 480], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
hold on;
hM = plot(t, Ds * 1e3, '-', 'Color', COL_M, 'LineWidth', 1.4, ...
          'DisplayName', '$E\{D\}$ measured');
hP = plot(t, Ps * 1e3, '-', 'Color', COL_P, 'LineWidth', 2.2, ...
          'DisplayName', '$E\{D\}$ model');
xline(0.5, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
xline(1.5, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
xline(3.5, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
yline(0, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
xlim([0 T_END]);
ylabel('$E\{D\}_z$  (nm)', 'FontSize', FS, 'Interpreter', 'latex');
xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
legend([hM hP], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'Interpreter', 'latex', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

fig_path = fullfile(out_dir, 'fig_det_display_z.png');
exportgraphics(f, fig_path, 'Resolution', 150);
fig_doc = fullfile(project_root, 'reference', 'eq17_analysis', 'derivation', ...
                   'figures', 'fig_taylor_det_inject_z.png');
exportgraphics(f, fig_doc, 'Resolution', 150);
close(f);
save(fullfile(out_dir, 'det_display_results.mat'), ...
     't', 'D_bar', 'e_bar', 'fd_bar', 'Fdx', 'dx_pred', 'Ds', 'Ps', 'Es', ...
     'cc', 'slope', 'seeds', 'SMOOTH_N');
fprintf('DONE: %s\n', fig_path);
