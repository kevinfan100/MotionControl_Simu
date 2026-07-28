%TEMP_DIAG_REALISTIC_INIT  What init information is actually available in a real rig?
%   Premise (project scope): wall POSITION known, c(h_bar) UNKNOWN.
%   Then a_h[0] cannot be computed -- but it can be MEASURED in situ during the
%   far-field hold, by the same IIR readout the controller already runs.
%   Q: is the hold long enough to beat the honest assumption a_h[0] = a_nom?
%   Scratch; delete with the rest of this round's temp files.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..');
addpath(genpath(proj));

cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init = 50;  cfg.h_bottom = 4.5;  cfg.amplitude = 2.5;
cfg.frequency = 1;  cfg.n_cycles = 2;
cfg.t_hold = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;
pc = physical_constants();
cfg.h_min = 1.05 * pc.R;
cfg.ctrl_enable = true;  cfg.thermal_enable = true;  cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;  cfg.Pf_a_frac = 0.06;  cfg.a_init_honest = 1;
az = 3;  seeds = [7 11 23 42 101 777 1234 5150];  ns = numel(seeds);

pv = calc_simulation_params(cfg);  Pv = pv.Value;
a_nom = Pv.common.Ts / Pv.common.gamma_N;  Ts = Pv.common.Ts;
hb0 = cfg.h_init / Pv.common.R;
[~, cpe0] = calc_correction_functions(hb0);
a_true0 = a_nom / cpe0;
err_anom = (a_nom - a_true0)/a_true0;

fprintf('=== the honest assumption: a_h[0] = a_nom (far field), no c anywhere ===\n');
fprintf('h_bar_init %.2f : a_nom %.4f vs a_true %.4f nm/pN  -> assumption error %+.2f %%\n\n', ...
        hb0, 1e3*a_nom, 1e3*a_true0, 100*err_anom);

% collect a_xm over the hold, honest-init runs
AX = [];  T = [];
for q = 1:ns
    s = temp_run_powerlaw_diag(cfg, struct('seed', seeds(q), 'verbose', false));
    AX(:,q) = s.a_xm_out(:,az); %#ok<AGROW>
    T = s.tout(:);
end
fprintf('=== can the far-field HOLD measure a_h[0] better than that? ===\n');
fprintf('(mean of a_xm over a window; scatter ACROSS SEEDS is the real std error)\n');
fprintf('%10s %8s | %11s %11s %10s | %s\n', 'window[s]','samples','mean nm/pN', ...
        'SE nm/pN','SE %','vs a_nom assumption');
for tw = [0.1 0.2 0.3 0.5]
    m = T > 0.02 & T <= tw;                     % skip the first few steps
    if nnz(m) < 20; continue; end
    mu = mean(AX(m,:), 1);
    SE = std(mu);
    fprintf('%10.2f %8d | %11.4f %11.4f %10.2f | %s\n', tw, nnz(m), 1e3*mean(mu), ...
        1e3*SE, 100*SE/a_true0, ...
        ternary(SE/a_true0 < abs(err_anom), 'BETTER', 'worse'));
end

% how long a hold would be needed
m = T > 0.02 & T <= 0.5;  mu = mean(AX(m,:),1);  SE05 = std(mu);  N05 = nnz(m);
N_need = N05 * (SE05/(abs(err_anom)*a_true0))^2;
fprintf('\nSE scales as 1/sqrt(N): to reach the %.2f %% of the a_nom assumption\n', ...
        100*abs(err_anom));
fprintf('  need N ~ %.0f samples = %.2f s of far-field hold (currently %.2f s)\n', ...
        N_need, N_need*Ts, cfg.t_hold);

fprintf('\n=== what the filter itself does with the honest init ===\n');
s = temp_run_powerlaw_diag(cfg, struct('seed', 7, 'verbose', false));
rel = (s.a_hat_out(:,az)-s.a_true_out(:,az))./s.a_true_out(:,az);
for tq = [0 0.02 0.05 0.1 0.2 0.3 0.5]
    [~,i1] = min(abs(s.tout - tq));
    fprintf('  t=%.2f s : a_hat error %+7.2f %%\n', s.tout(i1), 100*rel(i1));
end

function o = ternary(c,a,b); if c; o=a; else; o=b; end; end
