%TEMP_DIAG_POWERLAW_HONESTINIT  What is the one-time c(h_bar) peek at init worth,
%   on the CURRENT (post A1/A2) settings?
%   peek   : a_h[0] = a_nom / c(h_bar_init)   <- reads the true wall curve once
%   honest : a_h[0] = a_nom                   <- far-field only, no c at all
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
cfg.h_bar_safe = 1.5;
az = 3;  seeds = [7 11 23 42 101 777];

pv = calc_simulation_params(cfg);  Pv = pv.Value;
a_nom = Pv.common.Ts / Pv.common.gamma_N;
hb0 = cfg.h_init / Pv.common.R;
[~, cpe0] = calc_correction_functions(hb0);
fprintf('h_bar_init = %.2f , c_perp = %.4f\n', hb0, cpe0);
fprintf('peek   a_h[0] = a_nom/c_perp = %.4f nm/pN\n', 1e3*a_nom/cpe0);
fprintf('honest a_h[0] = a_nom        = %.4f nm/pN  -> init error %+.2f %%\n\n', ...
        1e3*a_nom, 100*(cpe0-1));

fprintf('%-28s | %10s %10s | %9s %9s | %8s %8s\n', 'arm', 'err@0.1s', 'settle<2%', ...
        'a_hat/aT', 'sc', 'p_osc', 'trk z');
arms = { 'peek   + Pf_a 0.03 (prod)', 0, 0.03; ...
         'honest + Pf_a 0.03',        1, 0.03; ...
         'honest + Pf_a 0.06',        1, 0.06; ...
         'honest + Pf_a 0.10',        1, 0.10 };
for r = 1:size(arms,1)
    c = cfg;  c.a_init_honest = arms{r,2};  c.Pf_a_frac = arms{r,3};
    e0=zeros(6,1); st=zeros(6,1); ar=zeros(6,1); po=zeros(6,1); tk=zeros(6,1);
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);
        rel = (s.a_hat_out(:,az)-s.a_true_out(:,az))./s.a_true_out(:,az);
        [~,i1] = min(abs(tt-0.1));  e0(q) = 100*rel(i1);
        k1 = find(abs(rel) < 0.02 & tt < 0.5, 1, 'first');
        if isempty(k1); st(q) = NaN; else; st(q) = tt(k1); end
        mo = tt > 1.6;
        ar(q) = mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
        po(q) = mean(s.p_hat_out(mo,az));
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        tk(q) = 1e3*std(e(te>1.6,az));
    end
    fprintf('%-28s | %9.2f%% %10.3f | %9.4f %9.4f | %8.4f %8.2f\n', arms{r,1}, ...
        mean(e0), mean(st,'omitnan'), mean(ar), std(ar), mean(po), mean(tk));
end
fprintf('\nsettle = first t (during the 0.5 s hold) with |a_hat err| < 2%%\n');
fprintf('steady-state columns should be IDENTICAL across arms if the peek only\n');
fprintf('buys a transient (i.e. it is not load-bearing).\n');
