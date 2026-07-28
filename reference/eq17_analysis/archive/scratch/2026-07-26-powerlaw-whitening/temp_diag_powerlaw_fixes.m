%TEMP_DIAG_POWERLAW_FIXES  Measure the candidate fixes for the descent p_hat swing.
%   6 seeds x 7 arms. Reported per arm:
%     p_desc_std  : descent-window p_hat scatter (the "oscillation")
%     p_osc / sc  : steady p_hat mean and its ACROSS-SEED scatter (truth ~0.98)
%     sig_p_end   : the filter's CLAIMED sigma_p (honesty = sc / sig_p_end)
%     a_ratio /sc : a_hat/a_true mean and across-seed scatter
%     trk z       : z tracking std [nm]
%   Scratch; delete after the diagnosis.

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
cfg.h_bar_safe = 1.5;  cfg.Pf_a_frac = 0.03;
az = 3;  seeds = [7 11 23 42 101 777];

arms = { ...
  'A baseline',        struct(); ...
  'B a_cov=0.20',      struct('a_cov', 0.20); ...
  'C R2 x39 (colour)', struct('r2_scale', 2/0.05 - 1); ...
  'D y2 decim /20',    struct('y2_decim', 20); ...
  'E H24 restored',    struct('h24_fix', 1); ...
  'F p frozen @1',     struct('Pf_p_std', 1e-3); ...
  'G B + E',           struct('a_cov', 0.20, 'h24_fix', 1) };

fprintf('%-18s | %9s | %8s %8s %9s %7s | %8s %8s | %7s\n', 'arm', 'p_desc_sd', ...
        'p_osc','sc','sig_p_end','honest','a_ratio','sc','trk z');
fprintf('%s\n', repmat('-', 1, 108));
for r = 1:size(arms,1)
    c = cfg;
    f = fieldnames(arms{r,2});
    for j = 1:numel(f); c.(f{j}) = arms{r,2}.(f{j}); end
    pd_sd = zeros(numel(seeds),1);  po = zeros(numel(seeds),1);
    sg = zeros(numel(seeds),1);     ar = zeros(numel(seeds),1);
    tk = zeros(numel(seeds),1);
    for q = 1:numel(seeds)
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  ph = s.p_hat_out(:,az);
        md = tt > 0.5 & tt < 1.5;  mo = tt > 1.6;
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        pd_sd(q) = std(ph(md));  po(q) = mean(ph(mo));
        sg(q) = s.P_p_out(end,az);
        ar(q) = mean(s.a_hat_out(mo,az) ./ max(s.a_true_out(mo,az), eps));
        tk(q) = 1e3*std(e(te>1.6,az));
    end
    fprintf('%-18s | %9.4f | %8.4f %8.4f %9.4f %7.1f | %8.4f %8.4f | %7.2f\n', ...
        arms{r,1}, mean(pd_sd), mean(po), std(po), mean(sg), ...
        std(po)/max(mean(sg),eps), mean(ar), std(ar), mean(tk));
end
fprintf('%s\n', repmat('-', 1, 108));
fprintf('truth: p_eff(z) ~ 0.976..1.004 ; honest = across-seed scatter / claimed sigma_p (1 = honest)\n');
