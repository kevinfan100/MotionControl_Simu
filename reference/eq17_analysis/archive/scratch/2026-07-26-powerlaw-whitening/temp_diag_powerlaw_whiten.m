%TEMP_DIAG_POWERLAW_WHITEN  Is there really a trade-off, or is R2 just wrong?
%
%   a_xm is EXACTLY an AR(1) filter of the single-sample gain readout u[k]:
%       a_xm[k] = (1-a_cov) a_xm[k-1] + a_cov u[k],   E[u[k]] = a_h[k-d]
%   The derivation feeds a_xm to the KF as a WHITE measurement of a_h[k-d].
%   The whitened (correct) measurement is the increment
%       y2'[k] = a_xm[k] - (1-a_cov) a_xm[k-1] = a_cov u[k]
%       H' = a_cov * H ,  R2' = a_cov^2 * 2 * IF * (a_h+xi)^2
%   Information ratio (current / whitened) = 2/K_var * a_cov^2/... = (2-a_cov)/a_cov.
%   -> the present R2 over-states y2's information by EXACTLY (2-a_cov)/a_cov.
%
%   TEST: if that is the true derivation, results must become a_cov-INVARIANT.
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
cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;  cfg.Pf_a_frac = 0.03;
az = 3;  seeds = [7 11 23 42 101 777];

fprintf('predicted over-count factor (2-a_cov)/a_cov:\n');
for acv = [0.02 0.05 0.10 0.20]
    fprintf('   a_cov=%.2f -> %.1f   (sqrt = %.2f, expect that as the honesty ratio)\n', ...
            acv, (2-acv)/acv, sqrt((2-acv)/acv));
end

fprintf('\n%-22s | %9s | %8s %8s %9s %7s | %8s %8s | %7s\n', 'arm', 'p_desc_sd', ...
        'p_osc','sc','sig_p_end','honest','a_hat/aT','sc','trk z');
fprintf('%s\n', repmat('-', 1, 110));
for wh = [0 1]
    for acv = [0.02 0.05 0.10 0.20]
        c = cfg;  c.a_cov = acv;  c.y2_whiten = wh;
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
        assert(isfield(s.ctrl_const,'y2_whiten') && s.ctrl_const.y2_whiten == wh, ...
               'y2_whiten knob did NOT reach the controller');
        nm = sprintf('%s a_cov=%.2f', ternary(wh>0,'WHITENED','as-derived'), acv);
        fprintf('%-22s | %9.4f | %8.4f %8.4f %9.4f %7.1f | %8.4f %8.4f | %7.2f\n', ...
            nm, mean(pd_sd), mean(po), std(po), mean(sg), ...
            std(po)/max(mean(sg),eps), mean(ar), std(ar), mean(tk));
    end
    fprintf('%s\n', repmat('-', 1, 110));
end
fprintf('truth p_eff(z) ~ 0.976..1.004 ; honest = across-seed scatter / claimed sigma_p (1 = honest)\n');
fprintf('ACCEPTANCE: whitened rows must be a_cov-INVARIANT and honest ~ 1 with NO knob.\n');

function o = ternary(c, a, b)
    if c; o = a; else; o = b; end
end
