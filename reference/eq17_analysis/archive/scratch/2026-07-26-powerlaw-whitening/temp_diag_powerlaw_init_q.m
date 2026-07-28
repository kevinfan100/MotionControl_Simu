%TEMP_DIAG_POWERLAW_INIT_Q  Audit of the init choices and the Q derivation.
%   A (analytic, no sim):
%     A1 true p_eff over the ACTUAL trajectory h_bar range, per axis
%        -> what an HONEST Pf_p_std should be (= power-law fit residual)
%     A2 a_h[0] seeding: honest (a_nom) vs the one-time c(h_bar) peek
%        -> is Pf_a_frac consistent with the init error it must cover?
%     A3 logistic sign margin: how close is a_hat[0] to the a_o zero-crossing?
%     A4 Q44 rank + the MISSING gain-model-error term vs the Q44 actually used
%   B (sim): whitened R2 x honest Pf_p_std, 6 seeds
%   C (sim): Path C's predicted P(3,3) undershoot (3-2*lc^2) vs realised
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

params = calc_simulation_params(cfg);  Pv = params.Value;
R = Pv.common.R;  a_nom = Pv.common.Ts / Pv.common.gamma_N;
kBT = Pv.ctrl.k_B * Pv.ctrl.T;  Ts = Pv.ctrl.Ts;  lc = cfg.lambda_c;

% ---------- A1: honest sigma_p from the power-law fit residual ----------
hb = linspace(2.0, 22.22, 4000).';
pe_perp = zeros(size(hb));  pe_para = zeros(size(hb));
for i = 1:numel(hb)
    [cpa, cpe, dv] = calc_correction_functions(hb(i), true);
    pe_perp(i) = -dv.dc_perp_dh * (hb(i) - 1) / (cpe - 1);
    pe_para(i) = -dv.dc_para_dh * (hb(i) - 1) / (cpa - 1);
end
fprintf('=========== A1: true p_eff over the trajectory range h_bar in [2.0, 22.2] ===========\n');
fprintf('%-14s %8s %8s %8s %10s %12s\n','axis','min','max','mean','std','max|p-1|');
fprintf('%-14s %8.4f %8.4f %8.4f %10.4f %12.4f\n','z (perp)', min(pe_perp), ...
        max(pe_perp), mean(pe_perp), std(pe_perp), max(abs(pe_perp-1)));
fprintf('%-14s %8.4f %8.4f %8.4f %10.4f %12.4f\n','x/y (para)', min(pe_para), ...
        max(pe_para), mean(pe_para), std(pe_para), max(abs(pe_para-1)));
fprintf('-> honest Pf_p_std (z)   ~ %.3f   (currently 0.300 default, %.0fx too loose)\n', ...
        std(pe_perp), 0.3/std(pe_perp));
fprintf('-> honest Pf_p_std (x/y) ~ %.3f   BUT p is NOT constant there (see spread)\n', ...
        std(pe_para));

% ---------- A2: a_h[0] seeding honesty ----------
hb0 = cfg.h_init / R;
[cpa0, cpe0] = calc_correction_functions(hb0);
fprintf('\n=========== A2: a_h[0] seeding ===========\n');
fprintf('h_bar_init = %.2f ; c_perp = %.4f ; c_para = %.4f\n', hb0, cpe0, cpa0);
fprintf('peeked  a_h[0] (z) = a_nom/c_perp = %.4f nm/pN   (code reads c(h_bar) once)\n', ...
        1e3*a_nom/cpe0);
fprintf('honest  a_h[0] (z) = a_nom        = %.4f nm/pN -> init error %+.2f %%\n', ...
        1e3*a_nom, 100*(cpe0-1));
fprintf('Pf_a_frac in use = %.3f  => prior sigma = %.2f %% of a_h\n', cfg.Pf_a_frac, ...
        100*cfg.Pf_a_frac);
fprintf('-> honest init error %.2f%% is %.1f sigma of that prior : ', 100*(cpe0-1), ...
        (cpe0-1)/cfg.Pf_a_frac);
if (cpe0-1)/cfg.Pf_a_frac > 1
    fprintf('INCONSISTENT (prior too tight without the peek)\n');
else
    fprintf('consistent\n');
end

% ---------- A3: logistic sign margin ----------
fprintf('\n=========== A3: logistic a_h(a_o-a_h)/a_o sign margin ===========\n');
for hbq = [22.22 10 5 3 2]
    [~, cpe] = calc_correction_functions(hbq);
    a_t = a_nom / cpe;
    fprintf('h_bar=%6.2f : a_o-a_true = %7.4f nm/pN  (%5.2f %% of a_o) -> ', ...
            hbq, 1e3*(a_nom - a_t), 100*(a_nom-a_t)/a_nom);
    fprintf('sign flips if a_hat err > %+.4f nm/pN\n', 1e3*(a_nom - a_t));
end

% ---------- A4: Q44 structure + missing model-error term ----------
fprintf('\n=========== A4: Q gain block ===========\n');
fprintf('Q = [Q33 Q34; Q34 Q44] = Q33*[1;-a''][1 -a'']  -> rank 1 BY CONSTRUCTION.\n');
fprintf('=> a_h receives NO process noise of its own; all of it is the thermal\n');
fprintf('   term perfectly correlated with q3. There is no term for "the power law\n');
fprintf('   is not exactly the true curve".  Size of that missing term:\n\n');
v_desc = (cfg.h_init - cfg.h_bottom) / cfg.t_descend_override;      % um/s
dhb_step = v_desc * Ts / R;                                          % per-step
fprintf('descent speed %.2f um/s -> Delta_hbar_d = %.3e per step\n', v_desc, dhb_step);
fprintf('%8s %12s %14s %14s %9s\n','h_bar','sqrt(Q44)','model_err/step','ratio','p_eff');
for hbq = [20 12 6 3 2]
    [~, cpe, dv] = calc_correction_functions(hbq, true);
    a_t = a_nom / cpe;
    s_h_true = -a_nom * dv.dc_perp_dh / cpe^2;         % da_h/dhbar (exact curve)
    a_pr = s_h_true / R;
    sq_Q44 = abs(a_pr) * sqrt(4*kBT*a_t);
    pe = -dv.dc_perp_dh * (hbq-1) / (cpe-1);
    merr = abs(pe - 1) * abs(s_h_true) * dhb_step;      % (p_eff - p_hat)*s_h/p*dhbar
    fprintf('%8.2f %12.3e %14.3e %14.1f %9.4f\n', hbq, sq_Q44, merr, ...
            merr/max(sq_Q44,eps), pe);
end
fprintf('(ratio >> 1 means the un-modelled gain-law error dwarfs the Q44 actually used)\n');

% ---------- B: whitened R2 x honest Pf_p_std ----------
fprintf('\n=========== B: whitened R2 x Pf_p_std (6 seeds) ===========\n');
fprintf('%-16s | %9s | %8s %8s %9s %7s | %8s %8s | %7s %6s\n', 'Pf_p_std', ...
        'p_desc_sd','p_osc','sc','sig_p_end','honest','a_hat/aT','sc','trk z','clamp');
RUN = struct();
for pp = [0.30 0.05 0.03]
    c = cfg;  c.y2_whiten = 1;  c.Pf_p_std = pp;
    pd_sd=zeros(6,1); po=zeros(6,1); sg=zeros(6,1); ar=zeros(6,1); tk=zeros(6,1); cl=0;
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  ph = s.p_hat_out(:,az);
        md = tt>0.5 & tt<1.5;  mo = tt>1.6;
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        pd_sd(q)=std(ph(md)); po(q)=mean(ph(mo)); sg(q)=s.P_p_out(end,az);
        ar(q)=mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
        tk(q)=1e3*std(e(te>1.6,az));
        cl = cl + any(ph <= 1e-9 | ph >= 5-1e-9);
        if pp == 0.03; RUN.(sprintf('s%d',q)) = s; end
    end
    fprintf('%-16.3g | %9.4f | %8.4f %8.4f %9.4f %7.1f | %8.4f %8.4f | %7.2f %6d\n', ...
        pp, mean(pd_sd), mean(po), std(po), mean(sg), std(po)/max(mean(sg),eps), ...
        mean(ar), std(ar), mean(tk), cl);
end
fprintf('clamp = how many of the 6 seeds hit the p in [0,5] clamp\n');

% ---------- C: Path C P(3,3) undershoot ----------
fprintf('\n=========== C: Path C predicted P(3,3) undershoot ===========\n');
fprintf('doc says keeping only the current white thermal makes P(3,3) ~ 1/(3-2lc^2)\n');
fprintf('of the true delta_h variance. For lc=%.2f that factor is %.3f (P too small by %.2fx)\n', ...
        lc, 1/(3-2*lc^2), 3-2*lc^2);
rat = zeros(6,1);
for q = 1:6
    s = RUN.(sprintf('s%d',q));
    tt = s.tout(:);
    dh_true = (s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:)) * Pv.wall.w_hat(:);
    dh_hat  = s.DG.delta_x_hat_3(2:end,az);
    P33     = s.DG.P_dx(2:end,az);
    mo = tt(2:end) > 1.6;
    err = dh_true(mo) - dh_hat(mo);
    rat(q) = var(err) / mean(P33(mo));
end
fprintf('realised var(delta_h3 - delta_h3_hat) / KF P(3,3), osc window:\n');
fprintf('   per seed: %s\n', sprintf('%.2f ', rat));
fprintf('   mean %.2f   (predicted %.2f if the Path C accounting is the whole story)\n', ...
        mean(rat), 3-2*lc^2);
