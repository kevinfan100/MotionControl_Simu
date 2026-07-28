%TEMP_DIAG_POWERLAW_DESCENT  Phase-1 evidence gathering for the descent-stage
%   p_hat oscillation / unsteady tracking in motion_control_law_5state_powerlaw.
%   NO code changes -- reads the existing smoke-test .mat and the analytic wall
%   curve only. Scratch; delete after the diagnosis.
%
%   Part A: raw p_hat[k] (z axis) around the descent, with h_bar, gates,
%           innovation, Kalman gain -- printed as a decimated table.
%   Part B: the TRUE effective exponent implied by the real c_perp curve,
%           p_eff(h_bar) := -c'(h_bar)*(h_bar-1)/(c(h_bar)-1), over the whole
%           trajectory range -- tests the "p is constant" modelling assumption.
%   Part C: observability budget of the p channel through y2.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..');
addpath(genpath(proj));

S   = load(fullfile(here, 'temp_smoke_5state_powerlaw.mat'));
sim = S.sim;  cfg = S.cfg;  P = sim.meta.params_value;

t    = sim.tout(:);
R    = P.common.R;
w    = P.wall.w_hat(:);
pz   = P.wall.pz;
a_nom = P.common.Ts / P.common.gamma_N;

pH   = sim.p_hat_out(:,3);
sdP  = sim.P_p_out(:,3);          % already sqrt(P55)
aH   = sim.a_hat_out(:,3);
aT   = sim.a_true_out(:,3);
aX   = sim.a_xm_out(:,3);
apH  = sim.a_prime_out(:,3);
inn  = sim.innov_y2_out(:,3);
Ka   = sim.K_a_y2_out(:,3);
gt   = sim.gate_out(:,3);
hb_m = sim.h_bar_out(:);                                    % measured h_bar
hb_d = (sim.p_d_out * w - pz) / R;                          % DESIRED h_bar (slope op. point)
hb_t = max((sim.p_true_out * w - pz) / R, 1.001);

fprintf('\n=========== PART A: raw p_hat (z) timeline ===========\n');
fprintf('a_o = a_nom = %.4f nm/pN ; R = %.3f um ; Ts = %.6f s\n', a_nom*1e3, R, P.common.Ts);
fprintf('h_bar_d range: [%.2f  %.2f] ; t_hold=%.2f  t_desc=%.2f\n', ...
        min(hb_d), max(hb_d), cfg.t_hold, cfg.t_descend_override);
fprintf('%8s %8s %9s %9s %9s %9s %9s %8s %6s\n', ...
        't[s]','hbar_d','p_hat','sig_p','a_hat','a_true','a_xm','innov2','gate');
idx = 1:32:numel(t);
for i = idx
    fprintf('%8.4f %8.3f %9.4f %9.4f %9.3f %9.3f %9.3f %8.3f %6d\n', ...
            t(i), hb_d(i), pH(i), sdP(i), aH(i)*1e3, aT(i)*1e3, aX(i)*1e3, inn(i)*1e3, gt(i));
end

fprintf('\n--- p_hat extrema / phase stats (z) ---\n');
[pmin, imin] = min(pH);  [pmax, imax] = max(pH);
fprintf('min p_hat = %.4f at t=%.4f (hbar_d=%.2f) ; max = %.4f at t=%.4f (hbar_d=%.2f)\n', ...
        pmin, t(imin), hb_d(imin), pmax, t(imax), hb_d(imax));
ph = {'hold  t<0.5', 'descend 0.5-1.5', 'osc  1.6-4.8'};
msk = {t < cfg.t_hold, t >= cfg.t_hold & t < cfg.t_hold + cfg.t_descend_override, t > 1.6};
for j = 1:3
    m = msk{j};
    fprintf('%-16s: p_hat mean %.4f  std %.4f  range [%.4f %.4f] | sig_p end %.4f\n', ...
            ph{j}, mean(pH(m)), std(pH(m)), min(pH(m)), max(pH(m)), sdP(find(m,1,'last')));
end

fprintf('\n=========== PART B: true effective exponent p_eff(h_bar) ===========\n');
fprintf('model:  s_h = (p/(hb-1)) * a(a_o-a)/a_o   <=>   -c''(hb) = p*(c(hb)-1)/(hb-1)\n');
fprintf('so      p_eff(hb) := -c''(hb)*(hb-1)/(c(hb)-1)   [=1 exactly if the model is exact]\n');
hb_grid = [1.2 1.5 2 2.5 3 4 5 6 8 10 12 15 18 20 22 25];
fprintf('%8s %10s %12s %12s %12s\n', 'hbar', 'c_perp', '-dc/dhbar', 'p_eff_perp', 'p_eff_para');
for hbv = hb_grid
    [cpa, cpe, dv] = calc_correction_functions(hbv, true);
    dperp = dv.dc_perp_dh;    % dc_perp/d h_bar
    dpara = dv.dc_para_dh;
    peff_perp = -dperp * (hbv - 1) / (cpe - 1);
    peff_para = -dpara * (hbv - 1) / (cpa - 1);
    fprintf('%8.2f %10.4f %12.5f %12.4f %12.4f\n', hbv, cpe, -dperp, peff_perp, peff_para);
end

fprintf('\n=========== PART C: p-channel observability budget through y2 ===========\n');
fprintf('H25 = -Delta_Hbar_d * s_h/p ; compare  |H25*sigma_p|  vs  sqrt(R2) (a_xm noise)\n');
% Delta_Hbar_d from the logged desired trajectory (d = 2)
d  = 2;
dHb = [zeros(d,1); (hb_d(1+d:end) - hb_d(1:end-d))];
sh_over_p = zeros(numel(t),1);
for i = 1:numel(t)
    den = hb_d(i) - 1;  den = sign(den + (den==0)) * max(abs(den), 0.05);
    sh_over_p(i) = aH(i) * (a_nom - aH(i)) / a_nom / den;
end
H25 = -dHb .* sh_over_p;
sig_axm = std(aX(t>1.6));                 % empirical a_xm noise (osc window)
fprintf('empirical std(a_xm) over osc window = %.4f nm/pN\n', sig_axm*1e3);
for tq = [0.6 0.8 1.0 1.2 1.4 1.45 2.0 3.0]
    [~, i] = min(abs(t - tq));
    fprintf('t=%.2f hbar_d=%6.2f  dHbar_d=%+9.5f  s_h/p=%9.5f  H25=%+10.3e ', ...
            t(i), hb_d(i), dHb(i), sh_over_p(i), H25(i));
    fprintf(' |H25|*0.1 = %8.3e nm/pN  (noise %.3f)\n', abs(H25(i))*0.1*1e3, sig_axm*1e3);
end

fprintf('\n--- sign health of the logistic factor a_hat*(a_o-a_hat)/a_o ---\n');
neg = (aH > a_nom);
fprintf('fraction of samples with a_hat > a_o (=> s_h sign FLIP): %.3f %%  (first at t=%.4f)\n', ...
        100*mean(neg), t(find(neg,1,'first')));

fprintf('\n--- tracking error (z) by phase, nm ---\n');
e_all = sim.p_d_out(2:end,:) - sim.p_true_out(1:end-1,:);
te = t(2:end);
for j = 1:3
    switch j
        case 1, m = te < cfg.t_hold;
        case 2, m = te >= cfg.t_hold & te < cfg.t_hold + cfg.t_descend_override;
        case 3, m = te > 1.6;
    end
    fprintf('%-16s: z std %7.2f  mean %+8.2f  max|e| %8.2f\n', ph{j}, ...
            1e3*std(e_all(m,3)), 1e3*mean(e_all(m,3)), 1e3*max(abs(e_all(m,3))));
end
