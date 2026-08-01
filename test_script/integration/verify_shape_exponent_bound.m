%VERIFY_SHAPE_EXPONENT_BOUND  Acceptance test for a gain-law SHAPE, and the
%   only defensible source for the exponent's prior variance P55[0].
%
%   Generates the table in Stage 2b of
%       reference/eq17_analysis/derivation/5state_expgain_hd.tex
%   and the companion numbers in 5state_powerlaw_hd.tex.
%
%   The idea: both gain laws write a "deficit" variable as a power of a shape
%   coordinate, so their exponent is a slope in log-log coordinates. Reading
%   that slope off the EXACT correction curve gives the exponent the truth
%   demands at each height. If the shape were right that would be a constant;
%   its variation IS the model error, and it bounds how far the filter's single
%   committed exponent can be from 1.
%
%       power law   c - 1        = K (h_bar - 1)^(-p)   -> p_eff = -c'(h-1)/(c-1)
%       exponential (c-1)/c      =     h_bar^(-b)       -> b_eff = -h c'/(c(c-1))
%
%   ACCEPTANCE:  sup |exponent - 1|  <~  sqrt(P55[0])   <=>  Q55 = 0 is honest.
%
%   Form B branch (formB_ws.tex S10): same acceptance idea, generalized to
%   sup |theta_eff - theta_0| with theta_0 the derived anchor (b0 = 9/8,
%   p0 = 1); see the comment block ahead of the Form B table below.
%
%   Also checks the far-field seed rule used for a_h[0] and Pf_a_frac:
%   c = 1/(1 - A/h + O(h^-3)) = 1 + A/h + A^2/h^2 + ..., so seeding with the
%   leading term alone leaves a relative gain error of A^2/h_bar_0^2, i.e. the
%   "1.25/h_bar_0^2" that used to be quoted as an offline fit is exactly
%   A^2 = (9/8)^2 = 81/64 for perp and (9/16)^2 = 81/256 for para.
%
%   Usage:  verify_shape_exponent_bound          (prints the tables)

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));

A_PERP = 9/8;      % leading method-of-reflections coefficient, perpendicular
A_PARA = 9/16;     % ... parallel
N_SWEEP = 60000;
H_MAX   = 500;
H_WORK  = 2;       % working-range floor used by the near-wall gate
B_FORMB_0 = A_PERP;  % Form B length anchor b0 = 9/8 (the same reflection coeff)
P_FORMB_0 = 1;       % Form B exponent anchor p0 = 1 (far-field 1/w decay)
H_FIT_LO  = 1.1;     % Form B minimax fit window (gainlaw_shape_selection.tex)
H_FIT_HI  = 10;

h = logspace(log10(1.001), log10(H_MAX), N_SWEEP);
[p_perp, p_para, b_perp, b_para] = deal(zeros(size(h)));
[bB_perp, pB_perp] = deal(zeros(size(h)));
for i = 1:numel(h)
    [c_para, c_perp, dv] = calc_correction_functions(h(i), true);
    p_perp(i) = -dv.dc_perp_dh * (h(i) - 1) / max(c_perp - 1, eps);
    p_para(i) = -dv.dc_para_dh * (h(i) - 1) / max(c_para - 1, eps);
    b_perp(i) = -h(i) * dv.dc_perp_dh / (c_perp * max(c_perp - 1, eps));
    b_para(i) = -h(i) * dv.dc_para_dh / (c_para * max(c_para - 1, eps));
    bB_perp(i) = (c_perp - 1) * (h(i) - 1);
    pB_perp(i) = -((h(i) - 1) + B_FORMB_0) * dv.dc_perp_dh ...
                 / (c_perp * max(c_perp - 1, eps));
end
work = h >= H_WORK;

fprintf('\n=========== SHAPE EXPONENT BOUND  sup |exponent - 1| ===========\n');
fprintf('%-26s %-4s | %8s %8s | %8s %8s\n', ...
        'shape', 'exp', 'perp all', 'perp>=2', 'para all', 'para>=2');
fprintf('%s\n', repmat('-', 1, 72));
report('power law   c-1 = K(h-1)^-p', 'p', p_perp, p_para, work);
report('exponential (c-1)/c = h^-b ', 'b', b_perp, b_para, work);

fprintf('\nlocation of the worst deviation (whole domain):\n');
locate('p perp', p_perp, h);  locate('p para', p_para, h);
locate('b perp', b_perp, h);  locate('b para', b_para, h);

fprintf('\nasymptotic anchors (both exponents must -> 1 at BOTH ends):\n');
fprintf('  %-8s h=1.001 %.4f   h=%g %.4f\n', 'p perp', p_perp(1), H_MAX, p_perp(end));
fprintf('  %-8s h=1.001 %.4f   h=%g %.4f\n', 'b perp', b_perp(1), H_MAX, b_perp(end));
fprintf('  %-8s h=1.001 %.4f   h=%g %.4f   <- no near-wall anchor\n', ...
        'p para', p_para(1), H_MAX, p_para(end));
fprintf('  %-8s h=1.001 %.4f   h=%g %.4f   <- no near-wall anchor\n', ...
        'b para', b_para(1), H_MAX, b_para(end));

% ---- Q55 is the wrong container: coherent (~N) vs diffusive (~sqrt N) -------
% Canonical descent of the smoke scenario: h_bar 22.2 -> 2 in 1 s at 1600 Hz.
H_HI = 22.2;  H_LO = 2;  N_STEP = 1600;
db   = gradient(b_perp, h);
kappa = max(abs(db(work)));
dh_step  = (H_HI - H_LO) / N_STEP;
coherent = abs(interp1(h, b_perp, H_HI) - interp1(h, b_perp, H_LO));
diffusive = kappa * sqrt(N_STEP) * dh_step;
fprintf('\n---- why Q55 = kappa^2 * Dhbar^2 cannot replace a correct shape ----\n');
fprintf('  kappa = sup |db_eff/dh_bar| (perp, h>=%g)      = %.4f\n', H_WORK, kappa);
fprintf('  b_eff travels %.4f -> %.4f over the descent   = %.4f  (coherent, ~N)\n', ...
        interp1(h, b_perp, H_HI), interp1(h, b_perp, H_LO), coherent);
fprintf('  random walk supplies kappa*sqrt(N)*dh          = %.4f  (diffusive, ~sqrt N)\n', diffusive);
fprintf('  short by %.2fx even with kappa taken as a supremum\n', coherent / diffusive);

% ---- far-field seed rule: the A^2/h^2 residual ------------------------------
fprintf('\n---- a_h[0] seed error vs the next reflection order A^2/h_bar_0^2 ----\n');
fprintf('%8s | %10s %10s %7s | %10s %10s %7s\n', 'h_bar_0', ...
        'perp meas', 'A^2/h^2', 'ratio', 'para meas', 'A^2/h^2', 'ratio');
for h0 = [50 22.2 10 5]
    [c_para, c_perp] = calc_correction_functions(h0);
    e_perp = c_perp / (1 + A_PERP / h0) - 1;    pr_perp = A_PERP^2 / h0^2;
    e_para = c_para / (1 + A_PARA / h0) - 1;    pr_para = A_PARA^2 / h0^2;
    fprintf('%8.1f | %9.4f%% %9.4f%% %7.3f | %9.4f%% %9.4f%% %7.3f\n', h0, ...
            100*e_perp, 100*pr_perp, e_perp/pr_perp, ...
            100*e_para, 100*pr_para, e_para/pr_para);
end
fprintf('=> Pf_a_frac should be per-axis: A^2/h_bar_0^2 = %.4f (perp) / %.4f (para)\n', ...
        A_PERP^2, A_PARA^2);
fprintf('   times the safety factor. The controller currently uses a single 5/h^2.\n');

% ---- acceptance verdict against the priors actually shipped -----------------
fprintf('\n---- verdict against the shipped priors ----\n');
check('expgain  Pf_b_std', 0.10,  max(abs(b_perp - 1)), 'perp, whole domain');
check('powerlaw Pf_p_std', 0.035, max(abs(p_perp - 1)), 'perp, whole domain');
fprintf('==================================================================\n');

% ---- Form B (w_s writing): theta_eff for (b, p), level vs slope readings ----
% Form B deficit: 1 - a_bar = (1 + (w-w_s)/b)^(-p); truth deficit (c-1)/c.
% With w_s at its truth (= 1, the Tier-1 setup) the deficit still carries TWO
% parameters, so each needs its own reading with the other pinned at its
% anchor (g := w - 1, the gap):
%   p (an exponent) -> log-log SLOPE of the deficit (house precedent: the
%     expgain/powerlaw theta_eff above are both slope readings), b = 9/8
%     pinned:   p_eff = -(g + 9/8) * c' / (c*(c-1))
%   b (a length)    -> deficit LEVEL, p = 1 pinned; invert b/(g+b) for b:
%                 b_eff = (c - 1) * g
%     A slope reading for b would be degenerate: far-field the log-log slope
%     tends to p for EVERY b -- it reads p's channel and misses even b's own
%     far anchor, so it cannot price the b commitment. The level can.
% Endpoints: b_eff 1 -> 9/8 and p_eff 9/8 -> 1 (contact -> far field): each
% reading travels exactly the 12.5% anchor tension, so the global sup IS the
% anchor tension -> sqrt(P_bb[0]) = sqrt(P_pp[0]) = 9/8 - 1 = 1/8 (S10).
fit = h >= H_FIT_LO & h <= H_FIT_HI;
fprintf('\n======== FORM B (w_s writing)  sup |theta_eff - theta_0|, perp ========\n');
fprintf('%-35s %-3s %6s | %8s %9s %8s\n', ...
        'reading', 'th', 'th_0', 'all', '[1.1,10]', '>=2');
fprintf('%s\n', repmat('-', 1, 76));
report_theta('level  b_eff = (c-1)(w-1)', 'b', B_FORMB_0, bB_perp, fit, work);
report_theta('slope  p_eff = -(g+9/8)c''/(c(c-1))', 'p', P_FORMB_0, pB_perp, fit, work);
fprintf('\nlocation of the worst deviation (whole domain):\n');
locate_theta('b_eff', bB_perp, B_FORMB_0, h);
locate_theta('p_eff', pB_perp, P_FORMB_0, h);
fprintf('=> global sup at the contact end = anchor tension; sqrt(P_bb[0]) = sqrt(P_pp[0]) = %.4f (= 9/8 - 1)\n', ...
        B_FORMB_0 - 1);
fprintf('==========================================================================\n');


function report(name, sym, v_perp, v_para, work)
    fprintf('%-26s %-4s | %8.4f %8.4f | %8.4f %8.4f\n', name, sym, ...
            max(abs(v_perp - 1)), max(abs(v_perp(work) - 1)), ...
            max(abs(v_para - 1)), max(abs(v_para(work) - 1)));
end

function locate(name, v, h)
    [m, ix] = max(abs(v - 1));
    fprintf('  %-8s %.4f @ h_bar = %.3f\n', name, m, h(ix));
end

function report_theta(name, sym, th0, v, fit, work)
    fprintf('%-35s %-3s %6.4f | %8.4f %9.4f %8.4f\n', name, sym, th0, ...
            max(abs(v - th0)), max(abs(v(fit) - th0)), max(abs(v(work) - th0)));
end

function locate_theta(name, v, th0, h)
    [m, ix] = max(abs(v - th0));
    fprintf('  %-8s %.4f @ w_bar = %.3f\n', name, m, h(ix));
end

function check(name, prior, bound, where)
    margin = prior / bound;
    if margin >= 2
        tag = 'PASS';
    elseif margin >= 1
        tag = 'TIGHT';
    else
        tag = 'FAIL';
    end
    fprintf('  %-20s prior %.4f vs bound %.4f -> margin %.2fx  [%s]  (%s)\n', ...
            name, prior, bound, margin, tag, where);
end
