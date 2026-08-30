% Stage A follow-up #2. The ap_known arm was fed the EXACT true slope at every
% step and still landed at -22.1% with the per-seed scatter collapsed 19x
% (8.8% -> 0.47%). A near-deterministic answer means a near-deterministic
% cause, so the candidate is no longer "wrong slope" but "right slope,
% integrated wrongly": the estimator adds a_bar' * Delta_w_bar once per step
% with a_bar' taken at the step's LEFT endpoint (driver:1128 uses hb_prev).
%
% Prediction written before the run: integrating the TRUE slope along the TRUE
% height with the left-endpoint rule must fail to reproduce the TRUE gain, by
% something of the order of the observed -22%, and the trapezoid/midpoint rule
% on the same data must do markedly better. If left-endpoint reproduces the
% truth to within a few percent, quadrature is NOT the mechanism and this dies.
%
% 3 seeds: this is a deterministic-integration question, not a noise question.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));
clear run_formC_b motion_control_law_formC_b;

AX = 3;
D = run_formC_b(struct('seeds', 1:3, 'arm', 'bmid', 'ap_known', true));
a_nom = D.runs{1}.a_nom;

fprintf('\n  seed |  left-endpt   trapezoid |   truth   |  left err   trap err | filter a_hat err\n');
for s = 1:numel(D.runs)
    r  = D.runs{s};
    % h_bar_true_out is Nx1 (one wall-normal height, not per-axis)
    hb = r.h_bar_true_out(2:end, 1);             % true height, init row dropped
    ap = r.a_prime_true_out(2:end, AX) / a_nom;  % d(a_bar)/d(w_bar), normalized
    at = r.a_true_out(2:end, AX) / a_nom;        % true a_bar
    ah = r.a_bar_hat_out(2:end, AX);
    ok = isfinite(hb) & isfinite(ap) & isfinite(at);
    hb = hb(ok); ap = ap(ok); at = at(ok); ah = ah(ok);

    dw = diff(hb);
    a_left = at(1) + [0; cumsum(ap(1:end-1) .* dw)];              % left endpoint
    a_trap = at(1) + [0; cumsum(0.5*(ap(1:end-1) + ap(2:end)) .* dw)];  % trapezoid

    i_end = round(0.95*numel(at)):numel(at);   % trough hold
    tr = mean(at(i_end));
    fprintf('  %4d | %10.5f %10.5f | %8.5f | %+8.2f%% %+8.2f%% | %+8.2f%%\n', s, ...
            mean(a_left(i_end)), mean(a_trap(i_end)), tr, ...
            100*(mean(a_left(i_end))/tr - 1), 100*(mean(a_trap(i_end))/tr - 1), ...
            100*(mean(ah(i_end))/tr - 1));
end
fprintf(['\n  READ: if "left err" is large and negative and "trap err" is much\n' ...
         '  smaller, the mechanism is the quadrature rule, not the slope value.\n' ...
         '  If "left err" is near zero, quadrature is dead and the -22%% comes\n' ...
         '  from elsewhere in the arm (A_a*M deletion or the F_e(4,3) rebuild).\n\n']);
