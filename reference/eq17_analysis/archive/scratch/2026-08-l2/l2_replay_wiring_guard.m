% L2 predict replay -- MANDATORY WIRING GUARD (team-lead instruction): with
% ctrl_const_override.ma2_aug = false the state is 7-long and M_tot == M_row4
% exactly, so a'*M_row4(reconstructed from logs) - predict(identity) must be
% numerical zero. This arm is a different filter; wiring test only, no physics.
% Registration: test_script/scratch/l2_replay_prereg.txt
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));
SEED = 7;  AX = 3;  R_um = physical_constants().R;
for arm = {'base', 'apknown'}
    clear run_formC_b motion_control_law_formC_b;
    o = struct('seeds', SEED, 'arm', 'bmid', 'ctrl_const_override', struct('ma2_aug', false));
    if strcmp(arm{1}, 'apknown'); o.ap_known = true; end
    D = run_formC_b(o);  r = D.runs{1};
    lc = r.ctrl_const.lambda_c;  al = 1 - lc;  a_nom = r.a_nom;  t = r.tout(:);
    ah = r.a_bar_hat_out(:, AX);  K1 = r.K_a_y1_out(:, AX);  I1 = r.innov_y1_out(:, AX);
    K2 = r.K_a_y2_out(:, AX);  I2 = r.innov_y2_out(:, AX);  ap = r.a_prime_out(:, AX) / a_nom;
    hd = r.h_bar_d_out(:);  dx3 = r.delta_x_hat_3_out(:, AX);
    pred = ah(2:end) - ah(1:end-1) - K1(2:end) .* I1(2:end) - K2(2:end) .* I2(2:end);
    dwd_prev = hd(2:end) - hd(1:end-1);  dwd_prev(1) = 0;      % init call: Delta_wbar_d_km1 = 0
    M4 = dwd_prev + al * dx3(1:end-1) / R_um;                  % delta_x_hat_3_out[k-1]/R
    res = ap(2:end) .* M4 - pred;
    fprintf('WIRING GUARD (ma2_aug=false, %s, seed %d): max|a''*M_row4 - predict| = %.3e  (max|pred| %.3e)  [must be numerical zero]\n', ...
            arm{1}, SEED, max(abs(res)), max(abs(pred)));
    % show what the two wrong conventions would have produced (diagnostic only)
    M4_wrong_shift = dwd_prev + al * dx3(2:end) / R_um;
    M4_wrong_units = dwd_prev + al * dx3(1:end-1) / a_nom;
    fprintf('   (diagnostic) wrong shift dx3[k]: max|res| %.3e ; wrong units /a_nom: max|res| %.3e\n', ...
            max(abs(ap(2:end) .* M4_wrong_shift - pred)), max(abs(ap(2:end) .* M4_wrong_units - pred)));
end
