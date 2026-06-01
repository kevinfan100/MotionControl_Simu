function [f_d, ekf_out] = motion_control_law_olmode(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_OLMODE V1 controller with ONE OL_mode feature swap.
%
%   This is a minimal-change variant of motion_control_law_7state.m used to
%   isolate the impact of single OL_mode features. The CHANGE is selectable
%   via `params.ctrl.olmode_feature` (default = 'H_d2'):
%     'H_d2'      : H(2,7) = -d (v2 backshift on δa_x), everything else v1
%     'IF_eff'    : R(2,2) overridden via Phase 9 IF_eff per-axis, everything else v1
%     'IF_and_H'  : both H_d2 and IF_eff
%     'none'      : pure v1 (sanity equivalent to motion_control_law_7state)

    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1); ekf_out = [1; 1; 1; 1]; return;
    end

    persistent del_p1_hat del_p2_hat del_p3_hat d_hat del_d_hat a_hat del_a_hat
    persistent Pf_1 Pf_2 Pf_3
    persistent pd_k1 pd_k2
    persistent del_pmd_k1 del_pmrd_k1 del_pmr2_avg a_m_k1 warmup_count
    persistent initialized
    persistent gamma_N Ts lambda_c k_B T_temp a_pd a_prd a_cov beta lamdaF
    persistent sigma2_noise sigma2_deltaXT
    persistent Q_1 Q_2 Q_3 Rz_scaling
    persistent C_dpmr_eff_const C_np_eff_const IIR_bias_factor_const
    persistent feature_mode d_delay IF_eff_per_axis xi_per_axis R22_prefactor

    if isempty(initialized)
        initialized = true;
        gamma_N = params.ctrl.gamma; Ts = params.ctrl.Ts;
        lambda_c = params.ctrl.lambda_c;
        a_pd = params.ctrl.a_pd; a_prd = params.ctrl.a_prd; a_cov = params.ctrl.a_cov;
        k_B = params.ctrl.k_B; T_temp = params.ctrl.T;
        beta = params.ctrl.beta; lamdaF = params.ctrl.lamdaF;
        sigma2_noise = params.ctrl.sigma2_noise;
        sigma2_deltaXT = params.ctrl.sigma2_deltaXT;
        a_nom = Ts / gamma_N;
        d_delay = 2;

        % Feature mode hardcoded (Simulink type inference can't handle
        % evalin at compile time). Edit this line to test different features:
        %   0 = pure v1 sanity (equivalent to motion_control_law_7state)
        %   1 = H(2,7) = -d only  ★ V6 default
        %   2 = R(2,2) per-axis Phase 9 IF_eff runtime only
        %   3 = both 1 and 2
        feature_mode = 1;

        p0_init = params.common.p0;
        h_init = p0_init(:)' * params.wall.w_hat(:) - params.wall.pz;
        h_bar_init = max(h_init / params.common.R, 1.001);
        [c_para_init, c_perp_init] = calc_correction_functions(h_bar_init);
        a_hat = [a_nom / c_para_init; a_nom / c_para_init; a_nom / c_perp_init];
        del_p1_hat = zeros(3,1); del_p2_hat = zeros(3,1); del_p3_hat = zeros(3,1);
        d_hat = zeros(3,1); del_d_hat = zeros(3,1); del_a_hat = zeros(3,1);
        Pf_diag = params.ctrl.Pf_init_diag;
        Pf_1 = diag(Pf_diag); Pf_2 = diag(Pf_diag); Pf_3 = diag(Pf_diag);

        Qz_scaling = params.ctrl.Qz_diag_scaling;
        Rz_scaling = params.ctrl.Rz_diag_scaling;
        Q_shared = sigma2_deltaXT * diag(Qz_scaling(1:6));
        Q_1 = zeros(7,7); Q_1(1:6,1:6) = Q_shared; Q_1(7,7) = sigma2_deltaXT * Qz_scaling(7);
        Q_2 = zeros(7,7); Q_2(1:6,1:6) = Q_shared; Q_2(7,7) = sigma2_deltaXT * Qz_scaling(8);
        Q_3 = zeros(7,7); Q_3(1:6,1:6) = Q_shared; Q_3(7,7) = sigma2_deltaXT * Qz_scaling(9);

        if isfield(params.ctrl, 'C_dpmr_eff') && all(params.ctrl.C_dpmr_eff > 0)
            C_dpmr_eff_const = params.ctrl.C_dpmr_eff(:);
            C_np_eff_const   = params.ctrl.C_np_eff(:);
        else
            C_dpmr_eff_const = (2 + 1/(1-lambda_c^2)) * ones(3,1);
            C_np_eff_const   = (2/(1+lambda_c)) * ones(3,1);
        end
        if isfield(params.ctrl, 'IIR_bias_factor') && all(params.ctrl.IIR_bias_factor > 0)
            IIR_bias_factor_const = params.ctrl.IIR_bias_factor(:);
        else
            IIR_bias_factor_const = ones(3, 1);
        end

        % Phase 9 X2a empirical IF_eff (eq17 calibration)
        IF_eff_per_axis = [3.44; 3.43; 3.34];
        xi_per_axis = zeros(3,1);
        for ax = 1:3
            xi_per_axis(ax) = (C_np_eff_const(ax)/C_dpmr_eff_const(ax)) * ...
                              sigma2_noise(ax) / (4 * k_B * T_temp);
        end
        R22_prefactor = 2 * a_cov / (2 - a_cov);

        pd_k1 = pd; pd_k2 = pd;
        del_pmd_k1 = zeros(3,1); del_pmrd_k1 = zeros(3,1);
        ar_init = a_hat / a_nom;
        del_pmr2_avg = C_dpmr_eff_const .* sigma2_deltaXT .* ar_init + C_np_eff_const .* sigma2_noise;
        del_pmr2_avg = max(del_pmr2_avg, 0.01 * sigma2_deltaXT);
        a_m_k1 = [a_nom; a_nom; a_nom];
        warmup_count = 2;

        f_d = zeros(3, 1);
        ekf_out = [a_nom; a_nom; a_nom; a_nom];
        return;
    end

    %% V1-style three-stage IIR (kept identical to v1)
    del_pm = pd_k2 - p_m;
    del_pmd  = (1 - a_pd) * del_pmd_k1 + a_pd * del_pm;
    del_pmr  = del_pm - del_pmd;
    del_pmrd = (1 - a_prd) * del_pmrd_k1 + a_prd * del_pmr;
    del_pmr2_avg = (1 - a_cov) * del_pmr2_avg + a_cov * del_pmr.^2;
    del_pmr_var  = max(del_pmr2_avg - del_pmrd.^2, 0);

    den        = C_dpmr_eff_const * (4 * k_B * T_temp);
    noise_corr = C_np_eff_const .* sigma2_noise;
    del_pmr_var_unbiased = del_pmr_var ./ IIR_bias_factor_const;
    a_m = max((del_pmr_var_unbiased - noise_corr) ./ den, 0);

    if warmup_count > 0
        warmup_count = warmup_count - 1;
        pd_k2 = pd_k1; pd_k1 = pd;
        del_pmd_k1 = del_pmd; del_pmrd_k1 = del_pmrd; a_m_k1 = a_m;
        if warmup_count == 0
            del_p1_hat = del_pmd; del_p2_hat = del_pmd; del_p3_hat = del_pmd;
        end
        f_d = zeros(3, 1);
        ekf_out = [a_hat(1); a_hat(3); a_m(1); a_m(3)];
        return;
    end

    f_d = (1 ./ a_hat) .* (del_pd + (1 - lambda_c) * del_p3_hat - d_hat);

    %% Build R per-axis based on feature_mode
    var_threshold = sigma2_deltaXT * 0.001;
    R_i = cell(3,1);
    use_IFeff_R22 = (feature_mode == 2 || feature_mode == 3);
    for i = 1:3
        r_pos_base_i  = sigma2_deltaXT * Rz_scaling(i);
        if use_IFeff_R22
            r_gain_i = R22_prefactor * IF_eff_per_axis(i) * (a_hat(i) + xi_per_axis(i))^2;
        else
            r_gain_i = sigma2_deltaXT * Rz_scaling(3+i);
        end
        if del_pmr_var(i) < var_threshold
            r_gain_i = 1e6;
        end
        R_i{i} = diag([r_pos_base_i, r_gain_i]);
    end

    %% Fe_err (v1 form, identical to motion_control_law_7state)
    Fe_err_x = [0 1 0  0  0  0       0;
                0 0 1  0  0  0       0;
                0 0 1 -1  0 -f_d(1)  0;
                0 0 0  1  1  0       0;
                0 0 0  0  1  0       0;
                0 0 0  0  0  1       1;
                0 0 0  0  0  0       1];
    Fe_err_y = Fe_err_x; Fe_err_y(3,6) = -f_d(2);
    Fe_err_z = Fe_err_x; Fe_err_z(3,6) = -f_d(3);
    if abs(beta) > 1e-12
        Fe_err_z(4,4) = 1+beta; Fe_err_z(4,5) = -beta;
        Fe_err_z(6,6) = 1+beta; Fe_err_z(6,7) = -beta;
    end

    %% H selector: v1 (H_27=0) or v2 (H_27=-d) based on feature_mode
    use_v2_H = (feature_mode == 1 || feature_mode == 3);
    if use_v2_H
        H_27 = -d_delay;
    else
        H_27 = 0;
    end

    states_x = [del_p1_hat(1); del_p2_hat(1); del_p3_hat(1); d_hat(1); del_d_hat(1); a_hat(1); del_a_hat(1)];
    states_y = [del_p1_hat(2); del_p2_hat(2); del_p3_hat(2); d_hat(2); del_d_hat(2); a_hat(2); del_a_hat(2)];
    states_z = [del_p1_hat(3); del_p2_hat(3); del_p3_hat(3); d_hat(3); del_d_hat(3); a_hat(3); del_a_hat(3)];

    % Use a_m (current) if v2 H, else a_m_k1 (v1 lag handling)
    if use_v2_H
        meas_x = [del_pm(1); a_m(1)];
        meas_y = [del_pm(2); a_m(2)];
        meas_z = [del_pm(3); a_m(3)];
    else
        meas_x = [del_pm(1); a_m_k1(1)];
        meas_y = [del_pm(2); a_m_k1(2)];
        meas_z = [del_pm(3); a_m_k1(3)];
    end

    [inj_x, Pf_1] = ekf_update_olmode(states_x, Pf_1, meas_x, Q_1, R_i{1}, Fe_err_x, 1, H_27);
    [inj_y, Pf_2] = ekf_update_olmode(states_y, Pf_2, meas_y, Q_2, R_i{2}, Fe_err_y, 1, H_27);
    [inj_z, Pf_3] = ekf_update_olmode(states_z, Pf_3, meas_z, Q_3, R_i{3}, Fe_err_z, lamdaF, H_27);

    %% State update — v1 simplified form (preserves stability)
    del_p1_hat_new = zeros(3,1); del_p2_hat_new = zeros(3,1); del_p3_hat_new = zeros(3,1);
    d_hat_new = zeros(3,1); del_d_hat_new = zeros(3,1);
    a_hat_new = zeros(3,1); del_a_hat_new = zeros(3,1);
    for ax = 1:2
        if ax == 1, inj = inj_x; else, inj = inj_y; end
        del_p1_hat_new(ax) = del_p2_hat(ax)                          + inj(1);
        del_p2_hat_new(ax) = del_p3_hat(ax)                          + inj(2);
        del_p3_hat_new(ax) = lambda_c * del_p3_hat(ax)               + inj(3);
        d_hat_new(ax)      = d_hat(ax) + del_d_hat(ax)               + inj(4);
        del_d_hat_new(ax)  = del_d_hat(ax)                           + inj(5);
        a_hat_new(ax)      = a_hat(ax) + del_a_hat(ax)               + inj(6);
        del_a_hat_new(ax)  = del_a_hat(ax)                           + inj(7);
    end
    % z-axis: at beta=0 use Jordan-block form (v1 chart-extension was buggy
    % at beta=0 — del_a_hat_new = a_hat + inj(7) creates positive feedback
    % when H_27 != 0; this OL_mode controller always uses correct Jordan form)
    inj = inj_z;
    del_p1_hat_new(3) = del_p2_hat(3)                                + inj(1);
    del_p2_hat_new(3) = del_p3_hat(3)                                + inj(2);
    del_p3_hat_new(3) = lambda_c * del_p3_hat(3)                     + inj(3);
    if abs(beta) < 1e-12
        d_hat_new(3)      = d_hat(3) + del_d_hat(3)                  + inj(4);
        del_d_hat_new(3)  = del_d_hat(3)                             + inj(5);
        a_hat_new(3)      = a_hat(3) + del_a_hat(3)                  + inj(6);
        del_a_hat_new(3)  = del_a_hat(3)                             + inj(7);
    else
        d_hat_new(3)      = (1+beta)*d_hat(3) - beta*del_d_hat(3)    + inj(4);
        del_d_hat_new(3)  = d_hat(3)                                 + inj(5);
        a_hat_new(3)      = (1+beta)*a_hat(3) - beta*del_a_hat(3)    + inj(6);
        del_a_hat_new(3)  = a_hat(3)                                 + inj(7);
    end

    del_p1_hat = del_p1_hat_new; del_p2_hat = del_p2_hat_new; del_p3_hat = del_p3_hat_new;
    d_hat = d_hat_new; del_d_hat = del_d_hat_new;
    a_hat = a_hat_new; del_a_hat = del_a_hat_new;

    pd_k2 = pd_k1; pd_k1 = pd;
    del_pmd_k1 = del_pmd; del_pmrd_k1 = del_pmrd;
    a_m_k1 = a_m;

    ekf_out = [a_hat(1); a_hat(3); a_m(1); a_m(3)];
end


function [inj, Pf_new] = ekf_update_olmode(states, Pf, meas, Q, R, Fe_err, lamdaF_i, H_27)
    H = [1 0 0 0 0 0 0;
         0 0 0 0 0 1 H_27];
    err = meas - H * states;
    S = H * Pf * H' + R;
    S = 0.5 * (S + S') + 1e-20 * eye(2);
    L = (Pf * H') / S;
    inj = L * err;
    P = (1 / lamdaF_i) * (eye(7) - L * H) * Pf;
    P = 0.5 * (P + P');
    Pf_new = Fe_err * P * Fe_err' + Q;
    Pf_new = 0.5 * (Pf_new + Pf_new');
end
