% STATUS: SCRATCH -- 2026-10-14 (user): is E[F_dx * eps_x] nonzero in the fixed hold (closed-loop identification condition)?
%   Same run as probe_mc_s9_holdbias (S8 estimator, w_s-hat locked at 0, 22.2 -> h_hold in 1 s, hold to T_sim, no regulation).
%   From the driver log, in R units (sec1_model.tex S1, d = 2):
%     fd = a_nom f_d / R,  aT = a_true f_T / R  (= a-bar f-bar_T),  n_x[k] = noise of the reading the controller uses at k = n_raw[k-d-1]
%     F_dx[k]  = fd[k] + (1-lc)(fd[k-1] + fd[k-2])
%     eps_T[k] = aT[k] + (1-lc)(aT[k-1] + aT[k-2]),   eps_N[k] = -(1-lc) n_x[k]
%   Prints over the hold (t >= 2 s): E[F eps]/E[F^2] for each part (= bias of a least-squares gain-error read from y1, a-bar units),
%   with a +-2 SE from 50-step block means; and the lag profile E[F[k] eps_T[k+j]]/E[F^2], j = -4..4.
function probe_mc_s9_fdx_eps(h_hold, seeds, T_sim, dith)
    if nargin < 1 || isempty(h_hold); h_hold = 1.105; end
    if nargin < 2 || isempty(seeds); seeds = 1:3; end
    if nargin < 3 || isempty(T_sim); T_sim = 6; end
    if nargin < 4; dith = []; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;  lc = 0.4;  d = 2;
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                'Pf_a_floor', 0.002, 'plaw_seed_cross', true, 'plaw_dc_slope', true, 'plaw_p', 2, 'c_init', 0, 'Pf_c_std', 1e-6, ...
                'delta_init', 0, 'Pf_delta_std', 1e-6, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-5, 5]);
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', h_hold * R, 'amplitude', 0, 't_hold', 0, ...
                't_descend_override', 1, 'T_sim', T_sim, 'lambda_c', lc);
    o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
               'seeds', seeds, 'log_P_full', false, 'cmd_dither', dith);
    clear run_formC_b motion_control_law_formC_b;
    evalc('Rr = run_formC_b(o);');
    fprintf('hold %.3f R, dither %s\n', h_hold, mat2str(struct2array(struct('a', 0)) * 0 + ~isempty(dith)));
    for q = 1:numel(Rr.runs)
        S = Rr.runs{q};  n = size(S.f_d_out, 1);  t = (0:n-1)' * Ts;  an = S.a_nom;
        fd = an * S.f_d_out(:, 3) / R;  aT = S.a_true_out(:, 3) .* S.F_th_out(:, 3) / R;
        nraw = (S.p_m_out(:, 3) - S.p_true_out(:, 3)) / R;  nx = [zeros(d + 1, 1); nraw(1:end - d - 1)];
        F = fd + (1 - lc) * ([0; fd(1:end-1)] + [0; 0; fd(1:end-2)]);
        eT = aT + (1 - lc) * ([0; aT(1:end-1)] + [0; 0; aT(1:end-2)]);  eN = -(1 - lc) * nx;
        H = find(t >= 2);  F2 = mean(F(H).^2);
        [bT, sT] = local_ratio(F(H) .* eT(H), F2);  [bN, sN] = local_ratio(F(H) .* eN(H), F2);
        lag = arrayfun(@(j) mean(F(H(5:end-5)) .* eT(H(5:end-5) + j)) / F2, -4:4);
        fprintf('  seed %d: std n_raw %.2f nm | E[F eps_T]/E[F^2] %+.4f +- %.4f | E[F eps_N]/E[F^2] %+.4f +- %.4f | sum %+.4f\n', ...
                seeds(q), std(nraw(H)) * R * 1e3, bT, 2 * sT, bN, 2 * sN, bT + bN);
        fprintf('          lag j=-4..4 E[F[k] eps_T[k+j]]/E[F^2]: %s\n', sprintf('%+.4f ', lag));
    end
end

function [b, se] = local_ratio(p, F2)
    nb = floor(numel(p) / 50);  m = mean(reshape(p(1:nb * 50), 50, nb), 1) / F2;
    b = mean(m);  se = std(m) / sqrt(nb);
end
