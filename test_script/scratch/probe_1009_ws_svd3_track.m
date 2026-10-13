% STATUS: SCRATCH -- 2026-10-11, 1009_conly_wall_position.tex, descent only (user 10-11: "window = 3 steps, O square"; focus v4, v5, v6).
%   At every step k of the run_1009_ws 'desc' run (seed 1, c and w_s, q55 = P_fix^2 I, q66 = 0), the 6 x 6 matrix
%     O(k) = [H R^-1/2 ; H R^-1/2 F_e[k] ; H R^-1/2 F_e[k+1] F_e[k]],  columns [dx1 dx2 dx3 a c w_s]  (slot 7 = delta = -w_s, sign-flipped)
%   and its SVD O = U S V'. Saved: sigma_1..6, v6, v5, the one-height alias 2(1 - c-hat)/(h_d - w_s-hat) (columns 5 and 6 of the
%   leading-order O(k) are proportional with this factor), the weight of v6 on (c, w_s), and h_d.
%   Output test_results/temp_1009_ws/svd3_<plant>.mat.
function probe_1009_ws_svd3_track(plants)
    if nargin < 1 || isempty(plants); plants = {'plane', 'cell'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    pc = physical_constants();  R = pc.R;  Ts = pc.Ts;
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                'c_init', 0, 'Pf_c_std', 0.10, 'Pf_a_floor', 0.002, 'plaw_seed_cross', true, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, ...
                'plaw_dc_slope', true, 'plaw_p', 2, 'delta_init', 0, 'Pf_delta_std', 0.64, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-0.9, 3], ...
                'obs_dump', true);
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * R, 't_hold', 0, 't_descend_override', 1.0, ...
                'T_sim', 1.2, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, 'lambda_c', 0.4);
    free = [1 2 3 4 5 7];  sgn = [1 1 1 1 1 -1];
    for ip = 1:numel(plants)
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', 1, 'log_P_full', false);
        if strcmp(plants{ip}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
        clear run_formC_b motion_control_law_formC_b;  obs_dump('reset');
        evalc('Rr = run_formC_b(o);');
        L = obs_dump('get');  S1 = Rr.runs{1};  Lz = L([L.ax] == 3);  n = numel(Lz);  ns = size(Lz(1).F, 1);
        wd = S1.h_bar_d_out(:);  hd = nan(n, 1);
        for i = 1:n; k = Lz(i).k + 1; if k >= 1 && k <= numel(wd); hd(i) = wd(k); end; end
        m = n - 2;  sig = nan(m, 6);  v6 = nan(m, 6);  v5 = nan(m, 6);  v4 = nan(m, 6);  alias = nan(m, 1);  nc = nan(m, 1);  orth = nan(m, 1);
        for k0 = 1:m
            Phi = eye(ns);  O = zeros(6, ns);  r = 0;
            for i = k0:(k0 + 2)
                for j = 1:2
                    r = r + 1;  O(r, :) = (Lz(i).H{j} / sqrt(Lz(i).R(j))) * Phi;
                end
                Phi = Lz(i).F * Phi;
            end
            Of = O(:, free) .* sgn;
            [~, S, V] = svd(Of);  sig(k0, :) = diag(S)';
            if V(4, 4) < 0; V(:, 4) = -V(:, 4); end                         % sign convention: a component of v4 positive,
            if V(5, 5) < 0; V(:, 5) = -V(:, 5); end                         %   c component of v5 positive,
            if V(6, 6) < 0; V(:, 6) = -V(:, 6); end                         %   w_s component of v6 positive (SVD fixes v_i up to sign)
            v6(k0, :) = V(:, 6)';  v5(k0, :) = V(:, 5)';  v4(k0, :) = V(:, 4)';
            orth(k0) = max(max(abs(V' * V - eye(6))));                       % V orthogonal: v_i . v_j = 0 (i ~= j), |v_i| = 1
            alias(k0) = 2 * (1 - Lz(k0).x_pred(5)) / (hd(k0) + Lz(k0).x_pred(7));   % h_d - w_s-hat = h_d + delta-hat
            nc(k0) = Lz(k0 + 1).F(4, 5);                                     % F_e(4,5) = -M-hat / h-hat^2 (the c column of row 4)
        end
        t = (1:m)' * Ts;
        res = struct('t', t, 'hd', hd(1:m), 'sig', sig, 'v6', v6, 'v5', v5, 'v4', v4, 'alias', alias, 'Fe45', nc, 'orth', orth, 'plant', plants{ip});
        save(fullfile(od, sprintf('svd3_%s.mat', plants{ip})), '-struct', 'res');
        w6 = sqrt(v6(:, 5).^2 + v6(:, 6).^2);  rat = v6(:, 5) ./ v6(:, 6);  desc = res.hd > 1.1 + 1e-6;
        fprintf('[%s] %d windows | weight of v6 on (c, w_s): min %.4f | v6 c:w_s vs alias, descent: max |diff| %.3f, median |rel diff| %.3f\n', ...
                plants{ip}, m, min(w6), max(abs(rat(desc) - alias(desc))), median(abs(rat(desc) ./ alias(desc) - 1)));
        dx456 = max(max(abs([v4(:, 1:3); v5(:, 1:3); v6(:, 1:3)])));
        fprintf('   max |V''V - I| over all k %.1e | largest dx1..3 component of v4, v5, v6 over all k %.1e\n', max(orth), dx456);
        for hm = [20 10 5 3 2 1.5 1.2]
            q = find(res.hd <= hm, 1);
            fprintf('   h_d %5.2f | sigma4..6 %s | v4 (a,c,w) %s | v5 %s | v6 %s | v6 c:w_s %.3f alias %.3f\n', res.hd(q), sprintf('%9.3g', sig(q, 4:6)), ...
                    sprintf('%+6.3f', v4(q, 4:6)), sprintf('%+6.3f', v5(q, 4:6)), sprintf('%+6.3f', v6(q, 4:6)), rat(q), alias(q));
        end
    end
end
