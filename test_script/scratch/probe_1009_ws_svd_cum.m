% STATUS: SCRATCH -- 2026-10-12, 1009_conly_wall_position.tex, descent only (user: "what if we use the whole descent, not small windows?").
%   Same run as probe_1009_ws_svd3_track.m (run_1009_ws 'desc', seed 1, c and w_s, obs_dump), same whitening (rows / sqrt(r)).
%   Instead of a 3-step window at every k0, the window starts at k = 1 and grows: O(1 -> k) = [H Phi(i, 1) / sqrt(R)], i = 1..k,
%   evaluated at every k through G(k) = O' O (6 x 6; eig(G) = sigma^2, eigenvectors = V). Columns [dx1 dx2 dx3 a c w_s]
%   (slot 7 = delta = -w_s, sign-flipped). Vectors picked by content (the a-direction becomes the LARGEST sigma once y2 readings
%   accumulate): 'v4' = largest a weight, 'v5' / 'v6' = the two largest (c, w_s) weights, larger / smaller sigma; sig(:, 4:6) = their sigmas,
%   idx = their positions in the sorted list. Signs as the 3-step figure (a > 0, c > 0, w_s > 0). Where sigma(a) is within 5 % of
%   the dx sigmas (~0.28 s) the a-direction is the most a-like vector of that equal-sigma group (user 10-12: the dip was an artefact).
%   Output test_results/temp_1009_ws/svdcum_<plant>.mat (same fields as svd3_<plant>.mat).
function probe_1009_ws_svd_cum(plants)
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
    free = [1 2 3 4 5 7];  D = diag([1 1 1 1 1 -1]);  EQ_TOL = 0.05;   % relative gap below which two singular values count as equal
    for ip = 1:numel(plants)
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', 1, 'log_P_full', false);
        if strcmp(plants{ip}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
        clear run_formC_b motion_control_law_formC_b;  obs_dump('reset');
        evalc('Rr = run_formC_b(o);');
        L = obs_dump('get');  S1 = Rr.runs{1};  Lz = L([L.ax] == 3);  n = numel(Lz);  ns = size(Lz(1).F, 1);
        wd = S1.h_bar_d_out(:);  hd = nan(n, 1);
        for i = 1:n; k = Lz(i).k + 1; if k >= 1 && k <= numel(wd); hd(i) = wd(k); end; end
        sig = nan(n, 6);  idx = nan(n, 3);  v4 = nan(n, 6);  v5 = nan(n, 6);  v6 = nan(n, 6);  orth = nan(n, 1);
        Phi = eye(ns);  G = zeros(ns);
        P0 = D * Lz(1).P_pred(free, free) * D;  P0i = inv(0.5 * (P0 + P0'));      % the filter's prior at the first step, (c, w_s) sign
        [bnd_c, bnd_w, flt_c, flt_w, r_flt, r_cum] = deal(nan(n, 1));
        for k = 1:n
            for j = 1:2
                M = (Lz(k).H{j} / sqrt(Lz(k).R(j))) * Phi;  G = G + M' * M;
            end
            Phi = Lz(k).F * Phi;
            Gf = D * G(free, free) * D;  Gf = 0.5 * (Gf + Gf');
            [V, E] = eig(Gf);  [e, ord] = sort(max(real(diag(E)), 0), 'descend');  V = V(:, ord);  sg = sqrt(e');
            % pick by content, not by index: the a-direction grows past the dx singular values (680) as y2 readings accumulate
            [~, ia] = max(V(4, :).^2);  wcw = V(5, :).^2 + V(6, :).^2;  wcw(ia) = -1;  [~, icw] = sort(wcw, 'descend');  icw = sort(icw(1:2));
            va = V(:, ia);  vb = V(:, icw(1));  vc = V(:, icw(2));                 % icw sorted => vb has the larger sigma
            cl = setdiff(find(abs(sg / sg(ia) - 1) < EQ_TOL), icw);            % singular values equal to sigma(a) within EQ_TOL: inside such a group
            if numel(cl) > 1                                                    %   any mix is a valid singular vector (the a-direction crossing the dx
                Vc = V(:, cl);  va = Vc * (Vc' * [0; 0; 0; 1; 0; 0]);  va = va / norm(va);   %   sigmas 680 at ~0.28 s); take the most a-like one
            end
            if va(4) < 0; va = -va; end;  if vb(5) < 0; vb = -vb; end;  if vc(6) < 0; vc = -vc; end
            sig(k, :) = [sg(1:3), sg(ia), sg(icw(1)), sg(icw(2))];  idx(k, :) = [ia, icw];
            v4(k, :) = va';  v5(k, :) = vb';  v6(k, :) = vc';  orth(k) = max(max(abs(V' * V - eye(6))));
            B = inv(P0i + Gf);  bnd_c(k) = sqrt(B(5, 5));  bnd_w(k) = sqrt(B(6, 6));      % prior + data of the cumulative window, no Q
            Pu = D * Lz(k).P_upd(free, free) * D;  flt_c(k) = sqrt(Pu(5, 5));  flt_w(k) = sqrt(Pu(6, 6));
            [W2, E2] = eig(0.5 * (Pu(5:6, 5:6) + Pu(5:6, 5:6)'));  [~, jl] = max(diag(E2));  r_flt(k) = W2(1, jl) / W2(2, jl);   % filter long axis dc/dw_s
            r_cum(k) = vc(5) / vc(6);
        end
        t = (1:n)' * Ts;  alias = nan(n, 1);
        res = struct('t', t, 'hd', hd, 'sig', sig, 'idx', idx, 'bnd_c', bnd_c, 'bnd_w', bnd_w, 'flt_c', flt_c, 'flt_w', flt_w, 'r_flt', r_flt, 'r_cum', r_cum, 'sP0', sqrt(diag(P0))', 'v6', v6, 'v5', v5, 'v4', v4, 'alias', alias, 'orth', orth, 'plant', plants{ip});
        save(fullfile(od, sprintf('svdcum_%s.mat', plants{ip})), '-struct', 'res');
        iend = find(hd <= min(hd) + 1e-9, 1);
        fprintf('[%s] cumulative window from t = 0 | max |V''V - I| %.1e | largest dx1..3 component of v4..v6 at the end of the descent %.1e\n', ...
                plants{ip}, max(orth), max(max(abs([v4(iend, 1:3); v5(iend, 1:3); v6(iend, 1:3)]))));
        fprintf('   prior sqrt P0 (c, w_s) = %.3f, %.3f\n   h_d    t     | filter sqrtP_w  cumulative bound sqrtP_w | filter sqrtP_c  bound sqrtP_c | filter long axis dc/dw_s  cumulative v6 dc/dw_s\n', sqrt(P0(5, 5)), sqrt(P0(6, 6)));
        for hm = [20 15 10 8 6 5 4 3 2 1.5 1.2 1.1]
            q = find(hd <= hm, 1);  if hm == 1.1; q = iend; end
            fprintf('  %5.2f  %.3f | %.3f           %.3f                    | %.3f           %.3f          | %.3f                    %.3f\n', hd(q), t(q), flt_w(q), bnd_w(q), flt_c(q), bnd_c(q), r_flt(q), r_cum(q));
        end
        for hm = [20 10 5 3 2 1.5 1.2 1.1]
            q = find(hd <= hm, 1);  if hm == 1.1; q = iend; end
            fprintf('   up to h_d %5.2f (t %.3f) | idx %s | sigma a/5/6 %s | v4 (a,c,w) %s | v5 %s | v6 %s | 1/sigma5 %.3g 1/sigma6 %.3g\n', hd(q), t(q), mat2str(idx(q, :)), ...
                    sprintf('%9.3g', sig(q, 4:6)), sprintf('%+6.3f', v4(q, 4:6)), sprintf('%+6.3f', v5(q, 4:6)), sprintf('%+6.3f', v6(q, 4:6)), 1 / sig(q, 5), 1 / sig(q, 6));
        end
    end
end
