% STATUS: SCRATCH -- 2026-10-11, 1009_conly_wall_position.tex, descent only. SVD of the whitened observability matrix with all six
%   states together (user 10-11: "each step H Phi is 2 x 6, fix the window so O is square").
%   O_N(k0) = [H_j Phi(k0 + i, k0) / sqrt(R_j)], i = 0..N-1, j = 1, 2  ->  2N x 6, columns [dx1 dx2 dx3 a c w_s] (w_s = -delta: the
%   delta column of formC_b slot 7 is sign-flipped). O_N = U S V'; S has 6 singular values, V is 6 x 6 for any N (N = 3: O square).
%   Printed per window: sigma_1..6, the last two right singular vectors v5, v6, the c : w_s ratio of v6 against the one-height alias
%   2 (1 - c-hat) / (h_d - w_s-hat), and the 2 x 2 Schur block of (c, w_s) (others unknown) for comparison with sigma_5, sigma_6.
%   Negative control: the inert slot 6 appended as a 7th column must give one extra sigma = 0 with v = that slot.
%   Run: run_1009_ws 'desc' settings with obs_dump, seed 1.
function probe_1009_ws_svd6(plants, HC)
    if nargin < 1 || isempty(plants); plants = {'cell'}; end
    if nargin < 2 || isempty(HC); HC = [10 3 1.5]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    pc = physical_constants();  R = pc.R;
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'ws0_perp', 0, 'Pf_w0_std', 0, ...
                'c_init', 0, 'Pf_c_std', 0.10, 'Pf_a_floor', 0.002, 'plaw_seed_cross', true, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, ...
                'plaw_dc_slope', true, 'plaw_p', 2, 'delta_init', 0, 'Pf_delta_std', 0.64, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-0.9, 3], ...
                'obs_dump', true);
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * R, 't_hold', 0, 't_descend_override', 1.0, ...
                'T_sim', 2.0, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, 'lambda_c', 0.4);
    free = [1 2 3 4 5 7];  sgn = [1 1 1 1 1 -1];                       % slot 7 = delta = -w_s
    NAMES = {'dx1', 'dx2', 'dx3', 'a', 'c', 'w_s'};
    for ip = 1:numel(plants)
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', 1, 'log_P_full', false);
        if strcmp(plants{ip}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
        clear run_formC_b motion_control_law_formC_b;  obs_dump('reset');
        evalc('Rr = run_formC_b(o);');
        L = obs_dump('get');  S1 = Rr.runs{1};  Lz = L([L.ax] == 3);  n = numel(Lz);  ns = size(Lz(1).F, 1);
        wd = S1.h_bar_d_out(:);  hd = nan(n, 1);
        for i = 1:n; k = Lz(i).k + 1; if k >= 1 && k <= numel(wd); hd(i) = wd(k); end; end
        fprintf('\n================ %s ================  columns: %s\n', plants{ip}, strjoin(NAMES, ' '));
        iend = find(hd <= min(hd) + 1e-9, 1);
        W = {};
        for h = HC
            ic = find(hd <= h, 1);
            W{end + 1} = struct('k0', ic - 1, 'N', 3, 'lab', sprintf('N = 3 (square) at h_d %.2f', hd(ic)));   %#ok<AGROW>
            W{end + 1} = struct('k0', max(1, ic - 100), 'N', 200, 'lab', sprintf('N = 200 centred at h_d %.2f', hd(ic)));   %#ok<AGROW>
        end
        W{end + 1} = struct('k0', 1, 'N', iend, 'lab', sprintf('whole descent, N = %d', iend));
        for q = 1:numel(W)
            w = W{q};  k1 = w.k0 + w.N - 1;
            O = local_O(Lz, w.k0, k1, ns);
            Of = O(:, free) .* sgn;  On = O(:, [free 6]) .* [sgn 1];
            [~, S, V] = svd(Of, 'econ');  s = diag(S);
            sn = svd(On);
            im = w.k0 + floor(w.N / 2);  ch = Lz(im).x_pred(5);  dl = Lz(im).x_pred(7);  alias = 2 * (1 - ch) / (hd(im) + dl);
            G = Of' * Of;  io = 1:4;  it = 5:6;
            Gs = G(it, it) - G(it, io) * (G(io, io) \ G(io, it));  ss = sqrt(sort(max(eig(0.5 * (Gs + Gs')), 0), 'descend'));
            fprintf('\n%s  (O is %d x 6; c-hat %+.3f, w_s-hat %+.3f)\n', w.lab, size(Of, 1), ch, -dl);
            fprintf('  sigma 1..6 : %s\n', sprintf('%10.3g', s));
            fprintf('  v5         : %s\n', sprintf('%+8.3f', V(:, 5)));
            fprintf('  v6         : %s\n', sprintf('%+8.3f', V(:, 6)));
            fprintf('  v6 c : w_s = %.3f   | one-height alias 2(1-c)/(h - w_s) = %.3f\n', V(5, 6) / V(6, 6), alias);
            fprintf('  Schur 2x2 (c, w_s), others unknown: sqrt eig = %.3g, %.3g   (compare sigma_5, sigma_6 = %.3g, %.3g)\n', ss(1), ss(2), s(5), s(6));
            fprintf('  negative control (inert slot appended): smallest sigma %.3g (next %.3g)\n', sn(end), sn(end - 1));
        end
    end
end

function O = local_O(Lz, k0, k1, ns)
    Phi = eye(ns);  O = zeros(0, ns);
    for i = k0:k1
        for j = 1:2
            Hj = Lz(i).H{j};  if isempty(Hj); continue; end
            O(end + 1, :) = (Hj / sqrt(Lz(i).R(j))) * Phi;               %#ok<AGROW>
        end
        Phi = Lz(i).F * Phi;
    end
end
