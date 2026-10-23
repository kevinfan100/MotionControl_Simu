% STATUS: SCRATCH -- 2026-10-16: closed-loop difference slope (run_formC_b opts.ap_diff) -- is it self-locked?
%   Splits the change of a_hat over the descent into the predict share (sum of fed slope x commanded step) and the
%   update share (the rest), and compares sqrt(P44) and K_a_y1 with the true-slope arm (same config, ap_known at the command).
%   Plane, seed 1, descent 22.2 -> 1.10 R in 1 s (run_mc_apslot_compare 'desc').
function probe_apdiff_selflock(N)
    if nargin < 1; N = 100; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    pc = physical_constants();  R = pc.R;
    OV = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.10 * R, 't_hold', 0, ...
                't_descend_override', 1.0, 'T_sim', 1.2, 'lambda_c', 0.4);
    cc = struct('lock_b', true);
    arms = {sprintf('diff%d', N), 'trueslope'};
    fprintf('arm       | t [s] | h_d [R] | a_true | a_hat | slope fed / true | dA pred | dA upd | sqrt P44 | K_a_y1\n');
    for ia = 1:1 + (N == 100)
        o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', ...
                   'verbose', false, 'seeds', 1, 'log_P_full', true);
        if ia == 1; o.ap_diff = struct('N', N, 'gate', 1e-3, 's0', 1 / 22.2^2);
        else; o.ap_known = true;  o.ap_known_at = 'cmd'; end
        clear run_formC_b motion_control_law_formC_b;
        evalc('Rr = run_formC_b(o);');  S = Rr.runs{1};
        hd = S.h_bar_d_out(:);  ah = S.a_bar_hat_out(:, 3);  n = numel(hd);
        hb = max(S.h_bar_true_out(:), 1.001);  at = zeros(n, 1);  apt = zeros(n, 1);
        for i = 1:n; [~, cp] = calc_correction_functions(hb(i)); at(i) = 1 / cp; end
        for i = 2:n-1; apt(i) = (at(i+1) - at(i-1)) / (hb(i+1) - hb(i-1)); end
        % fed slope, rebuilt exactly as the driver does (difference arm) or the true slope at the command (true arm)
        s = NaN(n, 1);  sk = 1 / 22.2^2;
        for k = 1:n
            if ia == 1 && k > N + 1
                dh = hd(k-1) - hd(k-1-N);
                if abs(dh) > 1e-3; sa = (ah(k-1) - ah(k-1-N)) / dh; if isfinite(sa) && sa > 0; sk = sa; end; end
            elseif ia == 2 && k > 1
                [~, cp1] = calc_correction_functions(max(hd(k-1) + 1e-4, 1.001)); [~, cp0] = calc_correction_functions(max(hd(k-1) - 1e-4, 1.001));
                sk = (1 / cp1 - 1 / cp0) / 2e-4;
            end
            s(k) = sk;
        end
        dpred = [0; s(2:end) .* diff(hd)];  dall = [0; diff(ah)];  dupd = dall - dpred;
        P44 = squeeze(S.P_full_out(:, 4, 4, 3));  Ky1 = S.K_a_y1_out(:, 3);
        t = (0:n-1)' * pc.Ts;
        for tc = [0.3 0.5 0.7 0.8 0.9 0.95 1.0 1.1]
            k = round(tc / pc.Ts) + 1;  w = 10:k;   % skip the first-sample command jump
            fprintf('%-9s | %4.2f  | %6.3f  | %5.3f  | %5.3f | %7.4f / %7.4f | %+6.3f | %+6.3f | %8.2e | %7.3f\n', arms{ia}, t(k), hd(k), at(k), ah(k), ...
                    s(k), apt(k), sum(dpred(w)), sum(dupd(w)), sqrt(P44(k)), Ky1(k));
        end
    end
end
