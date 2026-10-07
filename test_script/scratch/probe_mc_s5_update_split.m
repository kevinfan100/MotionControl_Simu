% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 5 explanation (user: why is the estimate so much steadier with a'?).
%   Fig. 5.1 runs (Section 2 trajectory, plane, lambda_c 0.4), seeds 1-5. Split each step's change of the gain estimate:
%     d a_hat[k] = a_hat[k] - a_hat[k-1] = (prediction: a_hat' M_hat, 0 in the first-order model) + (correction: l41 e_x1 + l42 e_am)
%   correction from the logged gain row and innovations; prediction = d a_hat - correction.
%   Instrument check: in the first-order model the prediction must be ~0 (correction = the whole step).
%   Reports per window (far 1-3 s, near 4.5-5 s): sqrt(P44), mean |l41|, |l42|, rms of the correction per step (noise injected),
%   and the share of the net change of a_hat carried by the prediction vs the correction.
function probe_mc_s5_update_split()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    R1 = run_mc_s4_first_order({'prod_xyz'}, 1:5, {'plane'});
    S = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_plane_PI_mcs5_desc_ramp5_lc040.mat'));
    W = {[1 3], [4.5 5]};  WN = {'far 1-3 s', 'near 4.5-5 s'};
    for est = 1:2
        for s = 1:5
            if est == 1
                r = R1.prod_xyz.plane(s);  t = r.t;  a = r.a_hat;  K1 = r.K_a_y1;  K2 = r.K_a_y2;  i1 = r.innov1;  i2 = r.innov2;  sP = sqrt(r.P_a);
            else
                t = S.t;  a = S.a_hat(:, s);  K1 = S.K_a_y1(:, s);  K2 = S.K_a_y2(:, s);  i1 = S.innov1(:, s);  i2 = S.innov2(:, s);  sP = S.sP_a(:, s);
            end
            da = [0; diff(a)];  corr = K1 .* i1 + K2 .* i2;  corr(~isfinite(corr)) = 0;  pred = da - corr;
            for w = 1:2
                m = t > W{w}(1) & t <= W{w}(2);
                T(est, s, w, :) = [mean(sP(m)), mean(abs(K1(m))), mean(abs(K2(m))), rms(corr(m)), rms(pred(m)), sum(pred(m)), sum(corr(m)), sum(da(m))]; %#ok<AGROW>
            end
        end
    end
    NM = {'first order', 'c-only'};
    for est = 1:2
        for w = 1:2
            v = squeeze(mean(T(est, :, w, :), 2));
            fprintf('%-11s %-13s | sqrtP44 %.4f | |l41| %.3g | |l42| %.3g | rms corr/step %.2e | rms pred/step %.2e | net change %+.3f = pred %+.3f + corr %+.3f\n', ...
                    NM{est}, WN{w}, v(1), v(2), v(3), v(4), v(5), v(8), v(6), v(7));
        end
    end
end
