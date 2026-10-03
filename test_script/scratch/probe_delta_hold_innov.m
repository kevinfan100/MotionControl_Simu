% PURPOSE (2026-09-20, after the iterated y2 update changed nothing, O38): is the slow crawl of delta_hat in the near-wall
%   hold an UNDER-GAINED filter (y2 innovation mean = (1 - S) e_a, the reading is right and P is too small) or a BIASED
%   reading (innovation mean ~ 0 while e_a != 0, the reading agrees with the wrong estimate)? Cell wall, plane init, canon
%   deep hold (t >= 3.5 s), 10 seeds; also the [1/1] P arm and the delta T arm for reference.
%   Console only. EXPIRES: with O38.
function probe_delta_hold_innov()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    D = load(fullfile(root, 'test_results', 'temp_delta_two_walls', 'cell_PI.mat'));
    E = load(fullfile(root, 'test_results', 'temp_11_two_walls', 'cell_A_P.mat'));
    T = load(fullfile(root, 'test_results', 'temp_delta_two_walls', 'cell_T.mat'));
    S_ECHO = 0.32;                                                 % y2 echo share: innovation mean should be (1 - S) e_a if the reading is unbiased
    for c = {{'delta PI', D}, {'[1/1] P', E}, {'delta T', T}}
        r = c{1}{2};  t = r.t;  hm = t >= r.seg(3);  ea = r.a_true - r.a_hat;
        in2 = r.innov2(hm, :);  S2 = r.S2(hm, :);  n = numel(in2);
        e_h = mean(ea(hm, :), 'all');  m_in = mean(in2, 'all');
        fprintf('%-9s hold: e_a %+.4f | innov2 mean %+.5f (SEM %.5f) sd %.4f | sqrt(S2) %.4f | NIS2 %.3f | innov2 mean / ((1-S) e_a) = %.2f\n', ...
                c{1}{1}, e_h, m_in, std(in2, 0, 'all') / sqrt(n), std(in2, 0, 'all'), mean(sqrt(S2), 'all'), mean(in2.^2 ./ S2, 'all'), m_in / ((1 - S_ECHO) * e_h));
    end
    d = D.dl_hat;  sp = D.sP_dl;  t = D.t;  k = find(t >= D.seg(3));
    fprintf('delta PI hold: delta_hat %.3f -> %.3f (target %.2f) | sqrtP %.3f -> %.3f | 1/P %.0f -> %.0f\n', ...
            mean(d(k(1), :)), mean(d(k(end), :)), D.dl_target, mean(sp(k(1), :)), mean(sp(k(end), :)), 1 / mean(sp(k(1), :))^2, 1 / mean(sp(k(end), :))^2);
    % descent-end snapshot: how much of the innovation the filter actually applies to delta per step in the hold
    Kd = (d(k(end), :) - d(k(1), :)) ./ sum(D.innov2(k, :), 1);
    fprintf('delta PI hold: mean per-step delta move per unit innovation %.4f R (= effective K7); with H = 0.68*0.21 = %.3f the consistent K would be P*H/S2 = %.4f\n', ...
            mean(Kd), 0.68 * 0.21, mean(sp(k, :), 'all')^2 * 0.68 * 0.21 / mean(D.S2(k, :), 'all'));
end
