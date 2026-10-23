% STATUS: ACTIVE (scratch) | PURPOSE: E = l41 + a' l31 (1 - l42 gamma) from the logged Kalman gains,
%   per segment, 10 seeds -- the residual of the rank-1 cancellation l41 ~ -a' l31 that the hold
%   level-mode equation (0902 tex, last section) says sets the hold drift. Also delta = E/(a' l31) =
%   the fractional deviation of P41/P31 from -a'. RESULT (2026-09-03): E = 0 to 1e-4 on both
%   trajectories, every segment -- the filter's gains satisfy l41 = -a' l31 EXACTLY, so the x3_hat-driven
%   flows cancel exactly in the filter's mean dynamics and the hold drift is not a P41/P31 imbalance.
%   Reads aptrue_kr1_full_<traj>.mat (gains are recipe-independent).
function probe_aptrue_E_from_log()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    lc = 0.7; al = 1 - lc; a_cov = 0.05; S = 0.32; gam = a_cov*(1-S); R1 = (0.00331/2.25)^2;
    for tr = {'canon','meng'}
        D = load(fullfile(od, sprintf('aptrue_kr1_full_%s.mat', tr{1})));  C = D.right.C;  t = D.right.t;  nS = size(C.K_a_y1_out, 2);  t3 = D.phases(3); t2 = D.phases(2);
        ap = C.a_prime_out ./ C.ad;                  % dabar/dwbar
        l31 = C.K_dx_y1_out; l41 = C.K_a_y1_out; l42 = C.K_a_y2_out;
        E = l41 + ap .* l31 .* (1 - l42*gam);  delta = E ./ (ap .* l31);
        switch tr{1}
            case 'canon'; SEG = {'osc 1.5-3.5', t>1.5&t<=3.5; 'hold', t>t3};
            case 'meng';  SEG = {'near 8-10.5', t>8&t<=t2; 'hold', t>t2};
        end
        fprintf('\n[%s] per segment (seed mean, SEM over %d seeds):\n', tr{1}, nS);
        fprintf('   %-12s %8s %8s %8s %9s %9s %9s\n', 'segment', 'l31', 'l41', 'a''', 'E', '(SEM)', 'delta');
        for si = 1:size(SEG,1)
            m = SEG{si,2};
            pe = mean(E(m,:),1); pd = mean(delta(m,:),1);
            fprintf('   %-12s %8.4f %8.4f %8.4f %+9.4f %9.4f %+9.3f\n', SEG{si,1}, mean(l31(m,:),'all'), mean(l41(m,:),'all'), mean(ap(m,:),'all'), mean(pe), std(pe)/sqrt(nS), mean(pd));
        end
        % E per step is identically zero to 1e-4: l41 = -a' l31 exactly (the filter's P41 = -a' P31, rank-1
        % law-consistent Q and F_e). The (3,4)-model hypothesis of a few-% imbalance is falsified here.
        fprintf('   max |E| over the run: %.2e ; max |delta|: %.2e\n', max(abs(E(:))), max(abs(delta(isfinite(delta)))));
        fprintf('   compare: measured drift/flow ratio ~ 0.03-0.05 (knob test); E here %+.4f, delta %+.3f\n', mean(mean(E(SEG{end,2},:),1)), mean(mean(delta(SEG{end,2},:),1)));
    end
end
