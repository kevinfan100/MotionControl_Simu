% STATUS: ACTIVE (scratch) | PURPOSE: 100-seed verification battery for the two
%   oracle arms on the Meng 10 s descent + 2 s hold scenario (deep band):
%     arm "b_true"  : opts.b_true = 1, b_true_at 'true'   (law runs, b exact)
%     arm "a'_true" : opts.ap_known = 1, ap_known_at 'cmd', lock_b (law bypassed)
%   Same scenario as arm_four_evalpoint_meng.mat (cols 1 and 4), seeds 1:100.
%   Saves COMPACT per-seed z-axis arrays (not full run structs) so the file
%   stays ~150 MB/arm, plus a separate full-P capture (log_P_full) for
%   seeds [7 23] per arm for P-structure analysis (P41 etc.).
%   Attribution target: the L0 residual bias -0.13% (SEM 0.04% at 6 seeds);
%   at 100 seeds SEM ~0.01% -> time profile of the bias becomes resolvable.
%
%   out = run_arm100_btrue_aptrue();          % run everything (~30-60 min)
function out = run_arm100_btrue_aptrue()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');

    SEEDS = 1:100; CHUNK = 20; ax = 3;
    OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10, ...
                'T_sim',12.5,'h_min',2.475);

    ARM = struct();
    ARM(1).tag  = 'btrue';
    ARM(1).name = 'b_true';
    ARM(1).opts = struct('arm','best','b_true',true,'b_true_at','true', ...
                         'config_override',OV,'scenario','deep','verbose',false);
    ARM(2).tag  = 'aptrue';
    ARM(2).name = 'a''_true';
    ARM(2).opts = struct('arm','best','ap_known',true,'ap_known_at','cmd', ...
                         'ctrl_const_override',struct('lock_b',true), ...
                         'config_override',OV,'scenario','deep','verbose',false);

    ZF = {'a_bar_hat_out','P_a_out','P_b_out','K_a_y1_out','K_a_y2_out', ...
          'K_dx_y1_out','K_dx_y2_out','innov_y1_out','innov_y2_out', ...
          'a_prime_out','a_prime_true_out'};   % z column kept per seed
    for a = 1:numel(ARM)
        t0 = tic; nS = numel(SEEDS); first = true;
        for lo = 1:CHUNK:nS
            sd = SEEDS(lo:min(lo+CHUNK-1, nS));
            clear run_formC_b motion_control_law_formC_b;
            o = ARM(a).opts; o.seeds = sd;
            evalc('R = run_formC_b(o);');
            if first
                t = R.runs{1}.tout(:); N = numel(t);
                D = struct('t', t, 'seeds', SEEDS, 'name', ARM(a).name, 'opts', ARM(a).opts, ...
                           'R', R.runs{1}.R, 'hd', R.runs{1}.p_d_out(:,ax)/R.runs{1}.R);
                for f = ZF; D.(f{1}) = zeros(N, nS); end
                D.a_true_norm = zeros(N, nS);   % a_true / a_d  (same scale as a_bar_hat)
                D.h_bar_true  = zeros(N, nS);
                D.ad          = zeros(1, nS);
                D.b_end       = zeros(1, nS);
                first = false;
            end
            for i = 1:numel(sd)
                q = lo + i - 1; r = R.runs{i};
                D.ad(q) = r.a_hat_out(1,ax) / r.a_bar_hat_out(1,ax);
                D.a_true_norm(:,q) = r.a_true_out(:,ax) / D.ad(q);
                D.h_bar_true(:,q)  = r.h_bar_true_out(:,1);
                D.b_end(q)         = r.b_hat_out(end,ax);
                for f = ZF; D.(f{1})(:,q) = r.(f{1})(:,ax); end
            end
            clear R;
            fprintf('[%s] seeds %d-%d done (%.0f s elapsed)\n', ARM(a).tag, sd(1), sd(end), toc(t0));
        end
        fn = fullfile(od, sprintf('arm100_%s.mat', ARM(a).tag));
        save(fn, '-struct', 'D', '-v7.3');
        fprintf('[%s] saved %s (%.0f s total)\n', ARM(a).tag, fn, toc(t0));

        % console summary (descent tail / hold), same windows as the 6-seed pages
        E = D.a_bar_hat_out - D.a_true_norm; t = D.t;
        m1 = t >= 7.5 & t <= 10.5; m2 = t > 10.5;
        for w = {m1, 'nearwall'; m2, 'hold'}'
            m = w{1}; sem = std(mean(E(m,:),1))/sqrt(nS);
            fprintf('[%s] %-8s mean %+.5f SEM %.5f | sd(seeds) %.5f | sqrtP44 %.5f | honesty %.2f\n', ...
                ARM(a).tag, w{2}, mean(E(m,:),'all'), sem, mean(std(E(m,:),0,2)), ...
                mean(mean(D.P_a_out(m,:),2)), mean(std(E(m,:),0,2))/mean(mean(D.P_a_out(m,:),2)));
        end
        clear D E;
    end

    % full-P capture, 2 seeds per arm, for P-structure analysis
    PF = struct();
    for a = 1:numel(ARM)
        clear run_formC_b motion_control_law_formC_b;
        o = ARM(a).opts; o.seeds = [7 23]; o.log_P_full = true;
        evalc('R = run_formC_b(o);');
        PF.(ARM(a).tag) = R;
        fprintf('[%s] full-P capture seeds [7 23] done\n', ARM(a).tag);
    end
    save(fullfile(od, 'arm100_Pfull.mat'), '-struct', 'PF', '-v7.3');
    fprintf('saved %s\n', fullfile(od, 'arm100_Pfull.mat'));
    out = 'done';
end
