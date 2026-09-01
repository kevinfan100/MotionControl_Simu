% STATUS: ACTIVE (scratch) | PURPOSE: both oracle arms with the seed moved onto
%   the full-series truth (the nominal wall enters the seed line only), 30
%   seeds. Question under test: with the init confound removed, does HOLD show
%   bias?
%     arm a'_true@cmd : expected zero (verified 2026-08-31, run_aptrue_seedtruth)
%     arm b_true      : if hold bias ~+0.8% persists, the law push is confirmed
%                       independent of the seed level.
%
%   TWO TRAJECTORIES, same battery (2026-09-01):
%     'meng'  (default) Meng 10 s cosine descent + 2 s hold, NO oscillation
%                       (amplitude 0); w_bar 6.667 -> 1.111, T = 12.5 s
%     'canon'           canonical deep, 1 Hz x 2 cycles oscillation on the band
%                       [1.10, 3.32]; w_bar 22.22 -> 1.10, T = 4.8 s
%   Both END holding at the trough, and the two troughs agree to 1% in w_bar,
%   so the HOLD window is comparable across the two; the descent/oscillation
%   windows are not.
%
%   THE SEED ANCHOR IS TRAJECTORY-DEPENDENT AND IS RECOMPUTED HERE. ws0_perp is
%   what makes "seed = truth" true: it solves
%       a_bar_seed = 1 - 1/[(8/9)(w_bar0 - (ws0_perp - 1))] = 1/c_perp(w_bar0)
%   at THAT trajectory's start height, so it may not be copied across the two
%   (meng w_bar0 = 6.667 -> 0.94366 ; canon w_bar0 = 22.222 -> 0.98099). The
%   canon value sits much closer to the plane default 1 for a physical reason:
%   the run starts in the far field, where the anchor law is already almost
%   exact, so on 'canon' the init confound this arm exists to remove is small
%   to begin with.
function out = run_arms_seedtruth_pair(traj, do_pfull, exact)
    if nargin < 1 || isempty(traj);      traj = 'meng';    end
    if nargin < 2 || isempty(do_pfull);  do_pfull = false; end
    if nargin < 3 || isempty(exact);     exact = false;    end   % law_exact_step both arms (incl ap_known ext, 09-01)
    traj = lower(traj);
    assert(any(strcmp(traj, {'meng','canon'})), 'traj must be meng|canon.');
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();

    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10, ...
                        'T_sim',12.5,'h_min',2.475);
            cfg0  = OV;
            fname = 'arms30_seedtruth_pair.mat';
            pname = 'arms30_seedtruth_Pfull.mat';
        case 'canon'
            OV    = struct();                       % pure canonical deep, no override
            cfg0  = canonical_scenario(0.05, 1.1, 'deep');
            fname = 'arms30_seedtruth_pair_canon.mat';
            pname = 'arms30_seedtruth_Pfull_canon.mat';
    end
    % seed = truth at THIS trajectory's start height
    w0bar = cfg0.h_init / pc.R;
    [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    % phase boundaries of the commanded trajectory (trajectory_generator)
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    fprintf('[%s] w_bar0 %.4f -> a_true %.5f -> ws0_perp %.5f | phases: hold<%.2f descend<%.2f osc<%.2f hold<=%.2f\n', ...
            traj, w0bar, at, ws0, t1, t2, t3, cfg0.T_sim);

    SEEDS = 1:30;
    ARMO = { ...
      struct('arm','best','ap_known',true,'ap_known_at','cmd', ...
             'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0), ...
             'config_override',OV,'scenario','deep','verbose',false,'seeds',SEEDS,'log_P_full',true), ...
      struct('arm','best','b_true',true,'b_true_at','true', ...
             'ctrl_const_override',struct('ws0_perp',ws0), ...
             'config_override',OV,'scenario','deep','verbose',false,'seeds',SEEDS,'log_P_full',true)};
    if exact
        for a = 1:2; ARMO{a}.ctrl_const_override.law_exact_step = true; end
        fname = strrep(fname, '.mat', '_exact.mat');
        pname = strrep(pname, '.mat', '_exact.mat');
        fprintf('[%s] law_exact_step ON (both arms, ap_known extension active)\n', traj);
    end
    TAG = {'aptrue','btrue'};
    out = struct('ws0', ws0, 'traj', traj, 'phases', [t1 t2 t3 cfg0.T_sim], 'exact', exact);
    for a = 1:2
        clear run_formC_b motion_control_law_formC_b;
        LOG = evalc('R = run_formC_b(ARMO{a});');
        t = R.runs{1}.tout(:); nS = numel(SEEDS);
        % full compact schema (= arm100 files + what the law-error-budget
        % verification reads: K/innov both channels, P_a, a_prime, heights)
        ZF = {'a_bar_hat_out','P_a_out','P_b_out','K_a_y1_out','K_a_y2_out', ...
              'K_dx_y1_out','K_dx_y2_out','innov_y1_out','innov_y2_out', ...
              'a_prime_out','a_prime_true_out','delta_x_hat_3_out'};
        E = zeros(numel(t), nS); AH = E; P44 = E; L41 = E; L42 = E;
        C = struct(); for f = ZF; C.(f{1}) = zeros(numel(t), nS); end
        C.a_true_norm = zeros(numel(t), nS); C.h_bar_true = C.a_true_norm; C.trk_true = C.a_true_norm; C.P33_sqrt = C.a_true_norm; C.ad = zeros(1, nS);
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad;
            P44(:,q) = r.P_a_out(:,3)/ad;
            for f = ZF; C.(f{1})(:,q) = r.(f{1})(:,3); end
            C.a_true_norm(:,q) = r.a_true_out(:,3)/ad; C.h_bar_true(:,q) = r.h_bar_true_out(:,1); C.ad(q) = ad;
            C.trk_true(:,q) = (r.p_d_out(:,3) - r.p_true_out(:,3)) / r.R;   % true tracking error [R]
            C.P33_sqrt(:,q) = sqrt(squeeze(r.P_full_out(:,3,3,3)));   % per-seed sqrt(P33), z
            R.runs{q}.P_full_out = [];   % drop the big matrix after extraction
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;
        % --- HEALTH FIRST (canonical-trajectory rule 3): wiring before RMS ---
        fprintf('[%s %s] health: min w_bar_true %.4f | min a_bar_hat %.5f | NaN in E %d | E(t=0) %+.2e (seed=truth => ~0)\n', ...
                traj, TAG{a}, min(C.h_bar_true(:)), min(AH(:)), sum(~isfinite(E(:))), mean(E(1,:)));
        ln = strsplit(LOG, newline);
        hit = find(~cellfun(@isempty, regexp(ln, 'scenario:|floor_a|P44\[0\]|b_true on the envelope|WARN|warning|clamp', 'once')));
        for ii = hit(1:min(6,numel(hit))); fprintf('    | %s\n', strtrim(ln{ii})); end
        % --- windows: same PHYSICAL phases on both trajectories ---
        W = {'descend', t> t1 & t<=t2; 'osc/flat', t> t2 & t<=t3; 'hold', t> t3};
        if strcmp(traj,'meng'); W(end+1,:) = {'legacy nearwall', t>=7.5 & t<=10.5}; end
        for w = 1:size(W,1)
            m = W{w,2}; pm = mean(E(m,:),1);
            fprintf('[%s %s] %-16s E %+.5f (SEM %.5f)\n', traj, TAG{a}, W{w,1}, mean(pm), std(pm)/sqrt(nS));
        end
        out.(TAG{a}) = struct('t',t,'E',E,'sdE',std(AH,0,2),'sP',mean(P44,2),'hd',hd,'L41',L41,'L42',L42,'C',C);
        out.(TAG{a}).L41 = C.K_a_y1_out; out.(TAG{a}).L42 = C.K_a_y2_out;
        clear R;
    end
    S = out; save(fullfile(od,fname), '-struct', 'S', '-v7.3');
    fprintf('saved %s\n', fname);
    if ~do_pfull; return; end
    % full-P capture, 2 seeds per arm, for the P41 line of the later checks
    PF = struct();
    for a = 1:2
        clear run_formC_b motion_control_law_formC_b;
        o = ARMO{a}; o.seeds = [7 23]; o.log_P_full = true;
        evalc('R = run_formC_b(o);'); PF.(TAG{a}) = R;
        fprintf('[%s] full-P seeds [7 23] done\n', TAG{a});
    end
    save(fullfile(od,pname), '-struct', 'PF', '-v7.3');
end
