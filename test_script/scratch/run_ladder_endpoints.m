% FORK OF test_script/scratch/run_aptrue_nw_mcorr_full.m (2026-09-04) | PURPOSE: the four END POINTS of the
%   September oracle ladder at STANDARD run length, same 10 seeds, seed-at-truth init (ws0_perp from the plant's
%   c_perp), both trajectories, one compact schema for plot_ladder_endpoints.m:
%     apcmd   a'_true read at the COMMANDED height, Euler step, no mean term, no correlated-noise predict (the earliest
%             a'_true arm: run_aptrue_seedtruth.m / run_arms_seedtruth_pair.m ARMO{1})
%     btcmd   b_true read at the COMMANDED height, Euler step, no mean term (the earliest b_true@cmd arm;
%             here with the seed-at-truth init so that only the estimator differs between the four figures)
%     apest   a'_true read at the ESTIMATED height + app_known, exact + pred_mean2 + nw_mcorr (the final a'_true recipe,
%             = the nwmcorr arm of run_aptrue_nw_mcorr_full.m)
%     btest   b_true read at the previous step's TRUE height, slope at a_hat, exact + pred_mean2 + nw_mcorr + pred_mean2_e4
%             (the current b_true arm, = the e4 arm of run_btrue_e4.m, standard length)
%     bhat    b_hat ESTIMATED from the 8/9 seed (production, driver arm 'best'), four blocks (= the prod arm of
%             run_prod_ladder.m at standard length); optional fifth figure
%   The four recipe flags are written EXPLICITLY in every arm (production defaults are ON since 2026-09-04 evening).
%   Negative controls (printed, not asserted): apest vs aptrue_nw_mcorr_full_<traj>.mat nwmcorr (bit-identical),
%   btest vs btrue_e4_<traj>.mat e4 over the standard length (bit-identical; that run had the hold extended).
%   Output: ladder_endpoints_<traj>.mat (traj, seeds, t_hold; per arm t, E, AH, AT, HB, hd, B = b the law used, sP5 = sqrt(P55),
%   a_nom [um/pN] for the absolute-units plot) | EXPIRES: with the ladder
%   | 產線改動不會自動跟上
function run_ladder_endpoints(traj, seeds, arms, kappa, tag)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(arms); arms = {'apcmd','btcmd','apest','btest','bhat'}; end
    if nargin < 4 || isempty(kappa); kappa = 1; end        % fe44_Aa_scale: 1 = the pre-kappa covariance (the 09-04 figure set), 0.5 = production since 09-04 evening
    if nargin < 5 || isempty(tag); tag = ''; if kappa ~= 1; tag = sprintf('_k%03d', round(100*kappa)); end; end   % 5th arg: explicit file tag (e.g. '_p0')
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t3 = cfg0.t_hold + cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency;
    switch traj; case 'meng'; p0_floor = 3e-4; otherwise; p0_floor = 1e-5; end
    OFF = struct('law_exact_step',false,'pred_mean2',false,'nw_mcorr',false,'pred_mean2_e4',false,'fe44_Aa_scale',kappa);
    ON3 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',false,'fe44_Aa_scale',kappa);
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',kappa);
    DEF = struct( ...
        'apcmd', struct('o', struct('ap_known',true,'ap_known_at','cmd','app_known',false), 'cc', setfield(setfield(OFF,'lock_b',true),'ws0_perp',ws0)), ...
        'btcmd', struct('o', struct('b_true',true,'b_true_at','cmd'),                        'cc', setfield(OFF,'ws0_perp',ws0)), ...
        'apest', struct('o', struct('ap_known',true,'ap_known_at','est','app_known',true),  'cc', setfield(setfield(ON3,'lock_b',true),'ws0_perp',ws0)), ...
        'btest', struct('o', struct('b_true',true,'b_true_at','true'),                       'cc', setfield(ON4,'ws0_perp',ws0)), ...
        'bhat',  struct('o', struct(),                                                       'cc', setfield(ON4,'ws0_perp',ws0)), ...   % b_hat ESTIMATED from 8/9 (production, arm 'best'), four blocks
        'btp0',  struct('o', struct('b_true',true,'b_true_at','true'),                       'cc', setfield(setfield(setfield(ON4,'ws0_perp',ws0),'Pf_w0_std',0),'Pf_a_floor',p0_floor)), ...
        'bhp0',  struct('o', struct(),                                                       'cc', setfield(setfield(setfield(ON4,'ws0_perp',ws0),'Pf_w0_std',0),'Pf_a_floor',p0_floor)), ...
        'bhq5',  struct('o', struct(),                                                       'cc', setfield(setfield(setfield(setfield(ON4,'ws0_perp',ws0),'Pf_w0_std',0),'Pf_a_floor',p0_floor),'q55_path',true)), ...
        'btq5',  struct('o', struct('b_true',true,'b_true_at','true'),                       'cc', setfield(setfield(setfield(setfield(ON4,'ws0_perp',ws0),'Pf_w0_std',0),'Pf_a_floor',p0_floor),'q55_path',true)), ...
        'bhq5s', struct('o', struct(),                                                       'cc', setfield(setfield(ON4,'ws0_perp',ws0),'q55_path',true)));
        % bhq5 / btq5 / bhq5s (09-07): the Q55 path container (0903 tex S11) on the estimated-b arm with the seed-at-truth P44[0], on the
        % b_true arm (must be bit-identical to btp0: slot 5 locked), and on production's own P44[0].
        % bhp0 (09-07): the PRODUCTION arm (b_hat estimated from the 8/9 seed, four blocks) with the same seed-at-truth prior P44[0]
        % as btp0 -- does the b_true result carry over when b is estimated?
        % btp0 (09-06): btest with the prior P44[0] matched to the seed-at-truth start (sqrt P44[0] 3e-4 Meng / 1e-5 canon instead of
        % 0.0031 / 0.00026) -- the b_true arm's fast-descent spread is that prior amplified by the law's own sensitivity (0903 tex S12).
    fname = fullfile(od, sprintf('ladder_endpoints_%s%s.mat', traj, tag));
    if exist(fname, 'file'); out = load(fname); else; out = struct(); end
    out.traj = traj; out.seeds = seeds; out.t_hold = t3; out.ws0 = ws0; out.kappa = kappa;  nS = numel(seeds);
    fprintf('[%s ladder] ws0 %.5f | hold from %.2f s | seeds %s | arms %s\n', traj, ws0, t3, mat2str(seeds), strjoin(arms, ','));
    for ia = 1:numel(arms)
        A = DEF.(arms{ia});
        o = struct('arm','best','ctrl_const_override',A.cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
        fn = fieldnames(A.o); for i = 1:numel(fn); o.(fn{i}) = A.o.(fn{i}); end
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:);  N = numel(t);  E = zeros(N,nS); AH = E; AT = E; HB = E;  B = E;  P5 = E;
        for q = 1:nS
            r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
            AH(:,q) = r.a_bar_hat_out(:,3); AT(:,q) = r.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q); HB(:,q) = r.h_bar_true_out(:,1);
            B(:,q) = r.b_hat_out(:,3);  P5(:,q) = r.P_b_out(:,3);   % b the law used (locked constant / fed b_true / estimate); P_b_out is sqrt(P55)
        end
        hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  a_nom = R.runs{1}.a_hat_out(1,3)/R.runs{1}.a_bar_hat_out(1,3);  clear R;
        out.(arms{ia}) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'B', B, 'sP5', P5, 'a_nom', a_nom);
        mh = t > t3;  pm = mean(E(mh,:), 1);
        fprintf('[%s %-5s] health: min w %.4f | min a_hat %.5f | NaN %d | hold est-true %+.5f (SEM %.5f) | sigma_seed %.5f | rel. to a(wall) %+.1f%%\n', ...
            traj, arms{ia}, min(HB(:)), min(AH(:)), sum(~isfinite(E(:))), mean(pm), std(pm)/sqrt(nS), mean(std(E(mh,:),0,2)), 100*mean(pm)/mean(AT(mh,:),'all'));
    end
    % negative controls against the existing recipe runs
    f1 = fullfile(od, sprintf('aptrue_nw_mcorr_full_%s.mat', traj));
    if isfield(out, 'apest') && exist(f1, 'file')
        S = load(f1);  fprintf('[%s] negative control apest vs aptrue_nw_mcorr_full nwmcorr: max |dE| %.2e\n', traj, max(abs(out.apest.E - S.nwmcorr.E), [], 'all'));
    end
    f3 = fullfile(od, sprintf('prod_ladder_%s.mat', traj));
    if isfield(out, 'bhat') && exist(f3, 'file')
        S = load(f3);  n = numel(out.bhat.t);  fprintf('[%s] negative control bhat vs prod_ladder prod (first %d steps): max |dE| %.2e\n', traj, n, max(abs(out.bhat.E - S.prod.E(1:n,:)), [], 'all'));
    end
    f2 = fullfile(od, sprintf('btrue_e4_%s.mat', traj));
    if isfield(out, 'btest') && exist(f2, 'file')
        S = load(f2);  n = numel(out.btest.t);  fprintf('[%s] negative control btest vs btrue_e4 e4 (first %d steps): max |dE| %.2e\n', traj, n, max(abs(out.btest.E - S.e4.E(1:n,:)), [], 'all'));
    end
    save(fname, '-struct', 'out', '-v7.3');
    fprintf('[%s ladder] saved ladder_endpoints_%s%s.mat\n', traj, traj, tag);
end
