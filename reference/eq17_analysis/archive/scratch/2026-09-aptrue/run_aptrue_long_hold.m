% FORK OF test_script/scratch/run_aptrue_kr1_full.m (2026-09-03) | PURPOSE: is the near-wall hold
%   drift of the final recipe (exact + @est + pred_mean2 + kr1_full) a transient carried in from the
%   motion segment or a persistent hold source? The level mode pulls back at only ~5e-5/step, so
%   in a 1-2 s hold both look alike; a LONG hold separates them by the slope of the seed mean.
%   Same 10 seeds, final hold extended by 4 s (canon T_sim 4.8 -> 8.8, meng 12.5 -> 16.5); the
%   first part is bit-identical to aptrue_kr1_full_<traj>.mat (right arm), checked here.
%   PRE-REGISTERED (est - true, 10-seed mean):
%     [hypothesis] descent-borne transient: flattens within ~2 s of hold entry near -0.001,
%                  slope over the last 3 s |m| < 0.1e-6/step (canon), < 0.15e-6 (meng)
%     [opponent]   persistent hold source: slope stays ~ -0.25e-6/step (canon) / -0.3e-6 (meng),
%                  i.e. a further -0.0016 / -0.0019 over the extra 4 s
%   recipe 'mean2' (09-03, second pre-registration): the probe shows Cov(a1,u) = -a'' Cov(x3_hat,u) =
%   +0.74e-6/step in hold, cancelling the three second-order feedthrough shares (-0.69), so the
%   truth's net n_w feedthrough is ~0 and kr1/kr1_full over-compensate by a'' (1-lc) K31 R1 per step.
%   PRE-REGISTERED for 'mean2': canon last-3 s slope turns to +0.1..+0.2e-6/step (only the y2 push
%   +0.23 x survival), meng 0..+0.1; opponent: slope stays ~ -0.2 (drift not made by kr1_full).
%   Output: aptrue_long_hold_<traj>[_mean2].mat + aptrue_long_hold_mean_band.png | EXPIRES: with the parent
function out = run_aptrue_long_hold(traj, seeds, recipe)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(recipe); recipe = 'full'; end     % 'full' (pred_mean2 + kr1_full) | 'mean2' (pred_mean2 only, no feedthrough term)
    assert(any(strcmp(recipe, {'full','mean2'})), 'recipe must be full or mean2');
    sfx = '';  if strcmp(recipe, 'mean2'); sfx = '_mean2'; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  EXTRA = 4.0;
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5 + EXTRA,'h_min',2.475);
            cfg0 = OV;
        case 'canon'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  OV = struct('T_sim', cfg0.T_sim + EXTRA);  cfg0.T_sim = cfg0.T_sim + EXTRA;
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    fprintf('[%s long %s] ws0 %.5f | hold from %.2f to %.2f s | seeds %s\n', traj, recipe, ws0, t3, cfg0.T_sim, mat2str(seeds));
    o = struct('arm','best','ap_known',true,'ap_known_at','est','app_known',true, ...
               'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true,'pred_mean2_kr1',strcmp(recipe,'full'),'pred_mean2_kr1_full',strcmp(recipe,'full')), ...
               'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    t = R.runs{1}.tout(:);  N = numel(t);  nS = numel(seeds);
    E = zeros(N, nS); AH = E; P44 = E; HB = E;
    for q = 1:nS
        r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
        AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; P44(:,q) = r.P_a_out(:,3)/ad; HB(:,q) = r.h_bar_true_out(:,1);
    end
    clear R;
    % bit-identity with the short run (first part)
    S = load(fullfile(od, sprintf('aptrue_kr1_full_%s.mat', traj)));  Es = S.right.E;  n0 = size(Es, 1);
    if strcmp(recipe, 'mean2'); S2 = load(fullfile(od, sprintf('aptrue_cmd_vs_est_%s.mat', traj)));  Es = S2.right.E;  n0 = size(Es, 1); end   % arm 4 short run (slope@est, pred_mean2, no kr1)
    fprintf('[%s long] health: min w %.4f | min a_hat %.5f | NaN %d | first %d rows vs short run: max |dE| %.2e\n', ...
        traj, min(HB(:)), min(AH(:)), sum(~isfinite(E(:))), n0, max(abs(E(1:n0,:) - Es), [], 'all'));
    % window means over the hold, 1 s bins, and the slope over the last 3 s (per seed -> SEM)
    edges = t3:1:cfg0.T_sim;  fprintf('[%s long] hold 1 s bins (est - true, mean, SEM):', traj);
    for b = 1:numel(edges)-1
        m = t > edges(b) & t <= edges(b+1);  pm = mean(E(m,:),1);
        fprintf('  [%.1f-%.1f] %+.5f (%.5f)', edges(b), edges(b+1), mean(pm), std(pm)/sqrt(nS));
    end; fprintf('\n');
    m = t > cfg0.T_sim - 3;  k = (1:sum(m)).';
    sl = zeros(1,nS); for q = 1:nS; p = polyfit(k, E(m,q), 1); sl(q) = p(1); end
    fprintf('[%s long] slope over the last 3 s: %+.3f e-6/step (SEM %.3f) | first-hold-second mean %+.5f -> last-second mean %+.5f\n', ...
        traj, 1e6*mean(sl), 1e6*std(sl)/sqrt(nS), mean(E(t>t3 & t<=t3+1,:),'all'), mean(E(t>cfg0.T_sim-1,:),'all'));
    out = struct('traj', traj, 'seeds', seeds, 't', t, 'E', E, 'sdE', std(AH,0,2), 'sP', mean(P44,2), 'phases', [t1 t2 t3 cfg0.T_sim], 'slope', sl);
    out.recipe = recipe;
    save(fullfile(od, sprintf('aptrue_long_hold_%s%s.mat', traj, sfx)), '-struct', 'out', '-v7.3');
    fprintf('[%s long %s] saved aptrue_long_hold_%s%s.mat\n', traj, recipe, traj, sfx);
end
