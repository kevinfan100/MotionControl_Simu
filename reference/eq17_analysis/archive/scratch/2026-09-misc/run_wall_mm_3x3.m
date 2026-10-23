% PURPOSE (2026-09-08): WALL-HYPOTHESIS DISCRIMINATION. Three plants (plane / sphere / cell, as in run_three_walls.m) x three
%   estimator priors, each prior = the production formC_b (four blocks, kappa = 1, narrow family-internal prior sqrt P55[0] 0.039,
%   Pf_w0_std 0.111 R) SEEDED AT ONE WALL's (b, w_s):
%     plane   production as is (b_init 8/9 anchor, ws0_perp 1)
%     sphere  b_init 1.156, ws0_perp 1 + 0.162
%     cell    b_init 0.877, ws0_perp 1 - 1.197
%   Each (plant, prior) runs its own closed loop (the deployable bank would share one loop; the model-mismatch signal is the same
%   to first order -- caveat recorded). Per step the innovation log-likelihood cost of each filter,
%     L(k) = 1/2 [innov1^2/S1 + ln S1] + 1/2 [innov2^2/S2 + ln S2]   (y2 term only while the gate is open),
%   is accumulated; the discrimination statistic is dL_j(t) = Lambda_j(t) - Lambda_right(t) for the two wrong priors j.
%   PRE-REGISTERED (canon 10 seeds; Meng 5 seeds):
%     (P1) on every plant the matching prior has the lowest Lambda by the end of the first descent (canon t 1.5 s, Meng 10.5 s),
%          and dL_wrong exceeds ln 3 = 1.10 before the descent ends (report the first-crossing time, median over seeds).
%     (P2) the derived wall w_wall = w - a_hat/(b_hat (1 - a_hat)) ramps along the descent for a wrong prior and is flat for
%          the right one (report the descent slope, R per R of travel).
%     (P3) a_hat - a_bar of the right prior on the sphere / cell plants is as good as production on the plane (desc / hold).
%     stop-loss: dL_wrong < ln 3 at the end of the descent on any plant => the bank cannot separate within the first traverse.
%   SANITY (rule 13): plane x plane must reproduce three_walls_<traj>.mat plane_prod bit-identically (logging edit + arm check);
%   B(1,:) must print the hypothesis b_init (override wiring); sphere x sphere and cell x cell (exact model) NIS1 ~ 1, hold ~ 0.
%   NOTE: the controller's default b_ceil = 1.05 clamps slot 5 every step (line ~1001 / 1366); a sphere prior at b = 1.156 is
%   unrepresentable under it (first launch: exact-model sphere x sphere hold +0.30, w_wall 0.42). All priors here run with b_ceil 1.5.
%   Output wall_mm_3x3_<traj>.mat | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function out = run_wall_mm_3x3(traj, seeds, walls, priors)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    end
    t1 = cfg0.t_hold; t2 = t1 + cfg0.t_descend_override; t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    WALLS = {'plane', NaN, [];  'sphere', 1.156, 0.162;  'cell', 0.877, -1.197};
    PRI   = {'plane', struct();  'sphere', struct('b_init', 1.156, 'ws0_perp', 1 + 0.162);  'cell', struct('b_init', 0.877, 'ws0_perp', 1 - 1.197)};
    if nargin >= 3 && ~isempty(walls);  WALLS = WALLS(ismember(WALLS(:,1), walls), :); end
    if nargin >= 4 && ~isempty(priors); PRI   = PRI(ismember(PRI(:,1), priors), :); end
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1, 'b_ceil', 1.5);
    nS = numel(seeds);  out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim], 'walls', {WALLS}, 'priors', {PRI(:,1)'});
    ref = []; rf = fullfile(od, sprintf('three_walls_%s.mat', traj)); if exist(rf, 'file'); ref = load(rf, 'plane_prod'); end
    LN3 = log(3); LN100 = log(100);
    for iw = 1:size(WALLS, 1)
        for ip = 1:size(PRI, 1)
            cc = ON4; fn = fieldnames(PRI{ip,2}); for i = 1:numel(fn); cc.(fn{i}) = PRI{ip,2}.(fn{i}); end
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', seeds, 'log_P_full', false);
            if ~isnan(WALLS{iw,2}); o.plant_law_b = WALLS{iw,2}; o.plant_law_w0 = WALLS{iw,3}; end
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            t = R.runs{1}.tout(:); N = numel(t); E = zeros(N,nS); AH = E; AT = E; HB = E; B = E; SP5 = E; WW = E; L1 = E; L2 = E; NIS1 = E; NIS2 = nan(N,nS);
            for q = 1:nS
                r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                AH(:,q) = r.a_bar_hat_out(:,3); AT(:,q) = r.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q); HB(:,q) = r.h_bar_true_out(:,1);
                B(:,q) = r.b_hat_out(:,3); SP5(:,q) = r.P_b_out(:,3);
                WW(:,q) = HB(:,q) - AH(:,q) ./ (B(:,q) .* (1 - AH(:,q)));
                e1 = r.innov_y1_out(:,3); s1 = r.S1_out(:,3); e2 = r.innov_y2_out(:,3); s2 = r.S2_out(:,3);
                l1 = 0.5*(e1.^2./s1 + log(s1)); l1(~isfinite(l1)) = 0;
                l2 = 0.5*(e2.^2./s2 + log(s2)); l2(~isfinite(l2)) = 0;
                L1(:,q) = cumsum(l1); L2(:,q) = cumsum(l2);
                NIS1(:,q) = e1.^2./s1; NIS2(:,q) = e2.^2./s2;
            end
            hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  a_nom = R.runs{1}.a_hat_out(1,3)/R.runs{1}.a_bar_hat_out(1,3);  clear R;
            key = sprintf('%s_%s', WALLS{iw,1}, PRI{ip,1});
            out.(key) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'B', B, 'sP5', SP5, 'W', WW, 'L1', L1, 'L2', L2, 'a_nom', a_nom);
            md = t > t1 & t <= t2; mo = t > t2 & t <= t3; mh = t > t3;
            [~, iwst] = max(abs(mean(E, 2)));
            % descent slope of the derived wall (R per R of travel): regress W on true height over the descent, seed mean
            sl = zeros(1,nS); for q = 1:nS; ok = md & isfinite(WW(:,q)); pp = polyfit(HB(ok,q), WW(ok,q), 1); sl(q) = pp(1); end
            san = '';
            if strcmp(WALLS{iw,1}, 'plane') && strcmp(PRI{ip,1}, 'plane') && ~isempty(ref); san = sprintf(' | SANITY vs three_walls plane_prod max|dE| %.1e', max(abs(E - ref.plane_prod.E(:, 1:min(nS, size(ref.plane_prod.E,2)))), [], 'all')); end
            fprintf('[%s plant %-6s prior %-6s] HEALTH min w %.3f NaN %d floor hits %d | b_hat start %.4f (wired %s) | desc: mean %+.4f sd %.4f worst t=%.2f %+.4f | osc: mean %+.4f sd %.4f | hold: mean %+.5f SEM %.5f sd %.5f | b_hat hold %.3f sqrtP55 %.3f | w_wall hold %.3f (sd %.3f) slope on descent %+.3f (sd %.3f) | NIS1 %.3f NIS2 %.3f%s\n', ...
                traj, WALLS{iw,1}, PRI{ip,1}, min(HB(:)), nnz(isnan(E)), nnz(min(AH,[],1) < 0.0312), mean(B(1,:)), mat2str(isfield(PRI{ip,2},'b_init') && abs(mean(B(1,:)) - PRI{ip,2}.b_init) < 1e-9 || ~isfield(PRI{ip,2},'b_init')), ...
                mean(E(md,:),'all'), mean(std(E(md,:),0,2)), t(iwst), mean(E(iwst,:)), mean(E(mo,:),'all'), mean(std(E(mo,:),0,2)), ...
                mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), mean(B(mh,:),'all'), mean(SP5(end,:)), mean(WW(mh,:),'all'), std(mean(WW(mh,:),1)), mean(sl), std(sl), mean(NIS1(:), 'omitnan'), mean(NIS2(:), 'omitnan'), san);
        end
        % discrimination table for this plant
        right = WALLS{iw,1}; if ~any(strcmp(PRI(:,1), right)); continue; end
        Lr = out.(sprintf('%s_%s', right, right)).L1 + out.(sprintf('%s_%s', right, right)).L2;  t = out.(sprintf('%s_%s', right, right)).t;
        fprintf('  --- plant %s: dL = Lambda_wrong - Lambda_right (per-seed; lower Lambda = better fit) ---\n', right);
        for ip = 1:size(PRI, 1)
            if strcmp(PRI{ip,1}, right); continue; end
            Lw = out.(sprintf('%s_%s', right, PRI{ip,1})).L1 + out.(sprintf('%s_%s', right, PRI{ip,1})).L2;  dL = Lw - Lr;
            at = @(tt) dL(find(t <= tt, 1, 'last'), :);
            tc3 = nan(1,nS); tc100 = nan(1,nS);
            for q = 1:nS; i3 = find(dL(:,q) > LN3 & t > t1, 1); if ~isempty(i3); tc3(q) = t(i3); end; i100 = find(dL(:,q) > LN100 & t > t1, 1); if ~isempty(i100); tc100(q) = t(i100); end; end
            fprintf('  prior %-6s: dL end-of-first-hold median %+7.1f | end-of-descent median %+8.1f [%+.1f, %+.1f] (seeds with dL>ln3: %d/%d) | end-of-osc %+8.1f | end %+8.1f | first t dL>ln3 median %.2f s (never: %d) | dL>ln100 median %.2f s (never: %d)\n', ...
                PRI{ip,1}, median(at(t1)), median(at(t2)), min(at(t2)), max(at(t2)), nnz(at(t2) > LN3), nS, median(at(t3)), median(dL(end,:)), median(tc3, 'omitnan'), nnz(isnan(tc3)), median(tc100, 'omitnan'), nnz(isnan(tc100)));
        end
    end
    tag = ''; if nargin >= 3 && ~isempty(walls); tag = ['_' strjoin(walls, '-')]; end
    save(fullfile(od, sprintf('wall_mm_3x3_%s%s.mat', traj, tag)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved wall_mm_3x3_%s%s.mat\n', traj, traj, tag);
end
