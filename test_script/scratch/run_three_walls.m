% PURPOSE (2026-09-07): the UNKNOWN-WALL test. The plant follows one of three walls fitted from Meng's measured curves
%   (IEEE 11072733 Fig 11, digitised 09-07; 1/(1-a) = b (w - w_s), rms 0.002-0.006 over 1.1-5.5 R):
%     plane   published Brenner polynomial (the fit gives b 0.870, w_s -0.168; Brenner itself rms 0.0016)
%     sphere  b 1.156, w_s +0.162   (wall where expected, weaker wall effect: b_B = 1/b = 0.87 vs the plane anchor 9/8)
%     cell    b 0.877, w_s -1.197   (same b as the plane, the no-slip wall 1 R BELOW the apparent cell top)
%   The estimator does not know which wall it meets. Three estimator arms (formC_b, four blocks, kappa = 1, production seed
%   from the 9/8 anchor -- NOT seed-at-truth):
%     prod    production as is: b_hat estimated with the plane prior (sqrt P55[0] 0.039), Pf_w0_std 0.111 R
%     wide    the same with the priors opened to the wall family: Pf_b_std 0.15, Pf_w0_std 1.0 R
%     lockb   b locked at 8/9 (arm bmid), production priors
%   PRE-REGISTERED (10 seeds, canonical deep; Meng ramp for prod / wide):
%     (1) sphere: wide's b_hat moves from ~0.89 toward 1.16 (the 0.29 signal is ~6x the 08-17 CRLB 0.05); its descent transient of
%         a is smaller than lockb's; the derived wall w_wall = w - a/(b(1-a)) -> 1.03 in hold.
%     (2) cell: b_hat stays ~0.88 in every arm; w_wall -> ~0; after the first hold (y2 reads the +0.023 seed error) a is normal.
%     (3) plane: wide's b_hat stays; hold level as production; only the near-wall spread grows with P44[0] -- report how much.
%     (4) opponent: with the wide prior b_hat wanders on the plane (sqrt P55 does not shrink) => the scheme fails, b from geometry.
%   Health first: min w_true > 1.0, no NaN, a_hat floor hits, and the truth log a_true_out must follow the plant curve (checked
%   against the law at the true height, max |dev|).
%   Output three_walls_<traj>.mat | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function out = run_three_walls(traj, seeds, arms, walls)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(arms); arms = {'prod','wide','lockb'}; end
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
    ARM = struct('prod',  struct('arm','best', 'cc', struct(), 'o', struct()), ...
                 'wide',  struct('arm','best', 'cc', struct('Pf_b_std', 0.15, 'Pf_w0_std', 1.0), 'o', struct()), ...
                 'lockb', struct('arm','bmid', 'cc', struct(), 'o', struct()), ...
                 'btrue', struct('arm','best', 'cc', struct(), 'o', struct('b_true', true, 'b_true_at', 'true')), ...          % law exactly right for THIS plant (b_true from the plant curve), slope at a_hat
                 'apest', struct('arm','best', 'cc', struct('lock_b', true), 'o', struct('ap_known', true, 'ap_known_at', 'est', 'app_known', true)));   % slope fed from the plant curve
    if nargin >= 4 && ~isempty(walls); WALLS = WALLS(ismember(WALLS(:,1), walls), :); end
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1);
    nS = numel(seeds);  out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim], 'walls', {WALLS});
    for iw = 1:size(WALLS, 1)
        for ia = 1:numel(arms)
            A = ARM.(arms{ia});  cc = ON4;  fn = fieldnames(A.cc); for i = 1:numel(fn); cc.(fn{i}) = A.cc.(fn{i}); end
            o = struct('arm', A.arm, 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', seeds, 'log_P_full', false);
            fo = fieldnames(A.o); for i = 1:numel(fo); o.(fo{i}) = A.o.(fo{i}); end
            if ~isnan(WALLS{iw,2}); o.plant_law_b = WALLS{iw,2}; o.plant_law_w0 = WALLS{iw,3}; end
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            t = R.runs{1}.tout(:); N = numel(t); E = zeros(N,nS); AH = E; AT = E; HB = E; B = E; SP5 = E; SP = E; WW = E;
            for q = 1:nS
                r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                AH(:,q) = r.a_bar_hat_out(:,3); AT(:,q) = r.a_true_out(:,3)/ad; E(:,q) = AH(:,q) - AT(:,q); HB(:,q) = r.h_bar_true_out(:,1);
                B(:,q) = r.b_hat_out(:,3); SP5(:,q) = r.P_b_out(:,3); SP(:,q) = r.P_a_out(:,3)/ad;
                WW(:,q) = HB(:,q) - AH(:,q) ./ (B(:,q) .* (1 - AH(:,q)));     % derived wall position from (true height, a_hat, b_hat)
            end
            hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;  a_nom = R.runs{1}.a_hat_out(1,3)/R.runs{1}.a_bar_hat_out(1,3);  clear R;
            % truth-log check against the plant law at the true height
            if ~isnan(WALLS{iw,2}); at_law = 1 - 1 ./ max(WALLS{iw,2} * (HB - WALLS{iw,3}), 1.05); dev = max(abs(AT - at_law), [], 'all'); else; dev = NaN; end
            key = sprintf('%s_%s', WALLS{iw,1}, arms{ia});
            out.(key) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'B', B, 'sP5', SP5, 'sP', SP, 'W', WW, 'a_nom', a_nom);
            md = t > t1 & t <= t2; mo = t > t2 & t <= t3; mh = t > t3; m0 = t <= t1;
            [~, iwst] = max(abs(mean(E, 2)));
            fprintf('[%s %-6s %-5s] HEALTH min w %.3f NaN %d min a_hat %.4f floor hits %d truth-vs-law %.1e | E first hold %+.4f -> end of first hold %+.4f | desc: mean %+.4f sd %.4f worst t=%.2f %+.4f | osc: mean %+.4f sd %.4f | hold: mean %+.5f SEM %.5f sd %.5f | b_hat: start %.3f desc-end %.3f hold %.3f sqrtP55 end %.3f | w_wall hold %.3f (sd %.3f) | a_true hold %.3f\n', ...
                traj, WALLS{iw,1}, arms{ia}, min(HB(:)), nnz(isnan(E)), min(AH(:)), nnz(min(AH,[],1) < 0.0312), dev, mean(E(1,:)), mean(E(find(m0,1,'last'),:)), ...
                mean(E(md,:),'all'), mean(std(E(md,:),0,2)), t(iwst), mean(E(iwst,:)), mean(E(mo,:),'all'), mean(std(E(mo,:),0,2)), ...
                mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), mean(B(1,:)), mean(B(find(md,1,'last'),:)), mean(B(mh,:),'all'), mean(SP5(end,:)), mean(WW(mh,:),'all'), std(mean(WW(mh,:),1)), mean(AT(mh,:),'all'));
        end
    end
    tag = ''; if nargin >= 4 && ~isempty(walls); tag = ['_' strjoin(walls, '-') '_' strjoin(arms, '-')]; end
    save(fullfile(od, sprintf('three_walls_%s%s.mat', traj, tag)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved three_walls_%s%s.mat\n', traj, traj, tag);
end
