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
%   09-08 SPLIT ARMS (the unknown wall = two unknowns of different type; each arm opens ONE of them; all three carry b_ceil 1.5
%   because the controller default 1.05 pinned the 09-07 wide arm's sphere b_hat -- see hypotheses R54):
%     widew   only the wall position opened: Pf_w0_std 1.0 R, b prior as production (0.039)   -- level-type unknown
%     wideb   only b opened:                 Pf_b_std 0.15, Pf_w0_std 0.111 (production)     -- shape-type unknown
%     wide15  both opened (the 09-07 wide arm rerun with b_ceil 1.5)
%   PRE-REGISTERED (canon 10 seeds / Meng 5 seeds, motion segment first):
%     (a) cell plant: widew recovers (descent / hold as wide15), wideb does not (hold stays ~ -0.08).
%     (b) sphere plant: wideb recovers (b_hat -> ~1.16, descent / hold ~ 0), widew does not (descent -0.02, hold +0.016 stay).
%     (c) plane plant: widew pays the near-wall descent spread (P44[0] amplified by the law), wideb pays little.
%   09-08 'bseed' ARM (the user's cell: start fully known + wall position known, c(h) unknown). With a_bar_0 at the start height
%   and the contact height w_c (where a_bar = 0) both known, the constant-b law has NO freedom left:
%       b_0 = (1/(1 - a_bar_0) - 1) / (w_0 - w_c),   ws0_perp = 1 + w_c - 1/b_0
%   so the only unknown left is how b varies along the path (the curvature of c). Production prior otherwise (sqrt P55[0] 0.039,
%   Pf_w0_std 0.111 R, b_ceil 1.5). PRE-REGISTERED: all three plants at the matching-prior floor (canon hold |E| < 0.005,
%   descent mean |E| < 0.01); sphere / cell reproduce their 3x3 diagonals within seed noise (b_0 = 1.156 / 0.877 exactly).
%   09-09 FORGETTING ON THE CHORD SEED (user: 'can a forgetting factor let b keep absorbing current information?'):
%     bseed0_lf999 / _lf9999  fixed slot-5 forgetting lambda_f_b (Menq (4.15) applied to slot 5 only), tau 0.6 s / 6 s
%     bseed0_lfa              adaptive: lambda = exp(-0.05 (NIS2 - 1)_+), floor 0.99 -- forgets only when y2 is surprised
%   on the 'plane' (b nearly constant) and the 'ramp' plant (b 1.16 -> 0.87 along the height). PRE-REGISTERED: on the plane every
%   forgetting arm pays spread (08-10 verdict, Q55 precedent) and buys nothing; on the ramp the fixed arms track b_hat toward the local
%   b (1.16 near the wall) and cut the near-wall error vs bseed0, the adaptive arm sits between. DECISION: forgetting earns a place only
%   if its ramp gain (paired vs bseed0, near-wall worst instant and osc mean) exceeds its plane loss (paired sigma, hold) -- else R57.
%   09-09 B-PER-BIN ARM bseed0_bins (4 height bins, controller flag b_bins_on; chord seed, P44[0] at the start truth).
%   PRE-REGISTERED (user approved 09-09, plane, no-hold): (1) first descent unchanged vs bseed0 (no data yet, any method equal);
%   (2) canon 2nd/3rd trough worst instant from -0.007 to <= -0.003 and mid-band revisit from 0.003 to <= 0.001;
%   (3) the near-wall bin's b_hat lands in 0.92 +- 0.02 and the mid-band bin in 0.87 +- 0.02 (= 'b follows the red curve');
%   (4) spread not worse than 1.5x bseed0; Meng (single traverse) only required not to get worse.
%   Fail any -> this closes too and only 'feed the physical curve' remains.
%   09-09 P55 FLOOR ARM bseed0_pf (directional forgetting in steady-state form, controller flag p55_floor_on, floor = Pf_b_std):
%   PRE-REGISTERED (user approved 09-09): (1) Meng plane: sqrt P55 stays 0.039 (no windup), desc / hold not worse than bseed0 (paired);
%   (2) canon plane near-wall worst instant <= -0.006 (what lambda 0.9999 reached) with hold within SEM of bseed0;
%   (3) ramp: b_hat near the wall >= 1.05 and both osc mean and hold |error| smaller than bseed0 on both trajectories.
%   All three or the forgetting family closes (R58) and the line moves to CUSUM + per-wall map.
%   Output three_walls_<traj>[_<walls>_<arms>].mat | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function out = run_three_walls(traj, seeds, arms, walls)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(arms); arms = {'prod','wide','lockb'}; end
    traj = lower(traj);  nohold = endsWith(traj, '_nohold'); traj_base = strrep(traj, '_nohold', '');   % 09-09: '<traj>_nohold' = no initial hold, run ends when the motion ends
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    switch traj_base
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    end
    if nohold
        OV.t_hold = 0;  cfg0.t_hold = 0;
        if strcmp(traj_base, 'meng'); OV.T_sim = cfg0.t_descend_override; else; OV.T_sim = cfg0.t_descend_override + cfg0.n_cycles/cfg0.frequency; end   % Meng: end at the bottom; canon: end after the last cycle
        cfg0.T_sim = OV.T_sim;
    end
    t1 = cfg0.t_hold; t2 = t1 + cfg0.t_descend_override; t3 = min(t2 + cfg0.n_cycles/cfg0.frequency, cfg0.T_sim);
    WALLS = {'plane', NaN, [], 0;  'sphere', 1.156, 0.162, 0;  'cell', 0.877, -1.197, 0;  'cellb', NaN, [], 1.03;  'ramp', [1.16 0.87 2.0 0.3], 0, 0};
    % 'ramp' (09-09): the driver's 4-vector plant [b_wall b_far w_c Delta] -- b changes ALONG THE HEIGHT (1.16 near the wall -> 0.87 far,
    % logistic step at 2.0 R, width 0.3 R): the 1-D proxy for 'the wall's property changes as the probe moves'. Contact (B = 1) found by fzero.
    % 4th column = shift [R]: 'cellb' = the published Brenner plane curve shifted DOWN by 1.03 R (the 09-07 fit: cell law origin -1.197 vs
    % plane -0.168), i.e. a cell whose no-slip surface sits 1.03 R below its visible top and whose b(w) is the plane's CURVE, not a constant.
    ARM = struct('prod',  struct('arm','best', 'cc', struct(), 'o', struct()), ...
                 'wide',  struct('arm','best', 'cc', struct('Pf_b_std', 0.15, 'Pf_w0_std', 1.0), 'o', struct()), ...
                 'lockb', struct('arm','bmid', 'cc', struct(), 'o', struct()), ...
                 'widew', struct('arm','best', 'cc', struct('Pf_w0_std', 1.0, 'b_ceil', 1.5), 'o', struct()), ...
                 'wideb', struct('arm','best', 'cc', struct('Pf_b_std', 0.15, 'b_ceil', 1.5), 'o', struct()), ...
                 'wide15', struct('arm','best', 'cc', struct('Pf_b_std', 0.15, 'Pf_w0_std', 1.0, 'b_ceil', 1.5), 'o', struct()), ...
                 'bseed', struct('arm','best', 'cc', struct('b_ceil', 1.5), 'o', struct()), ...   % b_init / ws0_perp filled per wall below
                 'btseed', struct('arm','best', 'cc', struct('b_ceil', 1.5), 'o', struct('b_true', true, 'b_true_at', 'true')), ...   % same seeds as bseed, but the law reads the PLANT's local b(w) (b_true arm): the b(w) ceiling in the user's cell
                 'bseed0',  struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0), 'o', struct()), ...                                   % bseed + P44[0] at the start truth (start FULLY known: Pf_a_floor set per traj below)
                 'bseed0_bins', struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'b_bins_on', true), 'o', struct()), ...   % 09-09 user-approved: b per height bin (4 bins), chord seed, P44[0] at the start truth
                 'bloc0',       struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0), 'o', struct()), ...   % 09-09 user: a_hat[0] AND b_hat[0] at the truth (local b_true(w_0)), then ESTIMATE (P55[0] = family prior 0.039)
                 'bloc0_ptiny', struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'Pf_b_std', 1e-3), 'o', struct()), ...   % 09-09 user check: b_hat[0] = LOCAL b_true(w_0) (not the chord), P55[0] tiny (b 'fully cheated')
                 'bseed0_ptiny', struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'Pf_b_std', 1e-3), 'o', struct()), ...  % chord seed with P55[0] tiny (b frozen at the chord)
                 'bseed0_pf',     struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'p55_floor_on', true), 'o', struct()), ...   % 09-09 directional forgetting as a P55 floor at the family prior (0.039): never more certain about b than the band's own b variation
                 'bseed0_lf999',  struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'lambda_f_b', 0.999), 'o', struct()), ...    % 09-09 forgetting on slot 5: fixed, tau 1000 steps (0.6 s)
                 'bseed0_lf9999', struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'lambda_f_b', 0.9999), 'o', struct()), ...   % fixed, tau 6 s
                 'bseed0_lfa',    struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'lambda_f_b_alpha', 0.05, 'lambda_f_b_floor', 0.99), 'o', struct()), ...   % adaptive: forget only when NIS2 excess > 0
                 'bhq5c', struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'q55_path', true), 'o', struct()), ...   % O25 re-verification on the CHORD seed: Q55 path container
                 'bhn1c', struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'l51_off', true), 'o', struct()), ...    % O25: E1 (b_hat fed by y2 only)
                 'bhdc',  struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0, 'da_slot', true), 'o', struct()), ...    % O25: E2 (da slot)
                 'prod_wc',     struct('arm','best', 'cc', struct('ws0_perp', 1 + 1.0 - 1/(8/9)), 'o', struct()), ...   % 09-08: production priors, seed LINE anchored at the contact height (a_bar = 0 at w = 1) instead of w0 = 0 (zero at 1.125)
                 'rep_bhp0',    struct('arm','best', 'cc', struct('Pf_w0_std', 0), 'o', struct()), ...                 % 09-08 discriminator: the ladder bhp0 arm rebuilt here (b_init 8/9, ws0 = ladder seed-at-truth line, b_ceil default 1.05)
                 'rep_bhp0_c15',struct('arm','best', 'cc', struct('Pf_w0_std', 0, 'b_ceil', 1.5), 'o', struct()), ...   % same + b_ceil 1.5
                 'bseed0_c105', struct('arm','best', 'cc', struct('Pf_w0_std', 0), 'o', struct()), ...                 % bseed0 with b_ceil default 1.05
                 'mix_b89_wc',  struct('arm','best', 'cc', struct('Pf_w0_std', 0, 'b_ceil', 1.5), 'o', struct()), ...   % b_init 8/9 with the CONTACT-anchored origin
                 'mix_bch_w99', struct('arm','best', 'cc', struct('Pf_w0_std', 0, 'b_ceil', 1.5), 'o', struct()), ...   % chord b_0 with the ladder's ws0 line
                 'btseed0', struct('arm','best', 'cc', struct('b_ceil', 1.5, 'Pf_w0_std', 0), 'o', struct('b_true', true, 'b_true_at', 'true')), ...
                 'btrue', struct('arm','best', 'cc', struct(), 'o', struct('b_true', true, 'b_true_at', 'true')), ...          % law exactly right for THIS plant (b_true from the plant curve), slope at a_hat
                 'apest', struct('arm','best', 'cc', struct('lock_b', true), 'o', struct('ap_known', true, 'ap_known_at', 'est', 'app_known', true)));   % slope fed from the plant curve
    if nargin >= 4 && ~isempty(walls); WALLS = WALLS(ismember(WALLS(:,1), walls), :); end
    ON4 = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1);
    nS = numel(seeds);  out = struct('traj', traj, 'seeds', seeds, 'phases', [t1 t2 t3 cfg0.T_sim], 'walls', {WALLS});
    for iw = 1:size(WALLS, 1)
        for ia = 1:numel(arms)
            A = ARM.(arms{ia});  cc = ON4;  fn = fieldnames(A.cc); for i = 1:numel(fn); cc.(fn{i}) = A.cc.(fn{i}); end
            if any(strcmp(arms{ia}, {'bseed0','btseed0','rep_bhp0','rep_bhp0_c15','bseed0_c105','mix_b89_wc','mix_bch_w99','bhq5c','bhn1c','bhdc','bseed0_lf999','bseed0_lf9999','bseed0_lfa','bseed0_pf','bloc0_ptiny','bseed0_ptiny','bloc0','bseed0_bins'})); if strcmp(traj, 'meng'); cc.Pf_a_floor = 3e-4; else; cc.Pf_a_floor = 1e-5; end; end   % ladder p0 convention (09-06)
            if any(strcmp(arms{ia}, {'rep_bhp0','rep_bhp0_c15','bseed0_c105','mix_b89_wc','mix_bch_w99'}))
                pc = physical_constants(); w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar, true); a0 = 1/cp;
                ws0_lad = 1 + w0bar - 1/((8/9)*(1 - a0));                          % the ladder's seed-at-truth line (b 8/9 through the start gain)
                b_ch = (1/(1 - a0) - 1) / (w0bar - 1.0);  ws0_ch = 1 + 1.0 - 1/b_ch;   % the chord line (through start gain and contact 1.0)
                switch arms{ia}
                    case {'rep_bhp0','rep_bhp0_c15'}; cc.b_init = 8/9;  cc.ws0_perp = ws0_lad;
                    case 'bseed0_c105';               cc.b_init = b_ch; cc.ws0_perp = ws0_ch;
                    case 'mix_b89_wc';                cc.b_init = 8/9;  cc.ws0_perp = 1 + 1.0 - 1/(8/9);
                    case 'mix_bch_w99';               cc.b_init = b_ch; cc.ws0_perp = ws0_lad;
                end
                fprintf('[%s %s %s] b_init %.4f ws0_perp %.4f (b_ceil %s)\n', traj, WALLS{iw,1}, arms{ia}, cc.b_init, cc.ws0_perp, mat2str(isfield(cc,'b_ceil')));
            end
            if any(strcmp(arms{ia}, {'bseed','btseed','bseed0','btseed0','bhq5c','bhn1c','bhdc','bseed0_lf999','bseed0_lf9999','bseed0_lfa','bseed0_pf','bloc0_ptiny','bseed0_ptiny','bloc0','bseed0_bins'}))
                pc = physical_constants(); w0bar = cfg0.h_init / pc.R;
                if ~isscalar(WALLS{iw,2}); pl = WALLS{iw,2}; w0p = WALLS{iw,3}; a0 = 1 - 1/local_B(w0bar - w0p, pl); w_c = w0p + fzero(@(u) local_B(u, pl) - 1, 0.9);
                elseif isnan(WALLS{iw,2}); [~, cp] = calc_correction_functions(w0bar + WALLS{iw,4}, true); a0 = 1/cp; w_c = 1.0 - WALLS{iw,4};
                else; a0 = 1 - 1/(WALLS{iw,2} * (w0bar - WALLS{iw,3})); w_c = WALLS{iw,3} + 1/WALLS{iw,2}; end
                cc.b_init = (1/(1 - a0) - 1) / (w0bar - w_c);  cc.ws0_perp = 1 + w_c - 1/cc.b_init;
                fprintf('[%s %s %s] a_bar_0 %.4f at w %.3f, contact %.3f => b_0 %.4f, ws0_perp %.4f\n', traj, WALLS{iw,1}, arms{ia}, a0, w0bar, w_c, cc.b_init, cc.ws0_perp);
                if any(strcmp(arms{ia}, {'bloc0_ptiny','bloc0'}))
                    [~, cpl, ddl] = calc_correction_functions(w0bar + WALLS{iw,4}, true); b_loc = (-ddl.dc_perp_dh / cpl^2) / (1 - 1/cpl)^2;   % local b_true at the start height
                    cc.b_init = b_loc; cc.ws0_perp = 1 + w0bar - 1/(b_loc * (1 - a0));                                                  % same start level, LOCAL slope
                    fprintf('[%s %s %s] LOCAL b_true(w_0) %.4f, ws0_perp %.4f, seed line zero at %.3f R\n', traj, WALLS{iw,1}, arms{ia}, b_loc, cc.ws0_perp, cc.ws0_perp - 1 + 1/b_loc);
                end
            end
            o = struct('arm', A.arm, 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', seeds, 'log_P_full', false);
            fo = fieldnames(A.o); for i = 1:numel(fo); o.(fo{i}) = A.o.(fo{i}); end
            if ~(isscalar(WALLS{iw,2}) && isnan(WALLS{iw,2})); o.plant_law_b = WALLS{iw,2}; o.plant_law_w0 = WALLS{iw,3}; end
            if isscalar(WALLS{iw,2}) && isnan(WALLS{iw,2}) && WALLS{iw,4} ~= 0; sh = WALLS{iw,4}; o.plant_cperp = @(hb) local_cperp_shifted(hb, sh); end
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
            if isscalar(WALLS{iw,2}) && ~isnan(WALLS{iw,2}); at_law = 1 - 1 ./ max(WALLS{iw,2} * (HB - WALLS{iw,3}), 1.05); dev = max(abs(AT - at_law), [], 'all'); else; dev = NaN; end
            key = sprintf('%s_%s', WALLS{iw,1}, arms{ia});
            out.(key) = struct('t', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'hd', hd, 'B', B, 'sP5', SP5, 'sP', SP, 'W', WW, 'a_nom', a_nom);
            md = t > t1 & t <= t2; mo = t > t2 & t <= t3; mh = t > t3; m0 = t <= t1;
            if ~any(mo); mo(end) = true; end; if ~any(mh); mh(end) = true; end; if ~any(m0); m0(1) = true; end   % nohold: keep the printf alive on empty segments (values then = last sample)
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

function cp = local_cperp_shifted(hb, shift)
%LOCAL_CPERP_SHIFTED  Brenner plane c_perp evaluated at hb + shift (the no-slip surface is `shift` R below the nominal wall).
    cp = zeros(size(hb));
    for i = 1:numel(hb); [~, cp(i)] = calc_correction_functions(hb(i) + shift, true); end
end

function B = local_B(u, pl)
%LOCAL_B  antiderivative of the driver's logistic b(w): B(u) = b_wall u + (b_far - b_wall) De [sp((u - w_c)/De) - sp(-w_c/De)],  sp(x) = log(1 + e^x).
    b_wall = pl(1); b_far = pl(2); w_c = pl(3); De = pl(4);
    sp = @(x) max(x, 0) + log1p(exp(-abs(x)));
    B = b_wall * u + (b_far - b_wall) * De * (sp((u - w_c)/De) - sp(-w_c/De));
end
