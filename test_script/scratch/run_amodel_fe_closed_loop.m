% PURPOSE (2026-09-11, step 4 of the green plan): closed loop of ctrl_const.amodel_fe (the green variant, 0911_estb_green_variant.tex)
%   against production on the SAME cell as steps 1-3: plane (Brenner) plant, chord seed, P44[0] at the start truth
%   (= run_three_walls arm 'bseed0': b_ceil 1.5, Pf_w0_std 0, Pf_a_floor 1e-5 canon / 3e-4 Meng, b_init = chord, ws0_perp = 2 - 1/b_init),
%   four production blocks explicit. Arms: orig (production), green (amodel_fe) and orange (amodel_fe + amodel_h), same
%   seeds (paired). The report compares one arm against orig; pass the arm name as the third argument of 'report'.
%   Windows: near wall = canon descent (0.5, 1.5] s, Meng (7, 10.5] s; second motion = canon osc (1.5, 3.5], Meng far (0.5, 7];
%   hold = after the motion (canon > 3.5, Meng > 10.5). E = a_hat - a_true (both / a_nom).
%   PRE-REGISTERED (written before the run):
%     [hypothesis] green shrinks sqrt(P55) (step 3, seed 7: canon 1.37x, Meng 1.03x) but b_hat is pulled by the part of the gain
%       error the a model does not explain (r = a(w_hat, b_hat) - a_hat, nonzero after y2 moves a_hat alone) => b_hat seed spread
%       grows beyond sqrt(P55) and the near-wall gain error is not better.
%     [opponent] benign: r stays small (the exact-step predict keeps a_hat on the curve), b_hat honest, motion error equal or better.
%     Separation = this run. Readouts, motion first:
%       M1 near-wall window: E mean, sigma_seed (time mean of the across-seed sd), worst instant of the seed-mean E;
%          paired per-seed worst |E|: green - orig, mean +- SEM
%       M2 second motion window: E mean, sigma_seed
%       M3 b: b_hat end mean +- sd, sqrt(P55) end, honesty_b = sd_seed(b_hat) / mean sqrt(P55) at the end of the near-wall
%          window and at the run end; paired b_hat end green - orig
%       M4 a honesty in the near-wall window: time mean of sd_seed(E) / mean_seed sqrt(P44)  (added after step 3: y1 no
%          longer informs a_w in green but the filter's sqrt(P44) did not grow)
%       M5 NIS1, NIS2 per window (production design: NIS1 ~ 1, NIS2 ~ 0.3)
%       M6 hold (last): E mean, SEM, sd
%     STOP-LOSS (either => green closes, ledger entry, no tuning): S1 honesty_b(green) > 2 at the near-wall end or the run end;
%       S2 paired per-seed worst |E| in the near-wall window, green - orig > 2 SEM.
%     WORTH KEEPING only if: near-wall sigma_seed or worst improved (paired, > 2 SEM), S1/S2 not triggered, NIS1 within 10% of orig.
%   HEALTH (checked first, before any readout): min w_true > 1.0, no NaN, a_hat floor hits, b_hat inside (0.5, 1.5) and ceiling
%     hits; orig must reproduce the saved 09-09 bseed0 plane run bit for bit on the shared seeds (canon 1:10, Meng 1:5); seed 7's
%     sqrt(P55)[end] must match step 3 (canon 0.00871 / 0.00634, Meng 0.01490 / 0.01443).
%   WALL OPTION (2026-09-14): pass the arm as '<arm>@cell' to put the plant on the Fig-11 cell law (b_true 0.877,
%   w_s,true -1.197, the 09-07 digitised fit) AND seed the estimator with those exact numbers. The plant is then a member
%   of the estimator's own law family and the init is exact, so there is NO model error left: b_true is a single constant
%   and "does b_hat track b_true" finally has a well-posed answer. Files get the _cell suffix; the bit-identity health
%   check against the saved plane run is skipped there.
%   PRE-REGISTERED for the 4state arm (2026-09-14, written before the run; derivation 0914_estb_4state_core.tex):
%     [hypothesis] removing the gain slot removes the buffer that absorbed r (the part of the gain error the law does not
%       explain, measured 09-12: 5-44% of the gain error while moving, 100% in hold). Motion should be unchanged or
%       slightly better, b_hat should MOVE (it owns the strongest direction now), and HOLD should get worse because r has
%       nowhere to go but b_hat. sqrt(P55) will shrink further and honesty_b should rise.
%     [opponent] the buffer was never doing anything: hold unchanged, b_hat honest, motion unchanged.
%     Extra stop-loss for this arm: any seed drives b_hat into the guard, or NIS1 moves more than 10% from the orig arm
%       (the y1 model is untouched, so NIS1 moving means a wiring error, not physics).
%   PRE-REGISTERED for the orange arm (2026-09-12, written before the run):
%     [hypothesis] the y2 substitution moves the gain reading onto b as well, so sqrt(P55) shrinks further (step 3, seed 7:
%       canon 0.0048, Meng 0.0049) and b_hat absorbs what a_w's error used to explain; with the a slot formally
%       unobservable (gate 4 rank 6 of 7, CRLB(a_w) = Inf) a_hat is corrected only through P_41, P_43, P_45, so the
%       near-wall gain error is no better and b_hat drifts / turns overconfident (honesty_b up).
%     [opponent] benign: r stays small, y2's information reaches a_hat through P_45 just as well, near-wall equal or
%       better, honesty_b < 2.
%     Same readouts M1-M6 and the same stop-losses S1 / S2 as the green arm, orange against orig.
%   Usage: run_amodel_fe_closed_loop('canon', 1:10, 'orig'); ... 'green'; ... 'orange'; ... '4state';
%          then run_amodel_fe_closed_loop('canon', [], 'report') or (..., 'report:orange')
%   Output test_results/apd_acov_meng/amodel_fe_cl_<traj>_<arm>.mat | EXPIRES: with the green verdict | 產線改動不會自動跟上
function out = run_amodel_fe_closed_loop(traj, seeds, arm)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(arm); arm = 'report'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    W = local_windows(traj);
    wall = 'plane';                                    % '<arm>@cell' puts the plant on the Fig-11 cell law with an exact init
    if contains(arm, '@'); wall = extractAfter(arm, '@');  arm = extractBefore(arm, '@'); end
    if startsWith(arm, 'report')
        cmp = 'green';  if contains(arm, ':'); cmp = extractAfter(arm, ':'); end
        out = local_report(od, traj, W, cmp, wall);  return;
    end
    B_CELL = 0.877;  WS_CELL = -1.197;                 % 09-07 digitised fit of Meng Fig 11, cell curve
    pc = physical_constants();
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                        't_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);
            h_init = OV.h_init;  p0 = 3e-4;
        otherwise
            OV = struct();  c0 = canonical_scenario(0.05, 1.1, 'deep');  h_init = c0.h_init;  p0 = 1e-5;
    end
    w0 = h_init / pc.R;  [~, cp] = calc_correction_functions(w0, true);  a0 = 1/cp;  w_c = 1.0;
    if startsWith(wall, 'cell') && ~startsWith(wall, 'cellmis'); a0 = 1 - 1/(B_CELL*(w0 - WS_CELL));  w_c = WS_CELL + 1/B_CELL; end
    cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1, ...
                'b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0);
    cc.b_init = (1/(1 - a0) - 1) / (w0 - w_c);  cc.ws0_perp = 1 + w_c - 1/cc.b_init;
    if strcmp(wall, 'cell')
        cc.b_init = B_CELL;  cc.ws0_perp = WS_CELL + 1;    % exact init: the estimator's law IS the plant's law
    end
    % 'cellmis': the plant is the cell law but the estimator keeps the PLANE chord seed (contact assumed at w = 1).
    % The model is then wrong by construction -- the estimator thinks a_bar = 0.09 at the valley where the truth is 0.50 --
    % so the innovations do carry a systematic part and b_hat has something to correct. This is the arm that ties the
    % observability numbers to the closed loop.
    cc.amodel_fe = any(strcmp(arm, {'green', 'orange', '4state'}));
    if any(strcmp(arm, {'orange', '4state'})); cc.amodel_h = true; end
    if strcmp(arm, '4state')
        % the gain slot is gone, so the START GAIN uncertainty has to be carried by b instead: pick sigma_b,0 so that
        % (1-a)/b * sigma_b,0 equals the production arm's sqrt(P44[0]) -- otherwise the comparison changes two things.
        cc.gain_algebraic = true;
        A = load(fullfile(od, local_name(traj, 'orig', wall)), 'sP4');
        sP44_0 = mean(A.sP4(1, :));  lev = (1 - a0) / cc.b_init;
        cc.Pf_b_std = sP44_0 / lev;
        fprintf('[%s 4state] sqrt(P44[0]) of the production arm %.5f, lever (1-a)/b %.4f => Pf_b_std %.4f (production 0.0389)\n', ...
                traj, sP44_0, lev, cc.Pf_b_std);
    end
    fprintf('[%s %s] a_bar_0 %.4f at w %.3f, contact %.3f => b_0 %.4f, ws0_perp %.4f, amodel_fe %d amodel_h %d, seeds %s\n', ...
            traj, arm, a0, w0, w_c, cc.b_init, cc.ws0_perp, cc.amodel_fe, isfield(cc, 'amodel_h'), mat2str(seeds));
    if strcmp(wall, 'cellmisw'); cc.Pf_w0_std = 1.0; end   % wall position declared unknown (09-07 'widew': it lands in P44[0])
    o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false, ...
               'seeds',seeds,'log_P_full',false);
    if startsWith(wall, 'cell'); o.plant_law_b = B_CELL;  o.plant_law_w0 = WS_CELL; end
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    t = R.runs{1}.tout(:);  N = numel(t);  nS = numel(seeds);
    [E, AH, AT, HB, B, SP5, SP4, NIS1, NIS2] = deal(zeros(N, nS));
    for q = 1:nS
        r = R.runs{q};  ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
        AH(:,q) = r.a_bar_hat_out(:,3);  AT(:,q) = r.a_true_out(:,3)/ad;  E(:,q) = AH(:,q) - AT(:,q);
        HB(:,q) = r.h_bar_true_out(:,1);  B(:,q) = r.b_hat_out(:,3);  SP5(:,q) = r.P_b_out(:,3);  SP4(:,q) = r.P_a_out(:,3)/ad;
        NIS1(:,q) = r.innov_y1_out(:,3).^2 ./ r.S1_out(:,3);  NIS2(:,q) = r.innov_y2_out(:,3).^2 ./ r.S2_out(:,3);
    end
    b_ceil = cc.b_ceil;  clear R;
    out = struct('traj', traj, 'arm', arm, 'seeds', seeds, 't', t, 'E', E, 'AH', AH, 'AT', AT, 'HB', HB, 'B', B, ...
                 'sP5', SP5, 'sP4', SP4, 'NIS1', NIS1, 'NIS2', NIS2, 'cc', cc);
    save(fullfile(od, local_name(traj, arm, wall)), '-struct', 'out', '-v7.3');
    % ---- health ----
    fprintf('[%s %s] HEALTH min w %.4f | NaN %d | min a_hat %.4f floor hits %d | b_hat %.4f .. %.4f, ceiling hits %d\n', traj, arm, ...
            min(HB(:)), nnz(isnan(E)), min(AH(:)), nnz(min(AH, [], 1) < 0.0312), min(B(:)), max(B(:)), nnz(B(:) >= b_ceil - 1e-9));
    if strcmp(arm, 'orig') && strcmp(wall, 'plane')
        ref = fullfile(od, sprintf('three_walls_%s_plane-ramp_bseed0-bseed0_pf.mat', traj));
        m = matfile(ref);  rs = m.seeds;  X = m.plane_bseed0;  [sh, ia, ib] = intersect(seeds, rs);
        d = max(abs(E(:, ia) - X.E(:, ib)), [], 'all');
        fprintf('[%s orig] HEALTH reproduces saved bseed0 (seeds %s): max |dE| = %.3e -> %s\n', traj, mat2str(sh), d, local_pf(d == 0));
    end
    i7 = find(seeds == 7, 1);
    if ~isempty(i7) && strcmp(wall, 'plane')
        ref7 = struct('canon_orig', 0.00871, 'canon_green', 0.00634, 'canon_orange', 0.00484, ...
                      'meng_orig', 0.01490, 'meng_green', 0.01443, 'meng_orange', 0.00490);
        fn7 = sprintf('%s_%s', traj, arm);
        if ~isfield(ref7, fn7); v = NaN; else; v = ref7.(fn7); end
        if isnan(v)
            fprintf('[%s %s] seed 7 sqrt(P55)[end] %.5f (no step-3 reference for this arm)\n', traj, arm, SP5(end, i7));
        else
            fprintf('[%s %s] HEALTH seed 7 sqrt(P55)[end] %.5f vs step 3 %.5f -> %s\n', traj, arm, SP5(end, i7), v, local_pf(abs(SP5(end, i7) - v) < 1e-5));
        end
    end
end

function W = local_windows(traj)
    switch traj
        case 'meng'; W = struct('near', [7 10.5], 'mot2', [0.5 7], 'hold', [10.5 Inf], 'n1', 'near', 'n2', 'far');
        otherwise;   W = struct('near', [0.5 1.5], 'mot2', [1.5 3.5], 'hold', [3.5 Inf], 'n1', 'descent', 'n2', 'osc');
    end
end

function fn = local_name(traj, arm, wall)
    if strcmp(wall, 'plane'); fn = sprintf('amodel_fe_cl_%s_%s.mat', traj, arm);
    else;                     fn = sprintf('amodel_fe_cl_%s_%s_%s.mat', traj, arm, wall); end
end

function out = local_report(od, traj, W, cmp, wall)
    A = load(fullfile(od, local_name(traj, 'orig', wall)));  G = load(fullfile(od, local_name(traj, cmp, wall)));
    assert(isequal(A.seeds, G.seeds), 'arms must share seeds');
    t = A.t;  nS = numel(A.seeds);  sq = sqrt(nS);
    mn = t > W.near(1) & t <= W.near(2);  m2 = t > W.mot2(1) & t <= W.mot2(2);  mh = t > W.hold(1);
    kN = find(mn, 1, 'last');
    fprintf('\n===== %s [%s wall], %d seeds %s, paired: orig (production) vs %s =====\n', traj, wall, nS, mat2str(A.seeds), cmp);
    % M1
    fprintf('M1 near wall (%s, %.1f-%.1f s)\n', W.n1, W.near);
    wq = zeros(nS, 2);
    for f = 1:2
        X = local_pick(A, G, f);  me = mean(X.E(mn, :), 2);  [~, iw] = max(abs(me));  tw = t(mn);
        wq(:, f) = max(abs(X.E(mn, :)), [], 1)';
        fprintf('   %-5s E mean %+.4f | sigma_seed %.4f | worst instant t %.3f s  %+.4f | per-seed worst |E| %.4f\n', X.arm, ...
                mean(X.E(mn,:), 'all'), mean(std(X.E(mn,:), 0, 2)), tw(iw), me(iw), mean(wq(:, f)));
    end
    d = wq(:,2) - wq(:,1);  s2 = mean(d) > 2*std(d)/sq;
    fprintf('   paired per-seed worst |E|, %s - orig: %+.4f +- %.4f (SEM)  -> S2 %s\n', cmp, mean(d), std(d)/sq, local_tf(s2, 'TRIGGERED', 'not triggered'));
    dsig = std(G.E(mn,:), 0, 2) - std(A.E(mn,:), 0, 2);
    fprintf('   sigma_seed(t) %s - orig, time mean %+.4f\n', cmp, mean(dsig));
    % M2
    fprintf('M2 second motion (%s, %.1f-%.1f s)\n', W.n2, W.mot2);
    for f = 1:2
        X = local_pick(A, G, f);
        fprintf('   %-5s E mean %+.4f | sigma_seed %.4f\n', X.arm, mean(X.E(m2,:), 'all'), mean(std(X.E(m2,:), 0, 2)));
    end
    % M3
    fprintf('M3 b\n');
    hb = zeros(2, 2);
    for f = 1:2
        X = local_pick(A, G, f);
        hb(f, 1) = std(X.B(kN,:)) / mean(X.sP5(kN,:));  hb(f, 2) = std(X.B(end,:)) / mean(X.sP5(end,:));
        fprintf('   %-5s b_hat near-end %.4f +- %.4f (sqrt P55 %.4f, honesty %.2f) | run end %.4f +- %.4f (sqrt P55 %.4f, honesty %.2f)\n', X.arm, ...
                mean(X.B(kN,:)), std(X.B(kN,:)), mean(X.sP5(kN,:)), hb(f,1), mean(X.B(end,:)), std(X.B(end,:)), mean(X.sP5(end,:)), hb(f,2));
    end
    if strcmp(wall, 'cell')
        bt = 0.877;
        fprintf('   b_true = %.4f (the plant IS a law member): offset at the run end  orig %+.4f | %s %+.4f\n', ...
                bt, mean(A.B(end,:)) - bt, cmp, mean(G.B(end,:)) - bt);
        fprintf('                                              offset at the near-wall end  orig %+.4f | %s %+.4f\n', ...
                mean(A.B(kN,:)) - bt, cmp, mean(G.B(kN,:)) - bt);
    end
    db = G.B(end,:) - A.B(end,:);
    fprintf('   paired b_hat end, %s - orig: %+.4f +- %.4f (SEM) | paired b_hat at the near-wall end: %+.4f +- %.4f\n', ...
            cmp, mean(db), std(db)/sq, mean(G.B(kN,:) - A.B(kN,:)), std(G.B(kN,:) - A.B(kN,:))/sq);
    fprintf('   -> S1 %s\n', local_tf(any(hb(2,:) > 2), 'TRIGGERED', 'not triggered'));
    % M4
    fprintf('M4 a honesty, near wall: sd_seed(E) / mean sqrt(P44)\n');
    for f = 1:2
        X = local_pick(A, G, f);
        fprintf('   %-5s %.2f  (sd_seed %.4f, sqrt P44 %.4f, time means)\n', X.arm, mean(std(X.E(mn,:), 0, 2) ./ mean(X.sP4(mn,:), 2)), ...
                mean(std(X.E(mn,:), 0, 2)), mean(X.sP4(mn,:), 'all'));
    end
    % M5
    fprintf('M5 NIS (mean over seeds and time)\n');
    for f = 1:2
        X = local_pick(A, G, f);
        fprintf('   %-5s %s: NIS1 %.3f NIS2 %.3f | %s: NIS1 %.3f NIS2 %.3f | hold: NIS1 %.3f NIS2 %.3f\n', X.arm, W.n1, ...
                mean(X.NIS1(mn,:), 'all', 'omitnan'), mean(X.NIS2(mn,:), 'all', 'omitnan'), W.n2, mean(X.NIS1(m2,:), 'all', 'omitnan'), ...
                mean(X.NIS2(m2,:), 'all', 'omitnan'), mean(X.NIS1(mh,:), 'all', 'omitnan'), mean(X.NIS2(mh,:), 'all', 'omitnan'));
    end
    % M6
    fprintf('M6 hold (last)\n');
    for f = 1:2
        X = local_pick(A, G, f);  hm = mean(X.E(mh,:), 1);
        fprintf('   %-5s E mean %+.5f SEM %.5f sd %.5f\n', X.arm, mean(hm), std(hm)/sq, mean(std(X.E(mh,:), 0, 2)));
    end
    dh = mean(G.E(mh,:), 1) - mean(A.E(mh,:), 1);
    fprintf('   paired hold E, %s - orig: %+.5f +- %.5f (SEM)\n', cmp, mean(dh), std(dh)/sq);
    out = struct('A', A, 'G', G, 'W', W);
end

function X = local_pick(A, G, f); if f == 1; X = A; else; X = G; end; end
function s = local_pf(b); if b; s = 'PASS'; else; s = 'FAIL'; end; end
function s = local_tf(b, a, c); if b; s = a; else; s = c; end; end
