% FORK OF test_script/scratch/verify_obs_b1_state.m (2026-09-14) | PURPOSE: per-state observability of the b1 model, every free
%   slot, prior-scaled. Same windows and Phi/G convention as verify_state_observability.m (500 steps, G += (H Phi)' R^-1 (H Phi),
%   Phi <- F Phi). Scaling D = diag(sqrt(P[0])) (the run's own P_pred at the first record; b1 = sup|db_true/dw| when it is added
%   offline). Per window: CRLB/sqrt(P[0]) for every free slot = sqrt(diag(inv(D G D))) and the weakest direction of D G D
%   (lambda_min < 1 = along that combination the window's data are weaker than the prior; |v_min| = which slots make it).
%   Variants as verify_obs_b1_state: V1 constant b, V2 b1 added at b1_hat = 0 on the constant-b records, V3 the b1 run's records.
%   NOTE (tool header): the ratio is a proper bound only for Q = 0 slots (b, b1); dw1-3, m1-2 (and a_w, which carries the rank-1
%   gain block of Q) are reported for completeness. | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
function out = analyze_obs_b1_states(traj, seed, win)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seed); seed = 7; end
    if nargin < 3 || isempty(win); win = 500; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV; p0 = 3e-4;
                     SEG = {'hold0', 0, 0.5; 'far', 0.5, 7; 'near', 7, 10.5; 'hold', 10.5, 12.5};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
                     SEG = {'hold0', 0, 0.5; 'descent', 0.5, 1.5; 'osc', 1.5, 3.5; 'hold', 3.5, 4.8};
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar, true); a0 = 1/cp;  w_c = 1.0;
    b_ch = (1/(1 - a0) - 1) / (w0bar - w_c);  ws0_ch = 1 + w_c - 1/b_ch;
    cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1, ...
                'b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws0_ch,'obs_dump',true);
    o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;  evalc('RA = run_formC_b(o);');
    LA = obs_dump('get');  LA = LA([LA.ax] == 3);
    ccB = cc;  ccB.b1_state = true;  ccB.b1_seed_couple = false;  o.ctrl_const_override = ccB;
    clear run_formC_b motion_control_law_formC_b;  evalc('RB = run_formC_b(o);');
    LB = obs_dump('get');  LB = LB([LB.ax] == 3);  pb1 = RB.runs{1}.ctrl_const.Pf_b1_std;
    L2 = LA;
    for k = 1:numel(L2)
        if k == 1; a_prev = L2(k).x_pred(4); else; a_prev = L2(k-1).x_upd(4); end
        a_sl = min(max(a_prev, 0.05), 1 - 1e-4);
        L2(k).F(5, 6) = L2(k).F(4, 5) / (1 - a_sl)^2;  L2(k).F(6, 6) = 1;
    end
    names = {'dw1','dw2','dw3','a_w','b','b1','w_s','m1','m2'};
    hd = RA.runs{1}.h_bar_d_out(:);  t = RA.runs{1}.tout(:);
    out = struct('traj', traj, 'seed', seed, 'win', win, 'SEG', {SEG}, 'names', {names}, 'hd', hd, 't', t, 'T', cfg0.T_sim);
    out.V1 = local_variant(LA, [1 2 3 4 5 8 9],   [], win, cfg0.T_sim);
    out.V2 = local_variant(L2, [1 2 3 4 5 6 8 9], pb1, win, cfg0.T_sim);
    out.V3 = local_variant(LB, [1 2 3 4 5 6 8 9], [], win, cfg0.T_sim);
    for v = {'V1', 'V2', 'V3'}
        R = out.(v{1});
        fprintf('\n===== %s %s | free %s | sqrt(P[0]) %s =====\n', traj, v{1}, strjoin(names(R.free), ' '), mat2str(R.scale, 3));
        fprintf('  %-8s %-26s', 'segment', 'lambda_min(DGD) med [min]');
        for j = 1:numel(R.free); fprintf('%9s', names{R.free(j)}); end
        fprintf('   weakest direction at the segment''s smallest lambda\n');
        for s = 1:size(SEG, 1)
            m = R.t_win >= SEG{s,2} & R.t_win < SEG{s,3}; if ~any(m); continue; end
            lam = R.lam_min(m);  [lmin, iw] = min(lam);  idw = find(m);  vv = R.v_min(idw(iw), :);
            [~, ord] = sort(abs(vv), 'descend');
            comp = strjoin(arrayfun(@(q) sprintf('%s %+.2f', names{R.free(q)}, vv(q)), ord(1:3), 'uni', 0), ', ');
            fprintf('  %-8s %10.3g [%9.3g]    ', SEG{s,1}, median(lam), lmin);
            fprintf('%9.3g', median(R.ratio(m, :), 1));
            fprintf('   %s\n', comp);
        end
    end
    save(fullfile(od, sprintf('analyze_obs_b1_states_%s_w%d.mat', traj, win)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved analyze_obs_b1_states_%s_w%d.mat\n', traj, traj, win);
end

function R = local_variant(L, free, pb1_added, W, T)
    N = numel(L);  n = size(L(1).F, 1);  P0 = L(1).P_pred;
    sc = sqrt(max(diag(P0), 0));  if ~isempty(pb1_added); sc(6) = pb1_added; end
    D = diag(sc(free));
    starts = 1:W:(N - W + 1);  nw = numel(starts);  nf = numel(free);
    ratio = nan(nw, nf);  lam_min = nan(nw, 1);  v_min = nan(nw, nf);
    for w = 1:nw
        Phi = eye(n);  G = zeros(n);
        for k = starts(w):(starts(w) + W - 1)
            for j = 1:numel(L(k).H)
                Hj = L(k).H{j};  if isempty(Hj); continue; end
                Mw = (Hj / sqrt(L(k).R(j))) * Phi;  G = G + Mw' * Mw;
            end
            Phi = L(k).F * Phi;
        end
        Gt = D * (0.5 * (G(free, free) + G(free, free)')) * D;
        [V, E] = eig(0.5 * (Gt + Gt'));  e = diag(E);  [e, ord] = sort(e, 'ascend');  V = V(:, ord);
        cut = max(e) * 1e-15;  inv_e = 1 ./ e;  inv_e(e <= cut) = Inf;
        ratio(w, :) = sqrt(abs((V.^2) * inv_e));
        lam_min(w) = e(1);  v_min(w, :) = V(:, 1).';
    end
    R = struct('free', free, 'scale', sc(free).', 't_win', (starts + W/2 - 1) * (T / N), 'ratio', ratio, 'lam_min', lam_min, 'v_min', v_min);
end
