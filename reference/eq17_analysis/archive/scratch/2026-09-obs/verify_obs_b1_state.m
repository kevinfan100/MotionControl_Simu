% FORK OF test_script/scratch/verify_obs_b_of_w.m (2026-09-14) | PURPOSE: gates 4-5 for b1 = db/dw as slot 6
%   (0914_estb_bprime_core.tex, ctrl_const.b1_state) on the chord-seed plane arm, one seed. Three readings, same tool:
%     V1 const   production linearisation of the constant-b arm (bseed0), free [1 2 3 4 5 8 9]            -> baseline / victim reference
%     V2 struct  the SAME records with slot 6 = b1 added per the tex at b1_hat = 0: F(5,6) = M_k (the row-4 bracket,
%                recovered as F(4,5)/(1-a_bar_sl)^2), F(6,6) = 1, H(:,6) = 0                               -> structural observability
%     V3 impl    the implemented b1 run's own records (b1_state, no extra P[0] cross term), free [1 2 3 4 5 6 8 9]
%   Negative control = slot 7 (inert). Priors: a_w from the run, b = Pf_b_std (envelope half-range), b1 = sup|db_true/dw|.
%   Also prints the absolute CRLB of b1 against the two scales that matter: the wall slope sup|b'_true| and the
%   anchor-consistent width sup|b_true - 8/9|/(w_0 - w_c) (tex, Seed and prior), and the filter's own sqrt(P) at the end.
%   Output verify_obs_b1_state_<traj>_w<win>.mat | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
function out = verify_obs_b1_state(traj, seed, win)
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
    % ---- run A: constant b ----
    o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;  evalc('RA = run_formC_b(o);');
    LA = obs_dump('get');  LA = LA([LA.ax] == 3);  rA = RA.runs{1};
    % ---- run B: with b1 (plain: no extra P[0] cross term) ----
    ccB = cc;  ccB.b1_state = true;  ccB.b1_seed_couple = false;
    o.ctrl_const_override = ccB;
    clear run_formC_b motion_control_law_formC_b;  evalc('RB = run_formC_b(o);');
    LB = obs_dump('get');  LB = LB([LB.ax] == 3);  rB = RB.runs{1};
    ad = rA.a_hat_out(1,3) / rA.a_bar_hat_out(1,3);
    pa = rA.P_a_out(1,3) / ad;  pb = rA.ctrl_const.Pf_b_std;  pb1 = rB.ctrl_const.Pf_b1_std;
    b_half = pb;  anchor_w = b_half / (w0bar - w_c);
    fprintf('[%s obs b1] seed %d | chord b_0 %.4f | records A %d B %d | win %d | priors: a_w %.2e  b %.4f  b1 %.4f /R | anchor-consistent b1 width %.2e /R (= %.4f / %.2f R)\n', ...
        traj, seed, b_ch, numel(LA), numel(LB), win, pa, pb, pb1, anchor_w, b_half, w0bar - w_c);
    % ---- V2 structural augmentation at b1_hat = 0 ----
    L2 = LA;  Mk = zeros(numel(L2), 1);
    for k = 1:numel(L2)
        if k == 1; a_prev = L2(k).x_pred(4); else; a_prev = L2(k-1).x_upd(4); end
        a_sl = min(max(a_prev, 0.05), 1 - 1e-4);
        Mk(k) = L2(k).F(4, 5) / (1 - a_sl)^2;
        L2(k).F(5, 6) = Mk(k);  L2(k).F(6, 6) = 1;
    end
    names = {'dw1','dw2','dw3','a_w','b','b1','w_s','m1','m2'};
    t_end = cfg0.T_sim;
    out = struct('traj', traj, 'seed', seed, 'win', win, 'priors', [pa pb pb1], 'anchor_w', anchor_w, 'SEG', {SEG}, ...
                 'hd', rA.h_bar_d_out(:), 't', rA.tout(:), 'M', Mk);
    out.V1 = local_run(LA, [1 2 3 4 5 8 9],   [NaN NaN NaN pa pb NaN NaN],     names([1 2 3 4 5 8 9]),   win, t_end, SEG, 'V1 const  (constant b)');
    out.V2 = local_run(L2, [1 2 3 4 5 6 8 9], [NaN NaN NaN pa pb pb1 NaN NaN], names([1 2 3 4 5 6 8 9]), win, t_end, SEG, 'V2 struct (b1 added at b1_hat = 0)');
    out.V3 = local_run(LB, [1 2 3 4 5 6 8 9], [NaN NaN NaN pa pb pb1 NaN NaN], names([1 2 3 4 5 6 8 9]), win, t_end, SEG, 'V3 impl   (b1 run own linearisation)');
    % ---- absolute CRLB of b1 vs the scales, and the filter's own sqrt(P) ----
    for v = {'V2', 'V3'}
        R = out.(v{1});  j6 = find(R.free == 6);  c6 = R.crlb(:, j6, 1);  j5 = find(R.free == 5);  c5 = R.crlb(:, j5, 1);
        fprintf('   %s b1 best CRLB %.3e /R  (vs wall slope %.3f: %.2e x | vs anchor width %.2e: %.2f x) | b best CRLB %.3e\n', ...
            v{1}, min(c6), pb1, min(c6)/pb1, anchor_w, min(c6)/anchor_w, min(c5));
    end
    fprintf('   filter sqrt(P) end: const b %.4f | b1 run: b %.4f  b1 %.4f /R\n', rA.P_b_out(end,3), rB.P_b_out(end,3), rB.P_p_out(end,3));
    save(fullfile(od, sprintf('verify_obs_b1_state_%s_w%d.mat', traj, win)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved verify_obs_b1_state_%s_w%d.mat\n', traj, traj, win);
end

function res = local_run(L, free, ps, lab, win, T, SEG, title)
    cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 7, 'prior_std', ps, 'labels', {lab}, 'window', win, 't_end', T, 'segments', {SEG});
    evalc('res = verify_state_observability(L, cfgv);');
    res.free = free;
    fprintf('\n===== %s | free %s =====\n', title, mat2str(free));
    fprintf('  gate 4: %s (neg control %s) | sigma_min/max median %.2e, min %.2e\n', local_tf(res.gate4_pass), local_tf(res.neg_ok), median(res.srat), min(res.srat));
    pri = res.prior; C = res.crlb; tw = res.t_win(:);
    for j = 1:numel(pri)
        if isnan(pri(j)); continue; end
        cj = C(:, j, 1); line = sprintf('  %-6s prior %.2e | best CRLB/prior %8.3f (%s)', lab{j}, pri(j), min(cj)/pri(j), res.verdict{j});
        for s = 1:size(SEG, 1)
            m = tw >= SEG{s,2} & tw < SEG{s,3}; if ~any(m); continue; end
            line = [line sprintf(' | %s %8.3f', SEG{s,1}, min(cj(m))/pri(j))];
        end
        if size(C, 3) >= 3; line = [line sprintf(' | y1-only %.3g  y2-only %.3g', min(C(:, j, 2))/pri(j), min(C(:, j, 3))/pri(j))]; end
        fprintf('%s\n', line);
    end
end
function s = local_tf(tf); if tf; s = 'PASS'; else; s = 'FAIL'; end; end
