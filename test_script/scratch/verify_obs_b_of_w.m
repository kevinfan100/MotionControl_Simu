% PURPOSE (2026-09-08, user's challenge: "judge b(w) strictly by observability"): gates 4-5 for THREE representations of b on the
%   same dumped linearisation (bseed0 arm: chord seed, P44[0] at the start truth, b estimated, obs_dump on):
%     const   b constant (production slot 5)                                   free = [1 2 3 4 5 8 9]
%     lin     b(w) = b0 + b1 (w - w_ref): shadow slot 10 = b1, its F/H columns = the b columns x (w_k - w_ref)   (exact by the chain rule)
%     bins    b(w) = b_j on J height bins: shadow slots 10..9+J, columns = the b columns x 1[w_k in bin j]; slot 5's columns zeroed
%   The augmentation is done OFFLINE on the obs_dump records (F, H per step), so the controller is untouched and the answer is the
%   production linearisation's. Negative control = slot 7 (inert). Priors: a_w from the run, b 0.039 (family half-range),
%   b1 = sup|db_true/dw| over the band (0.07 /R, Brenner 0.867 at 2 R -> 0.93 at 1.1 R), bins 0.039 each.
%   PRE-REGISTERED: (1) const: canon best CRLB/prior << 1 (O23: 0.0088), Meng ~0.5. (2) lin: b1 PASS on canon (oscillation revisits
%   give range), TIGHT/FAIL on Meng (single ramp). (3) bins: near-wall bins PASS on canon, far bin FAIL; Meng only the near bin, at best.
%   DECISION RULE: a b(w) representation is 'estimable' only if every b-state PASSes in motion windows AND a_w's own CRLB/prior does not
%   cross 1 when the states are added (the victim rule, observability-workflow conclusion 4). Everything else stays [假說].
%   Output verify_obs_b_of_w_<traj>.mat | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function out = verify_obs_b_of_w(traj, seed, win)
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
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar, true); a0 = 1/cp;
    b_ch = (1/(1 - a0) - 1) / (w0bar - 1.0);  ws0_ch = 1 + 1.0 - 1/b_ch;
    cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1, ...
                'b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws0_ch,'obs_dump',true);
    o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    L = obs_dump('get');  r = R.runs{1};  ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);  cc_run = r.ctrl_const;
    L = L([L.ax] == 3);                                   % axis 3 only (the tool would select, but the shadow columns need w per record)
    wk = r.h_bar_d_out(max(1, min(numel(r.h_bar_d_out), [L.k])));   % commanded height per record [R]
    fprintf('[%s obs b(w)] seed %d | b_0 %.4f ws0 %.4f | records %d | w range %.2f .. %.2f R | win %d\n', traj, seed, b_ch, ws0_ch, numel(L), min(wk), max(wk), win);
    names = {'dw1','dw2','dw3','a_w','b','da','w_s','m1','m2'};
    pa = r.P_a_out(1,3)/ad;  pb = cc_run.Pf_b_std;
    w_ref = 2.2;  PRIOR_B1 = 0.07;                          % [1/R] sup |db_true/dw| over the deep band
    switch traj; case 'canon'; EDGES = [1.10 1.5 2.0 2.6 3.4]; otherwise; EDGES = [1.10 1.5 2.0 3.0 6.7]; end
    J = numel(EDGES) - 1;
    out = struct('traj', traj, 'seed', seed, 'win', win, 'b0', b_ch, 'edges', EDGES);
    % ---------- variant 1: const ----------
    free = [1 2 3 4 5 8 9]; ps = nan(1, numel(free)); ps(free == 4) = pa; ps(free == 5) = pb; lab = names(free);
    out.const = local_run(L, free, ps, lab, win, cfg0.T_sim, SEG, 'const  (b constant)');
    % ---------- variant 2: lin ----------
    L2 = L;
    for k = 1:numel(L2)
        phi = wk(k) - w_ref; F = L2(k).F; col = F(:, 5); col(5) = 0;
        L2(k).F = [F, col * phi; zeros(1, size(F,2)), 1];
        for j = 1:numel(L2(k).H); if ~isempty(L2(k).H{j}); h = L2(k).H{j}; L2(k).H{j} = [h, h(5) * phi]; end; end
    end
    free = [1 2 3 4 5 8 9 10]; ps = nan(1, numel(free)); ps(free == 4) = pa; ps(free == 5) = pb; ps(free == 10) = PRIOR_B1;
    lab = [names([1 2 3 4 5 8 9]), {sprintf('b1 = db/dw (ref %.1f R)', w_ref)}];
    out.lin = local_run(L2, free, ps, lab, win, cfg0.T_sim, SEG, 'lin  (b = b0 + b1 (w - w_ref))');
    % ---------- variant 3: bins ----------
    L3 = L;
    for k = 1:numel(L3)
        F = L3(k).F; col = F(:, 5); col(5) = 0; n = size(F, 2);
        ind = double(wk(k) >= EDGES(1:end-1) & wk(k) < EDGES(2:end)); if wk(k) >= EDGES(end); ind(end) = 1; end
        F2 = F; F2(:, 5) = 0; F2(5, 5) = 1;
        L3(k).F = [F2, col * ind; zeros(J, n), eye(J)];
        for j = 1:numel(L3(k).H); if ~isempty(L3(k).H{j}); h = L3(k).H{j}; h5 = h(5); h(5) = 0; L3(k).H{j} = [h, h5 * ind]; end; end
    end
    free = [1 2 3 4 8 9, 10:(9+J)]; ps = nan(1, numel(free)); ps(free == 4) = pa; ps(free >= 10) = pb;
    lab = names([1 2 3 4 8 9]); for j = 1:J; lab{end+1} = sprintf('b bin [%.1f,%.1f)', EDGES(j), EDGES(j+1)); end
    out.bins = local_run(L3, free, ps, lab, win, cfg0.T_sim, SEG, sprintf('bins  (b_j on %d height bins)', J));
    save(fullfile(od, sprintf('verify_obs_b_of_w_%s_w%d.mat', traj, win)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved verify_obs_b_of_w_%s_w%d.mat\n', traj, traj, win);
end

function res = local_run(L, free, ps, lab, win, T, SEG, title)
    cfgv = struct('axis', 3, 'free', free, 'neg_ctrl', 7, 'prior_std', ps, 'labels', {lab}, 'window', win, 't_end', T, 'segments', {SEG});
    fprintf('\n===== %s | free %s =====\n', title, mat2str(free));
    evalc('res = verify_state_observability(L, cfgv);');   % the tool's own long report is suppressed; the compact table below is what we read
    fprintf('  gate 4: %s (neg control %s) | sigma_min/max median %.2e\n', local_tf(res.gate4_pass), local_tf(res.neg_ok), median(res.srat));
    nb = numel(res.labels) - 1;                              % last label = the neg control appended by the tool? keep robust:
    labs = res.labels; pri = res.prior; C = res.crlb; tw = res.t_win(:);
    for j = 1:numel(pri)
        if isnan(pri(j)); continue; end
        cj = C(:, j, 1); line = sprintf('  %-26s prior %.4f | best CRLB/prior %6.3f (%s)', labs{j}, pri(j), min(cj)/pri(j), res.verdict{j});
        for s = 1:size(SEG, 1)
            m = tw >= SEG{s,2} & tw < SEG{s,3}; if ~any(m); continue; end
            line = [line sprintf(' | %s %6.3f', SEG{s,1}, min(cj(m))/pri(j))];
        end
        if size(C, 3) >= 3; line = [line sprintf(' | y1-only %.3g  y2-only %.3g', median(C(:, j, 2))/pri(j), median(C(:, j, 3))/pri(j))]; end
        fprintf('%s\n', line);
    end
end
function s = local_tf(tf); if tf; s = 'PASS'; else; s = 'FAIL'; end; end
