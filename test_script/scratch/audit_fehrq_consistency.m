% PURPOSE (2026-09-09, user: "check F_e, H, R, Q before trusting the observability verdict"): an audit of the matrices AS THE
%   FILTER USES THEM, from obs_dump alone -- no re-derivation, no mirror code, so it cannot be fooled by a shared mistake.
%   A1 UPDATE REPLAY   Joseph from (P_pred, H, R) must reproduce the recorded P_upd (validates H and R as wired).
%   A2 Q RECOVERY      Q[k] = P_pred[k] - F[k] P_upd[k-1] F[k]' (exact, since the record's F is the one used, nw_mcorr included).
%                      Checks: symmetry, min eig >= 0, Q(5,5) == 0 (b is a constant state), gain-block rank.
%   A3 REPLAY          propagate P from P_upd[1] with the recorded (F, H, R) and the recovered Q -> must track P_upd (wiring).
%   B  STATISTICS      NIS1, NIS2 per segment (should be 1 if R and P are honest) + lag-1 whiteness of the normalised innovations.
%   C  THE 20x GAP     the same replay with Q = 0 (what verify_state_observability assumes) and with one channel at a time:
%                      how much of "CRLB says 0.009 x prior but the filter reaches 0.18 x prior" is the tool ignoring Q.
%   Output audit_fehrq_<traj>.mat | EXPIRES: when the b line closes | 產線改動不會自動跟上
function out = audit_fehrq_consistency(traj, seed)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV; p0 = 3e-4;
                     SEG = {'hold0',0,0.5; 'far',0.5,7; 'near',7,10.5; 'hold',10.5,12.5};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
                     SEG = {'hold0',0,0.5; 'descent',0.5,1.5; 'osc',1.5,3.5; 'hold',3.5,4.8};
    end
    w0 = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0, true); a0 = 1/cp;
    b_ch = (1/(1-a0) - 1)/(w0 - 1.0);  ws0 = 1 + 1.0 - 1/b_ch;
    cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1, ...
                'b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',ws0,'obs_dump',true);
    o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seed,'log_P_full',false);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    L = obs_dump('get'); L = L([L.ax] == 3);  r = R.runs{1};  t = r.tout(:);
    n = size(L(1).F, 1); N = numel(L);
    fprintf('\n===== [%s seed %d] F_e / H / R / Q audit, %d records, n = %d =====\n', traj, seed, N, n);
    lockz = find(all(abs(L(end).P_upd) < 1e-300, 1) & all(abs(L(end).P_upd) < 1e-300, 2).');   % locked/inert slots (exact zero rows+cols)
    fprintf('locked/inert slots (zero rows and cols of P): %s\n', mat2str(lockz));

    % ---------- A1 update replay ----------
    e1 = zeros(N,1);
    for k = 1:N
        P = L(k).P_pred;
        for j = 1:numel(L(k).H)
            Hj = L(k).H{j}; if isempty(Hj); continue; end
            S = Hj*P*Hj' + L(k).R(j);  K = (P*Hj')/S;  K(lockz) = 0;
            P = (eye(n) - K*Hj)*P*(eye(n) - K*Hj)' + K*L(k).R(j)*K';  P = 0.5*(P+P');
        end
        e1(k) = max(abs(P(:) - L(k).P_upd(:))) / max(1, max(abs(L(k).P_upd(:))));
    end
    fprintf('A1  Joseph replay of the update vs recorded P_upd: max rel err %.2e (median %.2e)  -> %s\n', max(e1), median(e1), local_pf(max(e1) < 1e-9));

    % ---------- A2 Q recovery ----------
    Qs = cell(N,1); q55 = zeros(N,1); mineig = zeros(N,1); asym = zeros(N,1); qrank = zeros(N,1);
    for k = 2:N
        Q = L(k).P_pred - L(k).F * L(k-1).P_upd * L(k).F';
        Qs{k} = Q; Qt = Q.'; asym(k) = max(abs(Q(:) - Qt(:)));  Q = 0.5*(Q+Q');
        fr = setdiff(1:n, lockz);  ev = eig(Q(fr,fr));  mineig(k) = min(real(ev));
        q55(k) = Q(5,5);  qrank(k) = sum(real(ev) > 1e-14*max(1,max(real(ev))));
    end
    fprintf('A2  Q = P_pred - F P_upd F'':  max asym %.2e | min eig over run %.2e | Q(5,5) max |.| %.2e | rank(free block) median %d\n', ...
        max(asym), min(mineig(2:end)), max(abs(q55)), median(qrank(2:end)));
    fprintf('    verdicts: symmetric %s | PSD %s | Q55 = 0 (b constant) %s\n', local_pf(max(asym) < 1e-18), local_pf(min(mineig(2:end)) > -1e-18), local_pf(max(abs(q55)) == 0));

    % ---------- A3 / C replay variants ----------
    [pa, sp] = replay(L, Qs, lockz, n, 'both', true);
    [~, sp0] = replay(L, Qs, lockz, n, 'both', false);       % Q = 0: the CRLB assumption
    [~, sp1] = replay(L, Qs, lockz, n, 'y1', true);
    [~, sp2] = replay(L, Qs, lockz, n, 'y2', true);
    act = arrayfun(@(s) sqrt(s.P_upd(5,5)), L);  act = act(:);   % column, else the comparison below broadcasts to N x N
    fprintf('A3  replay(F,H,R,Q) vs recorded P_upd: max rel err on sqrt(P55) %.2e -> %s\n', max(abs(sp - act)./max(act,eps)), local_pf(max(abs(sp - act)./max(act,eps)) < 1e-9));
    fprintf('C   sqrt(P55) at the end:  filter %.5f | same recursion with Q = 0 %.5f (%.1fx tighter) | y1 only %.5f | y2 only %.5f | prior %.5f\n', ...
        act(end), sp0(end), act(end)/sp0(end), sp1(end), sp2(end), act(1));

    % ---------- B statistics ----------
    e1n = r.innov_y1_out(:,3)./sqrt(r.S1_out(:,3));  e2n = r.innov_y2_out(:,3)./sqrt(r.S2_out(:,3));
    fprintf('B   NIS (should be 1):   ');
    for s = 1:size(SEG,1)
        m = t >= SEG{s,2} & t < SEG{s,3};
        fprintf('%s y1 %.2f y2 %.2f | ', SEG{s,1}, mean(e1n(m).^2, 'omitnan'), mean(e2n(m).^2, 'omitnan'));
    end
    fprintf('\n    lag-1 autocorr of normalised innovations: y1 %.3f | y2 %.3f (white => ~0)\n', ac1(e1n), ac1(e2n));
    if isfield(cfg0, 'meas_noise_std'); mn = cfg0.meas_noise_std; else; c2 = canonical_scenario(0.05, 1.1, 'deep'); mn = c2.meas_noise_std; end
    R1_used = L(end).R(1);  R1_spec = (mn(3)/pc.R)^2;
    fprintf('    R1 used %.3e vs (meas_noise_std_z / R)^2 %.3e  (ratio %.3f)\n', R1_used, R1_spec, R1_used/R1_spec);
    out = struct('traj',traj,'seed',seed,'a1',e1,'q55',q55,'mineig',mineig,'sp',sp,'sp0',sp0,'sp1',sp1,'sp2',sp2,'act',act);
    save(fullfile(od, sprintf('audit_fehrq_%s.mat', traj)), '-struct', 'out', '-v7.3');
end

function [P, sp] = replay(L, Qs, lockz, n, chan, useQ)
    N = numel(L); P = L(1).P_upd; sp = zeros(N,1); sp(1) = sqrt(P(5,5));
    for k = 2:N
        P = L(k).F * P * L(k).F';
        if useQ && ~isempty(Qs{k}); P = P + Qs{k}; end
        P = 0.5*(P+P');
        for j = 1:numel(L(k).H)
            if strcmp(chan,'y1') && j ~= 1; continue; end
            if strcmp(chan,'y2') && j ~= 2; continue; end
            Hj = L(k).H{j}; if isempty(Hj); continue; end
            S = Hj*P*Hj' + L(k).R(j); K = (P*Hj')/S; K(lockz) = 0;
            P = (eye(n) - K*Hj)*P*(eye(n) - K*Hj)' + K*L(k).R(j)*K'; P = 0.5*(P+P');
        end
        sp(k) = sqrt(max(P(5,5), 0));
    end
end
function s = local_pf(b); if b; s = 'PASS'; else; s = 'FAIL'; end; end
function a = ac1(x); x = x(isfinite(x)); x = x - mean(x); a = (x(1:end-1)'*x(2:end))/(x'*x); end
