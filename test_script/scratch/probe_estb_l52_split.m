% PURPOSE (2026-09-07): WHY does the estimated-b arm with the seed-at-truth P44[0] pull b_hat UP (canon: 0.889 -> 0.96 at the first
%   bottom; Meng: slowly to 0.94) while the mid-band truth is 0.867? The y2 gain on b has two paths with opposite signs on a descent:
%     readout path   P55 H25 / S2,   H25 = -(da'/db) grad_d w_d  > 0 on a descent  (a larger b predicts a HIGHER reading)
%     state path     P54 H24 / S2,   P54 built by F_e(4,5) = a' M / b < 0 on a descent (a larger b lowers a_hat)  -> pushes b DOWN
%   The readout path scales with the commanded speed (grad_d w_d), the state path with the accumulated P54. Prediction: on canon
%   (fast) the readout path dominates the b_hat update in the descent (b_hat up); on Meng (slow) the two are comparable.
%   Per step from obs_dump: P^(1) after the y1 update (Joseph), S2, the two terms of l52 x innov2, and the y1 leg l51 x innov1;
%   cumulative sums over the descent / oscillation / hold, seed mean. Arm = bhp0 (b_hat estimated, Pf_w0_std 0, Pf_a_floor p0).
%   Output probe_estb_l52_split_<traj>.mat | EXPIRES: with the estimated-b line
function out = probe_estb_l52_split(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:5; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV; p0 = 3e-4;
                     SEG = {'far 0.5-7',[0.5 7]; 'near 7-10.5',[7 10.5]; 'hold',[10.5 12.5]};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
                     SEG = {'desc 0.5-1.5',[0.5 1.5]; 'osc 1.5-3.5',[1.5 3.5]; 'hold',[3.5 4.8]};
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1,'ws0_perp',ws0,'Pf_w0_std',0,'Pf_a_floor',p0,'obs_dump',true);
    nS = numel(seeds); out = struct('traj', traj, 'seeds', seeds);
    RO = []; ST = []; Y1 = []; BH = []; T = [];
    for q = 1:nS
        o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds(q),'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');  Lz = L([L.ax] == 3);  n = numel(Lz);  r = R.runs{1};  t = r.tout(:);
        k = [Lz.k].';  kd = min(k + 1, numel(t));
        in2 = r.innov_y2_out(:,3);  in1 = r.innov_y1_out(:,3);  l51 = r.K_b_y1_out(:,3);
        ro = zeros(n,1); st = ro; y1 = ro;
        for j = 1:n
            Pp = Lz(j).P_pred;  H1 = Lz(j).H{1};  H2 = Lz(j).H{2};  R1 = Lz(j).R(1);  R2 = Lz(j).R(2);
            S1 = H1 * Pp * H1' + R1;  K1 = (Pp * H1') / S1;  I = eye(size(Pp));
            P1 = (I - K1*H1) * Pp * (I - K1*H1)' + K1 * R1 * K1';
            S2 = H2 * P1 * H2' + R2;  g = (P1 * H2') / S2;          % full y2 gain vector; g(5) = l52
            ro(j) = (P1(5,5) * H2(5) / S2) * in2(kd(j));            % readout path contribution to the b_hat update
            st(j) = (g(5) - P1(5,5) * H2(5) / S2) * in2(kd(j));     % state path (P54 H24 + the rest)
            y1(j) = l51(kd(j)) * in1(kd(j));
        end
        RO(:,q) = ro; ST(:,q) = st; Y1(:,q) = y1; BH(:,q) = r.b_hat_out(kd,3); T = t(kd);
    end
    out.t = T; out.readout = RO; out.state = ST; out.y1leg = Y1; out.b_hat = BH;
    fprintf('[%s bhp0] b_hat update budget (seed mean over %d seeds): segment | readout path sum | state path sum | y1 leg sum | total | b_hat change\n', traj, nS);
    for s = 1:size(SEG,1)
        m = T >= SEG{s,2}(1) & T <= SEG{s,2}(2);  i0 = find(m,1); i1 = find(m,1,'last');
        fprintf('  %-14s %+.4f  %+.4f  %+.4f  | %+.4f | %+.4f\n', SEG{s,1}, mean(sum(RO(m,:),1)), mean(sum(ST(m,:),1)), mean(sum(Y1(m,:),1)), mean(sum(RO(m,:)+ST(m,:)+Y1(m,:),1)), mean(BH(i1,:) - BH(i0,:)));
    end
    save(fullfile(od, sprintf('probe_estb_l52_split_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved probe_estb_l52_split_%s.mat\n', traj, traj);
end
