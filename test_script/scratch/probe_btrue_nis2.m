% PURPOSE (2026-09-04): is the y2 leg (gain readout) weighted right, segment by segment, on the b_true arm at kappa*?
%   Normalised innovation squared NIS2 = innov2^2 / S2, S2 = H2 P^(1) H2' + R2, with P^(1) = P after the y1 update,
%   reconstructed exactly from the obs_dump capture (P_pred, H1, R1 -> Joseph update -> P^(1)); H2 and R2 from the capture,
%   innov2 from the driver log (record j <-> driver index k_j + 1). NIS2 ~ 1 in a segment => R2 and P44 are consistent with the
%   readout there; NIS2 << 1 => the filter under-uses y2 (S2 too large); >> 1 => over-uses. Also the y1 NIS1 for reference.
%   Reports seed-mean NIS per segment for kappa in {1, 0.5} (both with the four blocks). | EXPIRES: with the b_true rung
function out = probe_btrue_nis2(traj, seeds, kappas)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    if nargin < 3 || isempty(kappas); kappas = [1 0.5]; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');  pc = physical_constants();
    switch traj
        case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg0 = OV;
                     SEG = {'far 2-5 s',[2 5]; 'mid 5-7 s',[5 7]; 'near 7-10 s',[7 10]; 'osc 10.5-11.5',[10.5 11.5]; 'hold',[11.5 12.5]};
        case 'canon'; OV = struct(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
                     SEG = {'pre 0.6-1.0',[0.6 1.0]; 'fast 1.0-1.5',[1.0 1.5]; 'osc 1.5-3.5',[1.5 3.5]; 'hold',[3.5 4.8]};
    end
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp; ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    nS = numel(seeds);  out = struct('traj', traj, 'seeds', seeds, 'kappas', kappas);
    for ik = 1:numel(kappas)
        cc = struct('law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'ws0_perp',ws0,'obs_dump',true,'fe44_Aa_scale',kappas(ik));
        NIS2 = []; NIS1 = []; GATE = []; T = [];
        for q = 1:nS
            o = struct('arm','best','b_true',true,'b_true_at','true','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds(q),'log_P_full',false);
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            L = obs_dump('get');  Lz = L([L.ax] == 3);  n = numel(Lz);  r = R.runs{1};  t = r.tout(:);
            k = [Lz.k].';  kd = min(k + 1, numel(t));
            in2 = r.innov_y2_out(:,3);  in1 = r.innov_y1_out(:,3);
            nis2 = nan(n,1); nis1 = nan(n,1); gate = false(n,1);
            for j = 1:n
                Pp = Lz(j).P_pred;  H1 = Lz(j).H{1};  H2 = Lz(j).H{2};  R1 = Lz(j).R(1);  R2 = Lz(j).R(2);
                S1 = H1 * Pp * H1' + R1;  K1 = (Pp * H1') / S1;  I = eye(size(Pp));
                P1 = (I - K1*H1) * Pp * (I - K1*H1)' + K1 * R1 * K1';
                S2 = H2 * P1 * H2' + R2;
                nis1(j) = in1(kd(j))^2 / S1;  nis2(j) = in2(kd(j))^2 / S2;  gate(j) = Lz(j).gate;
            end
            NIS2(:,q) = nis2; NIS1(:,q) = nis1; GATE(:,q) = gate; T = t(kd);
        end
        out.(sprintf('k%03d', round(100*kappas(ik)))) = struct('t', T, 'nis2', NIS2, 'nis1', NIS1, 'gate', GATE);
        fprintf('[%s kappa %.2f] segment: NIS2 mean (SEM over seeds) | NIS1 mean | gate-off fraction\n', traj, kappas(ik));
        for s = 1:size(SEG,1)
            m = T >= SEG{s,2}(1) & T <= SEG{s,2}(2);  ps2 = mean(NIS2(m,:), 1, 'omitnan');  ps1 = mean(NIS1(m,:), 1, 'omitnan');
            fprintf('[%s kappa %.2f] %-14s NIS2 %.2f (SEM %.2f) | NIS1 %.2f | gate off %.0f%%\n', traj, kappas(ik), SEG{s,1}, mean(ps2), std(ps2)/sqrt(nS), mean(ps1), 100*mean(GATE(m,:),'all'));
        end
    end
    save(fullfile(od, sprintf('probe_btrue_nis2_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s] saved probe_btrue_nis2_%s.mat\n', traj, traj);
end
