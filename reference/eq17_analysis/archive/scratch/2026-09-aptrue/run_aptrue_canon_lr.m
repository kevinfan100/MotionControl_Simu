% STATUS: ACTIVE (scratch) | PURPOSE: left/right comparison of the a'_true oracle arm on
%   the 1 Hz canonical deep trajectory, SAME 10 seeds (2026-09-02, user request):
%     left  = baseline  Euler tangent step, slope @cmd            (the 08-31/09-01 arm)
%     right = arm 4     exact step + slope @est + app_known + pred_mean2 (a'_true section)
%   The right column is the aptrue arm of arms10_seedtruth_comp_canon.mat (seeds 1:10,
%   identical options); only the left arm is run here. Same compact schema, so the
%   5x2 plot fork renders it. Output: aptrue_canon_lr.mat
function out = run_aptrue_canon_lr()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants(); cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    w0bar = cfg0.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at)); SEEDS = 1:10;
    o = struct('arm','best','ap_known',true,'ap_known_at','cmd','app_known',false, ...
               'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0), ...
               'config_override',struct(),'scenario','deep','verbose',false,'seeds',SEEDS,'log_P_full',true);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    t = R.runs{1}.tout(:); nS = numel(SEEDS);
    ZF = {'a_bar_hat_out','P_a_out','P_b_out','K_a_y1_out','K_a_y2_out','K_dx_y1_out','K_dx_y2_out', ...
          'innov_y1_out','innov_y2_out','a_prime_out','a_prime_true_out','delta_x_hat_3_out'};
    E = zeros(numel(t), nS); AH = E; P44 = E; C = struct(); for f = ZF; C.(f{1}) = zeros(numel(t), nS); end
    C.a_true_norm = zeros(numel(t), nS); C.h_bar_true = C.a_true_norm; C.trk_true = C.a_true_norm; C.P33_sqrt = C.a_true_norm; C.ad = zeros(1, nS);
    for q = 1:nS
        r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
        AH(:,q) = r.a_bar_hat_out(:,3); E(:,q) = AH(:,q) - r.a_true_out(:,3)/ad; P44(:,q) = r.P_a_out(:,3)/ad;
        for f = ZF; C.(f{1})(:,q) = r.(f{1})(:,3); end
        C.a_true_norm(:,q) = r.a_true_out(:,3)/ad; C.h_bar_true(:,q) = r.h_bar_true_out(:,1); C.ad(q) = ad;
        C.trk_true(:,q) = (r.p_d_out(:,3) - r.p_true_out(:,3)) / r.R;
        C.P33_sqrt(:,q) = sqrt(squeeze(r.P_full_out(:,3,3,3))); R.runs{q}.P_full_out = [];
    end
    hd = R.runs{1}.p_d_out(:,3)/R.runs{1}.R;
    left = struct('t',t,'E',E,'sdE',std(AH,0,2),'sP',mean(P44,2),'hd',hd,'L41',C.K_a_y1_out,'L42',C.K_a_y2_out,'C',C);
    fprintf('[canon aptrue Euler] health: min w %.4f | min a_hat %.5f | NaN %d | E(0) %+.1e | hold E %+.5f (SEM %.5f)\n', ...
        min(C.h_bar_true(:)), min(AH(:)), sum(~isfinite(E(:))), mean(E(1,:)), mean(E(t>3.5,:),'all'), std(mean(E(t>3.5,:),1))/sqrt(nS));
    S = load(fullfile(od, 'arms10_seedtruth_comp_canon.mat')); right = S.aptrue;
    fprintf('[canon aptrue arm 4 ] hold E %+.5f (SEM %.5f)  | paired Euler - arm4 hold %+.5f (SEM %.5f)\n', ...
        mean(right.E(t>3.5,:),'all'), std(mean(right.E(t>3.5,:),1))/sqrt(nS), ...
        mean(mean(E(t>3.5,:),1) - mean(right.E(t>3.5,:),1)), std(mean(E(t>3.5,:),1) - mean(right.E(t>3.5,:),1))/sqrt(nS));
    out = struct('left', left, 'right', right, 'seeds', SEEDS, 'ws0', ws0);
    save(fullfile(od, 'aptrue_canon_lr.mat'), '-struct', 'out', '-v7.3'); fprintf('saved aptrue_canon_lr.mat\n');
end
