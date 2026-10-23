% STATUS: ACTIVE (scratch) | PURPOSE: can an INITIAL gain offset be corrected?
%   (2026-09-02, user request, WORKING-TREE controller/driver as of 12:37 incl.
%   the uncommitted pred_mean2 / ap_known_at='est' / app_known knobs).
%   Two arms, same seeds, exact law step in both:
%     A4  a'_true fully compensated ladder arm 4 (run_aptrue_cheat_ladder.m):
%         ap_known @est + app_known + pred_mean2 + lock_b      (slope never reads a_hat)
%     BT  b_true arm: law on, b exact                            (slope = b (1-a_hat)^2)
%   The seed a_hat[0] is placed at (1+delta) x a_bar_true(w_bar[0]) through ws0_perp
%   (seed line a_bar = 1 - 1/((8/9)(w_bar - (ws0-1)))). The filter's own prior is NOT
%   touched: it is a WALL-POSITION prior (Pf_w0_std = 0.111 R), so each delta is also
%   printed as the implied wall offset and as a multiple of sqrt(P44[0]).
%   Health (before any bias number): E(0) must equal the requested offset (seed
%   wiring), min a_bar_hat, NaN count. delta = 0 is the negative control and must
%   reproduce the ladder arm-4 / b_true exact numbers.
%   Usage: run_ainit_offset_pair('canon')  |  run_ainit_offset_pair('meng')
function out = run_ainit_offset_pair(traj, seeds, DEL)
    if nargin < 1 || isempty(traj);  traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:8;   end
    if nargin < 3 || isempty(DEL);   DEL = [-0.20 -0.05 -0.01 0 +0.01 +0.05]; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();
    switch traj
        case 'canon'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep'); w0bar = cfg0.h_init / pc.R; OV = struct();
            t_hold = 3.5;
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10, ...
                        'T_sim',12.5,'h_min',2.475);
            w0bar = OV.h_init / pc.R; t_hold = 10.6;
    end
    [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    ws0_true = 1 + w0bar - 1/((8/9)*(1 - at));
    base = struct('arm','best','config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',false);
    A4 = base; A4.ap_known = true; A4.ap_known_at = 'est'; A4.app_known = true;
    A4.ctrl_const_override = struct('lock_b',true,'law_exact_step',true,'pred_mean2',true);
    BT = base; BT.b_true = true; BT.b_true_at = 'true';
    BT.ctrl_const_override = struct('law_exact_step',true);
    ARM = {A4, BT}; AN = {'A4 a''_true compensated', 'BT b_true exact'};
    nS = numel(seeds); nD = numel(DEL);
    out = struct('traj',traj,'DEL',DEL,'seeds',seeds,'arms',{AN},'ws0_true',ws0_true,'a0_true',at);
    E = cell(2,nD); t = [];
    fprintf('\n[%s] w0bar %.3f  a_bar_true(0) %.4f  ws0_true %.5f\n', traj, w0bar, at, ws0_true);
    for a = 1:2
        for i = 1:nD
            a0 = (1 + DEL(i)) * at;
            if a0 >= 1; fprintf('  skip delta %+.2f: a0 = %.3f >= 1\n', DEL(i), a0); E{a,i} = []; continue; end
            ws0 = 1 + w0bar - 1/((8/9)*(1 - a0));
            o = ARM{a}; o.ctrl_const_override.ws0_perp = ws0;
            clear run_formC_b motion_control_law_formC_b;
            evalc('R = run_formC_b(o);');
            t = R.runs{1}.tout(:); N = numel(t); Ei = zeros(N, nS); sP0 = zeros(1,nS);
            for q = 1:nS
                r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
                Ei(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad;
                sP0(q) = r.P_a_out(1,3)/ad;
            end
            E{a,i} = Ei;
            mh = t > t_hold; e0 = mean(Ei(1,:)); eh = mean(Ei(mh,:),'all');
            fprintf('  %-24s delta %+5.2f  a0 %.4f  wall off %+7.3f R  = %6.1f sigma | E(0) %+.5f (req %+.5f) | min a_hat %.4f NaN %d | hold %+.5f (SEM %.5f)  recovered %5.1f%%\n', ...
                AN{a}, DEL(i), a0, ws0 - ws0_true, abs(a0-at)/mean(sP0), e0, a0-at, ...
                min(min(cellfun(@(x) min(x.a_bar_hat_out(:,3)), R.runs))), sum(~isfinite(Ei(:))), ...
                eh, std(mean(Ei(mh,:),1))/sqrt(nS), 100*(1 - eh/e0)*(e0~=0));
            clear R;
        end
    end
    out.t = t; out.E = E; out.t_hold = t_hold;
    save(fullfile(od, sprintf('ainit_offset_pair_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('saved ainit_offset_pair_%s.mat\n', traj);
end
