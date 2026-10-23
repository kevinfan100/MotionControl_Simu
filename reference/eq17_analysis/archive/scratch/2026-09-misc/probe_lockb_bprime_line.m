% FORK OF test_script/scratch/probe_btrue_e4_line.m (2026-09-04) | PURPOSE: the locked-b arm (b pinned at 8/9, four blocks)
%   showed a paired hold-slope difference to the b_true-curve arm of -0.176 +- 0.025 e-6/step (canon). Candidate
%   mechanism (the S10 verifier's note, now S11): e_b = b_true(w_bar) - 8/9 depends on the TRUE height, so
%       Cov(e_b, e_D) = -b'_true(w_bar) Cov(e3, e_D) = -b'_true [ (1-lc)(P33 - P13) + alpha (P38 + P39) + F_dw P34 ]
%   and the error dynamics carry (d a'/d b) Cov(e_b, e_D) = (1 - a_hat)^2 (-b'_true) [...] per step (true - est), which
%   nothing in the filter compensates (b'_true is not c-free). Prediction of the est - true hold slope = minus that.
%   Reports the per-segment mean (e-6/step) and the running sum, from the base P log of the locked arm (log_P_full),
%   b'_true by central difference of a_true'/(1-a_true)^2 on the plant curve at the true height (oracle, probe only).
%   Same F_dw approximation as probe_btrue_e4_line (raw force history). | EXPIRES: with the production rung
function out = probe_lockb_bprime_line(traj, seeds)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    if nargin < 2 || isempty(seeds); seeds = 1:10; end
    traj = lower(traj);
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  EXTRA = 4.0;
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                        'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5 + EXTRA,'h_min',2.475);
            cfg0 = OV;
        case 'canon'
            cfg0 = canonical_scenario(0.05, 1.1, 'deep');  OV = struct('T_sim', cfg0.T_sim + EXTRA);  cfg0.T_sim = cfg0.T_sim + EXTRA;
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
    t1 = cfg0.t_hold;  t2 = t1 + cfg0.t_descend_override;  t3 = t2 + cfg0.n_cycles/cfg0.frequency;
    % plant curve b_true(w) and b'_true(w) on a fine grid
    wg = linspace(1.0, 8, 40000);  ag = zeros(size(wg));
    for i = 1:numel(wg); [~, c] = calc_correction_functions(wg(i)); ag(i) = 1/c; end
    apg = gradient(ag, wg);  bg = apg ./ (1 - ag).^2;  bpg = gradient(bg, wg);
    cc = struct('ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true,'nw_mcorr',true,'pred_mean2_e4',true,'fe44_Aa_scale',1);
    o = struct('arm','bmid','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds,'log_P_full',true);
    clear run_formC_b motion_control_law_formC_b;
    evalc('R = run_formC_b(o);');
    lc = R.cfg.lambda_c;  alpha = 1 - lc;  nS = numel(seeds);
    t = R.runs{1}.tout(:);  N = numel(t);
    TERM = zeros(N, nS, 2);  E = zeros(N, nS);  BP = zeros(N, nS);   % TERM: [b'-line est-true per step, e_b first-order line f*dW_hat est-true]
    for q = 1:nS
        rr = R.runs{q};  P = rr.P_full_out;  np = size(P, 2);
        ah = rr.a_bar_hat_out(:,3);  fb = rr.f_bar_out(:,3);  hb = rr.h_bar_true_out(:,1);  bu = rr.b_hat_out(:,3);
        ad = rr.a_hat_out(1,3)/rr.a_bar_hat_out(1,3);  E(:,q) = ah - rr.a_true_out(:,3)/ad;
        bt = interp1(wg, bg, min(max(hb, wg(1)), wg(end)));  bp = interp1(wg, bpg, min(max(hb, wg(1)), wg(end)));  BP(:,q) = bp;
        for k = 2:N
            Pc = squeeze(P(k-1, :, :, 3));
            Fdw = fb(k-1);  if k >= 3; Fdw = Fdw + alpha * fb(k-2); end;  if k >= 4; Fdw = Fdw + alpha * fb(k-3); end
            a1 = 1 - ah(k-1);
            c3 = alpha * Pc(3,3) + Fdw * Pc(3,4);  if np >= 9; c3 = c3 + alpha * (Pc(3,8) + Pc(3,9)); end
            c3 = c3 - alpha * Pc(1,3);                                    % nw_mcorr u
            TERM(k,q,1) = -( a1^2 * (-bp(k-1)) * c3 );                    % est - true per step
            TERM(k,q,2) = -( a1^2 * (bt(k-1) - bu(k)) * (hb(k) - hb(k-1)) );   % first-order model line, est - true, on the true step
        end
    end
    clear R;
    LAB = {'b''-line  -(1-a)^2 (-b'') Cov(e3,u)', 'model line -(1-a)^2 (b_true - b_hat) dw'};
    SEG = {'far half', t > t1 & t <= t1 + 0.5*(t2-t1); 'near half', t > t1 + 0.5*(t2-t1) & t <= t2; ...
           'osc/last', t > t2 & t <= t3; 'hold 1st s', t > t3 & t <= t3 + 1; 'hold last 3 s', t > cfg0.T_sim - 3};
    fprintf('[%s lockb b''-line] seeds %s | est - true per step (e-6/step, SEM over seeds) | cum [a_bar]\n', traj, mat2str(seeds));
    for v = 1:2
        T = TERM(:,:,v);  fprintf('  %-40s | ', LAB{v});
        for s = 1:size(SEG,1); m = SEG{s,2}; ps = mean(T(m,:), 1); fprintf('%s %+.3f(%.3f) ', SEG{s,1}, 1e6*mean(ps), 1e6*std(ps)/sqrt(nS)); end
        cs = cumsum(T, 1);  fprintf('| cum @t2 %+.5f @t3 %+.5f @end %+.5f\n', mean(cs(find(t > t2, 1), :)), mean(cs(find(t > t3, 1), :)), mean(cs(end, :)));
    end
    mh = t > t3;  fprintf('  hold: mean b''_true %.4f | (1-a_hat)^2 %.4f\n', mean(BP(mh,:), 'all'), mean((1 - E(mh,:) - 0).^2, 'all'));
    m3 = t > cfg0.T_sim - 3;  k = (1:sum(m3)).';  sl = zeros(1,nS); for q = 1:nS; p = polyfit(k, E(m3,q), 1); sl(q) = p(1); end
    fprintf('  lockb arm measured here: last-3-s slope %+.3f e-6/step (SEM %.3f)\n', 1e6*mean(sl), 1e6*std(sl)/sqrt(nS));
    out = struct('traj', traj, 'seeds', seeds, 't', t, 'TERM', TERM, 'E', E, 'BP', BP, 'phases', [t1 t2 t3 cfg0.T_sim], 'lab', {LAB});
    save(fullfile(od, sprintf('probe_lockb_bprime_line_%s.mat', traj)), '-struct', 'out', '-v7.3');
    fprintf('[%s lockb b''-line] saved\n', traj);
end
