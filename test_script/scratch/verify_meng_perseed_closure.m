function out = verify_meng_perseed_closure()
%VERIFY_MENG_PERSEED_CLOSURE  Two checks against the arms30 page (2026-08-31).
%
%   (a) Do our own 6 Meng b_true seeds reproduce the 30-seed page's numbers?
%       (bump ~+0.026 a_o at 8.5-9 s, hold ~+0.010, spread to 0.03)
%   (b) PER-SEED closure (the correct form after the ensemble-replay lesson):
%       seed 7's own loop A_k + its own FULL forcing G_tot -> tautology gate;
%       then replay only the SLOW part of its forcing (0.5 s moving mean of
%       rows 3+4; the noise stays out) and compare with the seed's own slow
%       e4 trajectory. If the slow bias trajectory (the bump) is the loop's
%       response to the slow mean forcing, the two curves should track.
% STATUS: ACTIVE | closed-loop mean-bias line, Meng leg

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    ax = 3;  lam = 0.7;  pcR = physical_constants().R;

    % ---- (a) our 6-seed numbers vs the 30-seed page ------------------------
    S = load(fullfile(od, 'meng_btrue_profile.mat'));
    t = S.t(:);  bias = -S.E4A;                                  % bias = estimate - truth = -e4
    mb = t > 8.3 & t < 9.3;  mh = t > 11.5;
    fprintf('(a) 6-seed vs arms30 page:\n');
    fprintf('  bump 8.3-9.3 s:  mean %+0.4f +- %0.4f a_o   (page: ~+0.026)\n', ...
            mean(mean(bias(mb,:),1)), std(mean(bias(mb,:),1))/sqrt(size(bias,2)));
    fprintf('  hold >11.5 s:    mean %+0.4f +- %0.4f a_o   (page: ~+0.010)\n', ...
            mean(mean(bias(mh,:),1)), std(mean(bias(mh,:),1))/sqrt(size(bias,2)));
    fprintf('  spread max:      %0.3f a_o                   (page: ~0.03)\n', max(std(bias, 0, 2)));
    [~, ipk] = max(movmean(mean(bias, 2), 801));
    fprintf('  bump peak at t = %.2f s   |mean forcing| peak (round 1) = 9.02 s\n', t(ipk));

    % ---- (b) per-seed closure, seed 7 --------------------------------------
    ov = struct('trajectory_type','osc','h_init',15.0,'h_bottom',2.5,'amplitude',0,'frequency',1,'n_cycles',1, ...
                't_hold',0.5,'t_descend_override',10.0,'T_sim',12.5,'h_min',1.1*pcR);
    O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, ...
                           'b_true', true, 'b_true_at', 'true', 'config_override', ov, ...
                           'ctrl_const_override', struct('law_exact_step', true, 'obs_dump', true)));
    r = O.runs{1};  L = obs_dump('get');  Lz = L([L.ax] == ax);
    n = numel(Lz);  nsl = numel(Lz(1).x_pred);  a_nom = r.a_nom;
    ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);  dw_true = hd - ht;
    wT = r.a_true_out(:, ax) .* r.F_th_out(:, ax) / pcR;
    H1 = zeros(1, nsl);  H1(1) = 1;
    A = zeros(nsl, nsl, n);  EU = zeros(nsl, n);  KK = zeros(1, n);
    for i = 1:n
        s = Lz(i);  k = s.k + 1;  KK(i) = k;
        P = s.P_pred;
        k1 = (P * H1.') / (H1 * P * H1.' + s.R(1));
        P1 = (eye(nsl) - k1 * H1) * P;
        H2 = s.H{2};  I2 = eye(nsl);
        if ~isempty(H2) && ~s.gate
            k2 = (P1 * H2.') / (H2 * P1 * H2.' + s.R(2));
            I2 = eye(nsl) - k2 * H2;
        end
        A(:,:,i) = I2 * (eye(nsl) - k1 * H1) * s.F;
        EU(:, i) = [dw_true(max(k-2,1)); dw_true(max(k-1,1)); dw_true(min(k,numel(dw_true))); ...
                    r.a_true_out(min(k,size(r.a_true_out,1)), ax)/a_nom; s.x_upd(5:7); wT(min(k,numel(wT))); wT(max(k-1,1))] - s.x_upd;
    end
    i0 = find(KK >= 3, 1);
    Gt = zeros(nsl, n);
    for i = i0+1:n; Gt(:, i) = EU(:, i) - A(:,:,i) * EU(:, i-1); end
    % tautology gate
    e = EU(:, i0);  gmax = 0;
    for i = i0+1:n; e = A(:,:,i)*e + Gt(:,i); gmax = max(gmax, max(abs(e - EU(:,i)))); end
    fprintf('\n(b) seed 7 per-seed closure: tautology gate %.2e\n', gmax);
    % slow-forcing replay: rows 3+4, 0.5 s moving mean, noise left out
    W = 801;
    g3s = movmean(Gt(3, :).', W);  g4s = movmean(Gt(4, :).', W);
    e = zeros(nsl, 1);  E4s = zeros(n, 1);
    for i = i0+1:n
        g = zeros(nsl, 1);  g(3) = g3s(i);  g(4) = g4s(i);
        e = A(:,:,i) * e + g;  E4s(i) = e(4);
    end
    tb = r.tout(min(KK, numel(r.tout)));  tb = tb(:);
    act_slow = movmean(EU(4, :).', W);
    mb7 = tb > 8.3 & tb < 9.3;  mh7 = tb > 11.5;
    fprintf('  slow replay vs seed-7 slow e4:  bump  pred %+0.4f  act %+0.4f | hold  pred %+0.4f  act %+0.4f | corr(t>2) %+0.2f\n', ...
            mean(E4s(mb7)), mean(act_slow(mb7)), mean(E4s(mh7)), mean(act_slow(mh7)), corr(E4s(tb > 2), act_slow(tb > 2)));
    out = struct('t', t, 'bias6', bias, 'tb', tb, 'E4s', E4s, 'act_slow', act_slow, 'Gt3', Gt(3,:).', 'Gt4', Gt(4,:).');
    save(fullfile(od, 'meng_perseed_closure.mat'), 'out');
    fprintf('PERSEED CLOSURE DONE\n');
end
