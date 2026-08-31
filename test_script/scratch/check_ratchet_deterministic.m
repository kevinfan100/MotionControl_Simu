function out = check_ratchet_deterministic()
%CHECK_RATCHET_DETERMINISTIC  The Euler ratchet with every random input off.
%   thermal off, measurement noise off, y2 off, one seed: the plant and the
%   filter are deterministic, so the ONLY things that can move a_hat away from
%   a_true over the closed oscillation path are (i) the quadrature of the law
%   step, (ii) the b anchor (<= 1 pp, btrue arm 08-30) and (iii) the y1 leg
%   acting on the deterministic tracking error.
%   Registered prediction: Euler arm ends the oscillation above a_true by
%   about the ratchet -1/2 sum a'' dw^2 computed from its own log (within 2x,
%   the amplifier); the exact-step arm ends within +-2 % of a_true.
% STATUS: ACTIVE | sanity check for law_exact_step (2026-08-30)

    ax = 3;
    CO = struct('thermal_enable', false, 'meas_noise_enable', false);
    ARMS = {'Euler (best)', struct(); 'exact step', struct('law_exact_step', true)};
    out = struct();
    for a = 1:2
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, ...
                               'y2_on', false, 'config_override', CO, 'ctrl_const_override', ARMS{a,2}));
        r = O.runs{1};  a_nom = r.a_nom;  t = r.tout(:);
        ah = r.a_bar_hat_out(:, ax);  at = r.a_true_out(:, ax)/a_nom;  bh = r.b_hat_out(:, ax);
        dx3 = r.delta_x_hat_3_out(:, ax) / physical_constants().R;  dwd = [0; diff(r.h_bar_d_out(:))];
        K1 = r.K_a_y1_out(:, ax);  n1 = r.innov_y1_out(:, ax);
        idx = find(t > 1.60 & t < 3.40);  k0 = idx(1);  k1 = idx(end);  kk = (k0+1):k1;
        dw  = dwd(kk) + 0.3 * dx3(kk-1);
        rat = 0.5 * sum(2 * bh(kk-1).^2 .* (1 - ah(kk-1)).^3 .* dw.^2);   % -1/2 sum a'' dw^2
        law = sum(bh(kk-1) .* (1 - ah(kk-1)).^2 .* dw);
        y1  = sum(K1(kk) .* n1(kk));
        ho  = t > 3.70;
        fprintf('%-14s osc: d a_hat %+.4f  d a_true %+.4f  | law leg %+.4f  y1 leg %+.4f  resid %+.4f | ratchet %+.4f a_o\n', ...
                ARMS{a,1}, ah(k1)-ah(k0), at(k1)-at(k0), law, y1, (ah(k1)-ah(k0))-(law+y1), rat);
        fprintf('%-14s end-hold a_hat/a_true - 1 = %+.2f %%   (descend end %+.2f %%)\n', '', ...
                100*(mean(ah(ho))/mean(at(ho)) - 1), 100*(ah(k0)/at(k0) - 1));
        out.(matlab.lang.makeValidName(ARMS{a,1})) = struct('t', t, 'ah', ah, 'at', at, 'rat', rat, 'law', law, 'y1', y1);
    end
    fprintf('CHECK DONE\n');
end
