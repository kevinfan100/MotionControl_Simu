function out = replay_seed_attribution(seed)
%REPLAY_SEED_ATTRIBUTION  Extract the ACTUAL per-step forcing of one seed and
%   attribute the gain bias by ablation (2026-08-31).
%
%   e[k] = x_true[k] - x_filter[k] on slots [dw1 dw2 dw3 a_bar] (slot 5 is b,
%   zero in the b_true arm; 6-9 treated as filter-internal, folded into g).
%   The measured forcing is defined by the filter's own linearisation:
%       g_meas[k] = e_pred_actual[k] - F_k e_upd_actual[k-1]
%   REPLAY IDENTITY (instrument gate): running the loop recursion with g_meas
%   must reproduce e_upd_actual to machine precision on slots 1-4.
%   ABLATION: replay named components of g_meas alone --
%       g_mob   row 3 share  (a'/a_hat) * dw_jitter * (commanded+feedback step)
%               = the mobility-fluctuation x force term (the F2 family, now
%               PER SEED, so whatever y1 can see is absorbed by the replayed
%               loop exactly as in production)
%       g_rest  = g_meas - g_mob   (thermal kick w_T + everything else)
%   and read each component's end-hold e4 (gain bias) share.
% STATUS: ACTIVE | closed-loop mean-bias derivation line

    if nargin < 1 || isempty(seed); seed = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    C = load(fullfile(od, sprintf('loop_capture_seed%d.mat', seed)));
    assert(C.ok, 'capture not PASS-stamped');
    R = load(fullfile(od, sprintf('run_log_seed%d.mat', seed)));  r = R.run_log;
    ax = 3;  lam = 0.7;  ns = size(C.F, 1);  n = size(C.F, 3);  kk = C.KK;  t = C.t(:);
    a_nom = r.a_nom;

    % ---- true state aligned with the filter's slots (ALL 9) ----------------
    % Slots: 1-3 delayed/current tracking error, 4 a_bar, 5 b (e5 = 0 in the
    % b_true arm), 6-7 inert (truth := filter value, e = 0), 8-9 MA memory
    % (truth := the realised thermal steps w_T, per F_aug: m1 <- w_T, m2 <- m1).
    dw_true = r.h_bar_d_out(:) - r.h_bar_true_out(:);          % (w_d - w)/R, current
    pcR = physical_constants().R;
    wT = r.a_true_out(:, ax) .* r.F_th_out(:, ax) / pcR;       % realised thermal step [R]
    MA_OFF = 0;                                                 % w_T timing vs the memory shift; calibrated below
    XT = zeros(ns, n);
    for i = 1:n
        k = kk(i);
        XT(:, i) = [dw_true(max(k-2, 1)); dw_true(max(k-1, 1)); dw_true(k); r.a_true_out(k, ax) / a_nom; ...
                    C.XU(5:7, i); wT(max(k - MA_OFF, 1)); wT(max(k - 1 - MA_OFF, 1))];
    end
    EU = XT - C.XU;                                             % e_upd actual (all slots)
    EP = XT - C.XP;
    i0 = find(kk >= 3, 1);                                      % skip the startup padding rows

    % calibrate the w_T timing: pick MA_OFF in {0,1,2} minimising the identity error
    best = struct('err', inf, 'off', NaN);
    for off = 0:2
        for i = 1:n
            k = kk(i);
            XT(8, i) = wT(max(k - off, 1));  XT(9, i) = wT(max(k - 1 - off, 1));
        end
        EUo = XT - C.XU;  EPo = XT - C.XP;
        Gm = zeros(ns, n);
        for i = i0+1:n
            Gm(:, i) = EPo(:, i) - C.F(:, :, i) * EUo(:, i-1);
        end
        Erep = local_replay_from(C, Gm, n, i0, EUo(:, i0));
        err = max(abs(Erep(1:4, i0+1:end) - EUo(1:4, i0+1:end)), [], 'all');
        fprintf('  [MA_OFF %d] identity err %.3e\n', off, err);
        if err < best.err; best = struct('err', err, 'off', off, 'Gm', Gm, 'EU', EUo, 'EP', EPo); end
    end
    MA_OFF = best.off;  Gm = best.Gm;  EU = best.EU;  EP = best.EP; %#ok<NASGU>
    fprintf('  using MA_OFF = %d\n', MA_OFF);

    % ---- replay identity gate ---------------------------------------------
    rep = @(G) local_replay_from(C, G, n, i0, EU(:, i0));
    E_id = rep(Gm);
    e_gate = max(abs(E_id(1:4, i0+1:end) - EU(1:4, i0+1:end)), [], 'all');
    fprintf('[replay identity] max |replay - actual| on slots 1-4: %.2e  %s\n', e_gate, string(local_pf(e_gate < 1e-9)));

    % ---- ablation ----------------------------------------------------------
    ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
    ab = r.a_true_out(kk, ax) / a_nom;
    ap = bh .* (1 - ab).^2;
    dwd = [0; diff(r.h_bar_d_out(:))];  dwd = dwd(kk);
    x3h = C.XU(3, :).';
    step_cmd = dwd + (1 - lam) * x3h;                           % commanded + feedback step the force realises
    dw_jit = -(dw_true(kk) - movmean(dw_true(kk), 801));        % position jitter about its slow mean (w - <w>)
    g_mob = (ap ./ max(ah, 0.02)) .* dw_jit .* step_cmd;        % mobility-fluctuation x step  [R], sign: a(w) higher when w above mean
    G_mob = zeros(ns, n);  G_mob(3, :) = -g_mob.';              % row 3 is w_d - w: extra TRUE displacement reduces dw error
    G_mob(:, 1:i0) = 0;
    E_mob  = rep(G_mob);
    G_rest = Gm - G_mob;
    E_rest = rep(G_rest);

    mh = t > 3.7;  mo = t >= 1.5 & t < 3.5;
    EU = EU(1:4, :);  E_id = E_id(1:4, :);  E_mob = E_mob(1:4, :);  E_rest = E_rest(1:4, :);
    fprintf('\nend-hold e4 (truth - estimate; bias = -e4), seed %d:\n', seed);
    fprintf('  %-28s %+10.4f a_o\n', 'actual', mean(EU(4, mh)));
    fprintf('  %-28s %+10.4f\n', 'replay(g_meas)  [gate]', mean(E_id(4, mh)));
    fprintf('  %-28s %+10.4f\n', 'replay(g_mob)', mean(E_mob(4, mh)));
    fprintf('  %-28s %+10.4f\n', 'replay(g_rest)', mean(E_rest(4, mh)));
    fprintf('  additivity check: mob + rest - meas = %+.2e\n', mean(E_mob(4, mh)) + mean(E_rest(4, mh)) - mean(E_id(4, mh)));
    fprintf('  oscillation-only shares: mob %+0.4f  rest %+0.4f\n', mean(E_mob(4, mo)), mean(E_rest(4, mo)));
    out = struct('t', t, 'EU', EU, 'E_id', E_id, 'E_mob', E_mob, 'E_rest', E_rest, 'Gm', Gm, 'g_mob', g_mob);
    save(fullfile(od, sprintf('replay_attribution_seed%d.mat', seed)), 'out');
    fprintf('REPLAY ATTRIB DONE\n');
end

function E = local_replay_from(C, G, n, i0, e0)
    ns = size(C.F, 1);  H1 = zeros(1, ns);  H1(1) = 1;
    e = e0;  E = zeros(ns, n);  E(:, i0) = e0;
    for i = i0+1:n
        ep = C.F(:,:,i) * e + G(:, i);
        e = ep - C.K1(:,i) * (H1 * ep);
        e = e - C.K2(:,i) * (C.H2S(:,i).' * e);
        E(:, i) = e;
    end
end

function s = local_pf(ok); if ok; s = "PASS"; else; s = "FAIL"; end; end
