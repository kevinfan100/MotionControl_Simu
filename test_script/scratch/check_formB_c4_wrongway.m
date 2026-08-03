% check_formB_c4_wrongway.m -- PURPOSE: ensemble audit of the doc-c4
%   wrong-direction ws_hat drag (user challenge 2026-08-02: "I don't buy that
%   per-run spread this large is physics; and in c4 the estimator first fails
%   to move toward the injected wall, or moves the WRONG way").
%   Two-step audit, step 2 (derivation-defined quantities vs logs):
%     (a) far-field hold [0, 0.5] s: max |ws_hat - 1|  -- the derivation says
%         zero sensitivity there, so ANY motion would be dishonest;
%     (b) per-phase delta ws_hat (descent / osc / hold) with cross-seed sign
%         counts -- same-sign across fresh seeds = systematic channel (bug),
%         random sign = posterior noise realization;
%     (c) dws_y1 / dws_y2 ledger attribution per phase + closure residual;
%     (d) end-of-run z = err / claimed sqrtP honesty (fresh 3000-band seeds);
%     (e) paired inject/no-inject response vs the Bayes-shrinkage expectation.
%   EXPIRES: verdict recorded (memory / formB doc); no permanent artifact
%   depends on this file.
%   NOTE: run_formB_ws saves its own arm .mat (t1_y2on tag) -- re-run the
%   default 6-seed arm afterwards if the cached arm-1 file matters.

here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
pdirs = {'model', 'model/dual_track', 'model/diag', 'model/thermal_force', ...
         'model/config', 'model/wall_effect', 'model/controller', ...
         'model/trajectory', 'test_script/integration'};
for i = 1:numel(pdirs); addpath(fullfile(proj, pdirs{i})); end
res_file = fullfile(proj, 'test_results', 'check_formB_c4_wrongway.mat');

AX      = 3;                       % z / perpendicular
N_SEED  = 15;
SEEDS   = 3000 + (1:N_SEED);       % fresh band (1:20 band is burned, see memory)
INJ     = 0.05;
WS_TRUE = 1 + INJ;
ARMS    = [INJ, 0];                % arm 1 = injected, arm 2 = paired baseline
T_PH    = [0.5, 1.5, 3.5];         % far-hold | descent | osc | final-hold edges
ov = struct('lock_b', false, 'lock_p', true, 'lock_ws', false, 'Pf_ws_std', 0.111);

if exist(res_file, 'file')
    load(res_file, 'R');
else
    R = struct('done', zeros(2, N_SEED));
end
for a = 1:2
    for i = 1:N_SEED
        if R.done(a, i); continue; end
        o = struct('seeds', SEEDS(i), 'ws_inject', ARMS(a), 'verbose', false, ...
                   'ctrl_const_override', ov);
        out = run_formB_ws(o);
        r  = out.runs{1};  t = r.tout(:);
        ws = r.ws_hat_out(:, AX);  sqP = r.P_ws_out(:, AX);
        y1 = r.dws_y1_out(:, AX);  y2  = r.dws_y2_out(:, AX);
        ph = 1 + (t >= T_PH(1)) + (t >= T_PH(2)) + (t >= T_PH(3));
        for p = 1:4
            k0 = find(ph == p, 1, 'first');  k1 = find(ph == p, 1, 'last');
            R.dws_tr(a, i, p) = ws(k1) - ws(k0);
            R.dws_y1(a, i, p) = sum(y1(ph == p));
            R.dws_y2(a, i, p) = sum(y2(ph == p));
        end
        R.ws_end(a, i)     = ws(end);
        R.sqP_end(a, i)    = sqP(end);
        R.b_end(a, i)      = r.b_hat_out(end, AX);
        R.hold_a_err(a, i) = out.metrics.rows(1, 3);   % hold mean gain err [%]
        R.far_maxdev(a, i) = max(abs(ws(ph == 1) - 1));
        R.done(a, i) = 1;  save(res_file, 'R');
        fprintf('arm %d seed %d: ws_end %.4f (sqP %.4f)\n', a, SEEDS(i), ws(end), sqP(end));
    end
end

% ---------------- statistics ----------------
err = R.ws_end(1, :) - WS_TRUE;
sq  = R.sqP_end(1, :);
z   = err ./ sq;
fprintf('\n== c4 ensemble, N = %d fresh seeds, true ws = %.2f ==\n', N_SEED, WS_TRUE);
fprintf('end err : mean %+.4f (SEM %.4f)  std %.4f | claimed sqrtP %.4f | honesty std/sqrtP %.2f\n', ...
        mean(err), std(err)/sqrt(N_SEED), std(err), mean(sq), std(err)/mean(sq));
fprintf('z-score : mean %+.2f  std %.2f | |z|>2: %d/%d | wrong side (ws_end < 1): %d/%d\n', ...
        mean(z), std(z), nnz(abs(z) > 2), N_SEED, nnz(R.ws_end(1, :) < 1), N_SEED);
resp = (R.ws_end(1, :) - R.ws_end(2, :)) / INJ;
fprintf('paired injection response: mean %.2f  std %.2f  (Bayes-shrinkage expectation ~0.6)\n', ...
        mean(resp), std(resp));
fprintf('far-hold max |ws-1| over seeds: %.2e  (structural zero-information check)\n', ...
        max(R.far_maxdev(1, :)));
pn = {'far', 'desc', 'osc', 'hold'};
for p = 2:4
    d  = squeeze(R.dws_tr(1, :, p));
    d1 = squeeze(R.dws_y1(1, :, p));
    d2 = squeeze(R.dws_y2(1, :, p));
    clos = max(abs(d(:) - (d1(:) + d2(:))));
    fprintf('%4s: dws mean %+.4f std %.4f  sign>0 %2d/%d | ledger y1 %+.4f y2 %+.4f | closure %.1e\n', ...
            pn{p}, mean(d), std(d), nnz(d > 0), N_SEED, mean(d1), mean(d2), clos);
end
dh = squeeze(R.dws_tr(1, :, 4));
fprintf('hold-phase drift t-stat: %+.2f  (|t| > 2 = systematic channel)\n', ...
        mean(dh) / (std(dh)/sqrt(N_SEED)));
cb = corrcoef(err(:), R.b_end(1, :)');    ca = corrcoef(err(:), R.hold_a_err(1, :)');
fprintf('corr(ws err, b_end) %+.2f | corr(ws err, hold gain err) %+.2f\n', cb(1,2), ca(1,2));
