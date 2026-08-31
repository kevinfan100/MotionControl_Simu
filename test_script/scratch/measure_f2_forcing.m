function out = measure_f2_forcing(seeds)
%MEASURE_F2_FORCING  Measure the closed-loop covariance forcing directly and
%   push it through the validated loop (2026-08-31).
%
%   The true displacement per step is  dw = a(w) * fbar + w_T.  The model
%   carries a(.)*fbar at point values; the second-order mean term it misses is
%       g3[k] = Cov( a_bar(w[k]), fbar[k] )        [R per step]
%   Here g3 is MEASURED as the cross-seed covariance at every step (8 seeds,
%   b_true + exact base) -- no hand-derived coefficient, so the velocity
%   structure and the hold behaviour come out of the data.  The derivation's
%   job (later) is to REPRODUCE this measured g3 in closed form.
%
%   Instrument check first:  mean(a*fbar) must reproduce the commanded step
%   during motion (validates units/normalisation of f_bar_out).
%   Then g3 is propagated through the PASS-stamped seed-7 loop
%   (capture_loop_matrices) and compared with the four signatures:
%   trough bias +0.013 a_o | rms nubar 2.3e-4 | velocity-locked | hold flat.
% STATUS: ACTIVE | closed-loop mean-bias derivation line

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'loop_mean_bias');
    ax = 3;
    KEEP = {'a_true_out', 'f_bar_out', 'h_bar_true_out', 'h_bar_d_out', 'a_bar_hat_out', 'innov_y1_out', 'tout'};
    O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                           'b_true', true, 'b_true_at', 'cmd', ...
                           'ctrl_const_override', struct('law_exact_step', true)));
    ns = numel(O.runs);  a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
    G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, min(ax, size(r.(f), 2))), O.runs, 'UniformOutput', false), 1, []));
    AB = G('a_true_out') / a_nom;                      % true a_bar, per seed
    FB = G('f_bar_out');                               % normalized force
    hd = O.runs{1}.h_bar_d_out(:);  dwd = [0; diff(hd)];

    % ---- instrument check: a*fbar reproduces the commanded step ------------
    disp_pred = mean(AB .* FB, 2);
    mm = abs(dwd) > 1e-4;
    sc = disp_pred(mm) \ dwd(mm);
    fprintf('[instrument] regress dwd on mean(a*fbar) over motion: scale %.4f  corr %.4f  (expect ~1)\n', ...
            sc, corr(disp_pred(mm), dwd(mm)));
    assert(abs(sc - 1) < 0.2, 'f_bar normalisation not understood -- stop');

    % ---- the measured forcing ---------------------------------------------
    ABc = AB - mean(AB, 2);  FBc = FB - mean(FB, 2);
    g3_raw = sum(ABc .* FBc, 2) / (ns - 1);            % Cov_seeds(a_bar, fbar) per step [R]
    g3 = movmean(g3_raw, 81);
    m_osc = t >= 1.5 & t < 3.5;  m_hold = t > 3.7;  m_h0 = t < 0.5;
    fprintf('[g3] osc mean %+.3e (per step)  hold-end mean %+.3e  first-hold mean %+.3e   sum over osc %+.4f R\n', ...
            mean(g3(m_osc)), mean(g3(m_hold)), mean(g3(m_h0)), sum(g3_raw(m_osc)));
    % velocity structure: correlate with dwd and with |dwd|
    fprintf('[g3] corr with dwd %+.3f   with |dwd| %+.3f   (osc)\n', ...
            corr(g3_raw(m_osc), dwd(m_osc)), corr(g3_raw(m_osc), abs(dwd(m_osc))));

    % ---- propagate through the validated loop (seed 7 capture) -------------
    C = load(fullfile(od, 'loop_capture_seed7.mat'));
    assert(C.ok, 'loop capture not PASS-stamped');
    nsC = size(C.F, 1);  n = size(C.F, 3);  H1 = zeros(1, nsC);  H1(1) = 1;
    gk = interp1(t, g3, C.t(:), 'nearest', 'extrap');
    e = zeros(nsC, 1);  E4 = zeros(n, 1);  NB = zeros(n, 1);
    for i = 1:n
        ep = C.F(:,:,i) * e + [0; 0; gk(i); zeros(nsC - 3, 1)];
        NB(i) = H1 * ep;
        e = ep - C.K1(:,i) * (H1 * ep);
        e = e - C.K2(:,i) * (C.H2S(:,i).' * e);
        E4(i) = e(4);
    end
    tc = C.t(:);
    mo = tc >= 1.5 & tc < 3.5;  mh = tc > 3.7;
    bias_end = -mean(E4(mh));
    drift_h = (-mean(E4(tc > 4.3)) + mean(E4(tc > 3.7 & tc < 4.0))) / 0.6;
    S8 = load(fullfile(od, '..', 'btrue_log_ledger', 'check_btrue_log_ledger.mat'));
    nm = mean(S8.out.noisyExact.n1(2:end,:), 2) - S8.out.detExact.n1(2:end);
    nm = interp1(S8.out.noisyExact.tt(:), movmean(nm, 81), tc, 'nearest', 'extrap');
    fprintf('\n[PROPAGATED measured g3]\n');
    fprintf('  predicted trough bias  %+.4f a_o   (measured +0.013)\n', bias_end);
    fprintf('  predicted hold drift   %+.4f a_o/s (measured ~0)\n', drift_h);
    fprintf('  rms nubar (osc)        %.2e     (measured 2.3e-4)\n', rms(NB(mo)));
    fprintf('  shape corr with measured nubar  %+.2f\n', corr(movmean(NB(mo), 81), nm(mo)));
    out = struct('t', t, 'g3', g3, 'g3_raw', g3_raw, 'E4', E4, 'NB', NB, 'tc', tc, ...
                 'bias_end', bias_end, 'drift_h', drift_h);
    save(fullfile(od, 'measure_f2_forcing.mat'), 'out');
    fprintf('F2 MEASURE DONE\n');
end
