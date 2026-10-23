% STATUS: ACTIVE (scratch) | PURPOSE: per-step PREDICT-stage error increment of
%   the a'_true (exogenous slope @ commanded height) arm with law_exact_step,
%   captured through model/diag/obs_dump.m (x_pred / x_upd / P_upd per step),
%   so the mean drift the predict injects can be compared term by term with the
%   second-order expansion in derivation/0902_formC_aptrue_4state.tex WITHOUT
%   the update's pull-back confounding it (the open-loop sum used on 2026-09-01
%   could not separate the two).
%   Truth side comes from the driver (a_true_out, h_bar_true_out, p_d/p_true,
%   f_bar_out); estimator side from obs_dump (one seed per run: the buffer
%   holds the LAST seed only, see obs_dump.m MULTI-SEED).
%   Output: test_results/apd_acov_meng/aptrue_predict_drift_probe.mat
%   Analysis lives in the companion analyze_aptrue_predict_drift.m.
function probe_aptrue_predict_drift(seeds)
    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();
    cfg0 = canonical_scenario(0.05, 1.1, 'deep');
    w0bar = cfg0.h_init / pc.R;
    [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));                     % seed = truth (canon: 0.98099)
    fprintf('[probe] canon deep, ws0_perp %.5f, law_exact_step ON, obs_dump ON, seeds %s\n', ws0, mat2str(seeds));
    S = struct('seed', {}, 'k_rec', {}, 'x_pred', {}, 'x_upd', {}, 'P_upd', {}, 'P_pred', {}, ...
               'a_true', {}, 'h_bar_true', {}, 'trk_true', {}, 'f_bar', {}, 'a_prime', {}, ...
               'a_bar_hat', {}, 'hd', {}, 't', {}, 'ad', {}, 'f_th', {}, 'a_bar_true_row', {});
    for q = 1:numel(seeds)
        clear run_formC_b motion_control_law_formC_b;
        o = struct('arm','best','ap_known',true,'ap_known_at','cmd', ...
                   'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'obs_dump',true), ...
                   'config_override',struct(),'scenario','deep','verbose',false,'seeds',seeds(q),'log_P_full',false);
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');
        Lz = L([L.ax] == 3);
        n = numel(Lz);
        r = R.runs{1};
        % the driver's row 1 is init-only, so a full capture has N-1 records
        assert(n == numel(r.tout) - 1, 'obs_dump holds %d z-records, expected %d (buffer reset mid-run?)', n, numel(r.tout) - 1);  ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3);
        s = struct();
        s.seed   = seeds(q);
        s.k_rec  = [Lz.k].';
        s.x_pred = cell2mat(arrayfun(@(z) z.x_pred(:).', Lz, 'UniformOutput', false).');   % n x nstate
        s.x_upd  = cell2mat(arrayfun(@(z) z.x_upd(:).',  Lz, 'UniformOutput', false).');
        np = size(Lz(1).P_upd, 1);
        s.P_upd  = zeros(n, np, np);  s.P_pred = zeros(n, np, np);
        for j = 1:n; s.P_upd(j,:,:) = Lz(j).P_upd; s.P_pred(j,:,:) = Lz(j).P_pred; end
        s.a_true     = r.a_true_out(:,3) / ad;                 % abar_true at the true height
        s.h_bar_true = r.h_bar_true_out(:,1);
        s.trk_true   = (r.p_d_out(:,3) - r.p_true_out(:,3)) / r.R;   % true tracking error [R]
        s.f_bar      = r.f_bar_out(:,3);                       % applied normalized force
        s.a_prime    = r.a_prime_out(:,3) / ad;                % exogenous slope used, dabar/dwbar
        s.a_bar_hat  = r.a_bar_hat_out(:,3);
        s.hd         = r.p_d_out(:,3) / r.R;
        s.t          = r.tout(:);
        s.ad         = ad;
        s.f_th       = r.F_th_out(:,3);                        % thermal force applied at row k [pN] -> true kick w_T[k] = a_bar[k] a_o f_th[k]
        s.a_bar_true_row = r.a_true_out(:,3) / ad;
        S(end+1) = s;
        fprintf('[probe] seed %2d: %d z-records, nstate %d, driver N %d, a_bar_hat match to x_upd(4): ', seeds(q), n, np, numel(s.t));
        % alignment check: x_upd(4) must reproduce a_bar_hat_out exactly at some offset
        best = NaN; for off = -2:2
            if off >= 0; a = s.x_upd(1:end-off,4); b = s.a_bar_hat(1+off:end); else; a = s.x_upd(1-off:end,4); b = s.a_bar_hat(1:end+off); end
            m = min(numel(a), numel(b)); d = max(abs(a(1:m) - b(1:m)));
            if d < 1e-12; best = off; end
        end
        fprintf('offset %s\n', mat2str(best));
    end
    save(fullfile(od, 'aptrue_predict_drift_probe.mat'), 'S', 'ws0', '-v7.3');
    fprintf('[probe] saved aptrue_predict_drift_probe.mat\n');
end
