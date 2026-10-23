% STATUS: ACTIVE (scratch) | PURPOSE: obs_dump capture of the FINAL a'_true @est recipe
%   (law_exact_step + slope AND curvature read at the filter's own height + pred_mean2
%   + pred_mean2_kr1) on either trajectory, one seed per run, so the PREDICT-stage
%   remainder AFTER compensation can be decomposed term by term per segment
%   (analyze_aptrue_est_final.m). Lineage: probe_aptrue_predict_drift.m (canon, @cmd,
%   no mean2). Stores only the P entries the accounting needs, plus the driver's own
%   pred_mean2_out / K_dx_y1_out so the code's injection is read, not reconstructed.
%   Output: test_results/apd_acov_meng/aptrue_est_final_<traj>[_full].mat  (recipe kr1 | kr1_full)
function probe_aptrue_est_final(traj, seeds, recipe)
    if nargin < 1 || isempty(traj); traj = 'meng'; end
    if nargin < 2 || isempty(seeds); seeds = 1:8; end
    if nargin < 3 || isempty(recipe); recipe = 'kr1'; end   % 'kr1' (a'' (1-lc)^2 K31 R1) | 'kr1_full' (a'' (1-lc) K31 R1)
    assert(any(strcmp(recipe, {'kr1','kr1_full','mcorr'})), 'recipe must be kr1, kr1_full or mcorr');
    sfx = '';  if strcmp(recipe, 'kr1_full'); sfx = '_full'; elseif strcmp(recipe, 'mcorr'); sfx = '_mcorr'; end   % mcorr = pred_mean2 + nw_mcorr, no kr1
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();
    switch traj
        case 'meng'
            OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                        'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475);  cfg0 = OV;
        case 'canon'
            OV = struct();  cfg0 = canonical_scenario(0.05, 1.1, 'deep');
        otherwise
            error('traj must be meng or canon');
    end
    w0bar = cfg0.h_init / pc.R;  [~, cp] = calc_correction_functions(w0bar);  at = 1/cp;
    ws0 = 1 + w0bar - 1/((8/9)*(1 - at));                     % seed = truth
    fprintf('[probe %s] ws0_perp %.5f | exact + @est + app_known + pred_mean2 + %s | obs_dump ON | seeds %s\n', traj, ws0, recipe, mat2str(seeds));
    PIDX = [3 3; 3 4; 4 4; 3 8; 3 9; 8 8; 9 9; 8 9; 4 8; 4 9];   % P_upd entries kept: columns of PU
    S = [];
    for q = 1:numel(seeds)
        clear run_formC_b motion_control_law_formC_b;
        o = struct('arm','best','ap_known',true,'ap_known_at','est','app_known',true, ...
                   'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true, ...
                                                'pred_mean2',true,'pred_mean2_kr1',~strcmp(recipe,'mcorr'),'pred_mean2_kr1_full',strcmp(recipe,'kr1_full'),'nw_mcorr',strcmp(recipe,'mcorr'),'obs_dump',true), ...
                   'config_override',OV,'scenario','deep','verbose',false,'seeds',seeds(q),'log_P_full',false);
        evalc('R = run_formC_b(o);');
        L = obs_dump('get');  Lz = L([L.ax] == 3);  n = numel(Lz);  r = R.runs{1};
        assert(n == numel(r.tout) - 1, 'obs_dump holds %d z-records, expected %d', n, numel(r.tout) - 1);
        ad = r.a_hat_out(1,3) / r.a_bar_hat_out(1,3);
        s = struct();
        s.seed   = seeds(q);
        s.k_rec  = [Lz.k].';
        s.x_pred = cell2mat(arrayfun(@(z) z.x_pred(:).', Lz, 'UniformOutput', false).');
        s.x_upd  = cell2mat(arrayfun(@(z) z.x_upd(:).',  Lz, 'UniformOutput', false).');
        s.PU     = zeros(n, size(PIDX,1));  s.Pp11 = zeros(n,1);  s.Pp31 = zeros(n,1);  s.F34 = zeros(n,1);  s.R1 = zeros(n,1);  s.gate = false(n,1);
        for j = 1:n
            for c = 1:size(PIDX,1); s.PU(j,c) = Lz(j).P_upd(PIDX(c,1), PIDX(c,2)); end
            s.Pp11(j) = Lz(j).P_pred(1,1);  s.Pp31(j) = Lz(j).P_pred(3,1);
            s.F34(j)  = Lz(j).F(3,4);       s.R1(j)   = Lz(j).R(1);  s.gate(j) = Lz(j).gate;
        end
        s.PIDX       = PIDX;
        s.a_true     = r.a_true_out(:,3) / ad;                  % abar_true at the true height, call instant
        s.h_bar_true = r.h_bar_true_out(:,1);                   % call instant
        s.trk_true   = (r.p_d_out(:,3) - r.p_true_out(:,3)) / r.R;   % POST-advance (do not use for the pre-step error)
        s.f_bar      = r.f_bar_out(:,3);
        s.a_prime    = r.a_prime_out(:,3) / ad;                 % fed slope, dabar/dwbar
        s.a_bar_hat  = r.a_bar_hat_out(:,3);
        s.hd         = r.p_d_out(:,3) / r.R;
        s.t          = r.tout(:);
        s.ad         = ad;
        s.f_th       = r.F_th_out(:,3);
        s.mean2      = r.pred_mean2_out(:,3);                   % what the code injected in call k (incl. gap + kr1)
        s.K31        = r.K_dx_y1_out(:,3);                      % K1(3) of call k
        s.Ka1        = r.K_a_y1_out(:,3);  s.Ka2 = r.K_a_y2_out(:,3);
        s.innov1     = r.innov_y1_out(:,3);
        s.dx3        = r.delta_x_hat_3_out(:,3) / r.R;
        best = NaN;
        for off = -2:2
            if off >= 0; a = s.x_upd(1:end-off,4); b = s.a_bar_hat(1+off:end); else; a = s.x_upd(1-off:end,4); b = s.a_bar_hat(1:end+off); end
            m = min(numel(a), numel(b));  if max(abs(a(1:m) - b(1:m))) < 1e-12; best = off; end
        end
        fprintf('[probe %s] seed %2d: %d z-records, nstate %d, x_upd(4) = a_bar_hat_out at offset %s | hold E %+.5f\n', ...
                traj, seeds(q), n, size(s.x_upd,2), mat2str(best), mean(s.a_bar_hat(s.t > s.t(end)-1) - s.a_true(s.t > s.t(end)-1)));
        if isempty(S); S = s; else; S(end+1) = s; end %#ok<AGROW>
    end
    save(fullfile(od, sprintf('aptrue_est_final_%s%s.mat', traj, sfx)), 'S', 'ws0', 'traj', 'recipe', '-v7.3');
    fprintf('[probe %s] saved aptrue_est_final_%s%s.mat\n', traj, traj, sfx);
end
