% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 4 (augmented estimator, first order): z 22.2 -> 1.10 R at constant
%   velocity in 5 s (the Section 2 z trajectory, x/y held), plane wall, lambda_c 0.4, epsilon = Q44/R2 = 1e-5.
%   Two estimator forms of 5state_noap (gain_order 1, lf const 1, seed at the plane value at the start height):
%     'prod'  : production refinements on (y2 whitening, y2 echo factor, MA(2) thermal memory, correlated n_x, exact
%               past-input bookkeeping) -- the Layer-1 closed form;
%     'plain' : all five off -- exactly the equations written in motion_ctrl Section 4 (state dx1 dx2 dx3 a, H = [1 0 0 0;
%               0 0 0 1], Q33 = Var(eps_x), Q44 = eps R2, R2 = K_var IF (a_hat + xi)^2);
%     'plain_<flag>...' : plain with only the named refinements on (leave-one-in check); suffix '_eps0' = Q44 0,
%                         '_nominal' = the control law uses a_nom (the estimator runs but is not used).
%   Cached in test_results/temp_5state_noap/mc_s4_<form>_<wall>_s<seed>.mat.
function R = run_mc_s4_first_order(forms, seeds, walls)
    if nargin < 1 || isempty(forms); forms = {'plain', 'prod'}; end
    if nargin < 2 || isempty(seeds); seeds = 1; end
    if nargin < 3 || isempty(walls); walls = {'plane'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));  addpath(here);
    id = fullfile(root, 'test_results', 'temp_5state_noap');  if ~exist(id, 'dir'); mkdir(id); end
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    T_D = 5;  LC = 0.4;  EPS = 1e-5;
    R = struct();
    for i_f = 1:numel(forms)
        for i_w = 1:numel(walls)
            for s = seeds
                fn = fullfile(id, sprintf('mc_s4_%s_%s_s%d.mat', forms{i_f}, walls{i_w}, s));
                if exist(fn, 'file'); r = load(fn);
                else
                    cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 1);  cfg.eq17_variant = '5state_noap';
                    cfg.t_descend_override = T_D;  cfg.T_sim = T_D;  cfg.lambda_c = LC;
                    if contains(forms{i_f}, '_tri10')                  % '<form>_tri10' (user 10-07): 22.2 -> 1.10 R in 1 s, then triangle 1.10 <-> 10 R, 2 Hz, 2 s
                        cfg = canonical_scenario(0.05, 1.10, 'ramp_tri', 2);  cfg.eq17_variant = '5state_noap';  cfg.lambda_c = LC;
                        cfg.amplitude = 0.5 * (10 - 1.10) * pc.R;
                    end
                    tkf = regexp(forms{i_f}, '_fix(\d+)', 'tokens', 'once');   % '<form>_fixNNN' (10-07): parked at NNN/100 R for 5 s
                    if ~isempty(tkf)
                        hf = str2double(tkf{1}) / 100 * pc.R;
                        ov = struct('trajectory_type', 'ramp_descent', 'h_init', hf, 'h_bottom', hf, 't_hold', 0, 't_descend_override', T_D, 'T_sim', T_D);
                        fo = fieldnames(ov);  for q = 1:numel(fo); cfg.(fo{q}) = ov.(fo{q}); end
                    end
                    if contains(forms{i_f}, '_xyz')                    % '<form>_xyz': the Section 2 trajectory (run_mc_s2_xyz.m), x/y 1 Hz 9 um sines
                        ov = struct('trajectory_type', 'ramp_descent', 'h_init', 50, 'h_bottom', 1.1 * pc.R, 't_hold', 0, ...
                                    't_descend_override', T_D, 'T_sim', T_D, 'amp_u', -9, 'freq_u', 1, 'amp_v', -9, 'freq_v', 1, 'tan_shape', 1);
                        fo = fieldnames(ov);  for q = 1:numel(fo); cfg.(fo{q}) = ov.(fo{q}); end
                    end
                    eps_k = EPS;  tk = regexp(forms{i_f}, '_eps([0-9.]+(e-[0-9]+)?)', 'tokens', 'once');   % '<form>_eps<val>': Q44 = val R2
                    if ~isempty(tk); eps_k = str2double(tk{1}); end
                    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', eps_k);
                    FL = {'y2_whiten', 'y2_mirror', 'ma2_aug', 'nw_mcorr', 'corr_input'};
                    if ~startsWith(forms{i_f}, 'prod')                 % 'prod...' = code defaults (all on); 'plain' = all off; 'plain_<flag>[_<flag>]' = only those on
                        for q = 1:numel(FL); cc.(FL{q}) = contains(forms{i_f}, ['_' FL{q}]); end
                    end
                    o = struct('seed', s, 'collect_diag', true, 'ctrl_const_override', cc);
                    if strcmp(walls{i_w}, 'cell'); o.plant_cperp = @cell_cperp_delta; end
                    if contains(forms{i_f}, '_nominal'); o.a_ctrl_const = a_nom * ones(3, 1); end   % '<form>_nominal': law uses a_nom (a-hat = 1)
                    S = run_pure_simulation(cfg, o);
                    r = struct('t', S.tout(:), 'a_true', S.a_true_out(:, 3) / a_nom, 'a_hat', S.diag.a_hat(:, 3) / a_nom, ...
                               'a_m', S.diag.a_xm(:, 3) / a_nom, 'dxm', S.diag.delta_x_m(:, 3), ...
                               'h_d', S.p_d_out(:, 3) / pc.R, 'h', S.p_true_out(:, 3) / pc.R, ...
                               'dxm_xyz', S.diag.delta_x_m(:, 1:3), 'a_true_xyz', S.a_true_out(:, 1:3) / a_nom, ...
                               'a_hat_xyz', S.diag.a_hat(:, 1:3) / a_nom, 'p_d_xyz', S.p_d_out(:, 1:3));
                    % 10-07: gain-row feedback for the update-share split (z axis); K maps innovation -> a [um/pN], /a_nom -> a-bar
                    if isfield(S.diag, 'K_kf_a_y1')
                        r.K_a_y1 = S.diag.K_kf_a_y1(:, 3) / a_nom;  r.K_a_y2 = S.diag.K_kf_a_y2(:, 3) / a_nom;
                        r.innov1 = S.diag.innovation_y1(:, 3);  r.innov2 = S.diag.innovation_y2(:, 3);  r.P_a = S.diag.P_a(:, 3) / a_nom^2;
                    end
                    save(fn, '-struct', 'r');
                end
                e = r.a_true - r.a_hat;  t = r.t;
                w1 = t > 1 & t < 3;  w2 = t > 4.5;  wb = t > 4.9;
                fprintf('%-5s %-5s s%-2d | min h %.3f | e_a mean 1-3 s %+.4f | 4.5-5 s %+.4f | last 0.1 s %+.4f | dx_m rms %.4f um\n', ...
                        forms{i_f}, walls{i_w}, s, min(r.h), mean(e(w1)), mean(e(w2)), mean(e(wb)), rms(r.dxm));
                R.(matlab.lang.makeValidName(forms{i_f})).(walls{i_w})(s) = r;
            end
        end
    end
end
