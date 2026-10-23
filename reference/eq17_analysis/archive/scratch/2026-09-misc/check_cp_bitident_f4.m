% PURPOSE (2026-09-23): after wiring plaw_cp_slope, confirm the F4 path (plaw_dp_slope) is untouched: rerun the fixed-height
%   plane A run at 1.10 R exactly as probe_obs_fixed_height did and compare the six singular values with its cache.
%   Expect max relative difference 0.  EXPIRES: with the plaw_cp_slope wiring check.
function check_cp_bitident_f4()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    S = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'obs_fixed_height.mat'));
    ref = S.f4(2);  h = ref.h;  pc = physical_constants();  Ts = pc.Ts;  R = pc.R;  FREE = S.FREE;
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, 'law_exact_step', false, ...
                'lock_b', false, 'gain_algebraic', false, 'amodel_fe', false, 'amodel_h', false, 'b_ceil', 1.5, 'b_floor', 0.2, ...
                'ws0_perp', 0, 'Pf_w0_std', 0, 'Pf_a_floor', 0.002, 'plaw_g_min', 0.5, 'plaw_delta_lim', [-0.9, 3], ...
                'delta_init', 0.059, 'Pf_delta_std', 0.64, 'obs_dump', true, 'plaw_dp_slope', true, 'p_init', 1.950, 'Pf_p_std', 0.10, 'plaw_p_lim', [1.2, 4]);
    OV = struct('trajectory_type', 'positioning', 'h_init', h * R, 'T_sim', S.T_HOLD, 'h_bottom', h * R, 'amplitude', 0, 't_hold', 0, 't_descend_override', 0);
    o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, 'seeds', S.seed, 'log_P_full', false);
    clear run_formC_b motion_control_law_formC_b;
    evalc('Rr = run_formC_b(o);');  L = obs_dump('get');  Lz = L([L.ax] == 3);
    n = 7;  G = zeros(n);  Phi = eye(n);
    for i = 1:numel(Lz)
        H1 = Lz(i).H{1};  H2 = Lz(i).H{2};  R1 = Lz(i).R(1);  R2 = Lz(i).R(2);
        r1 = H1 * Phi;  G = G + (r1' * r1) / R1;
        if ~isempty(H2) && R2 > 0; r2 = H2 * Phi;  G = G + (r2' * r2) / R2; end
        Phi = Lz(i).F * Phi;
    end
    Gf = G(FREE, FREE);  d = sort(eig((Gf + Gf') / 2), 'descend');  SIG = sqrt(max(d, 0));
    rel = max(abs(SIG - ref.SIG) ./ max(abs(ref.SIG), 1e-12));
    verdict = 'DIFFERS';  if rel == 0; verdict = 'PASS (bit-identical)'; end
    fprintf('F4 plane fixed 1.10 R: sigma now  %s\n                       cached     %s\n  max rel diff %.3g  -> %s\n', ...
            sprintf('%10.4f', SIG), sprintf('%10.4f', ref.SIG), rel, verdict);
    if rel ~= 0; fprintf('  (F4 path changed by the plaw_cp_slope edit -- inspect)\n'); end
end
