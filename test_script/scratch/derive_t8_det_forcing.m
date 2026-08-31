% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T8 FAILED INSTRUMENT: a T-to-0 run has a different loop; forcing not subtractable.
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
restoredefaultpath;
addpath(genpath(fullfile(WT, 'model')));
addpath(genpath(fullfile(WT, 'test_script')));
assert(contains(which('motion_control_law_formC_b'), 'law-error-budget'), 'WRONG WORKTREE');
cd(WT);
od = fullfile(WT, 'test_results', 'loop_mean_bias');
ax = 3;  lam = 0.7;  pcR = physical_constants().R;
% near-zero-temperature run, filter consistent (T_scale scales both sides); meas noise on
O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false, ...
                       'b_true', true, 'b_true_at', 'cmd', 'T_scale', 1e-4, ...
                       'ctrl_const_override', struct('law_exact_step', true, 'obs_dump', true)));
r = O.runs{1};  L = obs_dump('get');  Lz = L([L.ax] == ax);
n = numel(Lz);  nsl = numel(Lz(1).x_pred);  a_nom = r.a_nom;
ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);  dw_true = hd - ht;
wT = r.a_true_out(:, ax) .* r.F_th_out(:, ax) / pcR;
H1 = zeros(1, nsl);  H1(1) = 1;  EUp = [];  Gd = zeros(nsl, n);  KK = zeros(1, n);
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
    A = I2 * (eye(nsl) - k1 * H1) * s.F;
    xt = [dw_true(max(k-2,1)); dw_true(max(k-1,1)); dw_true(min(k,numel(dw_true))); ...
          r.a_true_out(min(k,size(r.a_true_out,1)), ax)/a_nom; s.x_upd(5:7); wT(min(k,numel(wT))); wT(max(k-1,1))];
    eu = xt - s.x_upd;
    if ~isempty(EUp);  Gd(:, i) = eu - A * EUp;  end
    EUp = eu;
end
S = load(fullfile(od, 'replay_v2_profile.mat'));
A4 = load(fullfile(od, 't4_adjoint.mat'));
t = A4.t(:);  w3 = A4.w3;  w4 = A4.w4;
g3d = Gd(3, :).';  g4d = Gd(4, :).';
G3m = mean(S.out.G3, 2);  G4m = mean(S.out.G4, 2);
g3n = G3m - g3d;  g4n = G4m - g4d;                 % noise share of the mean forcing
fprintf('adjoint dots: det part  row3 %+8.4f row4 %+8.4f  total %+8.4f\n', nansum(w3.*g3d), nansum(w4.*g4d), nansum(w3.*g3d)+nansum(w4.*g4d));
fprintf('              noise part row3 %+8.4f row4 %+8.4f  total %+8.4f   (target: total_meas -0.0115 minus det)\n', ...
    nansum(w3.*g3n), nansum(w4.*g4n), nansum(w3.*g3n)+nansum(w4.*g4n));
SEG = {'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'gap2', t >= 3.5 & t <= 3.7; 'hold2', t > 3.7};
fprintf('noise-share segment means (x1e-5):  g3n      g4n\n');
for g = 1:4
    m = SEG{g,2};
    fprintf('  %-6s %+8.2f %+8.2f\n', SEG{g,1}, 1e5*mean(g3n(m)), 1e5*mean(g4n(m)));
end
save(fullfile(od, 't8_det_forcing.mat'), 'g3d', 'g4d', 'g3n', 'g4n', 't');
fprintf('T8 DONE\n');
