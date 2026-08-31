% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T9: finite-dT (0.8/1.2) det/noise split; noise -0.0165 / det +0.0050; segment targets for the derivation.
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
TS = [0.8, 1.2];
G3s = cell(1,2);  G4s = cell(1,2);
for ti = 1:2
    G3 = [];  G4 = [];
    for sd = 1:8
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', sd, 'verbose', false, ...
                               'b_true', true, 'b_true_at', 'cmd', 'T_scale', TS(ti), ...
                               'ctrl_const_override', struct('law_exact_step', true, 'obs_dump', true)));
        r = O.runs{1};  L = obs_dump('get');  Lz = L([L.ax] == ax);
        n = numel(Lz);  nsl = numel(Lz(1).x_pred);  a_nom = r.a_nom;
        ht = r.h_bar_true_out(:);  hd = r.h_bar_d_out(:);  dw_true = hd - ht;
        wT = r.a_true_out(:, ax) .* r.F_th_out(:, ax) / pcR;
        H1 = zeros(1, nsl);  H1(1) = 1;  EUp = [];  Gt = zeros(nsl, n);
        for i = 1:n
            s = Lz(i);  k = s.k + 1;
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
            if ~isempty(EUp);  Gt(:, i) = eu - A * EUp;  end
            EUp = eu;
        end
        G3(:, sd) = Gt(3, :).';  G4(:, sd) = Gt(4, :).'; %#ok<SAGROW>
        fprintf('T%.1f seed %d done\n', TS(ti), sd);
    end
    G3s{ti} = G3;  G4s{ti} = G4;
end
save(fullfile(od, 't9_finite_dT.mat'), 'G3s', 'G4s', 'TS');
% ---- shares at T0: noise = (G(1.2)-G(0.8))/0.4 * 1.0 ; det = G(T0) - noise --
S = load(fullfile(od, 'replay_v2_profile.mat'));
A4 = load(fullfile(od, 't4_adjoint.mat'));
t = A4.t(:);  w3 = A4.w3;  w4 = A4.w4;
g3n = (mean(G3s{2},2) - mean(G3s{1},2)) / (TS(2)-TS(1));
g4n = (mean(G4s{2},2) - mean(G4s{1},2)) / (TS(2)-TS(1));
g3d = mean(S.out.G3, 2) - g3n;
g4d = mean(S.out.G4, 2) - g4n;
fprintf('\nadjoint dots: NOISE row3 %+8.4f row4 %+8.4f total %+8.4f\n', nansum(w3.*g3n), nansum(w4.*g4n), nansum(w3.*g3n)+nansum(w4.*g4n));
fprintf('              DET   row3 %+8.4f row4 %+8.4f total %+8.4f   (sum must be -0.0115; T-sweep says noise ~ -0.013, det ~ +0.001)\n', ...
        nansum(w3.*g3d), nansum(w4.*g4d), nansum(w3.*g3d)+nansum(w4.*g4d));
SEG = {'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'gap2', t >= 3.5 & t <= 3.7; 'hold2', t > 3.7};
fprintf('noise-share segment means (x1e-5):  g3n      g4n   | closed-form g3 pred\n');
pc = physical_constants();
R7 = load(fullfile(od, 'run_log_seed7.mat'));  r7 = R7.run_log;  kk7 = A4.t(:)*0;  % placeholder
C7 = load(fullfile(od, 'loop_capture_seed7.mat'));
abar = r7.a_true_out(C7.KK, ax) / r7.a_nom;  ah7 = r7.a_bar_hat_out(C7.KK, ax);  bh7 = r7.b_hat_out(C7.KK, ax);
b = 1 - lam;  a_o = r7.a_nom / pc.R;  kap = 4*pc.k_B*pc.T/pc.R*a_o;  rn = (3.31e-3/pc.R)^2;
q = kap*abar;  V = (q*(1+2*b^2) + 4*lam*b*q + b^2*rn)/(1-lam^2);  Cws = b*(2*q - V);
g3p = (bh7 .* (1-ah7).^2 ./ max(ah7,0.02)) .* Cws;
for g = 1:4
    m = SEG{g,2};
    fprintf('  %-6s %+8.2f %+8.2f | %+8.2f\n', SEG{g,1}, 1e5*mean(g3n(m)), 1e5*mean(g4n(m)), 1e5*mean(g3p(m)));
end
fprintf('T9 DONE\n');
