% STATUS: ACTIVE (scratch, derivation probe series 2026-08-31)
% T4: adjoint weights; bias map = osc row3 -0.0079 (phase-uniform) + hold2 row3 -0.0034; gap2 rows cancel.
% Part of the closed-loop mean-bias derivation chain; see memory
% project_formC_inject_response_y1_amplifies_2026-08-26 (08-31 entries).
% T4: adjoint attribution. The end-hold e4 is linear in the forcing:
%   e4(end-hold avg) = sum_t [ w3(t)*G3(t) + w4(t)*G4(t) ]
% where w_r(t) = sensitivity of the end-hold-average e4 to a unit impulse on
% row r at time t. Compute w by backward recursion (adjoint of the replay),
% then decompose the actual bias integral: by segment, and by correlating the
% bias-carrying component with candidate time shapes.
WT = '/Users/kevin/Code/MotionControl_Simu-law-error-budget';
od = fullfile(WT, 'test_results', 'loop_mean_bias');
C = load(fullfile(od, 'loop_capture_seed7.mat'));
S = load(fullfile(od, 'replay_v2_profile.mat'));
R = load(fullfile(od, 'run_log_seed7.mat'));  r = R.run_log;
ax = 3;  t = C.t(:);  n = numel(t);  nsl = size(C.F, 1);  kk = C.KK;
mh = t > 3.7;  Nh = sum(mh);
% adjoint: J = (1/Nh) sum_{k in hold} e4[k];  lambda[k] = dJ/de_upd[k]
lam = zeros(nsl, n);  sel4 = zeros(nsl, 1);  sel4(4) = 1;
acc = zeros(nsl, 1);
for i = n:-1:2
    acc = C.A(:,:,min(i+1, n)).' * acc;   % carry next-step adjoint back through A[i+1]
    if mh(i); acc0 = acc + sel4 / Nh; else; acc0 = acc; end
    lam(:, i) = acc0;  acc = acc0;
end
w3 = lam(3, :).';  w4 = lam(4, :).';
G3m = mean(S.out.G3, 2);  G4m = mean(S.out.G4, 2);
contrib = w3 .* G3m + w4 .* G4m;
fprintf('adjoint check: sum contrib = %+.4f  (measured-forcing replay was -0.0115)\n', nansum(contrib));
SEG = {'hold1', t < 0.55; 'desc', t > 0.55 & t < 1.45; 'gap1', t >= 1.45 & t < 1.5; ...
       'osc', t >= 1.5 & t < 3.5; 'gap2', t >= 3.5 & t <= 3.7; 'hold2', t > 3.7};
fprintf('bias contribution by segment (row3 | row4):\n');
for g = 1:6
    m = SEG{g,2};
    fprintf('  %-6s %+8.4f | %+8.4f\n', SEG{g,1}, nansum(w3(m).*G3m(m)), nansum(w4(m).*G4m(m)));
end
% what does the weight look like in the osc? correlate w with candidate shapes
mo = t >= 1.5 & t < 3.5;
hd = r.h_bar_d_out(:);  dwd = [0; diff(hd)];  dwd = dwd(kk);
ah = r.a_bar_hat_out(kk, ax);  bh = r.b_hat_out(kk, ax);
ap = bh .* (1 - ah).^2;
fprintf('osc: corr(w3, dwd) %+.2f  corr(w3, a''/a) %+.2f | corr(w4, dwd) %+.2f  corr(w4, a''/a) %+.2f\n', ...
    corr(w3(mo), dwd(mo)), corr(w3(mo), ap(mo)./ah(mo)), corr(w4(mo), dwd(mo)), corr(w4(mo), ap(mo)./ah(mo)));
% the bias-carrying component of G: project G onto w within the osc
fprintf('osc: <w3*G3> share = %+8.4f   <w4*G4> share = %+8.4f\n', nansum(w3(mo).*G3m(mo)), nansum(w4(mo).*G4m(mo)));
% where inside the cycle? phase-resolved contribution
ph = mod(t(mo) - 1.5, 1);  edges = 0:0.125:1;  io = find(mo);
fprintf('phase-resolved total contribution (x1e-3):\n');
for bph = 1:8
    s2 = io(ph >= edges(bph) & ph < edges(bph+1));
    fprintf('  ph %.3f-%.3f: %+7.2f\n', edges(bph), edges(bph+1), 1e3*nansum(contrib(s2)));
end
save(fullfile(od, 't4_adjoint.mat'), 'w3', 'w4', 'G3m', 'G4m', 't', 'contrib');
fprintf('T4 DONE\n');
