% PURPOSE (2026-09-14, user: "give me V in 5-state for the two versions"): the whitened observability SVD of the
%   5-STATE reading copy (x = [dw1 dw2 dw3 a_w b]) at one window per trajectory, for the two self-consistent versions:
%     original  0907 core: e_a independent -- F(3,4) = -F_dw and H2(4) = 1 - Grad A_a
%     new       0910 a model: e_a -> -a' e_dw3 + ((1-a)/b) e_b in BOTH the state rows and the y2 row
%     4state    the self-consistent version the substitution points at: x = [dw1 dw2 dw3 b], the gain is COMPUTED from
%               the law at the estimator's own height, so there is no a slot at all; y2's walk-back is exact (the law is
%               evaluated at the delayed height, no A_a term). Negative control = a synthetic all-zero 5th column, which
%               must return CRLB = Inf.
%   Linearisation point, R1/R2 and the window convention are the 09-09 ones (production chord-seed arm, seed 7, 500 steps).
%   Health written before the run: the new version's a_w column of O_w must be identically 0 and its rank 4 of 5;
%   the original must reproduce the 09-09 console (canon descent sigma_min 241, CRLB_b/prior 0.105).
%   Output printed only (no figure) | EXPIRES: with the b line | 產線改動不會自動跟上
function out = probe_obs5_V_two_versions(seed, win)
    if nargin < 1 || isempty(seed); seed = 7; end
    if nargin < 2 || isempty(win);  win = 500; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    pc = physical_constants();  TR = {'canon', 'meng'};  T_SHOW = [1.25, 8.75];  LAB = {'dw1','dw2','dw3','a_w','b'};
    NAME = {'original', 'new (a model in F_e and H)', '4-state (gain computed, no a slot)'};  out = struct();
    for c = 1:2
        switch TR{c}
            case 'meng'; OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0,'frequency',1, ...
                                     'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',12.5,'h_min',2.475); cfg = OV; p0 = 3e-4;
            otherwise;   OV = struct(); cfg = canonical_scenario(0.05, 1.1, 'deep'); p0 = 1e-5;
        end
        w0 = cfg.h_init / pc.R;  [~, cp] = calc_correction_functions(w0, true);  a0 = 1/cp;
        b_ch = (1/(1 - a0) - 1) / (w0 - 1.0);
        cc = struct('b_ceil',1.5,'Pf_w0_std',0,'Pf_a_floor',p0,'b_init',b_ch,'ws0_perp',2 - 1/b_ch);
        o = struct('arm','best','ctrl_const_override',cc,'config_override',OV,'scenario','deep','verbose',false, ...
                   'seeds',seed,'log_P_full',false);
        clear run_formC_b motion_control_law_formC_b;  evalc('Rr = run_formC_b(o);');  S = Rr.runs{1};
        R = S.R;  Ts = pc.Ts;  lc = canonical_scenario(0.05, 1.1, 'deep').lambda_c;  d = 2;  K_INIT = 4;
        wd = S.h_bar_d_out(:);  ah = S.a_bar_hat_out(:,3);  bh = S.b_hat_out(:,3);  dw3 = S.delta_x_hat_3_out(:,3)/R;
        fb = S.f_bar_out(:,3);  R2 = S.R2_out(:,3);  n = numel(wd);  t = (0:n-1)'*Ts;
        R1 = (canonical_scenario(0.05, 1.1, 'deep').meas_noise_std(3) / R)^2;
        dwd = [diff(wd); 0];  dwd(1:K_INIT) = 0;
        Fdw = fb + (1 - lc) * ([0; fb(1:end-1)] + [0; 0; fb(1:end-2)]);
        grad = [zeros(d,1); wd(d+1:end) - wd(1:end-d)];  grad(1:K_INIT) = 0;
        k0 = max(K_INIT, round(T_SHOW(c)/Ts));
        fprintf('\n================ %s, 5-state, window %d steps from t = %.2f s (seed %d) ================\n', TR{c}, win, t(k0), seed);
        for m = 1:3
            nx = 5 - (m == 3);  LABm = LAB;  if m == 3; LABm = LAB([1 2 3 5]); end
            Ow = zeros(2*(win+1), nx);  Phi = eye(nx);
            for i = 0:win
                k = k0 + i;  M = dwd(k) + (1-lc)*dw3(k);  om = 1 - ah(k);  b_ = bh(k);  F_ = Fdw(k);  g_ = grad(k);
                if m == 1
                    F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc -F_ 0;
                         0 0 (1-lc)*b_*om^2, 1 + b_*om^2*F_ - 2*b_*om*M, om^2*M;  0 0 0 0 1];
                    H = [1 0 0 0 0; 0 0 0, 1 + 2*b_*om*g_, -om^2*g_];
                elseif m == 2
                    F = [0 1 0 0 0; 0 0 1 0 0; 0 0 lc + F_*b_*om^2, 0, -F_*om/b_;
                         0 0 (1-lc)*b_*om^2 - F_*b_^2*om^4, 1 - 2*b_*om*M, om^2*M + F_*om^3;  0 0 0 0 1];
                    H = [1 0 0 0 0; 0 0 -(1 + 2*b_*om*g_)*b_*om^2, 0, om/b_ + om^2*g_];
                else
                    om_d = om + b_*om^2*g_;                       % 1 - a at the delayed height (law, exact walk-back)
                    F = [0 1 0 0; 0 0 1 0; 0 0 lc + F_*b_*om^2, -F_*om/b_;  0 0 0 1];
                    H = [1 0 0 0; 0 0 -b_*om_d^2, om_d/b_];
                end
                Ow(2*i + (1:2), :) = diag([1/sqrt(R1), 1/sqrt(max(R2(k), eps))]) * (H * Phi);
                Phi = F * Phi;
            end
            if m == 3; Ow = [Ow, zeros(size(Ow,1), 1)]; nx = 5; LABm = [LABm, {'neg(0)'}]; end   % synthetic negative control
            [~, Sm, Vm] = svd(Ow, 'econ');  sv = diag(Sm);  rk = sum(sv > max(size(Ow))*eps(sv(1)));
            G = Ow'*Ow;  [Ve, De] = eig(0.5*(G+G'));  dv = real(diag(De));  tol = max(dv)*1e-15;  ok = dv > tol;
            jb = 5 - 1*(m == 3);  wj = real(Ve(jb,:)).^2;
            wn = real(Ve(5,:)).^2;  cn = NaN;
            if m == 3; if any(wn(~ok) > 1e-12); cn = Inf; else; cn = sqrt(sum(wn(ok)./dv(ok).')); end; end
            if any(wj(~ok) > 1e-12); cb = Inf; else; cb = sqrt(sum(wj(ok)./dv(ok).')); end
            if m == 3
                fprintf('\n--- %s | rank %d of 4 free | CRLB_b = %.4g (prior %.4f) | negative control CRLB = %.3g (must be Inf)\n', ...
                        NAME{m}, rk, cb, S.ctrl_const.Pf_b_std, cn);
            else
                fprintf('\n--- %s | rank %d of 5 | max |O_w(:,a_w)| = %.1e | CRLB_b = %.4g (prior %.4f)\n', ...
                        NAME{m}, rk, max(abs(Ow(:,4))), cb, S.ctrl_const.Pf_b_std);
            end
            fprintf('  sigma :%s\n', sprintf('%12.4g', sv));
            fprintf('  V (rows = state, cols = v1..v%d):\n', size(Vm,2));
            for j = 1:size(Vm,1); fprintf('   %-6s %s\n', LABm{j}, sprintf('%9.3f', Vm(j,:))); end
            fprintf('  1/sigma:%s\n', sprintf('%12.4g', 1./sv));
            out.(TR{c})(m) = struct('sv', sv, 'V', Vm, 'rank', rk, 'crlb_b', cb, 't', t(k0));
        end
    end
end
