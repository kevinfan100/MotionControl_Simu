% FROZEN BATTERY, block I (identity layer, stack-independent), offline.
% Source: run_formC_b, arm 'best', ap_src 'post', deep band, 400 seeds, 3 axes.
% Every item is measured on the SAME stack; nothing is re-run.
%
%   I1  Var(a_m) / [K_var*IF_var*(a+xi)^2]
%   I2  Var(y2)  / [2*a_cov^2*(a+xi)^2]
%   I3  E[a_m] / a_true            (first moment -- I1/I2 are blind to it)
%   I4  rho_y2(tau) vs rho_dwr(tau)^2
%   LEVER  the xi lever arm: how much each axis's I1 ratio MOVES if xi is
%          dropped from the formula. A sweep with no lever arm proves nothing
%          (meng ch4), so the leverage is reported next to the invariance.
%
% x and y are the wall-PARALLEL axes: w_hat = z, so they carry no commanded
% motion. They are therefore the clean control for the deterministic-remnant
% term (4 m^2 sigma^2) that the Gaussian identity drops -- m = 0 there by
% construction.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
L = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
K = L.S400.K; ns = 400;
AX = squeeze_all(L.S400.a_xm_out);  AT = squeeze_all(L.S400.a_true_out);
DR = squeeze_all(L.S400.dx_r_out);  t = L.S400.t(:);
clear L
a_cov = K.a_cov; R = K.R; a_nom = K.a_nom;
kT = 4*(K.kBT/R)*K.a_o; ab = K.IF_abc(:);
axl = 'xyz';
fprintf('\n================ BLOCK I  (400 seeds, formC_b, deep band) ================\n');
for ax = 1:3
    s2n = K.sigma2_n_s(ax)/R^2;
    xi  = (K.C_n/K.C_dpmr)*s2n/kT;
    A = squeeze(AX(:,ax,:))/a_nom;  T = squeeze(AT(:,ax,:))/a_nom;
    D = squeeze(DR(:,ax,:))/R;
    A = A(2:end,:); T = T(2:end,:); D = D(2:end,:); tt = t(2:end);
    Y = A(2:end,:) - (1-a_cov)*A(1:end-1,:);
    am = mean(T,2); am2 = am(2:end);
    ife = @(a) 1 + 2*(((kT*a).^2*ab(1) + 2*(kT*a)*s2n*ab(2) + s2n^2*ab(3)) ./ ...
                      ((K.C_dpmr*kT*a + K.C_n*s2n).^2));
    fprintf('\n--- axis %c :  xi_bar = %.4g,  a range [%.4f %.4f],  xi/a at min a = %.1f %%\n', ...
            axl(ax), xi, min(am), max(am), 100*xi/min(am));
    W = {[0.05 0.45],'hold(far)'; [0.5 1.5],'descent'; [1.75 3.5],'oscillate'; [3.75 4.8],'hold(trough)'};
    fprintf('    window        a/a_o    I1      I2      I3(1st mom)\n');
    for i = 1:4
        i1 = tt >= W{i,1}(1) & tt <= W{i,1}(2);
        i2 = tt(2:end) >= W{i,1}(1) & tt(2:end) <= W{i,1}(2);
        c  = mean(am(i1));
        r1 = mean(var(A(i1,:),0,2)) / (K.K_var*ife(c)*(c+xi)^2);
        r2 = mean(var(Y(i2,:),0,2)) / (2*a_cov^2*(c+xi)^2);
        r3 = mean(mean(A(i1,:),2))/mean(am(i1)) - 1;
        fprintf('    %-12s %6.4f  %6.3f  %6.3f   %+6.2f %%\n', W{i,2}, c, r1, r2, 100*r3);
    end
    % I4 in the trough hold (stationary)
    ih = tt >= 3.75; ih2 = tt(2:end) >= 3.75;
    rd = acf(D(ih,:),4); ry = acf(Y(ih2,:),4);
    fprintf('    I4 rho: dwr^2 ='); fprintf(' %.3f', rd.^2);
    fprintf('  |  y2 ='); fprintf(' %.3f', ry); fprintf('\n');
    % LEVER ARM: drop xi from the formula, report the move
    c_lo = mean(am(tt>=3.75));
    lev  = ((c_lo+xi)/c_lo)^2 - 1;
    fprintf('    LEVER (xi term at the trough): dropping xi moves I1 by %+.1f %%\n', 100*lev);
end
function Y = squeeze_all(X); Y = X; end
function r = acf(X,L)
    Xf = X - mean(X,2); n = size(Xf,1); r = zeros(1,L);
    for k = 1:L
        a = Xf(1:n-k,:); b = Xf(1+k:n,:);
        a = a(:)-mean(a(:)); b = b(:)-mean(b(:));
        r(k) = (a'*b)/sqrt((a'*a)*(b'*b));
    end
end
