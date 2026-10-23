% Where does rho_y2(1) = 0.65 come from? Whitening cancels the EWMA(a_cov)
% pole EXACTLY, so what is left must be the colour of dw_r^2, and for a
% zero-mean Gaussian residual Isserlis gives rho_{dw_r^2}(tau) = rho_dwr(tau)^2.
% Testable: rho_y2(tau) should equal rho_dwr(tau)^2 step for step.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
S = load([root 'test_results/am_r22_deep/stack_deep400.mat']);
L = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
dr = squeeze(L.S400.dx_r_out(:,3,:)); dr = dr(2:end,:); clear L
a_cov = S.cc.a_cov; A = S.A_wm; t = S.t;
Y = A(2:end,:) - (1-a_cov)*A(1:end-1,:); t2 = t(2:end); dr2 = dr(2:end,:);
in  = t2 >= 3.75;                      % final hold, stationary
rd = local_acf(dr2(in,:), 8);          % residual itself
ry = local_acf(Y(in,:), 8);            % whitened readout increment
fprintf('\n tau        1      2      3      4      5      6      7      8\n');
fprintf(' rho_dwr  '); fprintf(' %6.3f', rd); fprintf('\n');
fprintf(' rho_dwr^2'); fprintf(' %6.3f', rd.^2); fprintf('\n');
fprintf(' rho_y2   '); fprintf(' %6.3f', ry); fprintf('\n');
fprintf(' ratio    '); fprintf(' %6.3f', ry./(rd.^2)); fprintf('\n');
fprintf('\n a_pd pole 1-a_pd = %.3f, lambda_c = %.3f  (a_cov pole is cancelled by the whitening)\n', ...
        1-S.cc.a_pd, 0.7);
function r = local_acf(X, L)
    Xf = X - mean(X,2); n = size(Xf,1); r = zeros(1,L);
    for k = 1:L
        a = Xf(1:n-k,:); b = Xf(1+k:n,:);
        a = a(:)-mean(a(:)); b = b(:)-mean(b(:));
        r(k) = (a'*b)/sqrt((a'*a)*(b'*b));
    end
end
