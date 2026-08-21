% Judge the true-a arm against the PRE-REGISTERED predictions.
% CONFOUND, recorded before reading anything: with the control law on the
% truth, the coupling "gain error -> tracking error" (F_e(3,4) = -F_dw) is
% gone from reality but still in the filter's model, so a_hat degrades badly
% (desc peak 54% against the baseline's 5.7%). Every a_hat-based prediction is
% therefore unreadable on this arm. P1-P4 are readout/residual quantities and
% the loop pole IS exactly lambda_c here, so they remain testable.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
T = load([root 'test_results/am_r22_deep/truea_100.mat']);
B = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
K = T.K; a_nom = T.a_nom; R = B.S400.K.R; a_cov = K.a_cov;
kT = 4*(B.S400.K.kBT/R)*B.S400.K.a_o; ab = K.IF_abc(:);
nb = 100;                                   % paired: baseline seeds 1..100
axl = 'xyz';
fprintf('\n=========== TRUE-A ARM vs BASELINE (paired seeds 1..100) ===========\n');
fprintf('P1 acf | P2 Var(dx_r) | P3 I1,I2 | P4 E[a_m]  -- final hold, trough\n');
for ax = 1:3
    s2n = B.S400.K.sigma2_n_s(ax)/R^2;
    xi  = (K.C_n/K.C_dpmr)*s2n/kT;
    ife = @(a) 1 + 2*(((kT*a).^2*ab(1) + 2*(kT*a)*s2n*ab(2) + s2n^2*ab(3)) ./ ...
                      ((K.C_dpmr*kT*a + K.C_n*s2n).^2));
    for arm = 1:2
        if arm == 1
            A = squeeze(B.S400.a_xm_out(:,ax,1:nb))/a_nom;
            Tr= squeeze(B.S400.a_true_out(:,ax,1:nb))/a_nom;
            D = squeeze(B.S400.dx_r_out(:,ax,1:nb))/R;
            tt= B.S400.t(:); nm='baseline';
        else
            A = squeeze(T.A_xm(:,ax,:))/a_nom;
            Tr= squeeze(T.A_tr(:,ax,:))/a_nom;
            D = squeeze(T.D_xr(:,ax,:))/R;
            tt= T.t(:); nm='true-a  ';
        end
        A=A(2:end,:); Tr=Tr(2:end,:); D=D(2:end,:); tt2=tt(2:end);
        Y = A(2:end,:) - (1-a_cov)*A(1:end-1,:);
        am = mean(Tr,2);
        ih = tt2>=3.75; ih2 = tt2(2:end)>=3.75;
        c  = mean(am(ih));
        vdr= mean(var(D(ih,:),0,2)) / (K.C_dpmr*kT*c + K.C_n*s2n);
        i1 = mean(var(A(ih,:),0,2)) / (K.K_var*ife(c)*(c+xi)^2);
        i2 = mean(var(Y(ih2,:),0,2)) / (2*a_cov^2*(c+xi)^2);
        e1 = mean(mean(A(ih,:),2))/mean(am(ih)) - 1;
        rr = acf(D(ih,:),4);
        fprintf('  %c %s  a=%.4f | P2 %.3f | I1 %.3f  I2 %.3f | E[a_m] %+5.2f%% | acf %.3f %.3f %.3f %.3f\n', ...
                axl(ax), nm, c, vdr, i1, i2, 100*e1, rr(1), rr(2), rr(3), rr(4));
    end
end
fprintf('\n  model acf at lambda_c = 0.700 (P1 target): 0.825 0.614 0.379 0.217\n');
function r = acf(X,L)
    Xf = X - mean(X,2); n=size(Xf,1); r=zeros(1,L);
    for k=1:L
        a=Xf(1:n-k,:); b=Xf(1+k:n,:); a=a(:)-mean(a(:)); b=b(:)-mean(b(:));
        r(k)=(a'*b)/sqrt((a'*a)*(b'*b));
    end
end
