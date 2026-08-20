% STATUS: ACTIVE (scratch) | PURPOSE: judge R(2,2) by what it DOES, not by
%   whether a variance formula reproduces -- (A) against the de-weighting the
%   data actually calls for, (B) by the filter's own NIS | EXPIRES: with the
%   R22 audit
%
% Angle A answers "is the number the filter uses the right number", which is
% NOT the same question as "does the closed form reproduce a variance". Three
% separate errors ride on R2 near the wall and they partially cancel, so the
% formula can be wrong in two places while the product is nearly right.
%
% Angle B is the standard consistency test and must be read with care here:
% E[NIS] = 1 is the criterion for an R that equals the per-sample variance.
% This family deliberately inflates R by IF_eff to pay for serial correlation,
% so E[NIS] ~ 1/IF ~ 0.3 is the CORRECT signature, not a defect.
% Is R(2,2) right in the only sense that matters operationally?
% Two independent angles, both on the sibling's 400-seed deep-band stack:
%   A. against the R the de-weighting actually calls for:
%        R_needed[window] = Var(y2)_measured * IF_sample(measured integrated
%        autocorrelation of y2 in that window)
%   B. the filter's own consistency: NIS = innov_y2^2 / S2, which must average
%      1 if R2 (and P) are honest. H2*P*H2'/R2 ~ 5e-5 in this family, so
%      S2 = R2 to well under a percent.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
S = load([root 'test_results/am_r22_deep/stack_deep400.mat']);
L = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
iv = squeeze(L.S400.innov_y2_out(:,3,:));  iv = iv(2:end,:);   % drop init row
clear L
a_cov = S.cc.a_cov; A = S.A_wm; T = S.A_tr; R2 = S.R2_u; t = S.t;
Y  = A(2:end,:) - (1-a_cov)*A(1:end-1,:);
t2 = t(2:end);  R2b = R2(2:end,:);  iv2 = iv(2:end,:);
am = mean(T,2); am2 = am(2:end);
W = {[0.5 1.5],'descent'; [1.75 3.5],'oscillate'; [3.75 4.8],'final hold'};
fprintf('\n window      a/a_o   IF_sample  Var(y2)m   R2_used   R2_used/R_needed   E[NIS]\n');
for i = 1:3
    in = t2 >= W{i,1}(1) & t2 <= W{i,1}(2);
    X = Y(in,:); Xf = X - mean(X,2); n = size(Xf,1);
    rho = zeros(1,40);
    for k = 1:40
        a1 = Xf(1:n-k,:); b1 = Xf(1+k:n,:);
        a1 = a1(:)-mean(a1(:)); b1 = b1(:)-mean(b1(:));
        rho(k) = (a1'*b1)/sqrt((a1'*a1)*(b1'*b1));
    end
    IFs = 1 + 2*sum(rho);
    vy  = mean(var(X,0,2));
    ru  = mean(mean(R2b(in,:),2));
    nis = mean(mean(iv2(in,:).^2 ./ R2b(in,:), 2));
    fprintf(' %-11s %6.3f   %7.3f   %9.3g %9.3g      %6.3f          %6.3f\n', ...
            W{i,2}, mean(am2(in)), IFs, vy, ru, ru/(vy*IFs), nis);
end
