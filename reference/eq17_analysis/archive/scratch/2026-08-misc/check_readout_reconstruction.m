% meng ch4 guard 1(a): before any quantity enters a criterion, rebuild it from
% raw logs by a SECOND path. Their 1 Hz line survived five parameter sweeps
% because the defect was in the log pairing, which no parameter sweep can see.
%
% Everything I concluded today rests on a_xm_out being the readout at step k.
% Second path: take dh_m_out (= delta_w_m * R, the tracking error the
% controller was handed) and re-run the readout chain myself:
%     LP  -> HP -> EWMA -> affine inverse
% Two comparisons, so a failure localises:
%   (A) my dw_r          vs logged dx_r_out    -> LP/HP stage + dh_m pairing
%   (B) my a_bar_wm      vs logged a_xm_out    -> EWMA + inversion + constants
% Plus an explicit OFF-BY-ONE probe: correlate at shift -1, 0, +1.
root = '/Users/kevin/Code/MotionControl_Simu-motion-test/';
L = load([root 'test_results/formC_cdpmr_var_check/raw_seeds.mat']);
K = L.S400.K;  ax = 3;  ns = 20;                 % 20 seeds is plenty for an identity
dh = squeeze(L.S400.dh_m_out(:,ax,1:ns));        % [um]
dr = squeeze(L.S400.dx_r_out(:,ax,1:ns));        % [um]
ax_m = squeeze(L.S400.a_xm_out(:,ax,1:ns));      % [um/pN]
clear L
R = K.R; a_pd = K.a_pd; a_cov = K.a_cov;
kappa_T = 4*(K.kBT/R)*K.a_o; s2n = K.sigma2_n_s(ax)/R^2;
dwm = dh / R;                                    % normalized tracking error
N = size(dwm,1);
my_dwr = zeros(N,ns); my_awm = zeros(N,ns);
for s = 1:ns
    lp = 0;  s2 = (ax_m(2,s)/K.a_nom)*K.C_dpmr*kappa_T + K.C_n*s2n;  % seed from k=2
    for k = 2:N
        lp = (1-a_pd)*lp + a_pd*dwm(k,s);
        r  = dwm(k,s) - lp;
        s2 = (1-a_cov)*s2 + a_cov*r^2;
        my_dwr(k,s) = r;
        my_awm(k,s) = (s2 - K.C_n*s2n)/(K.C_dpmr*kappa_T);
    end
end
bi = 400:N;                                       % burn-in past the EWMA memory
relA = abs(my_dwr(bi,:)*R - dr(bi,:)) ./ max(abs(dr(bi,:)), eps);
relB = abs(my_awm(bi,:)*K.a_nom - ax_m(bi,:)) ./ max(abs(ax_m(bi,:)), eps);
fprintf('\n(A) dw_r  : median rel err %.3g | p99 %.3g | max %.3g\n', ...
        median(relA(:)), prctile(relA(:),99), max(relA(:)));
fprintf('(B) a_m   : median rel err %.3g | p99 %.3g | max %.3g\n', ...
        median(relB(:)), prctile(relB(:),99), max(relB(:)));
fprintf('\noff-by-one probe (corr of my a_m against logged a_xm at shift):\n');
x = my_awm(bi,1)*K.a_nom;  y = ax_m(bi,1);
for sh = [-1 0 1]
    if sh < 0; a = x(1:end-1); b = y(2:end); elseif sh > 0; a = x(2:end); b = y(1:end-1);
    else; a = x; b = y; end
    a = a - mean(a); b = b - mean(b);
    fprintf('   shift %+d : corr %.6f\n', sh, (a'*b)/sqrt((a'*a)*(b'*b)));
end
