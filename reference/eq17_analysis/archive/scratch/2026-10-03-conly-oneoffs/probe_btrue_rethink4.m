% STATUS: ACTIVE (scratch) | PURPOSE: is the FAR-FIELD seed of the mean first- or
%   second-order in noise (09-02)? Read m(t) at t=2,3,4 (w_d ~ 6.4-5.2, before the
%   amplifier has acted) from arm30_btrue_noise_scale.mat, k in {0.5,1,2}.
%   First-order (lag / evaluation point): m(t) same for all k.
%   Second-order (rectified noise): m(t) ~ k^2 (x0.25 / x1 / x4).
function probe_btrue_rethink4()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    S = load(fullfile(od, 'arm30_btrue_noise_scale.mat'));
    fprintf('   t   |  m(k=0.5)   SEM   |  m(k=1)     SEM   |  m(k=2)     SEM   | ratio m2/m1  m1/m05\n');
    for tt = [1 2 3 4 5 6 8 9 10.5 12.4]
        r = zeros(1,3); s = r;
        for i = 1:3
            f = sprintf('k%02d', round(10*S.K(i))); d = S.(f);
            k = find(d.t >= tt, 1); r(i) = mean(d.E(k,:)); s(i) = std(d.E(k,:))/sqrt(30);
        end
        fprintf('%5.1f  | %+9.5f %7.5f | %+9.5f %7.5f | %+9.5f %7.5f |  %5.2f  %5.2f\n', ...
            tt, r(1), s(1), r(2), s(2), r(3), s(3), r(3)/r(2), r(2)/r(1));
    end
    fprintf('(second-order predicts ratios 4.00 / 4.00; first-order predicts 1.00 / 1.00)\n');
end
