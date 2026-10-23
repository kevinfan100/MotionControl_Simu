% STATUS: ACTIVE (scratch) | PURPOSE: two offline readings for the 09-02 rethink of
%   the b_true bias/spread problem, all from arms30_seedtruth_pair.mat (30 seeds):
%   (1) hold-segment slope of the cross-seed mean error, per arm (rule 9: slope, not
%       segment mean) -- is hold balanced (flat) or ramping?
%   (2) second-order split of the SE channel: P44-type term b*M*E[eps^2] vs the
%       cross term A_a*(1-lc)*C34 -- which second-order term carries the mean.
function probe_btrue_rethink()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, 'arms30_seedtruth_pair.mat'));
    R = 2.25; lc = 0.7; b = 8/9;
    for A = {'btrue','aptrue'}
        d = D.(A{1}); t = d.t(:); E = d.E; nS = size(E,2); C = d.C;
        mE = mean(E,2);
        % (1) hold slope, per seed then across seeds (t-stat)
        mh = t > 10.5;
        sl = zeros(1,nS);
        for q = 1:nS; p = polyfit(t(mh), E(mh,q), 1); sl(q) = p(1); end
        fprintf('[%s] hold: mean E %+.5f | slope %+.3e /s (SEM %.1e, t=%.1f) | E at 10.5 %+.5f, at end %+.5f\n', ...
            A{1}, mean(E(mh,:),'all'), mean(sl), std(sl)/sqrt(nS), mean(sl)/(std(sl)/sqrt(nS)), ...
            mE(find(mh,1)), mE(end));
        % (2) second-order split (a-reference expansion, same as verify_btrue_error_budget V3)
        N = numel(t); k1 = 2:N; dWd = diff(d.hd);
        Mh = dWd + (1-lc) * C.delta_x_hat_3_out(k1-1,:) / R;
        eps4 = E(k1-1,:); m4 = mean(eps4,2); v4 = var(eps4,0,2);
        abarh = mean(C.a_bar_hat_out(k1-1,:),2); A_a = -2*b*(1-abarh);
        e3 = C.delta_x_hat_3_out(k1,:)/R - C.trk_true(k1-1,:);
        C34 = zeros(numel(k1),1);
        for kk = 1:numel(k1); c = cov(e3(kk,:), eps4(kk,:)); C34(kk) = c(1,2); end
        Mmean = mean(Mh,2);
        T_lin   = A_a .* dWd .* m4;                       % linear amplifier on existing mean
        T_P44   = b .* Mmean .* v4;                       % pure rectifier (variance)
        T_m2    = b .* Mmean .* m4.^2;                    % mean^2 (second order in mean)
        T_cross = A_a .* (1-lc) .* movmean(C34,801);      % cross rectifier
        seg = @(m) [sum(T_lin(m)) sum(T_P44(m)) sum(T_m2(m)) sum(T_cross(m))];
        md = t(k1) <= 10.5; mh2 = t(k1) > 10.5;
        fprintf('[%s] SE second-order split  (lin | P44 | m^2 | cross)  x1e-3\n', A{1});
        fprintf('     descent  %+7.3f %+7.3f %+7.3f %+7.3f\n', 1e3*seg(md));
        fprintf('     hold     %+7.3f %+7.3f %+7.3f %+7.3f\n', 1e3*seg(mh2));
        fprintf('     total    %+7.3f %+7.3f %+7.3f %+7.3f   | sum %+7.3f\n', 1e3*seg(true(size(md))), 1e3*sum(seg(true(size(md)))));
        % where along the descent does the cross term accumulate (by w_bar_d band)
        hd = d.hd(k1);
        for band = {[5 7], [3 5], [2 3], [1.5 2], [1 1.5]}
            mb = md & hd >= band{1}(1) & hd < band{1}(2);
            if any(mb); fprintf('     w_d in [%.1f,%.1f): P44 %+7.3f  cross %+7.3f  (x1e-3), mean v4 %.2e, mean C34 %.2e\n', ...
                band{1}, 1e3*sum(T_P44(mb)), 1e3*sum(T_cross(mb)), mean(v4(mb)), mean(C34(mb))); end
        end
    end
end
