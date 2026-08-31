% STATUS: ACTIVE (scratch) | PURPOSE: row/segment split of the v2 ensemble mean
%   forcing + mobility-component sign audit + row4-origin proxy (2026-08-31).
%   Consumes test_results/loop_mean_bias/replay_v2_profile.mat and the per-seed
%   captures. Findings: row3 37%% / row4 63%% of the bias; row4 almost entirely
%   from the near-wall transition gaps; comp sign -1 matches G3 (osc 1.01,
%   hold 0.95); row4 not the measurement legs.
here = fileparts(mfilename('fullpath'));
WT = fileparts(fileparts(here));
od = fullfile(WT, 'test_results', 'loop_mean_bias');
S = load(fullfile(od, 'replay_v2_profile.mat'));  o = S.out;  t = o.t(:);
C7 = load(fullfile(od, 'loop_capture_seed7.mat'));
nsl = size(C7.F, 1);  n = size(C7.F, 3);  i0 = 3;
gb3 = mean(o.G3, 2);  gb4 = mean(o.G4, 2);
rep = @(r3, r4) local_rep(C7, r3, r4, n, i0, nsl);
mh = t > 3.7;
E_33 = rep(gb3, zeros(n,1));  E_44 = rep(zeros(n,1), gb4);  E_all = rep(gb3, gb4);
fprintf('[row split] end-hold e4: row3-only %+.4f   row4-only %+.4f   both %+.4f   actual %+.4f\n', ...
        mean(E_33(mh)), mean(E_44(mh)), mean(E_all(mh)), mean(mean(o.E4A(mh,:),1)));
% segment split of each row's contribution (replay forcing restricted to one segment)
SEG = {'hold1', t > 0.05 & t < 0.5; 'desc', t > 0.55 & t < 1.45; 'osc', t >= 1.5 & t < 3.5; 'hold2', t > 3.7};
fprintf('[segment x row] end-hold e4 contribution:\n');
for g = 1:4
    m = SEG{g,2};
    E3s = rep(gb3 .* m, zeros(n,1));  E4s = rep(zeros(n,1), gb4 .* m);
    fprintf('  %-6s row3 %+.4f   row4 %+.4f\n', SEG{g,1}, mean(E3s(mh)), mean(E4s(mh)));
end
% mobility component sign audit: 4 combos vs G3 mean profile (segment means)
mo = t >= 1.5 & t < 3.5;
comp = o.comp(:);
for sflip = [1 -1]
    c = sflip * comp;
    fprintf('[comp sign %+d] osc: comp %+0.2e vs G3 %+0.2e | hold: comp %+0.2e vs G3 %+0.2e | ratio osc %.2f hold %.2f\n', ...
        sflip, mean(c(mo)), mean(gb3(mo)), mean(c(mh)), mean(gb3(mh)), mean(c(mo))/mean(gb3(mo)), mean(c(mh))/mean(gb3(mh)));
end
% row-4 forcing vs y2-channel mean drive: G4 should equal K2(4)*(innov2 - H2*e_pred) mean-wise;
% proxy check: mean K2(4)*innov2 per segment vs mean G4 (needs innov2, K2 from captures/logs of all seeds)
fprintf('[row4 origin proxy] per-seed descend means (x1e-5): G4 vs K2(4)*nu2 vs K1(4)*nu1\n');
for q = 1:8
    Cq = load(fullfile(od, sprintf('loop_capture_seed%d.mat', q)));
    Rq = load(fullfile(od, sprintf('run_log_seed%d.mat', q)));  r = Rq.run_log;
    kk = Cq.KK;  td = Cq.t(:);  md = td > 0.55 & td < 1.45;
    k24 = Cq.K2(4, :).';  k14 = Cq.K1(4, :).';
    nu2 = r.innov_y2_out(kk, 3);  nu2(~isfinite(nu2)) = 0;  nu1 = r.innov_y1_out(kk, 3);
    fprintf('  seed %d: %+6.2f   %+6.2f   %+6.2f\n', q, 1e5*mean(o.G4(md, q)), 1e5*mean(k24(md).*nu2(md)), 1e5*mean(k14(md).*nu1(md)));
end
fprintf('SPLIT DONE\n');

function E = local_rep(C, r3, r4, n, i0, nsl)
    e = zeros(nsl, 1);  E = zeros(n, 1);
    for i = i0+1:n
        g = zeros(nsl, 1);  g(3) = r3(i);  g(4) = r4(i);
        e = C.A(:,:,i) * e + g;
        E(i) = e(4);
    end
end
