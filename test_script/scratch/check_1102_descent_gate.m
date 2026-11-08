% STATUS: SCRATCH -- 2026-11-02 (user): the angle does not converge during the descent 10-11 s although g grows, in conflict with "small g, little information". Per window: fraction of steps with the angle gate open, sd of the angle step, g-hat against the true g, gate level, and the information expected from the true g alone (info rate proportional to g^2, scaled so that 0-10 s gives the measured 4.0 deg). temp_hold_angle12.mat (5 seeds). Rival checks: gate closed in the descent | g-hat error. Sanity: hold window gate 90-98%.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  load(fullfile(fo, 'temp_hold_angle12.mat'), 'rs');  W = [8 10; 10 10.25; 10.25 10.5; 10.5 10.75; 10.75 11; 11 11.25; 11.25 12];  NS = numel(rs);
fprintf('window s | gate open %% | sd dth_u, dth_v deg/step | g true | g-hat mean (min seed) | 2 sqrt P gap | g-hat - g rms\n');
for j = 1:size(W, 1)
    G = [];  D = [];  gt = [];  gh = [];  sg = [];  mn = inf;
    for q = 1:NS
        r = rs{q};  m = r.t >= W(j, 1) & r.t < W(j, 2) & isfinite(r.g);  G = [G; r.gate(m) > 0];  D = [D; r.dth(m, 1), r.dth(m, 2) .* cosd(r.thx(m))];  gt = [gt; r.aT_tr(m) - r.aN_tr(m)];  gh = [gh; r.g(m)];  sg = [sg; r.sg(m)];  mn = min(mn, min(r.g(m)));
    end
    fprintf('%.2f-%.2f | %.1f | %.4f %.4f | %.3f | %.3f (%.3f) | %.3f | %.3f\n', W(j, 1), W(j, 2), 100 * mean(G), std(D(:, 1)), std(D(:, 2)), mean(gt), mean(gh), mn, mean(2 * sg), sqrt(mean((gh - gt).^2)));
end
r = rs{1};  dt = r.t(2) - r.t(1);  g2 = (r.aT_tr - r.aN_tr).^2;  g2(~isfinite(g2)) = 0;  ih = sum(g2(r.t < 10)) * dt;  c = (1 / 4.0^2 - 1 / 10^2) / ih;  tq = [10 10.5 11 11.5 12];
for t0 = tq; I = sum(g2(r.t < t0)) * dt;  fprintf('t %.1f s: integral g^2 dt %.3f, expected sigma from g alone %.2f deg\n', t0, I, 1 / sqrt(1 / 10^2 + c * I)); end
