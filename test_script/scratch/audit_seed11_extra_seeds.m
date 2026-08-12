% STATUS: ACTIVE | seed-11 outlier adjudication, part 3 (is the new arm just heavy-tailed?)
%   EXPIRES: with audit_seed11_rng_stream.m -- when the seed-11 verdict is on record.
% FORK OF nothing (new diagnostic) | PURPOSE: 12 seeds OUTSIDE the house set, both
%   arms, same canonical scenario. If the new arm produces further hold errors of
%   seed-11 size on fresh draws, seed 11 is an ordinary tail of a high-variance arm
%   rather than a special event. Zero production changes. 產線改動不會自動跟上

clear cd
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

EXTRA = [1 2 3 5 13 17 19 29 37 53 71 97];

oB = run_formB_ws(struct('seeds', EXTRA));
oC = run_formC_state(struct('seeds', EXTRA));

MB = oB.metrics.rows(:, 1:3);   MC = oC.metrics.rows(:, 1:3);
fprintf('\n=== 12 fresh seeds, side by side (z axis) ===\n');
fprintf('%6s | %17s | %17s | %19s\n', 'seed', 'desc pk % (B / C)', 'osc RMS % (B / C)', 'hold mean % (B / C)');
for q = 1:numel(EXTRA)
    fprintf('%6d | %7.2f %9.2f | %7.2f %9.2f | %+8.2f %+10.2f\n', EXTRA(q), ...
            MB(q, 1), MC(q, 1), MB(q, 2), MC(q, 2), MB(q, 3), MC(q, 3));
end
fprintf('\nformB over 12: desc %.2f+-%.2f  osc %.2f+-%.2f  hold %+.2f+-%.2f\n', ...
        mean(MB(:,1)), std(MB(:,1)), mean(MB(:,2)), std(MB(:,2)), mean(MB(:,3)), std(MB(:,3)));
fprintf('formC over 12: desc %.2f+-%.2f  osc %.2f+-%.2f  hold %+.2f+-%.2f\n', ...
        mean(MC(:,1)), std(MC(:,1)), mean(MC(:,2)), std(MC(:,2)), mean(MC(:,3)), std(MC(:,3)));
fprintf('formC hold: max %+.2f (seed %d), how many >= +10%%: %d of 12\n', ...
        max(MC(:,3)), EXTRA(find(MC(:,3) == max(MC(:,3)), 1)), sum(MC(:,3) >= 10));
fprintf('formC osc RMS: max %.2f (seed %d), how many >= 8%%: %d of 12\n', ...
        max(MC(:,2)), EXTRA(find(MC(:,2) == max(MC(:,2)), 1)), sum(MC(:,2) >= 8));

save(fullfile(proj, 'test_results', 'temp_audit_seed11_extra_seeds.mat'), 'EXTRA', 'MB', 'MC');
fprintf('saved: test_results/temp_audit_seed11_extra_seeds.mat\n');
