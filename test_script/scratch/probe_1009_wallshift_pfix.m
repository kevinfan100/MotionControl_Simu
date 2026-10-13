% STATUS: SCRATCH -- 2026-10-12, per-case sqrt(P_fix) for the c-only wall-position-error runs (user 10-12: "pick a good P_fix per case").
%   Reads run_1009_wallshift outputs (10 seeds), sqrt(P_fix) in PF; criterion = motion window first (wall-test protocol): the descent
%   (t <= 1 s) mean over seeds of mean |e_a|, e_a = a_true - a_hat. Prints the table and saves the choice to wallshift_best.mat.
%   REFERENCE ARM: the choice uses the known DELTA; the estimator itself cannot pick it.
function best = probe_1009_wallshift_pfix()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    DL = [0 -2 -4 2 4];  PF = [0.05 0.10 0.15 0.30 0.50 1.0];
    best = struct('delta', DL, 'pfix', nan(size(DL)), 'tag', {cell(size(DL))});
    fprintf('descent mean |e_a| (near-wall 0.9-1.0 s mean e_a | hold mean e_a), 10 seeds\n%-6s', 'DELTA');
    for j = 1:numel(PF); fprintf('%28s', sprintf('sqrtPfix %.2f', PF(j))); end;  fprintf('\n');
    for i = 1:numel(DL)
        base = strrep(strrep(sprintf('%+g', DL(i)), '+', 'p'), '-', 'm');  J = nan(size(PF));  fprintf('%+-6g', DL(i));
        for j = 1:numel(PF)
            tag = base;  if abs(PF(j) - 0.15) > 1e-12; tag = sprintf('%s_pf%03d', base, round(100 * PF(j))); end
            fn = fullfile(od, ['wallshift_' tag '.mat']);  if ~exist(fn, 'file'); fprintf('%28s', '-'); continue; end
            S = load(fn);  t = S.t;  e = S.a_true - S.a_hat;
            J(j) = mean(mean(abs(e(t <= 1, :)), 1));  m = mean(e, 2);
            fprintf('%28s', sprintf('%.4f (%+.3f | %+.3f)', J(j), mean(m(t > 0.9 & t <= 1)), mean(m(t > 1))));
        end
        [~, jb] = min(J);  best.pfix(i) = PF(jb);
        best.tag{i} = base;  if abs(PF(jb) - 0.15) > 1e-12; best.tag{i} = sprintf('%s_pf%03d', base, round(100 * PF(jb))); end
        fprintf('   -> best %.2f\n', PF(jb));
    end
    save(fullfile(od, 'wallshift_best.mat'), '-struct', 'best');
end
