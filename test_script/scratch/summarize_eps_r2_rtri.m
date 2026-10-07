% PURPOSE (2026-10-05, user: "find the best eps for the plane and for the cell separately"): Q55 = eps R2 on ramp_tri (Td = 1 s),
%   c-only f1p2, plane init, sqrt(P_0) = 0.1, 10 seeds.  Per wall: triangle mean|e_a|, mean |trough| (1/2/3 s), troughs, guard hits;
%   best eps = smallest triangle mean|e_a| among arms with no c guard hit.  Then draws the two winners next to the P_fix designs.
%   STATUS: SCRATCH
function summarize_eps_r2_rtri()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');
    E = {1e-5, 'eps5e'; 1e-4, 'eps4e'; 3e-4, 'epsv0p0003'; 1e-3, 'eps3e'; 3e-3, 'epsv0p003'; 5e-3, 'epsv0p005'; ...
         1e-2, 'eps2e'; 2e-2, 'epsv0p02'; 3e-2, 'r2r003'; 3e-1, 'r2r030'};
    for w = {'plane', 'cell'}
        fprintf('\n%s\n  eps      | tri mean|e_a| | mean|trough| | troughs 1 / 2 / 3 s       | guard steps (seeds)\n', w{1});
        best = Inf;  bname = '';
        for i = 1:size(E, 1)
            fn = fullfile(id, sprintf('f1p2_%s_PI_sig010_%s_rtri_f1.mat', w{1}, E{i, 2}));
            if ~exist(fn, 'file'); fprintf('  %-8g missing\n', E{i, 1}); continue; end
            S = load(fn);  t = S.t;  ea = S.a_true - S.a_hat;  at = @(tt) mean(ea(find(t >= tt - 1e-9, 1), :));
            tr = [at(1) at(2) at(3)];  tri = mean(mean(abs(ea(t >= 1, :))));  g = S.c_hat > 0.95;
            fprintf('  %-8g | %.4f        | %.4f       | %+.3f %+.3f %+.3f      | %d (%d)\n', E{i, 1}, tri, mean(abs(tr)), tr, nnz(g), nnz(any(g, 1)));
            if nnz(g) == 0 && tri < best; best = tri; bname = sprintf('%g', E{i, 1}); end
        end
        fprintf('  -> best eps without guard hits: %s (tri mean|e_a| %.4f)\n', bname, best);
    end
end
