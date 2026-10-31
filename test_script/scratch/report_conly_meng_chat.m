% PURPOSE (2026-10-07, branch test/conly-meng-law): does the journal law change the c ESTIMATE? From the stored stage-4 runs
%   (tags '' = Q55 = 0, 'pfix015_' = q55 = P_fix^2 I, sqrt P_fix 0.15), per wall x law x arm, 10 paired seeds: c_hat at the descent
%   end (1 s) and averaged over the triangle (1-3 s), its seed sd and the filter's own sqrt(P_c) (honesty = sd / sqrt P), and the
%   paired difference j6 - eq17. Reference c_eff(h_d) at the valley and the triangle mean (the constant's level-neutral value c*).
function report_conly_meng_chat()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');
    walls = {'plane', 'cell', 'sphere'};  laws = {'eq17', 'j6'};  tags = {'', 'pfix015_'};  tagname = {'Q55=0', 'P_fix 0.15'};
    fprintf('%-10s %-7s %-7s | %-22s %-22s | %-11s %-8s | %-14s\n', 'arm', 'wall', 'law', 'c_hat @1s (seed sd)', 'c_hat tri mean (sd)', 'sqrtP_c end', 'sd/sqrtP', 'c_eff valley/tri');
    for it = 1:2
        for iw = 1:3
            C = cell(1, 2);
            for il = 1:2
                S = load(fullfile(id, sprintf('f1p2_%s_PI_%s%s_rtri_f1.mat', walls{iw}, tags{it}, laws{il})));
                t = S.t;  i1 = find(t >= 1, 1);  tri = t >= 1 & t <= 3;
                c1 = S.c_hat(i1, :);  ctri = mean(S.c_hat(tri, :), 1);  sp = mean(S.sP_c(end, :));
                C{il} = struct('c1', c1, 'ctri', ctri);
                fprintf('%-10s %-7s %-7s | %+7.3f (%.3f)        %+7.3f (%.3f)        | %-11.3f %-8.2f | %+.3f / %+.3f\n', ...
                        tagname{it}, walls{iw}, laws{il}, mean(c1), std(c1), mean(ctri), std(ctri), sp, std(ctri) / max(sp, eps), S.c_eff_d(i1), mean(S.c_eff_d(tri)));
            end
            d1 = C{2}.c1 - C{1}.c1;  dt = C{2}.ctri - C{1}.ctri;  n = numel(d1);
            fprintf('%-10s %-7s %-7s | %+7.3f +- %.3f (t %+.1f) %+7.3f +- %.3f (t %+.1f)\n', '', '', 'j6-eq17', ...
                    mean(d1), std(d1) / sqrt(n), mean(d1) / (std(d1) / sqrt(n)), mean(dt), std(dt) / sqrt(n), mean(dt) / (std(dt) / sqrt(n)));
        end
    end
end
