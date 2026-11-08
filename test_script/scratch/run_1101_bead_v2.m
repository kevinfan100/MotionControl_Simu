% STATUS: SCRATCH -- 2026-11-01 (user): whole-bead raster after the rotation fix, same settings as run_1029_fillet_opts.m arms E (40 um/s) and D (10 um/s regression), seeds 1-3.
%   Old results printed first from temp_fillet_opts_kq40.mat and temp_fillet_opts_kq.mat. Pre-registered: 10 um/s must stay 3/3 finished without contact. Outputs temp_fillet_opts_kq40_v2.mat, temp_fillet_opts_kq_v2.mat, figure via plot_1030_bead40.m.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
old = {'temp_fillet_opts_kq40.mat', 'temp_fillet_opts_kq.mat'};  Xe = 13;  Rr0 = 2.25;
for j = 1:2
    S = load(fullfile(fo, old{j}));
    for id = 1:3
        r = S.rs{1, id};  kk = (1:r.kend)';  ok = kk(isfinite(r.aN(kk)) & isfinite(r.h(kk)));  ks = ok(r.t(ok) >= r.tsc);
        fprintf('OLD %-28s seed %d: contact %d, end t %.1f / %.1f s, last y %.1f um (~%d lines) | gap %.2f-%.2f um | mean|a33 - a_d| %.3f\n', old{j}, id, r.contact, r.t(ok(end)), r.t(end), r.p(ok(end), 2), max(0, round(r.p(ok(end), 2) + Xe) + 1), min(r.h(ks)) * Rr0, max(r.h(ks)) * Rr0, mean(abs(r.aN_tr(ks) - S.ad)));
    end
end
arms_in = 5;  sfx_in = '_kq40_v2';  run(fullfile(here, 'run_1029_fillet_opts.m'));
sfx40_in = sfx_in;  run(fullfile(here, 'plot_1030_bead40.m'));
arms_in = 4;  sfx_in = '_kq_v2';  run(fullfile(here, 'run_1029_fillet_opts.m'));
