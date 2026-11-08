fo = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_3d_wsonly');  load(fullfile(fo, 'temp_1104_flow_circle_s2.mat'), 'r');
n = find(isfinite(r.thx), 1, 'last');  fprintf('last finite t %.4f, tsc %.4f\n', r.t(n), r.tsc);
for i = [find(r.t >= 4.9, 1) find(r.t >= 5.0, 1) find(r.t >= 5.5, 1) find(r.t >= 5.9, 1) n - 30:3:n]
    fprintf('t %.4f gap %.3f | th (%.1f %.1f) tr (%.1f %.1f) | a11 %.3f a33 %.3f g %.3f sg %.3f gate %d | ap33 %.2f ap11 %.2f | kap %.2f | sP %.2f %.2f | mineig %.3g\n', r.t(i), r.h(i) * 2.25, r.thx(i), r.thy(i), r.th_tr(i, :), r.aT(i), r.aN(i), r.dgap(i), r.sgap(i), r.gate(i), r.apN(i), r.apT(i), r.kap(i), r.sPth(i, :), r.mineig(i));
end
