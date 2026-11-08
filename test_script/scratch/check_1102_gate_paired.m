% STATUS: SCRATCH -- 2026-11-02 (user): paired gate on against gate off from 2 s, only the seeds that survive, because a few diverged seeds dominate the rms. temp_gate_off.mat arms A, C and temp_gate_from2.mat arms E, F, flat wall, 20 seeds. Sanity: gate-on rms repeats 3.14 and 1.31 deg at 11, 12 s when no seed is dropped.
fo = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_3d_wsonly');  S1 = load(fullfile(fo, 'temp_gate_off.mat'));  S2 = load(fullfile(fo, 'temp_gate_from2.mat'));  NS = 20;  tq = [10 10.5 11 11.5 12];  pair = {[1 1 0 0], [3 2 8 -8]};
for p = 1:2
    a = pair{p}(1);  b = pair{p}(2);  Tx = pair{p}(3);  Ty = pair{p}(4);  r1 = S1.rs(S1.IA == a);  r2 = S2.rs(S2.IA == b);  good = false(NS, 1);
    for s = 1:NS; q = r2{s};  good(s) = isfinite(q.thx(find(q.t >= 11.99, 1))) && max(abs(q.thx(q.t >= 8 & q.t < 12))) < 45; end
    fprintf('true (%g, %g): seeds kept %d of %d, dropped %s\n', Tx, Ty, sum(good), NS, mat2str(find(~good)'));
    for j = 1:numel(tq)
        A = [];  B = [];  for s = find(good)'; i = find(r1{s}.t >= tq(j), 1);  A = [A; r1{s}.thx(i) - Tx, r1{s}.thy(i) - Ty];  i = find(r2{s}.t >= tq(j), 1);  B = [B; r2{s}.thx(i) - Tx, r2{s}.thy(i) - Ty]; end
        fprintf('  t %5.2f s: gate on rms %.2f deg, gate off from 2 s rms %.2f deg, paired rms difference %.2f deg\n', tq(j), rms(A(:)), rms(B(:)), rms(A(:) - B(:)));
    end
    for s = find(~good)'; q = r2{s};  k = find(q.t >= 2 & q.g < 0.03, 1);  m = find(~isfinite(q.thx) | abs(q.thx) > 45, 1);  fprintf('  dropped seed %d: first g-hat < .03 after 2 s at %.2f s, angle gone at %.2f s, g-hat min %.3f\n', s, q.t(k), q.t(m), min(q.g(q.t >= 2))); end
end
