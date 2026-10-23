% STATUS: SCRATCH -- 2026-10-16: what happens just before contact in run_mc_apreg (cell). Re-runs one seed with shorter T until it
%   survives (step 0.25 s), then prints the last 0.1 s: h_d, h, a_true, a_hat, e_a, law slope.
function probe_mc_apreg_contact(arm, seed, ad)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    fn = fullfile(od, sprintf('apreg_%s_cell_ad%03d.mat', arm, round(100 * ad)));  if ad == 0.50; fn = fullfile(od, sprintf('apreg_%s_cell.mat', arm)); end
    bk = [fn '.bak'];  copyfile(fn, bk);
    src = fileread(fullfile(here, 'run_mc_apreg.m'));
    Tok = NaN;
    for T = 5.75:-0.25:0.25
        s2 = strrep(src, 'T = 6;', sprintf('T = %g;', T));  s2 = strrep(s2, 'function run_mc_apreg(', 'function tmp_apreg_T(');
        tf = fullfile(here, 'tmp_apreg_T.m');  fid = fopen(tf, 'w');  fwrite(fid, s2);  fclose(fid);  clear tmp_apreg_T;
        evalc('tmp_apreg_T({arm}, {''cell''}, seed, ad);');  S = load(fn);
        if ~S.contact(1); Tok = T; break; end
    end
    delete(tf);
    fprintf('%s cell a_d %.2f seed %d: survives %g s, contact before %g s\n', arm, ad, seed, Tok, Tok + 0.25);
    n = find(~isnan(S.h_true(:, 1)), 1, 'last');
    for k = max(1, n - 160):20:n
        fprintf('  t %.3f | h_d %.3f | h %.3f | a_true %.3f | a_hat %.3f | e_a %+.3f | slope %.3f\n', S.t(k), S.wd(k), S.h_true(k), S.a_true(k), S.a_hat(k), S.a_true(k) - S.a_hat(k), S.ap_hat(k));
    end
    movefile(bk, fn);
end
