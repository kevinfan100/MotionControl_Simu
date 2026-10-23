% FORK OF nothing (new) | PURPOSE: V0 wiring check for ctrl_const.b1_state (0914_estb_bprime_core.tex): the arm bseed0_b1z
%   (b1 slot free, P66[0] = 0) must reproduce bseed0 bit for bit on both trajectories; plus a 1-seed health smoke of bseed0_b1
%   | EXPIRES: when b1_state is closed | 產線改動不會自動跟上
here = fileparts(mfilename('fullpath')); addpath(here);
for tr = {'canon', 'meng'}
    tic; o = run_three_walls(tr{1}, 1:2, {'bseed0', 'bseed0_b1z'}, {'plane'}); t1 = toc;
    A = o.plane_bseed0; Bz = o.plane_bseed0_b1z;
    dE = max(abs(A.E - Bz.E), [], 'all'); dB = max(abs(A.B - Bz.B), [], 'all'); b1max = max(abs(Bz.B1), [], 'all');
    fprintf('V0 %-5s bseed0_b1z vs bseed0: max|dE| = %.3e  max|db| = %.3e  max|b1| = %.3e  -> %s   (%.0f s for 4 runs)\n', ...
        tr{1}, dE, dB, b1max, char(string(dE == 0 && dB == 0 && b1max == 0)), t1);
end
tic; o = run_three_walls('canon', 7, {'bseed0_b1'}, {'plane'}); t2 = toc;
S = o.plane_bseed0_b1;
fprintf('SMOKE canon bseed0_b1 seed 7: NaN %d | min a_hat %.4f | b_hat [%.3f %.3f] end %.3f | b1 [%.4f %.4f] end %.4f | sqrtP55 start %.3f end %.3f | sqrtP66 end %.4f | %.0f s\n', ...
    nnz(isnan(S.E)), min(S.AH), min(S.B), max(S.B), S.B(end), min(S.B1), max(S.B1), S.B1(end), S.sP5(2), S.sP5(end), S.sP6(end), t2);
