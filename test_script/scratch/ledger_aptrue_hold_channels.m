% STATUS: ACTIVE (scratch) | PURPOSE: where does the closed-loop hold bias of the a'_true
%   @est recipe come from AFTER the predict stage is unbiased (pred_mean2_kr1_full)?
%   Per arm of aptrue_kr1_full_<traj>.mat (10 seeds), over the near-wall hold, the
%   gain-row budget in abar (seed mean):
%     dA        = a_hat[end] - a_hat[start]                 what the estimate did
%     S_pred    = dA - S_y1 - S_y2                          predict-stage sum (exact step + mean lines)
%     S_y1      = sum l41 * innov_y1                        y1's push on the gain (through P41)
%     S_y2      = sum l42 * innov_y2                        y2's push
%     S_mean2   = sum pred_mean2                            the mean lines alone
%   and the readout-target check: E[innov_y2] vs E[a_true - a_hat] (y2 = a_bar[k-d] + n_a,
%   y2_hat = a_hat in hold, so a y2 innovation mean that differs from the true error mean
%   is a biased readout target). E[innov_y1] = E[e3] lagged (A1 in hold).
%   [hypothesis] y2 readout target below truth near the wall  |  opponent: y1 channel
%   pushed by E[e3] != 0  |  separated here by S_y1 vs S_y2 and the innovation means.
function out = ledger_aptrue_hold_channels(traj)
    if nargin < 1 || isempty(traj); traj = 'canon'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, sprintf('aptrue_kr1_full_%s.mat', traj)));
    t = D.left.t;  t3 = D.phases(3);  m = t > t3;  m2 = t > t3 + 0.5*(t(end) - t3);   % hold, and its second half
    TAG = {'left','right'};
    fprintf('[%s] hold t > %.2f s (%d steps), 10 seeds, abar units; second half in brackets\n', traj, t3, sum(m));
    fprintf('%-9s %9s %9s %9s %9s %9s | %10s %10s %10s | %10s %10s\n', 'arm', 'E(hold)', 'dA', 'S_pred', 'S_y1', 'S_y2', 'E[inn2]', 'E[T-E]', 'inn2-(T-E)', 'E[inn1] R', 'l41 mean');
    out = struct();
    for a = 1:2
        d = D.(TAG{a});  C = d.C;  nS = size(d.E, 2);
        E = d.E;  TmE = -E;                                        % True - Est
        i0 = find(m, 1);  i1 = find(m, 1, 'last');
        dA   = mean(C.a_bar_hat_out(i1,:) - C.a_bar_hat_out(i0,:));
        Sy1  = mean(sum(C.K_a_y1_out(m,:) .* C.innov_y1_out(m,:), 1));
        Sy2  = mean(sum(C.K_a_y2_out(m,:) .* C.innov_y2_out(m,:), 1));
        Sm2  = mean(sum(C.pred_mean2_out(m,:), 1));
        Spred = dA - Sy1 - Sy2;
        inn2 = mean(C.innov_y2_out(m,:), 'all');  inn2b = mean(C.innov_y2_out(m2,:), 'all');
        tme  = mean(TmE(m,:), 'all');             tmeb  = mean(TmE(m2,:), 'all');
        inn1 = mean(C.innov_y1_out(m,:), 'all');
        l41  = mean(C.K_a_y1_out(m,:), 'all');    l42 = mean(C.K_a_y2_out(m,:), 'all');
        fprintf('%-9s %+9.5f %+9.5f %+9.5f %+9.5f %+9.5f | %+10.5f %+10.5f %+10.5f | %+10.2e %+10.4f\n', d.label, mean(E(m,:),'all'), dA, Spred, Sy1, Sy2, inn2, tme, inn2 - tme, inn1, l41);
        fprintf('%-9s %9s %9s %9s %9s %9s | [%+9.5f] [%+9.5f] [%+9.5f] | l42 mean %+.4f, S_mean2 %+.5f, SEM(E hold) %.5f\n', '', '', '', '', '', '', inn2b, tmeb, inn2b - tmeb, l42, Sm2, std(mean(E(m,:),1))/sqrt(nS));
        out.(TAG{a}) = struct('E', mean(E(m,:),'all'), 'dA', dA, 'Spred', Spred, 'Sy1', Sy1, 'Sy2', Sy2, 'inn2', inn2, 'tme', tme, 'inn1', inn1);
    end
end
