function out = probe_entryAB_cmdonly(seeds)
%PROBE_ENTRYAB_CMDONLY  TEMPORARY probe (2026-08-31) -- do NOT read as a fix.
%   Entry-A/B discriminator for the remaining thermal-power bias (~15 pp on the
%   b_true + exact base): the law predict's MEAN step integrates the COMMAND
%   displacement only (law_M_cmd_only), dropping the noisy 0.3*dw3 pull and the
%   MA feedthrough from the mean.  F_e / Q / K untouched (hybrid on purpose).
%     bias collapses  -> the rectifier eats through entry A (noisy displacement
%                        into the law); remedy-2 derivation targets the law row
%     bias stays      -> entry B (position-model mean, lambda_eff); derivation
%                        targets the position row
%   Also reports the hold segment (does dropping the pull kill the ability to
%   track the true gain's thermal wander?) and a first ORDER-OF-MAGNITUDE line
%   for the entry-A second-order candidate  sum 1/2|a''| Var(0.3 dw3)  -- the
%   per-step-independence assumption is WRONG (dw3 is correlated over steps),
%   so that number is a floor, not a prediction.
% STATUS: TEMPORARY | EXPIRES: entry decided + remedy-2 derivation lands

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'btrue_log_ledger');
    ax = 3;  R = physical_constants().R;  lam = 0.7;  ns = numel(seeds);
    ARMS = {'btrue+exact (ref)', struct('law_exact_step', true); ...
            'btrue+exact+cmdonly', struct('law_exact_step', true, 'law_M_cmd_only', true)};
    EB = cell(1,2);
    for a = 1:2
        O = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, 'verbose', false, ...
                               'b_true', true, 'b_true_at', 'cmd', 'ctrl_const_override', ARMS{a,2}));
        a_nom = O.runs{1}.a_nom;  t = O.runs{1}.tout(:);
        G = @(f) cell2mat(reshape(cellfun(@(r) r.(f)(:, ax), O.runs, 'UniformOutput', false), 1, []));
        ah = G('a_bar_hat_out');  at = G('a_true_out')/a_nom;
        K1 = G('K_a_y1_out');  n1 = G('innov_y1_out');  K2 = G('K_a_y2_out');  n2 = G('innov_y2_out');
        kk = 2:numel(t);  tt = t(kk);  m = tt >= 1.5 & tt < 3.5;  ho = tt > 3.7;
        eb = mean(ah(kk(ho),:),1) ./ mean(at(kk(ho),:),1) - 1;  EB{a} = eb;
        dah = ah(kk,:) - ah(kk-1,:);  dat = at(kk,:) - at(kk-1,:);
        % hold: does a_hat still track a_true's thermal wander? (corr of changes, 50 ms blocks)
        hb = find(ho);  B = 80;  nb = floor(numel(hb)/B);  U = zeros(nb, ns);  V = zeros(nb, ns);
        for bl = 1:nb
            j = hb((bl-1)*B + (1:B));
            U(bl,:) = ah(kk(j(end)),:) - ah(kk(j(1)),:);  V(bl,:) = at(kk(j(end)),:) - at(kk(j(1)),:);
        end
        Uc = U - mean(U,2);  Vc = V - mean(V,2);
        fprintf('%-22s trough bias %+6.2f %% (SEM %.2f) | osc excess %+8.4f | osc y1 %+8.4f y2 %+8.4f | hold track corr %+.3f slope %.2f\n', ...
                ARMS{a,1}, 100*mean(eb), 100*std(eb)/sqrt(ns), mean(sum(dah(m,:)-dat(m,:),1)), ...
                mean(sum(K1(kk(m),:).*n1(kk(m),:),1)), mean(sum(K2(kk(m),:).*n2(kk(m),:),1)), ...
                corr(Uc(:), Vc(:)), sum(Uc(:).*Vc(:))/sum(Vc(:).^2));
    end
    d = EB{2} - EB{1};
    fprintf('paired trough diff (cmdonly - ref): %+.2f +- %.2f %%  (t = %+.1f)   [TEMPORARY probe -- no conclusion yet]\n', ...
            100*mean(d), 100*std(d)/sqrt(ns), mean(d)/(std(d)/sqrt(ns)));

    % ---- entry-A candidate, order of magnitude only (per-step independence WRONG) ----
    S = load(fullfile(od, 'check_btrue_log_ledger.mat'));
    F = S.out.noisyExact;  mm = F.tt >= 1.5 & F.tt < 3.5;
    app = 2 * F.bh(2:end,:).^2 .* (1 - F.ah(2:end,:)).^3;            % |a''|
    v3 = var((1 - lam) * F.dx3(1:end-1,:), 0, 2);                    % per-step cross-seed Var of the pull level
    cand = sum(0.5 * mean(app(mm,:),2) .* v3(mm));
    fprintf('entry-A candidate floor: sum 1/2|a''''| Var(0.3 dw3) over osc = %+.4f a_o  (needed ~0.013; correlation-time factor missing by construction)\n', cand);
    out = struct('EB', {EB}, 'cand', cand);
    fprintf('PROBE AB DONE\n');
end
