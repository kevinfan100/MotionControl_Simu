% STATUS: ACTIVE (scratch) | PURPOSE: per-segment, term-by-term account of the PREDICT-stage
%   remainder AFTER compensation on the FINAL a'_true @est recipe captured by
%   probe_aptrue_est_final.m. Everything from logged quantities, no fit.
%   Per record j (call k = j+1), posterior at k-1 -> prior at k, all True - Est, [abar]:
%     M   = Dw_d + al (x3 + x8 + x9)             the estimator's known step (code M_pred)
%     Bt  = w[k] - w[k-1]                        true height increment
%     u   = Bt - M                               unknown step
%     e3  = (hd[k-1] - w[k-1]) - x3              slot-3 error before the step
%     De  = (a_true[k] - a_pred[k]) - (a_true[k-1] - a_upd[k-1])
%     r   = De - a' u                            post-compensation remainder (mean2 + kr1 already inside De)
%   Second-order identity (derivation 0902, 'est' reading, expanded about the filter's height):
%     r = A2 + A2m + Du_ + Dvar + Dcov + A1d + A1x - kr1 + R3
%       A2   = a'' al x3 u              known-step x unknown-step correlation (A2 of the tex; kr1 is its closed form)
%       A2m  = a'' al (x8 + x9) u       same, memory states
%       Du_  = a'' Dw_d u               command step x mean of the unknown step
%       Dvar = a''/2 (u^2 - var_code)   Jensen, true vs the filter's P
%       Dcov = -a'' (u e3 - cov_code)   start-point term, true vs the filter's P
%       A1d  = -a'' Dw_d e3             A1 of the tex: E[e3] = 0 (command step part)
%       A1x  = -a'' al (x3+x8+x9) e3    A1/orthogonality: E[x3 e3] = 0 (estimate part)
%       R3   = r - (all of the above)   third order and anything not in the expansion
%   miss = a'' (1-lc) K31 R1 - (what the recipe injected): the closed-form prediction of r
%   (kr1 recipe: a'' (1-lc) lc K31 R1; kr1_full: 0). Read r against miss per segment.
%   Units 1e-6 abar/step; cumulative sums in abar.
function out = analyze_aptrue_est_final(traj, recipe)
    if nargin < 1 || isempty(traj); traj = 'meng'; end
    if nargin < 2 || isempty(recipe); recipe = 'kr1'; end
    sfx = '';  if strcmp(recipe, 'kr1_full'); sfx = '_full'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = load(fullfile(od, sprintf('aptrue_est_final_%s%s.mat', traj, sfx)));  S = D.S;
    fprintf('[%s] recipe %s\n', traj, recipe);
    pc = physical_constants();  lc = 0.7;  al = 1 - lc;
    a_o = pc.Ts / (pc.gamma_N * pc.R);  kappa_T = 4 * (pc.k_B * pc.T / pc.R) * a_o;
    HFLOOR = 1.001;
    abar = @(w) 1 ./ arrayfun(@(x) local_cp(x), w);
    hh = 1e-3;
    ap_num  = @(w) (abar(w+hh) - abar(w-hh)) / (2*hh);
    app_num = @(w) (abar(w+hh) - 2*abar(w) + abar(w-hh)) / hh^2;
    switch traj
        case 'meng';  SEG = {'hold0', @(t) t<=0.5; 'far', @(t) t>0.5 & t<=6; 'near', @(t) t>6 & t<=10.5; 'hold', @(t) t>10.5; 'ALL', @(t) true(size(t))};
        case 'canon'; SEG = {'hold0', @(t) t<=0.5; 'desc', @(t) t>0.5 & t<=1.5; 'osc', @(t) t>1.5 & t<=3.5; 'hold', @(t) t>3.5; 'ALL', @(t) true(size(t))};
    end
    names = {'r','miss','A2','A2m','Du_','Dvar','Dcov','A1d','A1x','kr1','R3','Du','Eex'};
    aux   = {'e3','u','x3u','K31','app','Dwd','M','dT','dEp','dEu','a1M','a1u','m2','ex2','y1push'};   % mean budget of the gain row
    acc = struct();  for f = [names aux]; acc.(f{1}) = []; end;  acc.t = [];  acc.seed = [];
    nS = numel(S);
    ci = @(a,b) find(all(S(1).PIDX == [a b], 2));
    for q = 1:nS
        s = S(q);  n = size(s.x_upd, 1);
        w = s.h_bar_true;  at = s.a_true;  hd = s.hd;  apl = s.a_prime;
        x3 = s.x_upd(:,3);  x4 = s.x_upd(:,4);  x8 = s.x_upd(:,8);  x9 = s.x_upd(:,9);  xp4 = s.x_pred(:,4);
        j = (4:n).';  kk = j + 1;
        Dwd = hd(kk) - hd(kk-1);
        M   = Dwd + al*x3(j-1) + al*(x8(j-1) + x9(j-1));
        Bt  = w(kk) - w(kk-1);
        u   = Bt - M;
        a1  = apl(kk);
        heval = max(hd(kk-1) - x3(j-1), HFLOOR);
        app = app_num(heval);
        d3t = hd(kk-1) - w(kk-1);
        e3  = d3t - x3(j-1);
        x4p = x4(j-1);
        De  = (at(kk) - xp4(j)) - (at(kk-1) - x4p);
        r   = De - a1 .* u;
        % --- the code's compensation, reconstructed from the SAME P and F it used ---
        P33 = s.PU(j-1,ci(3,3));  P34 = s.PU(j-1,ci(3,4));  P44 = s.PU(j-1,ci(4,4));
        P38 = s.PU(j-1,ci(3,8));  P39 = s.PU(j-1,ci(3,9));  P88 = s.PU(j-1,ci(8,8));  P99 = s.PU(j-1,ci(9,9));
        P89 = s.PU(j-1,ci(8,9));  P48 = s.PU(j-1,ci(4,8));  P49 = s.PU(j-1,ci(4,9));
        Fdw = -s.F34(j);  R1 = s.R1(j);
        cov_code = al*P33 + al*(P38 + P39) + Fdw.*P34;
        var_code = al^2*P33 + al^2*(P88 + P99 + 2*P89) + 2*al*al*(P38 + P39) + Fdw.^2.*P44 ...
                 + 2*Fdw.*(al*P34 + al*(P48 + P49)) + kappa_T*x4p + al^2*R1;
        app_law  = -2 * a1.^2 ./ (1 - x4p);
        if strcmp(recipe, 'kr1_full'); kr1 = app .* al .* s.K31(kk-1) .* R1; else; kr1 = app .* al^2 .* s.K31(kk-1) .* R1; end
        miss     = app .* al .* R1 .* s.K31(kk-1) - kr1;              % the feedthrough share the recipe does NOT inject (0 for kr1_full)
        gap      = 0.5 * (app - app_law) .* M.^2;
        mean2_rec = -app.*cov_code + 0.5*app.*var_code + gap + kr1;
        % --- decomposition ---
        A2   = app .* al .* x3(j-1) .* u;
        A2m  = app .* al .* (x8(j-1) + x9(j-1)) .* u;
        Du_  = app .* Dwd .* u;
        Dvar = 0.5 * app .* (u.^2 - var_code);
        Dcov = -app .* (u .* e3 - cov_code);
        A1d  = -app .* Dwd .* e3;
        A1x  = -app .* al .* (x3(j-1) + x8(j-1) + x9(j-1)) .* e3;
        R3   = r - (A2 + A2m + Du_ + Dvar + Dcov + A1d + A1x - kr1);
        Du   = xp4(j) - x4(j);                                    % update stage, True - Est
        Eex  = at(kk) - x4(j);                                    % closed-loop error after the update
        x3u  = x3(j-1) .* u;  K31 = s.K31(kk-1);
        dT   = at(kk) - at(kk-1);                                 % true gain increment
        dEp  = xp4(j) - x4(j-1);                                  % estimator predict increment (exact step + mean lines)
        dEu  = x4(j) - xp4(j);                                    % estimator update increment (y1 + y2)
        a1M  = a1 .* M;  a1u = a1 .* u;  m2 = s.mean2(kk);  ex2 = 0.5 * app_law .* M.^2;
        y1push = s.Ka1(kk) .* s.innov1(kk);                       % l41 * innovation_y1 of call k
        if q == 1
            fprintf('[check %s] a_o %.4e kappa_T %.4e R1 %.3e | fed slope vs abar''(h_eval): max rel %.2e | app(h_eval) range [%.3f, %.3f]\n', ...
                traj, a_o, kappa_T, R1(1), max(abs(a1 - ap_num(heval)) ./ abs(a1)), min(app), max(app));
            fprintf('[check %s] code mean2 reproduced: max |mean2_rec - pred_mean2_out| %.2e (mean2 range [%.2e, %.2e])\n', ...
                traj, max(abs(mean2_rec - s.mean2(kk))), min(s.mean2(kk)), max(s.mean2(kk)));
            xp = x4p + a1 .* M ./ (1 + a1 .* M ./ (1 - x4p)) + s.mean2(kk);
            fprintf('[check %s] exact step + mean2 reproduces x_pred(4): max |err| %.2e\n', traj, max(abs(xp - xp4(j))));
            fprintf('[check %s] K31 from P_pred: max |P31/(P11+R1) - K_dx_y1_out| %.2e | gate closed %.1f%% of records\n', traj, ...
                max(abs(s.Pp31(j)./(s.Pp11(j) + s.R1(j)) - s.K31(kk))), 100*mean(s.gate));
        end
        for f = [names aux]; acc.(f{1}) = [acc.(f{1}); eval(f{1})]; end
        acc.t = [acc.t; s.t(kk)];  acc.seed = [acc.seed; q*ones(numel(kk),1)];
    end
    % --- report ---
    fprintf('\n[%s] segment means, 1e-6 abar/step (True - Est); SEM over %d seeds in the row below\n', traj, nS);
    fprintf('%-6s%6s', 'seg', 'steps'); for f = names; fprintf('%9s', f{1}); end; fprintf('\n');
    for si = 1:size(SEG,1)
        m = SEG{si,2}(acc.t);  nst = sum(m)/nS;
        fprintf('%-6s%6.0f', SEG{si,1}, nst);
        for f = names; fprintf('%9.3f', 1e6*mean(acc.(f{1})(m))); end; fprintf('\n');
        fprintf('%-6s%6s', ' SEM', '');
        for f = names
            pm = arrayfun(@(q) mean(acc.(f{1})(m & acc.seed==q)), 1:nS);
            fprintf('%9.3f', 1e6*std(pm)/sqrt(nS));
        end; fprintf('\n');
    end
    fprintf('\n[%s] cumulative per segment, abar (seed mean): sum over the segment''s steps\n', traj);
    fprintf('%-6s', 'seg'); for f = names(1:11); fprintf('%9s', f{1}); end; fprintf('\n');
    for si = 1:size(SEG,1)
        m = SEG{si,2}(acc.t);  fprintf('%-6s', SEG{si,1});
        for f = names(1:11); fprintf('%9.5f', sum(acc.(f{1})(m))/nS); end; fprintf('\n');
    end
    fprintf('\n[%s] closed-loop Eex (True - Est, abar) per segment: mean, and at the last step; aux means\n', traj);
    fprintf('%-6s%10s%10s | %10s%10s%10s%10s%9s%10s%10s\n', 'seg', 'Eex', 'Eex_end', 'E[e3]', 'E[u]', 'E[x3 u]', 'K31', 'app', 'Dwd', 'M');
    for si = 1:size(SEG,1)
        m = SEG{si,2}(acc.t);
        eend = arrayfun(@(q) acc.Eex(find(m & acc.seed==q, 1, 'last')), 1:nS);
        fprintf('%-6s%+10.5f%+10.5f | %+10.2e%+10.2e%+10.2e%10.3f%9.3f%+10.2e%+10.2e\n', SEG{si,1}, mean(acc.Eex(m)), mean(eend), ...
            mean(acc.e3(m)), mean(acc.u(m)), mean(acc.x3u(m)), mean(acc.K31(m)), mean(acc.app(m)), mean(acc.Dwd(m)), mean(acc.M(m)));
    end
    fprintf('\n[%s] mean budget of the gain row per segment, 1e-6 abar/step (seed mean): truth vs estimator predict / update\n', traj);
    fprintf('%-6s%9s%9s%9s | %9s%9s%9s%9s | %9s%10s\n', 'seg', 'E[dTrue]', 'E[dPred]', 'E[dUpd]', 'E[a1 M]', 'E[a1 u]', 'E[mean2]', 'E[ex2]', 'l41*inn1', 'E[e3] R');
    for si = 1:size(SEG,1)
        m = SEG{si,2}(acc.t);
        fprintf('%-6s%+9.3f%+9.3f%+9.3f | %+9.3f%+9.3f%+9.3f%+9.3f | %+9.3f%+10.2e\n', SEG{si,1}, 1e6*mean(acc.dT(m)), 1e6*mean(acc.dEp(m)), 1e6*mean(acc.dEu(m)), ...
            1e6*mean(acc.a1M(m)), 1e6*mean(acc.a1u(m)), 1e6*mean(acc.m2(m)), 1e6*mean(acc.ex2(m)), 1e6*mean(acc.y1push(m)), mean(acc.e3(m)));
    end
    out = acc;
end
function cp = local_cp(w)
    [~, cp] = calc_correction_functions(w);
end
