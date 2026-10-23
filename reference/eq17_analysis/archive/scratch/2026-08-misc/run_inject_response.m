function out = run_inject_response(seeds, opts)
%RUN_INJECT_RESPONSE  Injection-response test of the y1 -> a_bar path.
%
%   out = run_inject_response(1:8);                 % Meng 10 s ramp, +-5 % at 5.5 s
%
% STATUS: ACTIVE | diagnostic; uses the default-off controller hook
%   ctrl_const.a_inject_{step,frac,axis} (motion_control_law_formC_b, 2026-08-26)
%
% THE QUESTION. On the deep canonical band the trough estimate ends +18..20 %
% high, and the error account says that is the residue of two large cancelling
% terms: the law over-pushes DOWN (-58 % of trough) and the y1 path corrects UP
% (+90 %). A ledger cannot say whether "y1 over-corrects" -- the two channels
% share one posterior. A RESPONSE test can: inject a KNOWN gain error into the
% posterior a_bar once, mid-descent, and follow the paired difference
%       r(t) = (a_hat_inj - a_hat_base) / (injected amount)
% r decays from 1 to 0 if the correction path has unit gain; it crosses zero
% if the path over-corrects; +5 % and -5 % differ if the gain depends on the
% sign of the error (the a_bar'(a_hat) asymmetry hypothesis, since
% F_e(4,3) = (1-lc)*b*(1-a_hat)^2 is larger when a_hat is LOW).
%
% PRE-REGISTERED (written before the run, see inject_prereg.txt):
%   P1  r(t) decays toward 0 within ~1-2 s (y1 informative on the ramp)
%   P2  [hypothesis] the -5 % arm corrects harder and overshoots; ratio of
%       initial |dr/dt| (-5 % / +5 %) > 1
%   P3  [opponent]   both arms mirror each other, no overshoot => path gain 1;
%       the bias is not created by this asymmetry
%   P4  all three arms reach the SAME end-hold bias (attractor set by the model)
%
% SCENARIO. formC_b, arm best, ap_src post, on the Meng 10 s monotone ramp
% (h 15 -> 2.5 um, w_bar 6.667 -> 1.111, no oscillation; override copied
% from run_formC_dist_meng_pair). Base scenario is the deep house config, so
% h_bar_safe = 1.0 and y2 is never gated on this ramp. Percentages here are
% NOT comparable to canonical-band numbers.
%
% NEGATIVE CONTROLS, run first:
%   N1  hook off on the canonical scenario, seed 7: a_bar_hat_z[end] must equal
%       the fixture 0.108275 (2026-08-27, R2 colour factor IF(1); the
%       2026-08-24 smoke-test value under IF(s) was 0.107505).
%   N2  hook firing with frac = 0 on the Meng ramp, seed 1: bit-identical to
%       hook off.

    if nargin < 1 || isempty(seeds); seeds = 1:8; end
    if nargin < 2; opts = struct(); end
    % re-analysis form: run_inject_response(S) with S = load(...inject_response.mat)
    if isstruct(seeds)
        here = fileparts(mfilename('fullpath'));
        tg = ''; if isfield(seeds, 'tag'); tg = seeds.tag; end
        od = fullfile(fileparts(fileparts(here)), 'test_results', ['inject_response' tg]);
        out = local_analyze(seeds, od);  return;
    end
    if ~isfield(opts, 't_inj');  opts.t_inj  = 5.5;  end   % [s] mid-ramp, w_bar_d ~ 3.9
    if ~isfield(opts, 'frac');   opts.frac   = 0.05; end
    if ~isfield(opts, 'skip_neg'); opts.skip_neg = false; end
    if ~isfield(opts, 'extra_cc'); opts.extra_cc = struct(); end   % applied to ALL arms
    if ~isfield(opts, 'tag');      opts.tag = '';           end   % output subdir suffix

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', ['inject_response' opts.tag]);
    if ~exist(od, 'dir'); mkdir(od); end

    pc = physical_constants();
    ov = struct('trajectory_type', 'osc', 'h_init', 15.0, 'h_bottom', 2.5, ...
                'amplitude', 0, 'frequency', 1, 'n_cycles', 1, 't_hold', 0.5, ...
                't_descend_override', 10.0, 'T_sim', 12.5, 'h_min', 1.1 * pc.R);
    Ts = pc.Ts;
    % controller k_step: 1 on the init-only call, so log row k has k_step = k
    k_inj = round(opts.t_inj / Ts) + 1;

    base = struct('arm', 'best', 'ap_src', 'post', 'seeds', seeds, ...
                  'verbose', false, 'config_override', ov, ...
                  'ctrl_const_override', opts.extra_cc);
    mk = @(st, fr) local_merge(opts.extra_cc, struct('a_inject_step', st, 'a_inject_frac', fr));

    % ---- negative controls ---------------------------------------------
    if ~opts.skip_neg
        fprintf('\n[N1] canonical deep, seed 7, hook off\n');
        c7 = run_formC_b(struct('arm', 'best', 'ap_src', 'post', 'seeds', 7, 'verbose', false));
        v = c7.runs{1}.a_bar_hat_out(end, 3);
        fprintf('[N1] a_bar_hat_z[end] = %.6f  (fixture 0.108275)  %s\n', v, ...
                local_pf(abs(v - 0.108275) < 5e-7));
        fprintf('[N2] Meng ramp, seed %d, hook off vs firing with frac 0\n', seeds(1));
        b0 = base; b0.seeds = seeds(1);
        b1 = b0;   b1.ctrl_const_override = mk(k_inj, 0);
        r0 = run_formC_b(b0);  r1 = run_formC_b(b1);
        d = max(abs(r0.runs{1}.a_bar_hat_out(:) - r1.runs{1}.a_bar_hat_out(:)));
        fprintf('[N2] max|diff a_bar_hat| = %.3e  %s\n', d, local_pf(d == 0));
    end

    % ---- the three arms --------------------------------------------------
    fprintf('\n[arm] baseline\n');   A = run_formC_b(base);
    bp = base; bp.ctrl_const_override = mk(k_inj, +opts.frac);
    fprintf('[arm] +%.0f %%\n', 100*opts.frac);   P = run_formC_b(bp);
    bm = base; bm.ctrl_const_override = mk(k_inj, -opts.frac);
    fprintf('[arm] -%.0f %%\n', 100*opts.frac);   M = run_formC_b(bm);

    S = local_collect(A, P, M, k_inj, opts.frac);
    S.ov = ov; S.seeds = seeds; S.t_inj = opts.t_inj; S.extra_cc = opts.extra_cc;
    S.tag = opts.tag;
    save(fullfile(od, 'inject_response.mat'), '-struct', 'S');

    out = local_analyze(S, od);
end

% =======================================================================
function S = local_collect(A, P, M, k_inj, frac)
    ax = 3;  ns = numel(A.runs);
    r1 = A.runs{1};  N = numel(r1.tout);  a_nom = r1.a_nom;
    S = struct('t', r1.tout(:), 'k_inj', k_inj, 'frac', frac, 'a_nom', a_nom, ...
               'h_d', r1.h_bar_d_out(:), 'ns', ns);
    ARMS = {A, P, M};  NM = {'base', 'plus', 'minus'};
    for c = 1:3
        O = ARMS{c};
        F = struct();
        for f = {'a_bar_hat_out', 'a_true_out', 'K_a_y1_out', 'K_a_y2_out', ...
                 'innov_y1_out', 'innov_y2_out', 'P_a_out'}
            F.(f{1}) = zeros(N, ns);
            for q = 1:ns; F.(f{1})(:, q) = O.runs{q}.(f{1})(:, ax); end
        end
        F.a_true_out = F.a_true_out / a_nom;      % physical -> normalized
        F.P_a_out    = F.P_a_out    / a_nom;      % driver stores sqrt(P) physical
        S.(NM{c}) = F;
    end
end

% =======================================================================
function out = local_analyze(S, od)
    t = S.t;  kk = 2:numel(t);  t = t(kk);  ns = S.ns;  frac = S.frac;
    B = S.base; Pp = S.plus; Mm = S.minus;
    ab = B.a_bar_hat_out(kk,:); ap = Pp.a_bar_hat_out(kk,:); am = Mm.a_bar_hat_out(kk,:);
    at = B.a_true_out(kk,:);

    % where the jump actually landed (the hook fires on k_step == k_inj)
    dj = mean(ap - ab, 2);  [~, kj] = max(abs(diff(dj)));  kj = kj + 1;
    inj_amt_p = ab(kj, :) * frac;   inj_amt_m = -ab(kj, :) * frac;
    rp = (ap - ab) ./ inj_amt_p;    rm = (am - ab) ./ inj_amt_m;   % per seed
    fprintf('\n[inj] jump detected at row %d, t = %.4f s (requested %.4f s), w_bar_d = %.3f\n', ...
            kj, t(kj), S.t_inj, S.h_d(kj+1));
    fprintf('[inj] measured jump / requested: +arm %.4f  -arm %.4f\n', ...
            mean(rp(kj,:)), mean(rm(kj,:)));

    % response summaries
    win = @(dt) find(t >= t(kj) + dt, 1);
    fprintf('\n%-10s %10s %10s\n', 'after', 'r(+5%)', 'r(-5%)');
    for dt = [0.05 0.1 0.25 0.5 1 2 3 5]
        k = win(dt); if isempty(k); continue; end
        fprintf('%8.2f s %+10.3f %+10.3f\n', dt, mean(rp(k,:)), mean(rm(k,:)));
    end
    k1 = win(0.1);
    slope_p = (mean(rp(k1,:)) - 1) / 0.1;  slope_m = (mean(rm(k1,:)) - 1) / 0.1;
    fprintf('\ninitial |dr/dt| over 0.1 s:  +arm %.3f /s   -arm %.3f /s   ratio (-/+) = %.3f\n', ...
            abs(slope_p), abs(slope_m), abs(slope_m)/abs(slope_p));
    mn_p = min(mean(rp(kj:end,:),2));  mn_m = min(mean(rm(kj:end,:),2));
    fprintf('overshoot (min r after injection):  +arm %+.3f   -arm %+.3f\n', mn_p, mn_m);

    % end-hold bias, three arms
    hold_end = t > t(end) - 0.9;
    eb = mean(ab(hold_end,:) ./ at(hold_end,:) - 1, 1);
    ep = mean(ap(hold_end,:) ./ at(hold_end,:) - 1, 1);
    em = mean(am(hold_end,:) ./ at(hold_end,:) - 1, 1);
    fprintf('\nend-hold bias %%:  base %+.2f+-%.2f   +arm %+.2f+-%.2f   -arm %+.2f+-%.2f\n', ...
            100*mean(eb), 100*std(eb)/sqrt(ns), 100*mean(ep), 100*std(ep)/sqrt(ns), ...
            100*mean(em), 100*std(em)/sqrt(ns));
    fprintf('paired (arm - base):  +arm %+.3f+-%.3f   -arm %+.3f+-%.3f  [%%]\n', ...
            100*mean(ep-eb), 100*std(ep-eb)/sqrt(ns), 100*mean(em-eb), 100*std(em-eb)/sqrt(ns));

    % ledger of the RESPONSE (legitimate: response to a known perturbation)
    L = struct();
    for c = {'base','plus','minus'}
        F = S.(c{1});
        da  = diff(F.a_bar_hat_out(kk,:), 1, 1);
        y1  = F.K_a_y1_out(kk(2:end),:) .* F.innov_y1_out(kk(2:end),:);
        y2  = F.K_a_y2_out(kk(2:end),:) .* F.innov_y2_out(kk(2:end),:);
        L.(c{1}) = struct('y1', y1, 'y2', y2, 'pr', da - y1 - y2);
    end
    resp = struct();
    for c = {'plus','minus'}
        amt = mean(S.(c{1}).a_bar_hat_out(kj+1,:) - B.a_bar_hat_out(kj+1,:));
        resp.(c{1}) = struct( ...
            'y1', cumsum(mean(L.(c{1}).y1 - L.base.y1, 2)) / amt, ...
            'y2', cumsum(mean(L.(c{1}).y2 - L.base.y2, 2)) / amt, ...
            'pr', cumsum(mean(L.(c{1}).pr - L.base.pr, 2)) / amt);
    end
    kend = numel(t) - 1;
    fprintf('\nresponse ledger at end (cumulative diff / injected, expect sum -> -1 if fully corrected):\n');
    fprintf('        %8s %8s %8s\n', 'y1', 'y2', 'predict');
    fprintf('  +arm  %+8.3f %+8.3f %+8.3f\n', resp.plus.y1(kend), resp.plus.y2(kend), resp.plus.pr(kend));
    fprintf('  -arm  %+8.3f %+8.3f %+8.3f\n', resp.minus.y1(kend), resp.minus.y2(kend), resp.minus.pr(kend));

    % honesty (spread / sqrt P), three arms
    hon = @(F) std(F.a_bar_hat_out(kk,:) - F.a_true_out(kk,:), 0, 2) ./ mean(F.P_a_out(kk,:), 2);
    hb = hon(B); hp = hon(Pp); hm = hon(Mm);
    fprintf('\nhonesty spread/sqrtP, end hold:  base %.2f  +arm %.2f  -arm %.2f\n', ...
            mean(hb(hold_end)), mean(hp(hold_end)), mean(hm(hold_end)));

    out = struct('t', t, 'kj', kj, 'rp', rp, 'rm', rm, 'ab', ab, 'ap', ap, 'am', am, 'at', at, ...
                 'K1b', B.K_a_y1_out(kk,:), 'K1p', Pp.K_a_y1_out(kk,:), 'K1m', Mm.K_a_y1_out(kk,:), ...
                 'resp', resp, 'hb', hb, 'hp', hp, 'hm', hm, 'h_d', S.h_d(kk), 'ns', ns, 'frac', frac);

    local_fig_time(out, fullfile(od, 'inject_response_time.png'));
    local_fig_gain(out, fullfile(od, 'inject_response_gain.png'));
    local_fig_map(out,  fullfile(od, 'inject_response_map.png'));
    fprintf('\nfigures -> %s\n', od);
end

function m = local_merge(a, b)
    m = a; fn = fieldnames(b);
    for q = 1:numel(fn); m.(fn{q}) = b.(fn{q}); end
end

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end

% =======================================================================
function local_fig_time(o, fpath)
    FS = 17; AXLW = 2.0;
    RED=[0.85 0.1 0.1]; BLU=[0 0.2 0.9]; ORG=[0.9 0.5 0]; GRN=[0.1 0.6 0.2]; GRY=[0.5 0.5 0.5];
    f = figure('Position',[40 40 1300 1050],'Color','w','Visible','off');
    tl = tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    tj = o.t(o.kj);

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1,o.t,mean(o.ab,2),'-','Color',BLU,'LineWidth',2.2,'DisplayName','a_{hat} baseline');
    plot(a1,o.t,mean(o.ap,2),'-','Color',ORG,'LineWidth',2.2,'DisplayName',sprintf('a_{hat} +%.0f%%',100*o.frac));
    plot(a1,o.t,mean(o.am,2),'-','Color',GRN,'LineWidth',2.2,'DisplayName',sprintf('a_{hat} -%.0f%%',100*o.frac));
    plot(a1,o.t,mean(o.at,2),'-','Color',RED,'LineWidth',2.2,'DisplayName','a_{true}');
    xline(a1,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    ylabel(a1,'a / a_o');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    yline(a2,1,':','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    local_band(a2,o.t,o.rp,ORG,sprintf('r(t)  +%.0f%% arm',100*o.frac));
    local_band(a2,o.t,o.rm,GRN,sprintf('r(t)  -%.0f%% arm',100*o.frac));
    xline(a2,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xlim(a2,[tj-0.5, o.t(end)]); ylim(a2,[-0.3 2.8]);
    ylabel(a2,'(a_{hat,inj} - a_{hat,base}) / injected');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a3 = nexttile(tl,3); hold(a3,'on'); box(a3,'on');
    yline(a3,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    plot(a3,o.t,100*(mean(o.ab,2)./mean(o.at,2)-1),'-','Color',BLU,'LineWidth',2.2,'DisplayName','error baseline [%]');
    plot(a3,o.t,100*(mean(o.ap,2)./mean(o.at,2)-1),'-','Color',ORG,'LineWidth',2.2,'DisplayName','error +arm [%]');
    plot(a3,o.t,100*(mean(o.am,2)./mean(o.at,2)-1),'-','Color',GRN,'LineWidth',2.2,'DisplayName','error -arm [%]');
    xline(a3,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xlabel(a3,'t [s]'); ylabel(a3,'[%]');
    legend(a3,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a3,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end

function local_band(a, t, R, col, nm)
    m = mean(R,2); s = std(R,0,2)/sqrt(size(R,2));
    fill(a,[t;flipud(t)],[m+s;flipud(m-s)],col,'FaceAlpha',0.2,'EdgeColor','none','HandleVisibility','off');
    plot(a,t,m,'-','Color',col,'LineWidth',2.4,'DisplayName',nm);
end

% =======================================================================
function local_fig_gain(o, fpath)
    FS = 17; AXLW = 2.0;
    BLU=[0 0.2 0.9]; ORG=[0.9 0.5 0]; GRN=[0.1 0.6 0.2]; GRY=[0.5 0.5 0.5];
    f = figure('Position',[40 40 1300 800],'Color','w','Visible','off');
    tl = tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
    tj = o.t(o.kj);

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    plot(a1,o.t,abs(mean(o.K1b,2)),'-','Color',BLU,'LineWidth',2.2,'DisplayName','|K_1(4)| baseline');
    plot(a1,o.t,abs(mean(o.K1p,2)),'-','Color',ORG,'LineWidth',2.2,'DisplayName','|K_1(4)| +arm');
    plot(a1,o.t,abs(mean(o.K1m,2)),'-','Color',GRN,'LineWidth',2.2,'DisplayName','|K_1(4)| -arm');
    xline(a1,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    set(a1,'YScale','log'); ylabel(a1,'|K_1(4)|  y_1 \rightarrow a');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'XTickLabel',[]);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    yline(a2,-1,':','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    tt = o.t(2:end);
    plot(a2,tt,o.resp.plus.y1,'-','Color',ORG,'LineWidth',2.4,'DisplayName','+arm: \Sigma K_1 innov_1');
    plot(a2,tt,o.resp.plus.y2,'--','Color',ORG,'LineWidth',2.0,'DisplayName','+arm: \Sigma K_2 innov_2');
    plot(a2,tt,o.resp.plus.pr,':','Color',ORG,'LineWidth',2.0,'DisplayName','+arm: \Sigma predict');
    plot(a2,tt,o.resp.minus.y1,'-','Color',GRN,'LineWidth',2.4,'DisplayName','-arm: \Sigma K_1 innov_1');
    plot(a2,tt,o.resp.minus.y2,'--','Color',GRN,'LineWidth',2.0,'DisplayName','-arm: \Sigma K_2 innov_2');
    plot(a2,tt,o.resp.minus.pr,':','Color',GRN,'LineWidth',2.0,'DisplayName','-arm: \Sigma predict');
    xline(a2,tj,':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    xlim(a2,[tj-0.5, o.t(end)]); ylim(a2,[-6 4]);
    xlabel(a2,'t [s]'); ylabel(a2,'cumulative (arm - base) / injected');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',11,'NumColumns',3);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end

% =======================================================================
function local_fig_map(o, fpath)
    FS = 17; AXLW = 2.0;
    BLU=[0 0.2 0.9]; ORG=[0.9 0.5 0]; GRN=[0.1 0.6 0.2]; GRY=[0.5 0.5 0.5];
    f = figure('Position',[40 40 1400 560],'Color','w','Visible','off');
    tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');

    a1 = nexttile(tl,1); hold(a1,'on'); box(a1,'on');
    yline(a1,0,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    yline(a1,1,':','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    m = o.kj:numel(o.t);
    plot(a1,o.h_d(m),mean(o.rp(m,:),2),'-','Color',ORG,'LineWidth',2.4,'DisplayName','r  +arm');
    plot(a1,o.h_d(m),mean(o.rm(m,:),2),'-','Color',GRN,'LineWidth',2.4,'DisplayName','r  -arm');
    set(a1,'XScale','log','XDir','reverse'); ylim(a1,[-0.3 2.8]);
    xlabel(a1,'w_{bar,d}  (descending \rightarrow)'); ylabel(a1,'r');
    legend(a1,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a1,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);

    a2 = nexttile(tl,2); hold(a2,'on'); box(a2,'on');
    yline(a2,1,'-','Color',GRY,'LineWidth',1.5,'HandleVisibility','off');
    plot(a2,o.t,o.hb,'-','Color',BLU,'LineWidth',2.2,'DisplayName','spread/\surdP_{44} baseline');
    plot(a2,o.t,o.hp,'-','Color',ORG,'LineWidth',2.2,'DisplayName','+arm');
    plot(a2,o.t,o.hm,'-','Color',GRN,'LineWidth',2.2,'DisplayName','-arm');
    xline(a2,o.t(o.kj),':','Color',GRY,'LineWidth',1.8,'HandleVisibility','off');
    set(a2,'YScale','log'); ylim(a2,[0.1 10]);
    xlabel(a2,'t [s]'); ylabel(a2,'[-]');
    legend(a2,'Location','northoutside','Orientation','horizontal','FontSize',12);
    set(a2,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW);
    exportgraphics(f,fpath,'Resolution',150); close(f);
end
