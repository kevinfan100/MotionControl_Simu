% STATUS: ACTIVE (Layer-0 instrument) -- targets
%   model/controller/motion_control_law_formC_b.m via
%   test_script/integration/run_formC_b.m. Made on commit 10e51db (test/motion-test).
%L0_FLAG_LIVENESS  Flip every knob of the formC_b controller / driver one at a
%   time and ask the ONLY question a wiring audit can answer: does the output
%   change at all?
%
%   Rule (stacked-fix-audit.md B6): bit-identical when flipped = flag not
%   wired. The verdict is on max|d a_bar_hat| over the full run and all three
%   axes (plus b_hat and the final a_bar_hat), so a knob that only touches x/y
%   still registers. A knob whose consumption is CONDITIONAL on another knob
%   (ap_ewma_a needs ap_src = 'ewma'; lambda_f_b_floor needs
%   lambda_f_b_alpha > 0; law_b in the driver needs ap_law_bias finite) is
%   run twice: alone against the baseline (expected inert by design) and
%   stacked against its enabling arm (must be LIVE there).
%
%   NEGATIVE CONTROL first: the baseline re-run with an identical (empty)
%   override struct must be bit-identical. If it is not, the whole table is
%   uninterpretable and the script says so.
%
%   Baseline = canonical deep band, seed 7, arm 'best', driver defaults
%   (the multi-seed form with opts.seeds = 7, so the driver's own envelope
%   priors / par-law package / arm flags are all in the ctrl_const exactly as
%   in production; the single-seed compatibility form skips them).
%
%   Output: console table + <tmp>/flag_liveness.txt. No figure.
%   Runs ~60 arms x ~7 s. Nothing under model/ or test_script/integration/ is
%   touched; test_results/run_formC_b_*.mat is rewritten by the driver on every
%   call (gitignored, as always).

clear; clc;
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
root = pwd;
l0_local_addpaths(root);

OUT_DIR  = '/Users/kevin/.claude/jobs/8581427c/tmp/l0b';
if ~exist(OUT_DIR, 'dir'); mkdir(OUT_DIR); end
OUT_FILE = fullfile(OUT_DIR, 'flag_liveness.txt');

SEED = 7;            % canonical first house seed
AX_Z = 3;

% ------------------------------------------------------------------
% Arm list. Each row: name | level | override struct | reference arm name
% ('' = baseline) | expected verdict (from reading the code) | note.
% level 'cc' = opts.ctrl_const_override field (merged LAST in the driver);
% level 'drv' = driver opts field; level 'cfg' = opts.config_override field.
% ------------------------------------------------------------------
arms = {};
mk = @(nm, lvl, ov, ref, expct, note) ...
    struct('name', nm, 'level', lvl, 'ov', ov, 'ref', ref, 'expect', expct, 'note', note);

% --- negative control -------------------------------------------------
arms{end+1} = mk('NEG CONTROL (empty override)', 'cc', struct(), '', 'DEAD', 'must be 0 or table is void');

% --- controller-level knobs (ctrl_const_override) ---------------------
arms{end+1} = mk('law_b_formC', 'cc', struct('law_b_formC', 2.0), '', 'DEAD', 'read into persistent, never consumed (known)');
arms{end+1} = mk('r22_delay_scale', 'cc', struct('r22_delay_scale', 1e6), '', 'LIVE', 'positive control');
arms{end+1} = mk('a_bar_floor', 'cc', struct('a_bar_floor', 0.5), '', 'LIVE', 'trough a_bar ~0.09 < 0.5, must bind');
arms{end+1} = mk('a_bar_ceil', 'cc', struct('a_bar_ceil', 0.5), '', 'LIVE', 'far-field a_bar ~0.95 > 0.5, must bind');
arms{end+1} = mk('b_floor', 'cc', struct('b_floor', 0.95), '', 'LIVE', 'seed 8/9 < 0.95, binds at k=1');
arms{end+1} = mk('b_ceil (lead value)', 'cc', struct('b_ceil', 0.90), '', '?', 'seed 8/9 < 0.90; binds only if b_hat rises past 0.90');
arms{end+1} = mk('b_ceil (binding)', 'cc', struct('b_ceil', 0.85), '', 'LIVE', 'seed 8/9 > 0.85, binds at k=1');
arms{end+1} = mk('Pf_b_std x10', 'cc', 'PF_B_X10', '', 'LIVE', 'value filled at run time from the baseline ctrl_const');
arms{end+1} = mk('Pf_a_floor x10', 'cc', 'PF_A_X10', '', 'LIVE', 'value filled at run time');
arms{end+1} = mk('Pf_w0_std x10', 'cc', struct('Pf_w0_std', 1.11), '', 'LIVE', '');
arms{end+1} = mk('Pf_da_std', 'cc', struct('Pf_da_std', 1e-3), '', 'DEAD', 'controller reads it into Pf_da_std_unused');
arms{end+1} = mk('b_init', 'cc', struct('b_init', 1.0), '', 'LIVE', '');
arms{end+1} = mk('lock_b', 'cc', struct('lock_b', true), '', 'LIVE', 'arm best -> slot 5 locked');
arms{end+1} = mk('ws0_perp', 'cc', struct('ws0_perp', 1.1), '', 'LIVE', 'nominal wall w0 = ws0_perp - 1');
arms{end+1} = mk('ws_margin', 'cc', struct('ws_margin', 0.5), '', '?', 'gap_floor; binds only if w_bar - w0 < 0.5');
arms{end+1} = mk('w0_par', 'cc', struct('w0_par', 0.2), '', 'LIVE', 'x/y only');
arms{end+1} = mk('Pf_a_floor_par', 'cc', struct('Pf_a_floor_par', 0.5), '', 'LIVE', 'x/y only');
arms{end+1} = mk('ap_src=pred', 'cc', struct('ap_src', 'pred'), '', 'LIVE', '');
arms{end+1} = mk('ap_src=ewma', 'cc', struct('ap_src', 'ewma'), '', 'LIVE', '');
arms{end+1} = mk('ap_src=cmd', 'cc', struct('ap_src', 'cmd'), '', 'LIVE', '');
arms{end+1} = mk('ap_src=act', 'cc', struct('ap_src', 'act'), '', 'LIVE', '');
arms{end+1} = mk('ap_ewma_a alone', 'cc', struct('ap_ewma_a', 0.5), '', 'DEAD', 'only consumed under ap_src=ewma');
arms{end+1} = mk('ap_ewma_a | ap_src=ewma', 'cc', struct('ap_src', 'ewma', 'ap_ewma_a', 0.5), 'ap_src=ewma', 'LIVE', 'stacked vs the ewma arm');
arms{end+1} = mk('lambda_f', 'cc', struct('lambda_f', 0.98), '', 'LIVE', 'Menq (4.15)');
arms{end+1} = mk('lambda_f_b', 'cc', struct('lambda_f_b', 0.98), '', 'LIVE', 'slot-5 congruence');
arms{end+1} = mk('lambda_f_b_alpha', 'cc', struct('lambda_f_b_alpha', 1.0), '', 'LIVE', 'adaptive slot-5 forgetting');
arms{end+1} = mk('lambda_f_b_floor alone', 'cc', struct('lambda_f_b_floor', 0.5), '', 'DEAD', 'only consumed when alpha > 0');
arms{end+1} = mk('lambda_f_b_floor | alpha', 'cc', struct('lambda_f_b_alpha', 1.0, 'lambda_f_b_floor', 0.5), 'lambda_f_b_alpha', '?', 'binds only if exp(-alpha*excess) < 0.99 somewhere');
arms{end+1} = mk('Q_theta_floor', 'cc', struct('Q_theta_floor', 1e-6), '', 'LIVE', '');
arms{end+1} = mk('amlpf_var_factor', 'cc', struct('amlpf_var_factor', 0.1), '', 'LIVE', 'R2 scale');
arms{end+1} = mk('y2_whiten=false', 'cc', struct('y2_whiten', false), '', 'LIVE', '');
arms{end+1} = mk('fe_row4_full=false', 'cc', struct('fe_row4_full', false), '', 'LIVE', '');
arms{end+1} = mk('use_fdet=false', 'cc', struct('use_fdet', false), '', 'LIVE', '');
arms{end+1} = mk('y2_off=true', 'cc', struct('y2_off', true), '', 'LIVE', '');
arms{end+1} = mk('y2_echo_corr=false', 'cc', struct('y2_echo_corr', false), '', 'LIVE', '');
arms{end+1} = mk('ma2_aug=false', 'cc', struct('ma2_aug', false), '', 'LIVE', '9 -> 7 states');
arms{end+1} = mk('q33_dc_match | ma2_aug=false', 'cc', struct('ma2_aug', false, 'q33_dc_match', true), 'ma2_aug=false', 'LIVE', 'mutually exclusive with ma2_aug');
arms{end+1} = mk('y1_gain_off=true', 'cc', struct('y1_gain_off', true), '', 'LIVE', '');
arms{end+1} = mk('t2_pure_prop=true', 'cc', struct('t2_pure_prop', true), '', 'LIVE', '');
arms{end+1} = mk('obs_dump=true', 'cc', struct('obs_dump', true), '', 'DEAD', 'diagnostic capture, documented bit-identical');
arms{end+1} = mk('h_bar_safe (cc)', 'cc', struct('h_bar_safe', 3.0), '', 'LIVE', 'y2 gate; band is w_bar < 3.32');
arms{end+1} = mk('h_bar_safe (cfg)', 'cfg', struct('h_bar_safe', 3.0), '', 'LIVE', 'same knob through config_override');
arms{end+1} = mk('par_law=false (cc)', 'cc', struct('par_law', false), '', 'LIVE', 'x/y only');

% --- driver-level opts ------------------------------------------------
arms{end+1} = mk('drv y2_on=false', 'drv', struct('y2_on', false), '', 'LIVE', '');
arms{end+1} = mk('drv par_law=false', 'drv', struct('par_law', false), '', 'LIVE', 'x/y only');
arms{end+1} = mk('drv ap_src=pred', 'drv', struct('ap_src', 'pred'), '', 'LIVE', '');
arms{end+1} = mk('drv arm=bmid', 'drv', struct('arm', 'bmid'), '', 'LIVE', '');
arms{end+1} = mk('drv arm=b1', 'drv', struct('arm', 'b1'), '', 'LIVE', '');
arms{end+1} = mk('drv da_known=true', 'drv', struct('da_known', true), '', 'LIVE', '');
arms{end+1} = mk('drv ap_known=true', 'drv', struct('ap_known', true), '', 'LIVE', '');
arms{end+1} = mk('drv ap_law_bias=0.05', 'drv', struct('ap_law_bias', 0.05), '', 'LIVE', 'implies ap_known');
arms{end+1} = mk('drv law_b=2 alone', 'drv', struct('law_b', 2), '', 'DEAD', 'feeds law_b_formC (dead) + ap_law path (off)');
arms{end+1} = mk('drv law_b=2 | ap_law_bias', 'drv', struct('ap_law_bias', 0.05, 'law_b', 2), 'drv ap_law_bias=0.05', 'LIVE', 'stacked vs the ap_law_bias arm');
arms{end+1} = mk('drv lambda_f=0.98', 'drv', struct('lambda_f', 0.98), '', 'LIVE', '');
arms{end+1} = mk('drv ap_ewma_a=0.5 alone', 'drv', struct('ap_ewma_a', 0.5), '', 'DEAD', 'ap_src=post');
arms{end+1} = mk('drv Pf_da_std=1e-3', 'drv', struct('Pf_da_std', 1e-3), '', 'DEAD', 'driver prints it, never puts it in ov');
arms{end+1} = mk('drv Pf_w0_std=1.0', 'drv', struct('Pf_w0_std', 1.0), '', 'LIVE', '');
arms{end+1} = mk('drv floor_from_envelope', 'drv', struct('floor_from_envelope', true), '', 'LIVE', 'Pf_a_floor env sup vs seed-local');
arms{end+1} = mk('drv a_cov_scale=2', 'drv', struct('a_cov_scale', 2), '', 'LIVE', '');
arms{end+1} = mk('drv plant_law_b=1.0', 'drv', struct('plant_law_b', 1.0), '', 'LIVE', 'plant-side truth changes');
arms{end+1} = mk('drv ws_inject=0.1', 'drv', struct('ws_inject', 0.1), '', 'LIVE', 'plant-side wall shift');
arms{end+1} = mk('drv a_ctrl_override=true', 'drv', struct('a_ctrl_override', 'true'), '', 'LIVE', 'controller runs the true gain');

n_arms = numel(arms);

% ------------------------------------------------------------------
% Baseline
% ------------------------------------------------------------------
fprintf('=== L0 flag liveness: baseline canonical deep, seed %d, arm best ===\n', SEED);
[base, t_base] = l0_local_run(struct(), 'cc', SEED);
fprintf('baseline run time %.1f s; N = %d steps; a_bar_hat(end,z) = %.17g\n', ...
        t_base, size(base.a_bar_hat_out, 1), base.a_bar_hat_out(end, AX_Z));
cc_base = base.ctrl_const;
fprintf('baseline ctrl_const: Pf_b_std = %.6g  Pf_a_floor = %.6g  b_init = %.6g  lock_b = %d  par_law = %d\n', ...
        cc_base.Pf_b_std, cc_base.Pf_a_floor, cc_base.b_init, cc_base.lock_b, cc_base.par_law);

% ------------------------------------------------------------------
% Arms
% ------------------------------------------------------------------
res = struct('name', {}, 'level', {}, 'value', {}, 'ref', {}, 'd_abar_all', {}, ...
             'd_abar_z', {}, 'd_bhat', {}, 'd_abar_end', {}, 'verdict', {}, ...
             'expect', {}, 't_run', {}, 'note', {});
runs = containers.Map();      % name -> simOut (for stacked references)
runs('') = base;
for ia = 1:n_arms
    A = arms{ia};
    ov = A.ov;
    if ischar(ov)      % values that depend on the baseline ctrl_const
        switch ov
            case 'PF_B_X10';  ov = struct('Pf_b_std',   10 * cc_base.Pf_b_std);
            case 'PF_A_X10';  ov = struct('Pf_a_floor', 10 * cc_base.Pf_a_floor);
        end
    end
    vstr = l0_local_valstr(ov);
    err = '';
    try
        [s, t_run] = l0_local_run(ov, A.level, SEED);
        runs(A.name) = s;
        if isKey(runs, A.ref); r = runs(A.ref); else; r = base; end
        d_all = l0_local_maxdiff(s.a_bar_hat_out, r.a_bar_hat_out);
        d_z   = l0_local_maxdiff(s.a_bar_hat_out(:, AX_Z), r.a_bar_hat_out(:, AX_Z));
        d_b   = l0_local_maxdiff(s.b_hat_out, r.b_hat_out);
        d_end = l0_local_maxdiff(s.a_bar_hat_out(end, :), r.a_bar_hat_out(end, :));
        if max([d_all, d_b]) > 0; v = 'LIVE'; else; v = 'DEAD'; end
    catch ME
        d_all = NaN; d_z = NaN; d_b = NaN; d_end = NaN; t_run = NaN;
        v = 'ERROR'; err = strrep(ME.message, newline, ' ');
    end
    res(end+1) = struct('name', A.name, 'level', A.level, 'value', vstr, 'ref', A.ref, ...
                        'd_abar_all', d_all, 'd_abar_z', d_z, 'd_bhat', d_b, ...
                        'd_abar_end', d_end, 'verdict', v, 'expect', A.expect, ...
                        't_run', t_run, 'note', [A.note ' ' err]); %#ok<SAGROW>
    fprintf('[%2d/%2d] %-32s %-4s %-38s d_abar %.3e d_b %.3e -> %s (exp %s) %.1fs\n', ...
            ia, n_arms, A.name, A.level, vstr, d_all, d_b, v, A.expect, t_run);
end

% ------------------------------------------------------------------
% Table (console + file)
% ------------------------------------------------------------------
fid = fopen(OUT_FILE, 'w');
outs = [1, fid];
neg = res(1);
for o = outs
    fprintf(o, 'L0 flag liveness -- motion_control_law_formC_b via run_formC_b, commit 10e51db, canonical deep, seed %d, arm best\n', SEED);
    fprintf(o, 'baseline: N = %d, run time %.1f s, a_bar_hat(end,z) = %.17g, b_hat(end,z) = %.17g\n', ...
            size(base.a_bar_hat_out, 1), t_base, base.a_bar_hat_out(end, AX_Z), base.b_hat_out(end, AX_Z));
    if neg.d_abar_all == 0 && neg.d_bhat == 0
        fprintf(o, 'NEGATIVE CONTROL: PASS (bit-identical rerun, max diff exactly 0)\n');
    else
        fprintf(o, 'NEGATIVE CONTROL: FAIL (d_abar %.3e, d_bhat %.3e) -- TABLE BELOW IS UNINTERPRETABLE\n', ...
                neg.d_abar_all, neg.d_bhat);
    end
    fprintf(o, 'verdict: LIVE if max|d a_bar_hat| (all axes, full run) > 0 or max|d b_hat| > 0; DEAD if both exactly 0.\n');
    fprintf(o, 'ref = what the arm is compared against ("" = baseline).\n\n');
    fprintf(o, '%-30s | %-4s | %-40s | %-24s | %11s | %13s | %11s | %12s | %-6s | %-5s | %5s | %s\n', ...
            'knob', 'lvl', 'value used', 'ref', 'max|d abar|', 'max|d abar_z|', 'max|d bhat|', '|d abar end|', 'verdict', 'exp', 't[s]', 'note');
    fprintf(o, '%s\n', repmat('-', 1, 200));
    for i = 1:numel(res)
        r = res(i);
        fprintf(o, '%-30s | %-4s | %-40s | %-24s | %11.3e | %13.3e | %11.3e | %12.3e | %-6s | %-5s | %5.1f | %s\n', ...
                r.name, r.level, r.value, r.ref, r.d_abar_all, r.d_abar_z, r.d_bhat, r.d_abar_end, ...
                r.verdict, r.expect, r.t_run, strtrim(r.note));
    end
    fprintf(o, '\n');
    mism = find(~strcmp({res.verdict}, {res.expect}) & ~strcmp({res.expect}, '?'));
    if isempty(mism)
        fprintf(o, 'EXPECTATION MISMATCHES: none\n');
    else
        fprintf(o, 'EXPECTATION MISMATCHES (%d):\n', numel(mism));
        for i = mism
            fprintf(o, '  %-30s got %s expected %s\n', res(i).name, res(i).verdict, res(i).expect);
        end
    end
    dead = find(strcmp({res.verdict}, 'DEAD'));
    fprintf(o, 'DEAD knobs (%d): %s\n', numel(dead), strjoin({res(dead).name}, '; '));
    errs = find(strcmp({res.verdict}, 'ERROR'));
    if ~isempty(errs)
        fprintf(o, 'ERROR arms (%d): %s\n', numel(errs), strjoin({res(errs).name}, '; '));
    end
    fprintf(o, 'mean run time per arm: %.1f s\n', mean([res.t_run], 'omitnan'));
end
fclose(fid);
fprintf('written: %s\n', OUT_FILE);


%% ================= local helpers =================
function [s, t_run] = l0_local_run(ov, level, seed)
%L0_LOCAL_RUN  One seed through the production multi-seed driver form, so the
%   arm flags / envelope priors / par-law package are built exactly as in
%   production. Console output is captured (the driver prints a full report).
    opts = struct('seeds', seed);
    switch level
        case 'cc';  opts.ctrl_const_override = ov;
        case 'cfg'; opts.config_override = ov;
        case 'drv'
            fn = fieldnames(ov);
            for i = 1:numel(fn); opts.(fn{i}) = ov.(fn{i}); end
    end
    t0 = tic;
    evalc('out = run_formC_b(opts);');
    t_run = toc(t0);
    s = out.runs{1};
end

function d = l0_local_maxdiff(a, b)
%L0_LOCAL_MAXDIFF  max|a-b| with NaN bookkeeping: both-NaN counts as equal,
%   one-sided NaN or a size mismatch counts as Inf (definitely different).
    if ~isequal(size(a), size(b)); d = Inf; return; end
    na = isnan(a); nb = isnan(b);
    if any(na(:) ~= nb(:)); d = Inf; return; end
    df = abs(a - b);
    df(na) = 0;
    d = max(df(:));
    if isempty(d); d = 0; end
end

function s = l0_local_valstr(ov)
    fn = fieldnames(ov);
    if isempty(fn); s = '(none)'; return; end
    parts = cell(1, numel(fn));
    for i = 1:numel(fn)
        v = ov.(fn{i});
        if ischar(v);          vs = ['''' v ''''];
        elseif islogical(v);   vs = mat2str(v);
        else;                  vs = sprintf('%.6g', v);
        end
        parts{i} = sprintf('%s=%s', fn{i}, vs);
    end
    s = strjoin(parts, ', ');
end

function l0_local_addpaths(root)
%L0_LOCAL_ADDPATHS  Active model dirs + integration drivers; archive/ excluded
%   so no superseded function can shadow production.
    p = strsplit(genpath(fullfile(root, 'model')), pathsep);
    p = p(~cellfun('isempty', p));
    p = p(~contains(p, [filesep 'archive']));
    addpath(p{:});
    addpath(fullfile(root, 'test_script', 'integration'));
    addpath(fullfile(root, 'test_script', 'build_helpers'));
end
