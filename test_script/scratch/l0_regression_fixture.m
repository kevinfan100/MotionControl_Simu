% STATUS: ACTIVE (Layer-0 instrument) -- bit-identical regression fixture for
%   model/controller/motion_control_law_formC_b.m via
%   test_script/integration/run_formC_b.m.
%   FIXTURE MADE ON: commit 10e51db445af37f35e65219ff1fd49af78ec5a56
%   (branch test/motion-test, 2026-08-25). Re-make whenever a production
%   change is INTENDED; every other controller edit must leave 'check' at 0.
%L0_REGRESSION_FIXTURE  Freeze the production formC_b output on the canonical
%   deep scenario (seeds 7 and 11, driver defaults, arm 'best') and compare a
%   rerun against it bit for bit.
%
%   MODE = 'make'   run both seeds, save test_results/l0_fixture/fixture_formC_b.mat
%                   (a_bar_hat_out, b_hat_out, R2_out, a_hat_out, P_a_out per
%                   seed + the ctrl_const and commit), print the hash lines
%   MODE = 'check'  rerun both seeds, report max|diff| per channel; PASS only
%                   when every channel is EXACTLY 0 (roundoff is a diff)
%
%   Why this exists (stacked-fix-audit.md B7): the controller has no unit
%   test, so "did my edit change anything I did not intend" had no answer.
%   The rule is the same as the liveness one, inverted: an edit that should
%   be inert must be bit-identical here; an edit that is meant to change
%   behaviour must be followed by a deliberate 'make' with a new commit line.
%
%   The hash lines are printed with %.17g so they can be pasted into a ledger
%   and compared without loading the .mat. FNV-1a 64-bit over the raw bytes
%   of each series is also printed (pure MATLAB, no toolbox).
%
%   Usage: set MODE below (or define it in the workspace before running) and
%   run(<this file>). test_results/ is gitignored, so the fixture is a local
%   artefact: it travels with the machine, not the repo, by design.

if ~exist('MODE', 'var'); MODE = 'check'; end
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
root = pwd;
l0fx_addpaths(root);

SEEDS     = [7 11];
CHANNELS  = {'a_bar_hat_out', 'b_hat_out', 'R2_out', 'a_hat_out', 'P_a_out'};
FIX_DIR   = fullfile(root, 'test_results', 'l0_fixture');
FIX_FILE  = fullfile(FIX_DIR, 'fixture_formC_b.mat');
COMMIT    = '1a705991002159f580cadb9550425f91b4fcc93b';   % IF(1) colour factor fix, 2026-08-27

assert(any(strcmp(MODE, {'make', 'check'})), 'l0_regression_fixture:mode', ...
       'MODE must be ''make'' or ''check''.');

% ---- run both seeds through the production driver form -----------------
fprintf('=== L0 regression fixture [%s] -- canonical deep, seeds %s, arm best ===\n', ...
        MODE, mat2str(SEEDS));
cur = struct();
for i = 1:numel(SEEDS)
    t0 = tic;
    evalc('out = run_formC_b(struct(''seeds'', SEEDS(i)));');
    t_run = toc(t0);
    s = out.runs{1};
    key = sprintf('seed%d', SEEDS(i));
    cur.(key) = struct();
    for c = 1:numel(CHANNELS)
        cur.(key).(CHANNELS{c}) = s.(CHANNELS{c});
    end
    cur.(key).ctrl_const = s.ctrl_const;
    cur.(key).t_run = t_run;
    fprintf('seed %3d: N = %d, run time %.1f s\n', SEEDS(i), size(s.a_bar_hat_out, 1), t_run);
    for c = 1:numel(CHANNELS)
        v = s.(CHANNELS{c});
        fprintf('  HASH seed %d %-14s sum=%.17g first=%.17g last=%.17g fnv1a64=%s\n', ...
                SEEDS(i), CHANNELS{c}, sum(v(:)), v(1), v(end), l0fx_fnv1a64(v));
    end
end

switch MODE
    case 'make'
        if ~exist(FIX_DIR, 'dir'); mkdir(FIX_DIR); end
        fixture = struct('commit', COMMIT, 'made', datestr(now, 'yyyy-mm-dd HH:MM:SS'), ...
                         'seeds', SEEDS, 'channels', {CHANNELS}, 'data', cur, ...
                         'matlab', version); %#ok<TNOW1,DATST>
        save(FIX_FILE, 'fixture');
        fprintf('fixture written: %s (commit %s)\n', FIX_FILE, COMMIT(1:7));

    case 'check'
        assert(exist(FIX_FILE, 'file') == 2, 'l0_regression_fixture:noFixture', ...
               'no fixture at %s -- run with MODE = ''make'' first.', FIX_FILE);
        L = load(FIX_FILE);
        ref = L.fixture.data;
        fprintf('fixture from commit %s (%s)\n', L.fixture.commit(1:7), L.fixture.made);
        all_zero = true;
        fprintf('%-8s | %-14s | %12s | %s\n', 'seed', 'channel', 'max|diff|', 'verdict');
        for i = 1:numel(SEEDS)
            key = sprintf('seed%d', SEEDS(i));
            for c = 1:numel(CHANNELS)
                a = cur.(key).(CHANNELS{c});
                b = ref.(key).(CHANNELS{c});
                d = l0fx_maxdiff(a, b);
                if d == 0; v = 'identical'; else; v = 'DIFF'; all_zero = false; end
                fprintf('%-8d | %-14s | %12.3e | %s\n', SEEDS(i), CHANNELS{c}, d, v);
            end
        end
        if all_zero
            fprintf('REGRESSION CHECK: PASS -- bit-identical to fixture (commit %s)\n', L.fixture.commit(1:7));
        else
            fprintf('REGRESSION CHECK: FAIL -- output differs from fixture (commit %s)\n', L.fixture.commit(1:7));
        end
end


%% ================= local helpers =================
function d = l0fx_maxdiff(a, b)
    if ~isequal(size(a), size(b)); d = Inf; return; end
    na = isnan(a); nb = isnan(b);
    if any(na(:) ~= nb(:)); d = Inf; return; end
    df = abs(a - b); df(na) = 0;
    d = max(df(:)); if isempty(d); d = 0; end
end

function h = l0fx_fnv1a64(v)
%L0FX_FNV1A64  FNV-1a 64-bit over the raw little-endian double bytes of v(:).
%   Done in uint64 with explicit mod-2^64 multiplication split into 32-bit
%   halves, so no toolbox and no overflow saturation.
    bytes = typecast(double(v(:)), 'uint8');
    PRIME_LO = uint64(0x1B3);          % 0x100000001B3 = 2^40 + 0x1B3
    hash  = uint64(0xCBF29CE484222325);
    for i = 1:numel(bytes)
        hash = bitxor(hash, uint64(bytes(i)));
        % hash * 0x100000001B3 mod 2^64 = (hash << 40) + hash * 0x1B3 (mod 2^64)
        hi = bitshift(hash, 40);                          % wraps mod 2^64
        lo = l0fx_mul64(hash, PRIME_LO);
        hash = l0fx_add64(hi, lo);
    end
    h = sprintf('%016x', hash);
end

function r = l0fx_add64(a, b)
%   (a + b) mod 2^64 without saturation.
    MASK32 = uint64(0xFFFFFFFF);
    lo = bitand(a, MASK32) + bitand(b, MASK32);
    hi = bitshift(a, -32) + bitshift(b, -32) + bitshift(lo, -32);
    r  = bitor(bitshift(bitand(hi, MASK32), 32), bitand(lo, MASK32));
end

function r = l0fx_mul64(a, b)
%   (a * b) mod 2^64 with b < 2^16, via 16-bit limbs of a.
    r = uint64(0);
    for k = 0:3
        limb = bitand(bitshift(a, -16*k), uint64(0xFFFF));
        r = l0fx_add64(r, bitshift(limb * b, 16*k));      % limb*b < 2^32, no overflow
    end
end

function l0fx_addpaths(root)
    p = strsplit(genpath(fullfile(root, 'model')), pathsep);
    p = p(~cellfun('isempty', p));
    p = p(~contains(p, [filesep 'archive']));
    addpath(p{:});
    addpath(fullfile(root, 'test_script', 'integration'));
    addpath(fullfile(root, 'test_script', 'build_helpers'));
end
