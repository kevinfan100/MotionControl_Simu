% PURPOSE (2026-09-21): the one-shot closed-loop test of the advisor's power law a' = (1 - c)/h^p under the wall-test protocol
%   (.claude/rules/wall-test-protocol.md): version A (flag plaw_c_state: slot 7 = c, p FIXED at the plane band fit 1.9666,
%   gain = law at the estimator's own height, no a slot), estimator init at the PLANE (c = 0, prior 0.5 two-sided), wall given
%   only to the plant.
%     plants    plane   Brenner polynomial (production plant)
%               cell    plane polynomial slid down 1.05 R (cell_cperp_delta.m)
%               sphere  Meng Fig. 11 sphere, data + [1/1] tail + near-contact patch (sphere_cperp_fig11.m)
%     arms      PI  plane init c = 0 (the protocol arm)
%               T   seed = the wall's own band-fit c (fit_advisor_law_walls.m; sanity / reference only, not a verdict arm)
%   Reference line for c: c_eff(h_d) = 1 - h_d^p a'_plant(h_d) at the commanded height (one height pins one c, C57).
%   Controller flags: plaw_c_state + gain_algebraic + amodel_fe + amodel_h, pure form (ma2_aug / nw_mcorr / pred_mean2 /
%   law_exact_step off), slot 5 locked. Paired seeds, canon deep (opts.traj 'meng' = 15 -> 2.5 um ramp over 10 s).
%   e_a = a_true - a_hat absolute; motion segments first. Output test_results/temp_plaw_walls/<plant>_<arm>[_meng].mat +
%   console; figures by plot_plaw_walls.m.  FORK OF run_delta_two_walls.m | EXPIRES: with the advisor-law test
function out = run_plaw_walls(opts)
    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds');  opts.seeds  = 1:10; end
    if ~isfield(opts, 'plants'); opts.plants = {'plane', 'cell', 'sphere'}; end
    if ~isfield(opts, 'arms');   opts.arms   = {'PI'}; end
    if ~isfield(opts, 'tag_suffix'); opts.tag_suffix = ''; end
    if ~isfield(opts, 'traj'); opts.traj = 'canon'; end
    if ~isfield(opts, 'p'); opts.p = 1.9666; end                        % plane band fit on [1.10, 22.2], uniform in h (fit_advisor_law_walls.m)
    user_sig = isfield(opts, 'sig');
    if ~user_sig; opts.sig = 0.5; end                                   % prior sqrt(P) on c, two-sided (same family width as the delta prior); f2 default 0.10 below
    if ~isfield(opts, 'law'); opts.law = 'plaw'; end                    % 'plaw' (advisor a' = (1-c)/h^p, p fixed, c state, version A) | 'p11d1' ([1/1] d1 = 1, c0 state, version A) | 'v2' (0921 tex: a slot kept, slope (1-c)/h^p at the estimator height, c in slot 5, pure form) | 'v2p' (v2 + ma2_aug + nw_mcorr) | 'f2' (0922 tex: two-constant slope (1-c)/(h+delta)^2, slots 5 = c, 7 = delta) | 'f2p' (f2 + ma2_aug + nw_mcorr)
    user_sig_d = isfield(opts, 'sig_d');
    if ~user_sig_d; opts.sig_d = 0.64; end                              % f2: prior sqrt(P) on delta, half the wall-family spread (delta_cell - delta_sphere)/2; f1 default 0.10 (p) below
    if ~isfield(opts, 'seed_cross'); opts.seed_cross = true; end        % f2/f4: false = O40 discriminator arm (seed cross terms P45[0], P47[0] zeroed; tag your suffix '_nocross')
    if ~isfield(opts, 'kind'); opts.kind = 'traj'; end                  % 'traj' (canon / meng) | 'fixed' (positioning at opts.heights for opts.T_hold s, as run_delta_fixed_speed)
    if ~isfield(opts, 'heights'); opts.heights = [22.2, 2.0, 1.10]; end
    if ~isfield(opts, 'T_hold'); opts.T_hold = 3.0; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration')); addpath(here);
    od = fullfile(root, 'test_results', 'temp_plaw_walls');  if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  Ts = pc.Ts;  R = pc.R;
    W_S = 0.0;  P0 = opts.p;  SIG = opts.sig;
    CFIT = struct('plane', 0.0035, 'cell', 0.2918, 'sphere', 0.1219);   % band fits (uniform h), fit_advisor_law_walls.m 09-21
    fm = fullfile(root, 'test_results', 'temp_delta_two_walls', 'advisor_law_fit.mat');
    if exist(fm, 'file'); S = load(fm); for w = 1:numel(S.RES); CFIT.(S.RES(w).name) = S.RES(w).c; end; end
    PFX = '';
    if strcmp(opts.law, 'p11d1')                                          % [1/1] with d1 = 1: band-fit c0 per wall (fit_pade11_d1_walls.m)
        CFIT = struct('plane', -0.4901, 'cell', -0.0848, 'sphere', -0.2968);
        fm = fullfile(root, 'test_results', 'temp_plaw_walls', 'pade11_d1_fit.mat');
        if exist(fm, 'file'); S = load(fm); for w = 1:numel(S.RES); CFIT.(S.RES(w).name) = S.RES(w).c0; end; end
        PFX = 'p11d1_';
    end
    if startsWith(opts.law, 'v2')                                          % version 2: c fitted on the SLOPE over the band (fit_advisor_two_layers.m, p = 2.042)
        CFIT = struct('plane', 0.0372, 'cell', 0.6363, 'sphere', -0.0621);
        PFX = [opts.law '_'];
    end
    DFIT = struct('plane', 0, 'cell', 0, 'sphere', 0);  SIG_D = opts.sig_d;
    if startsWith(opts.law, 'f2')                                          % two-constant slope law: (delta, c) fitted on the SLOPE over the band, p = 2 (fit_two_param_height_walls.m F2)
        CFIT = struct('plane', -0.085, 'cell', -0.163, 'sphere', 0.011, 'sphere_pt', 0.014);      % sphere_pt = C^1 exponential relaxation to the plane tail above 4.5 R (chk_tail2 09-22)
        DFIT = struct('plane',  0.104, 'cell',  1.238, 'sphere', -0.039, 'sphere_pt', -0.041);
        P0 = 2;  if ~user_sig; SIG = 0.10; end                             % p pinned; c prior = wall family -0.16..+0.01 and in-wall walk 0.16 (0922 tex, Seed)
        PFX = [opts.law '_'];
    end
    if startsWith(opts.law, 'f4')                                          % same family, other coordinate: 1/(h+delta)^p, contact slope pinned at 1 (0923 tex); CFIT holds p per wall
        CFIT = struct('plane', 1.950, 'cell', 1.938, 'sphere', 2.015, 'sphere_pt', 2.018);
        DFIT = struct('plane', 0.059, 'cell', 1.115, 'sphere', -0.034, 'sphere_pt', -0.035);
        P0 = 2;  if ~user_sig; SIG = 0.10; end                             % p prior = wall family 1.94..2.02 and in-wall walk 0.04-0.14 (0923 tex, Seed)
        PFX = [opts.law '_'];
    end
    if startsWith(opts.law, 'f1')                                          % the advisor's (1-c)/h^p, c AND p states, no shift (plaw_cp_slope): CFIT = c, DFIT = p per wall (fit_two_param_height_walls.m F1, 09-23)
        CFIT = struct('plane', 0.096, 'cell', 0.732, 'sphere', -0.068, 'sphere_pt', -0.068);
        DFIT = struct('plane', 1.894, 'cell', 1.365, 'sphere', 2.054, 'sphere_pt', 2.054);   % sphere_pt not refitted: the sphere's band pair stands in
        P0 = DFIT.plane;  if ~user_sig; SIG = 0.10; end;  if ~user_sig_d; SIG_D = 0.10; end   % c and p priors 0.10 each (as F2 / F4); plane in-wall walk c 0.14, p 0.10
        PFX = [opts.law '_'];
    end
    if startsWith(opts.law, 'f1p2')                                        % user 09-23 evening: "unify p = 2" -- c the only constant, p pinned by a ~0 prior in the f1 wiring;
        CFIT = struct('plane', 0.0533, 'cell', 0.6405, 'sphere', NaN, 'sphere_pt', NaN);   %   CFIT = c at p = 2 (LS on a', uniform h, band [1.10, 22.2]; fit_c_p2 09-23)
        DFIT = struct('plane', 2, 'cell', 2, 'sphere', 2, 'sphere_pt', 2);
        P0 = 2;  if ~user_sig; SIG = 0.6; end;  SIG_D = 1e-6;                %   c prior = wall-family width (plane 0.05 .. cell 0.64); pass opts.sig 0.10 for the narrow arm
        PFX = [opts.law '_'];
    end
    SEG = [0.5, 1.5, 3.5];  OV = struct();
    if strcmp(opts.traj, 'meng')
        OV  = struct('trajectory_type', 'osc', 'h_init', 15, 'h_bottom', 2.5, 'amplitude', 0, 'frequency', 1, 'n_cycles', 1, ...
                     't_hold', 0.5, 't_descend_override', 10, 'T_sim', 12.5, 'h_min', 2.475);
        SEG = [0.5, 10.5, 10.5];
        opts.tag_suffix = [opts.tag_suffix '_meng'];
    end
    if strcmp(opts.traj, 'osc')                                           % 10-01 (user): canon oscillation with the ramp's descent time and final hold, so the two trajectories differ only by the 2 s oscillation
        if ~isfield(opts, 'Td'); opts.Td = 2.0; end
        if ~isfield(opts, 'T_hold_end'); opts.T_hold_end = 0.5; end
        OV  = struct('t_descend_override', opts.Td, 'T_sim', 0.5 + opts.Td + 2.0 + opts.T_hold_end);   % canon deep otherwise (2 cycles at 1 Hz, band 1.10-3.32 R)
        SEG = [0.5, 0.5 + opts.Td, 0.5 + opts.Td + 2.0];
        opts.tag_suffix = [opts.tag_suffix strrep(sprintf('_osc%g', opts.Td), '.', 'p')];
        if isfield(opts, 'descent_linear') && opts.descent_linear                 % 10-02 (user): constant-velocity descent (same as the ramp) before the oscillation
            OV.trajectory_type = 'osc_linear';  opts.tag_suffix = [opts.tag_suffix 'lin'];
            if isfield(opts, 'wave') && strcmp(opts.wave, 'tri'); OV.trajectory_type = 'osc_tri';  opts.tag_suffix = [opts.tag_suffix 'tri']; end   % 10-02 (user): triangle wave
        end
        if isfield(opts, 'f_osc') && opts.f_osc ~= 1                           % 10-02 (user): oscillation frequency [Hz], 2 s of oscillation kept (n_cycles = 2 f)
            OV.frequency = opts.f_osc;  OV.n_cycles = 2 * opts.f_osc;  opts.tag_suffix = [opts.tag_suffix sprintf('_f%g', opts.f_osc)];
        end
        if isfield(opts, 'h_bottom') && isfield(opts, 'h_top')                   % 10-02 (user): narrower band -- trough h_bottom, top h_top [R]; amplitude = half the band
            OV.h_bottom  = opts.h_bottom * R;  OV.amplitude = 0.5 * (opts.h_top - opts.h_bottom) * R;
            opts.tag_suffix = [opts.tag_suffix sprintf('_b%03d_t%03d', round(100 * opts.h_bottom), round(100 * opts.h_top))];   % 1.05 / 2.0 -> _b105_t200
        end
        if opts.T_hold_end ~= 0.5; opts.tag_suffix = [opts.tag_suffix sprintf('_hold%g', opts.T_hold_end)]; end
    end
    if strcmp(opts.traj, 'ramp')                                          % 09-23 template: hold 0.5 s - constant-velocity descent 22.2 -> 1.10 R in opts.Td - hold 0.5 s
        if ~isfield(opts, 'Td'); opts.Td = 2.0; end
        if ~isfield(opts, 'T_hold_end'); opts.T_hold_end = 0.5; end   % 09-24: longer final hold for the forgetting windup check
        OV  = struct('trajectory_type', 'ramp_descent', 't_hold', 0.5, 't_descend_override', opts.Td, 'T_sim', 0.5 + opts.Td + opts.T_hold_end, ...
                     'amplitude', 0, 'frequency', 1, 'n_cycles', 1);
        SEG = [0.5, 0.5 + opts.Td, 0.5 + opts.Td];
        opts.tag_suffix = [opts.tag_suffix strrep(sprintf('_ramp%g', opts.Td), '.', 'p')];   % 09-30: T_d = 0.5 -> _ramp0p5 (the tag is a struct field)
        if opts.T_hold_end ~= 0.5; opts.tag_suffix = [opts.tag_suffix sprintf('_hold%g', opts.T_hold_end)]; end
        if isfield(opts, 'h_bottom') && ~isempty(opts.h_bottom)             % 09-28: trough below the canonical 1.10 R (deeper-bottom test, c-only line)
            OV.h_bottom = opts.h_bottom * R;                                %   R units in; the driver gate at 1.10 is bypassed explicitly below
            opts.tag_suffix = [opts.tag_suffix sprintf('_hb%03d', round(100 * opts.h_bottom))];   % 1.05 -> _hb105 (no dot: the tag is also a struct field)
        end
    end
    out = struct();
    for ip = 1:numel(opts.plants)
        pl = opts.plants{ip};
        switch pl
            case 'plane';  plant_cperp = [];
            case 'cell';   plant_cperp = @cell_cperp_delta;
            case 'sphere'; plant_cperp = @sphere_cperp_fig11;
            case 'sphere_pt'; plant_cperp = @sphere_cperp_fig11_ptail;   % sphere with the PLANE far tail above 4.5 R (brackets the untested tail, 09-22)
            otherwise; error('unknown plant %s', pl);
        end
        for ia = 1:numel(opts.arms)
            arm = opts.arms{ia};
            switch arm
                case 'PI'; c_seed = 0;  d_seed = DFIT.plane;             % plane init (advisor law: c = 0 is the plane)
                    if strcmp(opts.law, 'p11d1'); c_seed = CFIT.plane; end   % [1/1] d1 = 1: the plane has no exact c0, its band fit (-0.49) is the plane init
                    if startsWith(opts.law, 'v2') || startsWith(opts.law, 'f2') || startsWith(opts.law, 'f4') || startsWith(opts.law, 'f1'); c_seed = CFIT.plane; end    % version 2 / F2 / F4 / F1: plane init = the plane's slope fit (slot 5 = c or p)
                case 'T';  c_seed = CFIT.(pl);  d_seed = DFIT.(pl);      % the wall's own band fit (reference)
                otherwise; error('unknown arm %s', arm);
            end
            cc = local_ctrl_const(c_seed, SIG, W_S, P0, opts.law, d_seed, SIG_D);
            if isfield(cc, 'plaw_dc_slope') || isfield(cc, 'plaw_dp_slope') || isfield(cc, 'plaw_cp_slope'); cc.plaw_seed_cross = opts.seed_cross; end
            if isfield(opts, 'Pf_a_floor'); cc.Pf_a_floor = opts.Pf_a_floor; end   % 09-23 lock control: constants pinned (sig ~ 0) with P44[0] kept at the free arm's value
            if isfield(opts, 'lambda_f_b'); cc.lambda_f_b = opts.lambda_f_b; end   % 09-23 user: fixed forgetting on slot 5 (= c here), R57's knob, reported as a sweep not a chosen value
            if isfield(opts, 'r2_scale'); cc.r2_scale = opts.r2_scale; end         % 09-23 user: whole-R2 scale (y2 weight sweep), diagnostic
            if isfield(opts, 'q55_per_R'); cc.q55_path = true; cc.q55_per_R = opts.q55_per_R; end       % 09-29 layer 1: random-walk container on slot 5 (= c), Q55 = q55_per_R |M| per step; Delta_c^2 / W
            if isfield(opts, 'law_err_db'); cc.law_err_q44 = true; cc.law_err_db = opts.law_err_db; end  % 09-29 layer 1: Q44 += ((da'/dc) delta_c M)^2, the a slot admits the law's c error (0915 tex S7)
            if isfield(opts, 'q55_adapt')                                                                   % 09-29 layer 2 (exploratory): innovation-adaptive Q55 from the y1 residual
                cc.q55_adapt = opts.q55_adapt;
                for fq = {'q55_adapt_N', 'q55_adapt_thr', 'q55_adapt_gain', 'q55_adapt_kcu', 'q55_adapt_P0', 'q55_adapt_rho', 'q55_adapt_Imin', 'q55_adapt_hgate'}; if isfield(opts, fq{1}); cc.(fq{1}) = opts.(fq{1}); end; end
            end
            for fld = {'lambda_f_b_alpha', 'lambda_f_b_floor', 'lambda_f_b_chan', 'lambda_f_b_info'}   % 09-24: conditional forgetting on slot 5 (Meng (6.11) / information-driven)
                if isfield(opts, fld{1}); cc.(fld{1}) = opts.(fld{1}); end
            end
            if strcmp(opts.kind, 'fixed')                                % positioning at one height, no velocity: y2 alone must pin c
                for ih = 1:numel(opts.heights)
                    h = opts.heights(ih);
                    OV = struct('trajectory_type', 'positioning', 'h_init', h * R, 'T_sim', opts.T_hold, ...
                                'h_bottom', h * R, 'amplitude', 0, 't_hold', 0, 't_descend_override', 0);
                    tag = sprintf('%sfixed_%s_%s_h%s%s', PFX, pl, arm, strrep(sprintf('%.2f', h), '.', 'p'), opts.tag_suffix);
                    o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                               'seeds', opts.seeds, 'log_P_full', false);
                    if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
                    fprintf('\n=== %s | hold at w = %.2f R (a_true %.4f) | law %s, seed c = %+.3f (band fit %+.3f), prior %.2f | %d seeds ===\n', ...
                            tag, h, local_a_plant(h, plant_cperp), opts.law, c_seed, CFIT.(pl), SIG, numel(opts.seeds));
                    clear run_formC_b motion_control_law_formC_b;
                    tic; evalc('Rr = run_formC_b(o);'); tr = toc;
                    res = local_collect(Rr, plant_cperp, Ts, P0, opts.law);
                    res.tag = tag; res.plant = pl; res.arm = arm; res.W_S = W_S; res.c_seed = c_seed; res.c_fit = CFIT.(pl);
                    res.d_seed = d_seed; res.d_fit = DFIT.(pl); res.sig_d = SIG_D; res.cc = cc;
                    res.p = P0; res.sig = SIG; res.kind = 'fixed'; res.h = h; res.seg = [0 0 0]; res.law = opts.law;
                    local_report_fixed(res);
                    fprintf('    (%.0f s)\n', tr);
                    save(fullfile(od, [tag '.mat']), '-struct', 'res');
                    out.(tag) = res;
                end
                continue
            end
            tag = sprintf('%s%s_%s%s', PFX, pl, arm, opts.tag_suffix);
            o = struct('arm', 'best', 'ctrl_const_override', cc, 'config_override', OV, 'scenario', 'deep', 'verbose', false, ...
                       'seeds', opts.seeds, 'log_P_full', false);
            if ~isempty(plant_cperp); o.plant_cperp = plant_cperp; end
            if isfield(opts, 'h_bottom') && ~isempty(opts.h_bottom) && opts.h_bottom < 1.1; o.allow_below_floor = true; end   % 09-28 deeper-bottom test
            a11 = local_a_plant(1.1, plant_cperp);  a11_seed = local_law(1.1, P0, c_seed);
            if strcmp(opts.law, 'p11d1'); a11_seed = (1.1 + c_seed) / (1.1 + 1); end
            if startsWith(opts.law, 'v2') || startsWith(opts.law, 'f2') || startsWith(opts.law, 'f4') || startsWith(opts.law, 'f1'); a11_seed = NaN; end   % version 2 / F2 / F4 / F1 have no value law (the a slot is a state)
            fprintf('\n=== %s | law %s, seed c = %+.3f (band fit %+.3f), prior %.2f | seed delta = %+.3f (band fit %+.3f), prior %.2f | a_bar(1.1): plant %.3f, seed law %.3f | %d seeds ===\n', ...
                    tag, opts.law, c_seed, CFIT.(pl), SIG, d_seed, DFIT.(pl), SIG_D, a11, a11_seed, numel(opts.seeds));
            clear run_formC_b motion_control_law_formC_b;
            tic; evalc('Rr = run_formC_b(o);'); tr = toc;
            res = local_collect(Rr, plant_cperp, Ts, P0, opts.law);
            res.tag = tag; res.plant = pl; res.arm = arm; res.W_S = W_S; res.c_seed = c_seed; res.c_fit = CFIT.(pl);
            res.d_seed = d_seed; res.d_fit = DFIT.(pl); res.sig_d = SIG_D; res.cc = cc;
            res.p = P0; res.sig = SIG; res.seg = SEG; res.law = opts.law;
            local_report(res, SEG);
            fprintf('    (%.0f s)\n', tr);
            save(fullfile(od, [tag '.mat']), '-struct', 'res');
            out.(tag) = res;
        end
    end
end

function a = local_law(h, p, c)
    a = 1 - (1 - c) ./ ((p - 1) * max(h, 1).^(p - 1));
end

function a = local_a_plant(h, plant_cperp)
    h = max(h, 1.001);
    if isempty(plant_cperp)                                % production plane plant is scalar-only
        a = zeros(size(h));
        for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
    else
        a = 1 ./ plant_cperp(h);
    end
end

function cc = local_ctrl_const(c_seed, sig, ws, p, law, d_seed, sig_d)
    if nargin < 5; law = 'plaw'; end
    if nargin < 6; d_seed = 0; end
    if nargin < 7; sig_d = 0; end
    cc = struct('ma2_aug', false, 'nw_mcorr', false, 'pred_mean2', false, 'pred_mean2_e4', false, ...
                'law_exact_step', false, 'lock_b', true, 'b_ceil', 1.5, 'b_floor', 0.2, 'Pf_a_floor', 1e-5);
    cc.gain_algebraic = true;  cc.amodel_fe = true;  cc.amodel_h = true;
    cc.ws0_perp = ws;  cc.Pf_w0_std = 0;
    switch law
        case 'plaw'                                        % advisor power law: slot 7 = c, slot 5 locked and inert
            cc.plaw_c_state = true;  cc.plaw_p = p;  cc.c_init = c_seed;  cc.Pf_c_std = sig;
            cc.b_init = 8/9;  cc.Pf_b_std = 0;
        case 'p11d1'                                       % [1/1] Pade, version A: slot 7 = c0 (estimated), slot 5 = d1 LOCKED at 1 (lock_b)
            cc.c0_state = true;  cc.c0_init = c_seed;  cc.Pf_c0_std = sig;
            cc.d1_init = 1.0;  cc.Pf_d1_std = 0;
        case {'v2', 'v2p'}                                 % version 2 (0921 tex): a slot kept, slope (1-c)/h^p at the estimator height, slot 5 = c FREE
            cc.gain_algebraic = false;  cc.amodel_fe = false;  cc.amodel_h = false;  cc.lock_b = false;
            cc.plaw_c_slope = true;  cc.plaw_p = p;  cc.c_init = c_seed;  cc.Pf_c_std = sig;
            cc.Pf_a_floor = 0.01;                          % level prior of the a slot at the start height: the wall family spans 0.003-0.01 there
            cc.Pf_w0_std  = 0.5;                           % position-type family prior (C53 container, same 0.5 R as the delta prior): P44[0] = a'^2 0.25 --
                                                           %   0.001 at 22 R, 0.12 at 2 R, 0.39 at 1.1 R; without it a fixed start at 1.10 R on the cell
                                                           %   keeps a_hat at the plane's 0.087 (true 0.505), the force is 5.8x too large and the particle hits the wall
            if strcmp(law, 'v2p'); cc.ma2_aug = true;  cc.nw_mcorr = true; end   % the two law-independent production patches
        case {'f2', 'f2p'}                                 % two-constant slope law (0922 tex): a slot kept, slope (1-c)/(h+delta)^2 at the estimator height, slots 5 = c, 7 = delta FREE
            cc.gain_algebraic = false;  cc.amodel_fe = false;  cc.amodel_h = false;  cc.lock_b = false;
            cc.plaw_dc_slope = true;  cc.plaw_p = 2;  cc.c_init = c_seed;  cc.Pf_c_std = sig;
            cc.delta_init = d_seed;  cc.Pf_delta_std = sig_d;  cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-0.9, 3];
            cc.Pf_a_floor = 0.002;                         % plane polynomial vs the family's value law at the start height (0922 tex, Seed); the position-type
            cc.Pf_w0_std  = 0;                             %   container is now slot 7 itself (P44[0] += a'^2 Pf_delta^2 + Pf_c^2/g^2 with cross terms), w_s is the given surface
            if strcmp(law, 'f2p'); cc.ma2_aug = true;  cc.nw_mcorr = true; end
        case {'f4', 'f4p'}                                 % same family, other coordinate (0923 tex): slope 1/(h+delta)^p, slots 5 = p, 7 = delta FREE, contact slope pinned at 1
            cc.gain_algebraic = false;  cc.amodel_fe = false;  cc.amodel_h = false;  cc.lock_b = false;
            cc.plaw_dp_slope = true;  cc.p_init = c_seed;  cc.Pf_p_std = sig;  cc.plaw_p_lim = [1.2, 4];
            cc.delta_init = d_seed;  cc.Pf_delta_std = sig_d;  cc.plaw_g_min = 0.5;  cc.plaw_delta_lim = [-0.9, 3];
            cc.Pf_a_floor = 0.002;  cc.Pf_w0_std = 0;      % as f2: the position-type container is slot 7 itself, w_s is the given surface
            if strcmp(law, 'f4p'); cc.ma2_aug = true;  cc.nw_mcorr = true; end
        case {'f1', 'f1p', 'f1p2'}                         % the advisor's (1-c)/h^p with BOTH constants as states, no shift (plaw_cp_slope, 09-23 evening): slots 5 = c, 7 = p FREE ('f1p2': p pinned at 2 by sig_d ~ 0)
            cc.gain_algebraic = false;  cc.amodel_fe = false;  cc.amodel_h = false;  cc.lock_b = false;
            cc.plaw_cp_slope = true;  cc.c_init = c_seed;  cc.Pf_c_std = sig;
            cc.p_init = d_seed;  cc.Pf_p_std = sig_d;  cc.plaw_p_lim = [1.2, 4];     % the runner's "d" slot carries p here
            cc.Pf_a_floor = 0.002;  cc.Pf_w0_std = 0;
            if strcmp(law, 'f1p'); cc.ma2_aug = true;  cc.nw_mcorr = true; end
        otherwise; error('unknown law %s', law);
    end
end

function res = local_collect(Rr, plant_cperp, Ts, p, law)
    if nargin < 5; law = 'plaw'; end
    ns = numel(Rr.runs);  S1 = Rr.runs{1};  n = numel(S1.h_bar_d_out);
    res = struct('t', (0:n-1)' * Ts, 'wd', S1.h_bar_d_out(:), 'ns', ns);
    res.h_true = zeros(n, ns); res.a_true = zeros(n, ns); res.a_hat = zeros(n, ns);
    res.c_hat = zeros(n, ns); res.sP_c = zeros(n, ns); res.sP_a = zeros(n, ns); res.K_a_y2 = zeros(n, ns);
    res.d_hat = zeros(n, ns); res.sP_d = zeros(n, ns);
    res.innov1 = zeros(n, ns); res.innov2 = zeros(n, ns); res.S1 = zeros(n, ns); res.S2 = zeros(n, ns);
    for q = 1:ns
        S = Rr.runs{q};  hb = S.h_bar_true_out(:);
        res.h_true(:, q) = hb;  res.a_true(:, q) = local_a_plant(hb, plant_cperp);  res.a_hat(:, q) = S.a_bar_hat_out(:, 3);
        if startsWith(law, 'v2') || startsWith(law, 'f2') || startsWith(law, 'f4') || startsWith(law, 'f1')   % version 2 / F2 / F4 / F1: slot 5 (c, or p under F4) reported through b_hat / P_b
            res.c_hat(:, q) = S.b_hat_out(:, 3);  res.sP_c(:, q) = S.P_b_out(:, 3);
            if startsWith(law, 'f2') || startsWith(law, 'f4') || startsWith(law, 'f1'); res.d_hat(:, q) = S.ws_hat_out(:, 3);  res.sP_d(:, q) = S.P_ws_out(:, 3); end   % F2 / F4: delta is slot 7 (ws_hat / P_ws); F1: p is slot 7
        else                                                                     % version A flags: slot 7 = c / c0 (ws_hat / P_ws)
            res.c_hat(:, q) = S.ws_hat_out(:, 3);  res.sP_c(:, q) = S.P_ws_out(:, 3);
        end
        res.innov1(:, q) = S.innov_y1_out(:, 3);  res.innov2(:, q) = S.innov_y2_out(:, 3);
        if isfield(S, 'P_a_out'); res.sP_a(:, q) = S.P_a_out(:, 3) / S.a_nom;  res.K_a_y2(:, q) = S.K_a_y2_out(:, 3); end   % sqrt(P44) in a/a_nom units, y2 gain on the a slot
        if isfield(S, 'S1_out'); res.S1(:, q) = S.S1_out(:, 3);  res.S2(:, q) = S.S2_out(:, 3); end
        if isfield(S, 'lam_b_out'); res.lam_b(:, q) = S.lam_b_out(:, 3); end   % 09-24: per-step slot-5 forgetting factor
        if isfield(S, 'a_xm_out'); res.a_m(:, q) = S.a_xm_out(:, 3) / S.a_nom; end   % 09-29: the measured gain a_m (y2 reading) in a/a_nom units
        if isfield(S, 'q55a_q_out'); res.q55a_q(:, q) = S.q55a_q_out(:, 3);  res.q55a_stat(:, q) = S.q55a_stat_out(:, 3); end   % 09-29: adaptive Q55 added per step and its y1 statistic
    end
    hd = max(res.wd, 1.0);  dh = 1e-4;                                       % c_eff at the commanded height: 1 - h^p a'_plant(h)
    ap = (local_a_plant(hd + dh, plant_cperp) - local_a_plant(hd - dh, plant_cperp)) / (2 * dh);
    res.c_eff_d = 1 - hd.^p .* ap;
end

function local_report_fixed(res)
    t = res.t;  ea = res.a_true - res.a_hat;  x = res.c_hat;  sp = res.sP_c;  last = t >= t(end) - 1.0;
    m = mean(ea, 2);  [~, iw] = max(abs(m));
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d\n', min(res.h_true(:)), nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9));
    fprintf('  e_a    : last 1 s mean %+.4f sd %.4f | worst |mean| %.4f at t = %.2f s\n', mean(m(last)), mean(std(ea(last, :), 0, 2)), abs(m(iw)), t(iw));
    fprintf('  c      : seed %+.3f band fit %+.3f c_eff here %+.3f | end %+.3f +- %.3f (seed sd) | sqrtP end %.3f | sd/sqrtP %.2f | hold slope %+.5f /s\n', ...
            res.c_seed, res.c_fit, res.c_eff_d(end), mean(x(end, :)), std(x(end, :)), mean(sp(end, :)), std(x(end, :)) / max(mean(sp(end, :)), eps), ...
            mean((x(end, :) - x(find(last, 1), :)) / 1.0));
    if isfield(res, 'd_seed') && (startsWith(res.law, 'f2') || startsWith(res.law, 'f4') || startsWith(res.law, 'f1'))
        d = res.d_hat;  sd = res.sP_d;  lab7 = 'delta';  if startsWith(res.law, 'f1'); lab7 = 'p'; end
        fprintf('  %-6s : seed %+.3f band fit %+.3f | end %+.3f +- %.3f (seed sd) | sqrtP end %.3f | sd/sqrtP %.2f | hold slope %+.5f /s\n', ...
                lab7, res.d_seed, res.d_fit, mean(d(end, :)), std(d(end, :)), mean(sd(end, :)), std(d(end, :)) / max(mean(sd(end, :)), eps), ...
                mean((d(end, :) - d(find(last, 1), :)) / 1.0));
    end
end

function local_report(res, SEG)
    t = res.t;  ea = res.a_true - res.a_hat;                     % e_a = a_true - a_hat, absolute
    m = mean(ea, 2);  s = std(ea, 0, 2);
    desc = t >= SEG(1) & t < SEG(2);  osc = t >= SEG(2) & t < SEG(3);  hold = t >= SEG(3);
    [~, iw] = max(abs(m) .* desc);
    fprintf('  health : min h_true %.4f | NaN %d | floor hits %d | c_hat range [%.3f, %.3f]\n', min(res.h_true(:)), ...
            nnz(~isfinite(res.a_hat(:))), nnz(res.a_hat(:) <= 0.05 + 1e-9), min(res.c_hat(:)), max(res.c_hat(:)));
    fprintf('  e_a    : descent worst %+.4f at t = %.2f s (sd %.4f) | descent mean %+.4f sd %.4f | osc mean %+.4f sd %.4f | hold mean %+.4f sd %.4f\n', ...
            m(iw), t(iw), s(iw), mean(m(desc)), mean(s(desc)), mean(m(osc)), mean(s(osc)), mean(m(hold)), mean(s(hold)));
    x = res.c_hat;  sp = res.sP_c;  lab5 = 'c';  if startsWith(res.law, 'f4'); lab5 = 'p'; end
    fprintf('  %-6s : seed %+.3f band fit %+.3f | end %+.3f +- %.3f (seed sd) | c_eff at hold height %+.3f, at descent end %+.3f | sqrtP end %.3f | sd/sqrtP %.2f\n', ...
            lab5, res.c_seed, res.c_fit, mean(x(end, :)), std(x(end, :)), res.c_eff_d(end), res.c_eff_d(find(t < SEG(2), 1, 'last')), ...
            mean(sp(end, :)), std(x(end, :)) / max(mean(sp(end, :)), eps));
    dt = t(2) - t(1);  hs = find(hold);
    fprintf('  hold   : %s_hat slope %+.5f /s (seed mean)\n', lab5, mean((x(hs(end), :) - x(hs(1), :)) / (numel(hs) * dt)));
    if isfield(res, 'd_seed') && (startsWith(res.law, 'f2') || startsWith(res.law, 'f4') || startsWith(res.law, 'f1'))
        d = res.d_hat;  sd = res.sP_d;  ie = find(t < SEG(2), 1, 'last');  lab7 = 'delta';  if startsWith(res.law, 'f1'); lab7 = 'p'; end
        fprintf('  %-6s : seed %+.3f band fit %+.3f | descent end %+.3f +- %.3f | end %+.3f +- %.3f (seed sd) | sqrtP end %.3f | sd/sqrtP %.2f | range [%.3f, %.3f]\n', ...
                lab7, res.d_seed, res.d_fit, mean(d(ie, :)), std(d(ie, :)), mean(d(end, :)), std(d(end, :)), mean(sd(end, :)), ...
                std(d(end, :)) / max(mean(sd(end, :)), eps), min(d(:)), max(d(:)));
    end
end
