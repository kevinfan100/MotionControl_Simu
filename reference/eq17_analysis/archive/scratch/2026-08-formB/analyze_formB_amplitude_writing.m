function res = analyze_formB_amplitude_writing()
%ANALYZE_FORMB_AMPLITUDE_WRITING  Audit the proposed writing
%   a_bar = 1 - b*(1 + (w_bar - w_s))^(-p)   ("amplitude writing")
%   against the production Form B
%   a_bar = 1 - (1 + (w_bar - w_s)/b)^(-p)   ("length writing").
%
% QUESTION (user, 2026-08-06): "near wall b = 1, but far wall b = ?"
%
% SCOPE: OFFLINE ONLY.  Curve algebra + minimax fits against the SSOT truth
%   (calc_correction_functions, Brenner/Jeffrey-Onishi series).  No filter is
%   built, nothing under model/ is touched.
%
% DELIVERABLES
%   D1  the two writings are the SAME 3-parameter family: the map
%           b' = b^p ,  p' = p ,  w_s' = w_s - b + 1
%       is a bijection, so every fit number is shared.  Checked to eps.
%   D2  minimax ladder on the documented protocol (w_bar in [1.1,10], 2e4
%       samples) -- reproduces the tex numbers, then prices the two anchors.
%   D3  the anchor question itself: near wall b = 1 (from BOTH the intercept
%       a_bar(w_s) = 1-b = 0 and the slope), far field b = 9/8 (reflection
%       coefficient).  Same 12.5% over-determination as Form B -- relocated.
%   D4  where the tension lands: in the length writing on the near-wall SLOPE
%       (w_s stays the wall); in the amplitude writing on w_s ITSELF (the
%       reported surface moves by b-1 = 0.125 R).
%   D5  sensitivity geometry on the canonical envelope, LEVEL convention
%       dA/dtheta (the Fisher convention of analyze_formB_fisher_2param.m):
%       det F is invariant (|det dtheta'/dtheta| = 1), so the amplitude
%       writing buys no information -- it only rotates the coordinates
%       towards the strong direction, which is the (A_ref, w_s) decoupling of
%       the 08-06 Fisher audit seen from the form side, and it makes the wall
%       a DERIVED quantity (w_s + b - 1) instead of a state.
%
% Truth curve, protocol and reference numbers:
%   reference/eq17_analysis/derivation/gainlaw_shape_selection.tex
%   reference/eq17_analysis/derivation/formB_ws_ref.tex  (S1, S10)

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(fullfile(root, 'model', 'wall_effect'));
    addpath(fullfile(root, 'model', 'config'));

    pc     = physical_constants();
    R_um   = pc.R;
    B_FAR  = 9/8;              % reflection coefficient, perpendicular
    B_NEAR = 1;                % lubrication: a_bar -> gap exactly

    % ---- truth on the documented fit protocol ---------------------------
    w_fit  = linspace(1.1, 10, 2e4)';
    a_fit  = truth_abar(w_fit);

    % canonical scenario envelope (h_init 50 um, amplitude 2.5 um, R 2.25 um)
    env    = [1.9, 22.3];
    w_env  = linspace(env(1), env(2), 4000)';
    a_env  = truth_abar(w_env);

    fprintf('\n==== D1  the two writings are one family ====================\n');
    th = [1.0566, 0.9563, 0.9936];          % documented 3-free Form B optimum
    b = th(1); p = th(2); ws = th(3);
    map = [b^p, p, ws - b + 1];
    dmax = max(abs(len_law(w_fit, b, p, ws) - amp_law(w_fit, map(1), map(2), map(3))));
    fprintf('  length  (b,p,w_s)  = (%.4f, %.4f, %.4f)\n', b, p, ws);
    fprintf('  amplitude (b,p,w_s)= (%.4f, %.4f, %.4f)   [b''=b^p, w_s''=w_s-b+1]\n', map);
    fprintf('  sup |a_len - a_amp| over [1.1,10] = %.3e  (identical curve)\n', dmax);
    fprintf('  bare-writing pole  C = w_s - b = %+.4f  <- the tex''s "w_s reads -0.063"\n', ws - b);

    fprintf('\n==== D2  minimax ladder, w_bar in [1.1,10] ==================\n');
    lad = struct('name', {}, 'sup', {}, 'par', {});
    lad(end+1) = fitrow('length   3 free (b,p,w_s)', ...
        @(x) len_law(w_fit, x(1), x(2), x(3)), [1.05 0.96 0.99], a_fit);
    lad(end+1) = fitrow('amplitude 3 free (b,p,w_s)', ...
        @(x) amp_law(w_fit, x(1), x(2), x(3)), [1.05 0.96 0.94], a_fit);
    lad(end+1) = fitrow('p=1, b free, w_s free', ...
        @(x) len_law(w_fit, x(1), 1, x(2)), [1.05 0.99], a_fit);
    lad(end+1) = fitrow('p=1, b=9/8 FAR anchor, w_s free', ...
        @(x) len_law(w_fit, B_FAR, 1, x(1)), 1.0, a_fit);
    lad(end+1) = fitrow('p=1, b=1   NEAR anchor, w_s free', ...
        @(x) len_law(w_fit, B_NEAR, 1, x(1)), 1.0, a_fit);
    for i = 1:numel(lad)
        fprintf('  %-34s sup %6.3f%%   par %s\n', lad(i).name, ...
            100*lad(i).sup, mat2str(round(lad(i).par, 4)));
    end

    fprintf('\n==== D2b same ladder on the CANONICAL ENVELOPE [%.1f, %.1f] ====\n', env);
    lad2 = struct('name', {}, 'sup', {}, 'par', {});
    lad2(end+1) = fitrow('p=1, b free, w_s free', ...
        @(x) len_law(w_env, x(1), 1, x(2)), [1.12 0.99], a_env);
    lad2(end+1) = fitrow('p=1, b=9/8 FAR anchor, w_s free', ...
        @(x) len_law(w_env, B_FAR, 1, x(1)), 1.0, a_env);
    lad2(end+1) = fitrow('p=1, b=1   NEAR anchor, w_s free', ...
        @(x) len_law(w_env, B_NEAR, 1, x(1)), 1.0, a_env);
    for i = 1:numel(lad2)
        fprintf('  %-34s sup %6.3f%%   par %s   wall err %+.3f um\n', lad2(i).name, ...
            100*lad2(i).sup, mat2str(round(lad2(i).par, 4)), ...
            (lad2(i).par(end) - 1)*R_um);
    end

    fprintf('\n==== D3  the anchor question ================================\n');
    fprintf('  amplitude writing at contact: a_bar(w_s) = 1 - b   -> b = 1\n');
    fprintf('  amplitude writing slope at contact: p*b            -> b = 1 (p=1)\n');
    fprintf('  amplitude writing far field: 1-a_bar -> b/gap      -> b = 9/8 (perp)\n');
    fprintf('                                                        b = 9/16 (para)\n');
    fprintf('  over-determination 9/8 vs 1 = %.1f%%  (identical to Form B''s p/b tension)\n', ...
        100*(B_FAR - B_NEAR)/B_NEAR);

    fprintf('\n==== D4  where the tension lands ============================\n');
    ws_far = lad(4).par(1);
    fprintf('  length writing, b=9/8 pinned:  a_bar(w_s)=0 by construction\n');
    fprintf('     fitted w_s = %.4f -> wall report error %+.4f R = %+.3f um\n', ...
        ws_far, ws_far - 1, (ws_far - 1)*R_um);
    fprintf('  amplitude writing, b=9/8 pinned: a_bar(w_s) = %+.4f  (NEGATIVE mobility)\n', ...
        1 - B_FAR);
    fprintf('     model stalls at w_s + (b-1) = w_s + %.4f R = w_s + %.3f um\n', ...
        B_FAR - 1, (B_FAR - 1)*R_um);
    fprintf('     so the reported w_s misses the surface by %.3f um (positioning RMSE 0.25 um)\n', ...
        (B_FAR - 1)*R_um);
    fprintf('  amplitude writing, b=1 pinned (wall preserved): far field off by %.1f%%\n', ...
        100*(1 - 1/B_FAR));

    fprintf('\n==== D5  sensitivity geometry on the envelope [%.1f, %.1f] ======\n', env);
    % LEVEL sensitivities dA/dtheta (the Fisher convention of
    % analyze_formB_fisher_2param.m), p = 1, anchored b = 9/8.
    % length writing   : gap g = w - w_s ,  D = g + b
    % amplitude writing: same curve at (b, w_s-b+1) -> same D
    g_env = w_env - 1;
    D     = g_env + B_FAR;
    sL    = [-g_env./D.^2, -B_FAR./D.^2];      % [dA/db  dA/dws]  length
    sA    = [-1./D,        -B_FAR./D.^2];      % [dA/db  dA/dws]  amplitude
    cosL  = abs(dot(sL(:,1), sL(:,2))/(norm(sL(:,1))*norm(sL(:,2))));
    cosA  = abs(dot(sA(:,1), sA(:,2))/(norm(sA(:,1))*norm(sA(:,2))));
    FL = sL'*sL;  FA = sA'*sA;                 % uniform weight, unit R
    fprintf('  |cos(s_b, s_ws)|       length %.4f    amplitude %.4f\n', cosL, cosA);
    fprintf('  det F                  length %.6e  amplitude %.6e   ratio %.6f\n', ...
        det(FL), det(FA), det(FA)/det(FL));
    fprintf('  (ratio = 1 exactly: dtheta''/dtheta = [1 0; -1 1], |det| = 1 -> same information)\n');
    CL = inv(FL); CA = inv(FA);
    fprintf('  sigma_b  (data only)   length %.4e  amplitude %.4e\n', sqrt(CL(1,1)), sqrt(CA(1,1)));
    fprintf('  sigma_ws (data only)   length %.4e  amplitude %.4e\n', sqrt(CL(2,2)), sqrt(CA(2,2)));
    sig_wall_A = sqrt(CA(1,1) + CA(2,2) + 2*CA(1,2));   % wall = w_s'' + b - 1
    fprintf('  sigma of the WALL      length %.4e  amplitude %.4e (= w_s''+b-1)\n', ...
        sqrt(CL(2,2)), sig_wall_A);
    fprintf('  -> identical, as it must be; the amplitude writing just makes the\n');
    fprintf('     estimand a DERIVED combination instead of a state.\n');

    % what a diagonal prior means in each coordinate
    sb = 0.0157; sw = 0.111;                   % production envelope priors
    fprintf('  production prior (b, w_s) = (%.4f, %.4f), diagonal\n', sb, sw);
    fprintf('     pushed to amplitude coords: sigma_ws = %.4f, corr(b,w_s) = %+.3f\n', ...
        sqrt(sw^2 + sb^2), -sb^2/(sb*sqrt(sw^2+sb^2)));
    fprintf('     keeping it diagonal there instead = a DIFFERENT belief about the wall\n');

    res = struct('ladder', lad, 'map', map, 'cos_len', cosL, 'cos_amp', cosA, ...
                 'detF_ratio', det(FA)/det(FL));
end

% ---------------------------------------------------------------------------
function a = truth_abar(w)
    a = zeros(size(w));
    for i = 1:numel(w)
        [~, cp] = calc_correction_functions(w(i));
        a(i) = 1/cp;
    end
end

function a = len_law(w, b, p, ws)
    a = 1 - (1 + (w - ws)/b).^(-p);
end

function a = amp_law(w, b, p, ws)
    a = 1 - b*(1 + (w - ws)).^(-p);
end

function row = fitrow(name, model, x0, a_true)
%   Metric = sup |a_model/a_true - 1| (relative gain error), the protocol of
%   gainlaw_shape_evidence.m: RMS seed (smooth) then minimax (kinked).
    rel  = @(x) model(x)./a_true - 1;
    frms = @(x) sqrt(mean(rel(x).^2));
    fsup = @(x) max(abs(rel(x)));
    opt  = optimset('MaxFunEvals', 4e4, 'MaxIter', 2e4, 'TolX', 1e-11, 'TolFun', 1e-13);
    x = fminsearch(frms, x0, opt);
    for k = 1:8
        x = fminsearch(fsup, x, opt);
    end
    row = struct('name', name, 'sup', fsup(x), 'par', x);
end
