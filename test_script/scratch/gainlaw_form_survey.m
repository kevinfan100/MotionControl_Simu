%GAINLAW_FORM_SURVEY  Are there better shapes than Form B?
%
%   Every candidate is written in the gap g = w - w_s and must satisfy, by
%   construction, the two things the truth fixes:
%       g -> 0 : a -> 0          (contact)
%       g -> oo: a -> 1          (free space)
%   A rational function whose numerator and denominator have the same degree
%   AND the same leading coefficient satisfies both automatically, and its
%   far-field deficit is then automatically O(1/g) -- the Lorentz tail.  That
%   makes Pade [n/n] in g the natural family to test against Form B.
%
%   Reported for each: sup relative gain error on a plane and on a 7 um cell,
%   and the Fisher condition number in log-parameter space (how badly the
%   parameters are coupled).  Accuracy alone is not the criterion: a form that
%   is 20% better but 10x worse conditioned is not an improvement.
%
%   Usage:  gainlaw_form_survey

clear; clc;
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'model', 'wall_effect'));
addpath(fullfile(root, 'model', 'config'));
addpath(here);
pc = physical_constants();  R = pc.R;
OPT = optimset('MaxFunEvals', 3e5, 'MaxIter', 3e5, 'TolX', 1e-14, 'TolFun', 1e-14);
N = 8000;  w = logspace(log10(1.1), log10(10), N)';

% ---- candidates: f(theta, w), theta(end) is always w_s ------------------
gp = @(t, w) max(w - t(end), 0);
C = {};
C{end+1} = struct('name','B   1-[lam/(g+lam)]^b', 'np',3, ...
    'f', @(t,w) 1 - (exp(t(1))./(gp(t,w)+exp(t(1)))).^exp(t(2)), ...
    'seed',[log(1) 0 1; log(0.8) log(1.2) 0.98; log(1.06) log(0.96) 1; log(0.6) log(1.4) 1.02]);
C{end+1} = struct('name','P11 g/(g+lam)          ', 'np',2, ...
    'f', @(t,w) gp(t,w)./(gp(t,w)+exp(t(1))), ...
    'seed',[log(9/8) 1; log(1) 0.98; log(1.2) 1.02]);
C{end+1} = struct('name','P22 (a1 g+a2 g^2)/(1+b1 g+a2 g^2)', 'np',4, ...
    'f', @(t,w) (exp(t(1))*gp(t,w)+exp(t(2))*gp(t,w).^2) ./ ...
                (1+exp(t(3))*gp(t,w)+exp(t(2))*gp(t,w).^2), ...
    'seed',[log(1) log(1) log(2) 1; log(0.9) log(0.5) log(1.5) 1; ...
            log(1.1) log(2) log(3) 0.98; log(0.8) log(0.3) log(1.2) 1.02]);
C{end+1} = struct('name','P22b same, b1 = a1+lam (far pinned)', 'np',3, ...
    'f', @(t,w) (exp(t(1))*gp(t,w)+exp(t(2))*gp(t,w).^2) ./ ...
                (1+(exp(t(1))+exp(t(2))*9/8)*gp(t,w)+exp(t(2))*gp(t,w).^2), ...
    'seed',[log(1) log(1) 1; log(0.9) log(0.5) 1; log(1.1) log(2) 0.98]);
C{end+1} = struct('name','H   g^n/(g^n+lam^n)    ', 'np',3, ...
    'f', @(t,w) gp(t,w).^exp(t(2)) ./ (gp(t,w).^exp(t(2)) + exp(t(1)).^exp(t(2))), ...
    'seed',[log(9/8) 0 1; log(1) log(1.1) 0.98; log(1.2) log(0.9) 1]);

WALLS = {'plane', inf; 'cell 7um', 7/R};

fprintf('\n%-38s %4s | %10s %10s | %10s\n', 'form', 'np', 'plane sup%', 'cell sup%', 'cond(F)');
fprintf('%s\n', repmat('-', 1, 80));
for k = 1:numel(C)
    row = zeros(1,2);  th = cell(1,2);
    for j = 1:2
        if isinf(WALLS{j,2})
            D = zeros(N,1);
            for i = 1:N, [~, cc] = calc_correction_functions(w(i)); D(i) = 1/cc; end
        else
            D = 1 ./ build_truth_two_sphere(w, WALLS{j,2});
        end
        [row(j), th{j}] = fitmm(C{k}.f, D, w, OPT, C{k}.seed);
    end
    cn = condF(C{k}.f, th{1}, w);          % conditioning at the plane optimum
    fprintf('%-38s %4d | %9.3f%% %9.3f%% | %10.0f\n', C{k}.name, C{k}.np, ...
            100*row(1), 100*row(2), cn);
end
fprintf('%s\n', repmat('-', 1, 80));
fprintf('cond(F): Fisher condition number in log-parameter space at the plane fit.\n');
fprintf('Reference: the dominant closed-loop error today is defect 2 at 4-5%%.\n');


function [sup, tb] = fitmm(f, D, w, opt, seeds)
    gd = @(t) 1e6*max(0, t(end) - 1.09)^2;
    gs = @(t) max(abs(f(t,w)./D - 1)) + gd(t);
    gr = @(t) sqrt(mean((f(t,w)./D - 1).^2)) + gd(t);
    best = inf; tb = [];
    for s = 1:size(seeds,1)
        t = fminsearch(gs, fminsearch(gr, seeds(s,:), opt), opt);
        t = fminsearch(gs, t, opt);
        if gs(t) < best, best = gs(t); tb = t; end
    end
    sup = max(abs(f(tb,w)./D - 1));
end

function cn = condF(f, t, w)
%CONDF  Fisher condition number, log-parameter space for the positive
%   parameters and linear for w_s (which is the last entry).
    n = numel(t);  d = 1e-6;  J = zeros(numel(w), n);
    for k = 1:n
        tp = t; tm = t;
        tp(k) = t(k) + d;  tm(k) = t(k) - d;
        J(:,k) = (f(tp,w) - f(tm,w))/(2*d);
    end
    cn = cond(J'*J);
end
