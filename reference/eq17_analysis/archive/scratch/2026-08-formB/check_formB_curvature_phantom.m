% check_formB_curvature_phantom.m -- S11 V1- stall: convict the curvature
%   phantom (Stage 3 diagnosis, 2026-08-02).
%   DESIGN (paired cancellation): run the STALLED condition (delta=-0.05,
%   ws_init = truth+0.025) and its MIRROR (delta=+0.05, ws_init = truth-0.025)
%   with the same seeds. First-order innovation content flips sign between
%   the two arms; any curvature phantom (proportional to P_ws, sign fixed) is
%   COMMON -- so (mean_stalled + mean_mirror)/2 isolates the phantom, per
%   channel and per phase. Scaling leg: repeat at prior/2 -- a curvature
%   phantom ( ~ a''*P_ws/2 ) must drop 4x.
%   Candidates it discriminates:
%     H1 y2-measurement curvature: phantom in innov_y2, all phases, ~ a''(gap)
%     H2 a-propagation curvature (descent): phantom in innov_y1/ws-push,
%        descent-localized
%   EXPIRES: when the V1- stall is resolved and re-examed.

proj = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(proj, 'model')));
addpath(fullfile(proj, 'test_script', 'integration'));

SEEDS = 60000 + (1:6);
PRIORS = [0.044, 0.022];
base = struct('lock_b', false, 'lock_p', true, 'lock_ws', false);

res = struct();
for ip = 1:2
  pw = PRIORS(ip);
  arms = {struct('name','stall',  'delta',-0.05, 'init',0.95+0.025), ...
          struct('name','mirror', 'delta',+0.05, 'init',1.05-0.025)};
  for ia = 1:2
    arm = arms{ia};
    iy1 = []; iy2 = []; d1 = []; d2 = []; tt = [];
    for c = 1:numel(SEEDS)
      o = struct(); o.seeds = SEEDS(c) + 100*ip; o.ws_inject = arm.delta;
      o.verbose = false;
      o.ctrl_const_override = base;
      o.ctrl_const_override.Pf_ws_std = pw;
      o.ctrl_const_override.ws_init   = arm.init;
      [~, r] = evalc('run_formB_ws(o)');
      s = r.runs{1};
      if isempty(iy1)
        n = numel(s.tout); tt = s.tout;
        iy1 = zeros(n,1); iy2 = zeros(n,1); d1 = zeros(n,1); d2 = zeros(n,1);
      end
      iy1 = iy1 + s.innov_y1_out(:,3) / numel(SEEDS);
      iy2 = iy2 + s.innov_y2_out(:,3) / numel(SEEDS);
      d1  = d1  + s.dws_y1_out(:,3)  / numel(SEEDS);
      d2  = d2  + s.dws_y2_out(:,3)  / numel(SEEDS);
    end
    res.(sprintf('p%d_%s', ip, arm.name)) = struct( ...
        'iy1', iy1, 'iy2', iy2, 'd1', d1, 'd2', d2, 'tt', tt);
  end
end

W = {'desc', @(t) t>=0.5 & t<1.5; 'osc', @(t) t>=1.6 & t<3.5; 'hold', @(t) t>=3.8};
fprintf('\n=== curvature-phantom extraction: (stall + mirror)/2, per phase ===\n');
fprintf('%-10s %-6s %12s %12s %14s %14s\n', 'prior', 'phase', ...
        'ph_innov_y1', 'ph_innov_y2', 'ph_wspush_y1', 'ph_wspush_y2');
for ip = 1:2
  S = res.(sprintf('p%d_stall', ip)); M = res.(sprintf('p%d_mirror', ip));
  for iw = 1:3
    w = W{iw,2}(S.tt);
    fprintf('%-10.3f %-6s %12.3e %12.3e %14.3e %14.3e\n', PRIORS(ip), W{iw,1}, ...
      (mean(S.iy1(w)) + mean(M.iy1(w)))/2, ...
      (mean(S.iy2(w)) + mean(M.iy2(w)))/2, ...
      (sum(S.d1(w)) + sum(M.d1(w)))/2, ...
      (sum(S.d2(w)) + sum(M.d2(w)))/2);
  end
end
fprintf('\nH1 prediction (y2 phantom, trough): 0.5*a''''*P = %.3e (P=0.044^2) / %.3e (P=0.022^2)\n', ...
        -0.5*0.2343*0.044^2, -0.5*0.2343*0.022^2);
fprintf('scaling check: phantom(p1)/phantom(p2) should be ~4 for curvature terms\n');
save(fullfile(proj, 'test_results', 'check_formB_curvature_phantom.mat'), 'res', '-v7.3');
fprintf('saved: test_results/check_formB_curvature_phantom.mat\n');
