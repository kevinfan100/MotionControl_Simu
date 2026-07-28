% TEMP checkcode runner for the dither driver (chat 2026-07-21).
here = fileparts(mfilename('fullpath'));
proj = fileparts(here);
addpath(genpath(fullfile(proj, 'model')));
addpath(here);
f = fullfile(here, 'temp_run_pure_sim_dither.m');
msgs = checkcode(f, '-struct');
if isempty(msgs)
    fprintf('CHECKCODE: 0 issues\n');
else
    fprintf('CHECKCODE: %d issue(s)\n', numel(msgs));
    for i = 1:numel(msgs)
        fprintf('  L%d: %s\n', msgs(i).line, msgs(i).message);
    end
end
