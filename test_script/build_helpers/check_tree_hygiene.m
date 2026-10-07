% CHECK_TREE_HYGIENE  End-of-session check of the work tree against the repository rules (2026-10-04).
%
%   r = check_tree_hygiene()                       report only, nothing is moved or staged
%   r = check_tree_hygiene('move_orphans', true)   also move unreferenced untracked figures to test_results/figs_unused/
%   r = check_tree_hygiene('exclude', 'noap|fig_noap')   hide paths matching a regexp (e.g. the other session's files)
%
%   Checks (rules: CLAUDE.md core rules 4-5, .claude/rules/matlab-conventions.md, dual-machine-workflow.md item 9)
%     [HARD] every tracked .tex includes only tracked figures (a committed tex must rebuild from the repository).
%            The 2026-10-03 commit round missed 35 figures included as "figures/<name>.png"; this is that check.
%     [INFO] untracked figures in a figures/ folder that no .tex includes and the ledger does not name -> orphans
%            (regenerable; belong in test_results/figs_unused/, which is gitignored).
%     [INFO] untracked scripts under test_script/ with who refers to them (documents, ledger, other scripts):
%            referred -> commit with its document; not referred -> scratch triage (archive or keep with a FORK header).
%     [INFO] untracked files in reference/**/data/ that are not CSV (third-party files do not belong in the repo).
%     [INFO] days since the last commit and the size of the uncommitted tree.
%   r.pass is true when the HARD check passes. Prints a summary; returns the lists in r.
function r = check_tree_hygiene(varargin)
    ip = inputParser;
    ip.addParameter('move_orphans', false, @islogical);
    ip.addParameter('exclude', '', @ischar);
    ip.parse(varargin{:});
    opt = ip.Results;
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    old = cd(root);  cleanup = onCleanup(@() cd(old));

    tracked   = local_git('ls-files');
    untracked = local_git('ls-files --others --exclude-standard');
    modified  = local_git('diff --name-only');
    if ~isempty(opt.exclude)
        keep = @(c) c(cellfun(@isempty, regexp(c, opt.exclude, 'once')));
        untracked = keep(untracked);  modified = keep(modified);
    end
    is_tracked = containers.Map(tracked, true(size(tracked)));

    % ---- [HARD] tracked tex -> tracked figures
    tex_tr  = tracked(endsWith(tracked, '.tex'));
    tex_all = [tex_tr; untracked(endsWith(untracked, '.tex'))];
    missing = {};  missing_disk = {};  archive_broken = {};  included = containers.Map();
    for i = 1:numel(tex_all)
        f = tex_all{i};  d = fileparts(f);  [incs, gpath] = local_includes(f);
        for j = 1:numel(incs)
            cands = local_candidates(d, incs{j}, gpath);
            for c = 1:numel(cands); included(cands{c}) = true; end
            if ~any(strcmp(f, tex_tr)); continue; end
            if any(cellfun(@(c) isKey(is_tracked, c), cands)); continue; end
            if contains(f, '/archive/')                                      % history (CLAUDE.md rule 3): reported, never fails the check
                archive_broken{end + 1, 1} = sprintf('%s  <-  %s', incs{j}, f); continue; %#ok<AGROW>
            end
            if any(cellfun(@(c) exist(c, 'file') == 2, cands))
                missing{end + 1, 1} = sprintf('%s  <-  %s', cands{find(cellfun(@(c) exist(c, 'file') == 2, cands), 1)}, f); %#ok<AGROW>
            else
                missing_disk{end + 1, 1} = sprintf('%s  <-  %s', incs{j}, f); %#ok<AGROW>
            end
        end
    end

    % ---- [INFO] orphan figures
    ledger = '';
    for lf = {'reference/eq17_analysis/hypotheses_formC_b.md', 'agent_docs/eq17/mainline-gainlaw.md'}
        if exist(lf{1}, 'file') == 2; ledger = [ledger fileread(lf{1})]; end %#ok<AGROW>
    end
    figs = untracked(contains(untracked, '/figures/') & endsWith(untracked, {'.png', '.pdf', '.jpg'}));
    orph = {};
    for i = 1:numel(figs)
        [~, nm] = fileparts(figs{i});
        if isKey(included, figs{i}) || contains(ledger, nm); continue; end
        orph{end + 1, 1} = figs{i}; %#ok<AGROW>
    end
    if opt.move_orphans && ~isempty(orph)
        dest = fullfile('test_results', 'figs_unused');  if ~exist(dest, 'dir'); mkdir(dest); end
        for i = 1:numel(orph); movefile(orph{i}, dest); end
        fid = fopen(fullfile(dest, sprintf('MOVED_%s.txt', datestr(now, 'yyyy-mm-dd_HHMM'))), 'w');  fprintf(fid, '%s\n', orph{:});  fclose(fid); %#ok<TNOW1,DATST>
    end

    % ---- [INFO] untracked scripts and who refers to them
    scr = untracked(startsWith(untracked, 'test_script/') & endsWith(untracked, {'.m', '.py'}));
    src = [tex_all; tracked(startsWith(tracked, 'test_script/') & endsWith(tracked, '.m'))];
    parts = cellfun(@fileread, src, 'UniformOutput', false);             % one join, not 600 growing copies
    corpus = [ledger, strjoin(parts', newline)];
    scr_ref = {};  scr_unref = {};
    for i = 1:numel(scr)
        [~, nm] = fileparts(scr{i});
        if contains(corpus, nm); scr_ref{end + 1, 1} = scr{i}; else; scr_unref{end + 1, 1} = scr{i}; end %#ok<AGROW>
    end

    % ---- [INFO] third-party files in data folders
    data3 = untracked(contains(untracked, '/data/') & ~endsWith(untracked, '.csv'));

    % ---- [INFO] age and size
    last = local_git('log -1 --format=%ct');  age_d = (posixtime(datetime('now', 'TimeZone', 'UTC')) - str2double(strtrim(last{end}))) / 86400;

    r = struct('pass', isempty(missing) && isempty(missing_disk), 'tex_untracked_figures', {missing}, ...
               'tex_missing_on_disk', {missing_disk}, 'archive_tex_unresolved', {archive_broken}, 'orphan_figures', {orph}, 'scripts_referenced', {scr_ref}, ...
               'scripts_unreferenced', {scr_unref}, 'data_third_party', {data3}, 'days_since_commit', age_d, ...
               'n_untracked', numel(untracked), 'n_modified', numel(modified));

    fprintf('\n== check_tree_hygiene (%s)\n', root);
    fprintf('[HARD] tracked tex including untracked figures: %d   (included figure missing on disk: %d)  -> %s\n', ...
            numel(missing), numel(missing_disk), local_pf(r.pass));
    local_list(missing, 12);  local_list(missing_disk, 12);
    fprintf('[INFO] archived tex whose figures do not resolve (history, not a failure): %d\n', numel(archive_broken));
    fprintf('[INFO] orphan figures (untracked, no tex includes them, ledger does not name them): %d%s\n', numel(orph), ...
            local_if(opt.move_orphans && ~isempty(orph), ' -> moved to test_results/figs_unused/', ''));
    if ~opt.move_orphans; local_list(orph, 8); end
    fprintf('[INFO] untracked scripts: referred to by a document / ledger / tracked script %d (commit them), not referred %d (triage)\n', ...
            numel(scr_ref), numel(scr_unref));
    local_list(scr_ref, 8);  local_list(scr_unref, 8);
    fprintf('[INFO] third-party files in data folders (should be gitignored): %d\n', numel(data3));  local_list(data3, 5);
    fprintf('[INFO] %.1f days since the last commit; %d untracked, %d modified%s\n', age_d, numel(untracked), numel(modified), ...
            local_if(isempty(opt.exclude), '', sprintf(' (excluding /%s/)', opt.exclude)));
end

function out = local_git(args)
    [st, txt] = system(['git --no-pager -c color.ui=false ' args]);   % no pager: MATLAB's system() has a tty, so git would page and add escape codes
    if st ~= 0; error('check_tree_hygiene:git', 'git %s failed: %s', args, txt); end
    out = splitlines(strtrim(txt));  out = out(~cellfun(@isempty, out));
end

function [incs, gpath] = local_includes(f)
    s = fileread(f);  s = regexprep(s, '(?<!\\)%[^\n]*', '');          % drop comments to end of line (not \%; MATLAB has no (?m))
    t = regexp(s, '\\includegraphics\s*(?:\[[^\]]*\])?\s*\{([^}]+)\}', 'tokens');
    incs = cellfun(@(c) strtrim(c{1}), t, 'UniformOutput', false);
    incs = incs(~contains(incs, '#'));                                   % macro arguments
    g = regexp(s, '\\graphicspath\s*\{((?:\{[^}]*\})+)\}', 'tokens', 'once');
    gpath = {};  if ~isempty(g); gpath = regexp(g{1}, '\{([^}]*)\}', 'tokens'); gpath = cellfun(@(c) c{1}, gpath, 'UniformOutput', false); end
end

function c = local_candidates(d, g, gpath)
    dirs = [{''}, {'figures/'}, {'../figures/'}, gpath];                % ../figures/: tex \input from a subfolder (motion_ctrl/*.tex) resolves against the parent's figures/ (10-07)
    base = cellfun(@(p) [d '/' p g], dirs, 'UniformOutput', false);
    [~, ~, ext] = fileparts(g);
    if isempty(ext); base = [strcat(base, '.png'), strcat(base, '.pdf'), strcat(base, '.jpg')]; end
    c = cellfun(@local_norm, base, 'UniformOutput', false);
end

function p = local_norm(p)                                                % collapse '//', './' and 'dir/..' so paths match git ls-files
    parts = strsplit(strrep(p, '\', '/'), '/');  out = {};
    for k = 1:numel(parts)
        if isempty(parts{k}) || strcmp(parts{k}, '.'); continue; end
        if strcmp(parts{k}, '..') && ~isempty(out) && ~strcmp(out{end}, '..'); out(end) = []; else; out{end + 1} = parts{k}; end %#ok<AGROW>
    end
    p = strjoin(out, '/');
end

function local_list(c, n)
    for i = 1:min(n, numel(c)); fprintf('    %s\n', c{i}); end
    if numel(c) > n; fprintf('    ... (%d more)\n', numel(c) - n); end
end

function s = local_pf(p); if p; s = 'PASS'; else; s = 'FAIL'; end; end
function s = local_if(c, a, b); if c; s = a; else; s = b; end; end
