% STATUS: ACTIVE (scratch) | PURPOSE: bundle the readout-chain figures of
%   2026-08-25/26 into one multi-page PDF for the record. Each page is one PNG,
%   placed on its own axes at its native aspect ratio.
function make_readout_pdf(files, outfile)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(files)
        % Page order of record (2026-08-26): chain -> step-5 histogram ->
        % two-arm chain -> house comparison page (seed 7) -> y2 ledger ->
        % seed-averaged arm performance.
        files = {'dhm_to_ahm.png', 'hist_step5_t1_4.png', ...
                 'dhm_to_ahm_arms_both.png', 'arms_pair_s007.png', ...
                 'y2_contribution.png', 'arms_performance.png'};
    end
    if nargin < 2 || isempty(outfile)
        outfile = fullfile(od, 'readout_chain_record.pdf');
    end
    if exist(outfile, 'file'); delete(outfile); end

    for k = 1:numel(files)
        fn = fullfile(od, files{k});
        assert(exist(fn,'file')==2, 'missing: %s', fn);
        I = imread(fn);
        [h, w, ~] = size(I);
        f = figure('Color','w','Visible','off','Units','pixels', ...
                   'Position', [10 10 min(w,1600) min(w,1600)*h/w]);
        a = axes(f, 'Position', [0 0 1 1]);
        imshow(I, 'Parent', a, 'Border', 'tight');
        exportgraphics(f, outfile, 'Append', k > 1, 'Resolution', 200);
        close(f);
        fprintf('  page %d : %-30s %5d x %5d\n', k, files{k}, w, h);
    end
    d = dir(outfile);
    fprintf('\nPDF -> %s   (%d pages, %.1f MB)\n', outfile, numel(files), d.bytes/1e6);
end
