% STATUS: ACTIVE (scratch) | PURPOSE: bundle the readout-chain figures of
%   2026-08-25/26 into one multi-page PDF for the record, one PNG per page at
%   its NATIVE pixel size.
%
%   Route 1 (default): python3 + Pillow embeds each PNG losslessly -- no
%   figure, no imshow, no re-rasterisation. The first version of this script
%   drew each PNG with imshow inside a figure capped at 1600 px and exported
%   at 200 dpi, i.e. a 2500 px figure was shrunk to 63 % and blown back up:
%   two resamplings, visibly blurred (2026-08-26).
%   Route 2 (fallback, no Pillow): figure at the PNG's own pixel size and
%   exportgraphics at 2x screen DPI, so the only resampling is a 2x upsample.
function make_readout_pdf(files, outfile)

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    if nargin < 1 || isempty(files)
        % Page order of record (2026-08-26): chain -> step-5 histogram ->
        % two-arm chain -> house comparison page (seed 7) -> y2 ledger.
        % (arms_performance.png was page 6 for one build and dropped again
        % the same day; pass it in `files` to include it.)
        files = {'dhm_to_ahm.png', 'hist_step5_t1_4.png', ...
                 'dhm_to_ahm_arms_both.png', 'arms_pair_s007.png', ...
                 'y2_contribution.png'};
    end
    if nargin < 2 || isempty(outfile)
        outfile = fullfile(od, 'readout_chain_record.pdf');
    end
    paths = cellfun(@(f) fullfile(od, f), files, 'UniformOutput', false);
    for k = 1:numel(paths)
        assert(exist(paths{k}, 'file') == 2, 'missing: %s', paths{k});
    end
    if exist(outfile, 'file'); delete(outfile); end

    % ---- Route 1: lossless embed via Pillow -----------------------------
    listfile = [tempname '.txt'];
    fid = fopen(listfile, 'w'); fprintf(fid, '%s\n', paths{:}); fclose(fid);
    py = ['import sys; from PIL import Image; ' ...
          'fs=[l.strip() for l in open(sys.argv[1]) if l.strip()]; ' ...
          'ims=[Image.open(f).convert("RGB") for f in fs]; ' ...
          'ims[0].save(sys.argv[2], "PDF", resolution=150.0, save_all=True, append_images=ims[1:]); ' ...
          '[print("  page %d : %-30s %5d x %5d" % (i+1, fs[i].split("/")[-1], im.width, im.height)) for i,im in enumerate(ims)]'];
    [st, out] = system(sprintf('python3 -c ''%s'' "%s" "%s"', py, listfile, outfile));
    delete(listfile);
    if st == 0
        fprintf('%s', out);
    else
        % ---- Route 2: MATLAB rasterisation at native size ---------------
        warning('make_readout_pdf:noPillow', ...
                'python3/Pillow route failed (%s); falling back to exportgraphics.', strtrim(out));
        dpi = get(0, 'ScreenPixelsPerInch');
        for k = 1:numel(paths)
            I = imread(paths{k});  [h, w, ~] = size(I);
            f = figure('Color', 'w', 'Visible', 'off', 'Units', 'pixels', ...
                       'Position', [10 10 w h]);
            a = axes(f, 'Position', [0 0 1 1]);
            image(a, I); axis(a, 'image', 'off');
            exportgraphics(f, outfile, 'Append', k > 1, 'Resolution', 2*dpi);
            close(f);
            fprintf('  page %d : %-30s %5d x %5d\n', k, files{k}, w, h);
        end
    end
    d = dir(outfile);
    fprintf('\nPDF -> %s   (%d pages, %.1f MB)\n', outfile, numel(files), d.bytes/1e6);
end
