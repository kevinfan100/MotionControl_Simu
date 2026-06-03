function out_path = save_figure(fig, name, style)
%SAVE_FIGURE  Export figure with consistent DPI / path / naming.
%
%   out_path = save_figure(fig, name, style)
%
%   Saves to:
%       <style.save.dir>/<style.save.prefix><name>.<style.save.format>
%   at <style.save.dpi> resolution. Creates the directory if missing.
%
%   Returns the resolved absolute path of the saved file.

    if ~exist(style.save.dir, 'dir')
        mkdir(style.save.dir);
    end

    file_name = sprintf('%s%s.%s', style.save.prefix, name, style.save.format);
    out_path  = fullfile(style.save.dir, file_name);

    exportgraphics(fig, out_path, 'Resolution', style.save.dpi);

end
