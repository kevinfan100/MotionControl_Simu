function [fig, tl] = new_stacked_figure(n_panels, style, opts)
%NEW_STACKED_FIGURE  Create a tall stacked time-series figure.
%
%   [fig, tl] = new_stacked_figure(n_panels, style)
%   [fig, tl] = new_stacked_figure(n_panels, style, opts)
%
%   Returns a figure with a tiledlayout(n_panels, 1) ready for stacked
%   time-series. Caller uses nexttile to fill panels and shares the x-axis
%   via linkaxes at the end.
%
%   opts (optional struct):
%       .figure_size  [w h] in inches (default style.layout.figure_size_stacked)
%       .name         figure name (default '')
%       .visible      'on' / 'off' (default 'on')

    if nargin < 3 || isempty(opts), opts = struct(); end
    if ~isfield(opts, 'figure_size') || isempty(opts.figure_size)
        opts.figure_size = style.layout.figure_size_stacked;
    end
    if ~isfield(opts, 'name'),    opts.name    = ''; end
    if ~isfield(opts, 'visible'), opts.visible = 'on'; end

    fig = figure('Units', 'inches', ...
                 'Position', [1, 1, opts.figure_size(1), opts.figure_size(2)], ...
                 'Color', 'w', ...
                 'Name', opts.name, ...
                 'Visible', opts.visible);

    tl = tiledlayout(fig, n_panels, 1, ...
                     'Padding',     style.layout.tile_padding, ...
                     'TileSpacing', style.layout.tile_spacing);

end
