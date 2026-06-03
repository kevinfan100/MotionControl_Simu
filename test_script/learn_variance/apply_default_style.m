function apply_default_style(style)
%APPLY_DEFAULT_STYLE  Push style into MATLAB groot defaults for the session.
%
%   apply_default_style(style)
%
%   Sets root-level defaults so subsequent plot calls inherit consistent
%   font, line width, and grid settings without per-call boilerplate.
%
%   Call once at the top of an analysis script:
%       style = plot_style();
%       apply_default_style(style);

    set(groot, 'defaultAxesFontName',     style.font.family);
    set(groot, 'defaultAxesFontSize',     style.font.tick_size);
    set(groot, 'defaultAxesLabelFontSizeMultiplier', ...
               style.font.label_size / style.font.tick_size);
    set(groot, 'defaultAxesTitleFontSizeMultiplier', ...
               style.font.title_size / style.font.tick_size);
    set(groot, 'defaultAxesTitleFontWeight', 'normal');

    set(groot, 'defaultTextFontName',     style.font.family);
    set(groot, 'defaultTextFontSize',     style.font.label_size);

    set(groot, 'defaultLegendFontName',   style.font.family);
    set(groot, 'defaultLegendFontSize',   style.font.legend_size);
    set(groot, 'defaultLegendBox',        'off');

    set(groot, 'defaultLineLineWidth',    style.lw.signal);

    set(groot, 'defaultAxesXGrid',        'on');
    set(groot, 'defaultAxesYGrid',        'on');
    set(groot, 'defaultAxesGridAlpha',    0.25);
    set(groot, 'defaultAxesGridLineStyle', '-');
    set(groot, 'defaultAxesLineWidth',    style.lw.gridline);
    set(groot, 'defaultAxesBox',          'on');

end
