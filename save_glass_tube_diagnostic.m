function save_glass_tube_diagnostic(stripImg, result, cfg, outputPath)
%SAVE_GLASS_TUBE_DIAGNOSTIC Save a compact visual verification report.
side = result.side;
rod = result.rod;
rect = side.roiRect;
x1 = max(1, floor(rect(1)) + 1);
y1 = max(1, floor(rect(2)) + 1);
x2 = min(size(stripImg, 2), x1 + round(rect(3)) - 1);
y2 = min(size(stripImg, 1), y1 + round(rect(4)) - 1);
roi = stripImg(y1:y2, x1:x2, :);
if ndims(roi) == 3, roi = im2gray(roi); end

maxDisplayCols = 2600;
displayScale = min(1, maxDisplayCols / size(roi, 2));
roiSmall = imresize(roi, displayScale, 'bilinear');
xPlot = (1:numel(side.topPx)) * displayScale;
topPlot = side.topPx * displayScale;
botPlot = side.bottomPx * displayScale;

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 1800 1100]);
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile([1 2]);
imshow(roiSmall, [], 'Parent', ax1); hold(ax1, 'on');
plot(ax1, xPlot, topPlot, 'r-', 'LineWidth', 1.4);
plot(ax1, xPlot, botPlot, 'g-', 'LineWidth', 1.4);
unsupported = ~side.observed;
if any(unsupported)
    scatter(ax1, xPlot(unsupported), topPlot(unsupported), 5, [1 .65 0], 'filled');
    scatter(ax1, xPlot(unsupported), botPlot(unsupported), 5, [1 .65 0], 'filled');
end
title(ax1, sprintf('Canny inner-wall tracks — observed %.2f%%', 100 * side.quality.observedFraction));
xlabel(ax1, 'Stitched ROI column (display-scaled)'); ylabel(ax1, 'ROI row');
legend(ax1, {'Upper inner wall','Lower inner wall','Not directly observed'}, ...
    'Location','southoutside','Orientation','horizontal');

ax2 = nexttile;
plot(ax2, rod.axisXmm, rod.cavityTopMm, 'r-', 'LineWidth', 1.2); hold(ax2, 'on');
plot(ax2, rod.axisXmm, rod.cavityBottomMm, 'g-', 'LineWidth', 1.2);
plot(ax2, rod.axisXmm, rod.rodTopMm, 'k--', 'LineWidth', 1.5);
plot(ax2, rod.axisXmm, rod.rodBottomMm, 'k--', 'LineWidth', 1.5);
plot(ax2, rod.axisXmm, rod.axisYmm, 'b-', 'LineWidth', 1.0);
scatter(ax2, rod.topContactXmm, rod.cavityTopMm(rod.topContactIndex), 55, 'r', 'filled');
scatter(ax2, rod.bottomContactXmm, rod.cavityBottomMm(rod.bottomContactIndex), 55, 'g', 'filled');
grid(ax2, 'on'); axis(ax2, 'tight');
xlabel(ax2, 'Axial position (mm)'); ylabel(ax2, 'Transverse position (mm)');
title(ax2, sprintf('Maximum conservative rod: %.3f mm', rod.diameterMm));
legend(ax2, {'Cavity top','Cavity bottom','Rod top','Rod bottom','Rod axis','Top contact','Bottom contact'}, ...
    'Location','best');

ax3 = nexttile; axis(ax3, 'off');
lines = {
    sprintf('Maximum rod diameter: %.3f mm', result.maximumRodDiameterMm)
    sprintf('Axis slope: %.8f mm/mm', rod.axisSlope)
    sprintf('Active limit: %s', rod.activeLimit)
    sprintf('Left conservative end diameter: %.3f mm', result.leftEnd.conservativeDiameterMm)
    sprintf('Right conservative end diameter: %.3f mm', result.rightEnd.conservativeDiameterMm)
    sprintf('Side direct Canny support: %.2f %%', 100 * side.quality.observedFraction)
    sprintf('Median side confidence: %.3f', side.quality.medianConfidence)
    sprintf('Stitch peak min/median: %.4f / %.4f', min(result.stitch.quality.peak), median(result.stitch.quality.peak))
    sprintf('Stitch PSR min/median: %.3f / %.3f', min(result.stitch.quality.psr), median(result.stitch.quality.psr))
    sprintf('Processing time: %.3f s', result.timing.total)
    sprintf('Scale: %.6f side px/mm, %.6f end px/mm', cfg.scale.sidePxPerMm, cfg.scale.endPxPerMm)
    };
text(ax3, 0.02, 0.98, strjoin(lines, newline), 'VerticalAlignment','top', ...
    'FontName','Consolas','FontSize',12, 'Interpreter','none');
title(ax3, 'Quality and traceability');

sgtitle(fig, 'Glass-tube maximum passage analysis', 'FontWeight','bold', 'FontSize',17);
exportgraphics(fig, outputPath, 'Resolution', 150);
close(fig);
end
