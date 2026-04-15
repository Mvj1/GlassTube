function stageTimings = pj4_debug(~, debugData)
stageTimings = table('Size', [0, 2], ...
    'VariableTypes', {'string', 'double'}, ...
    'VariableNames', {'Stage', 'Seconds'});

if isfield(debugData, 'roiGray') && isfield(debugData, 'topMask') && isfield(debugData, 'botMask')
    tStage = tic;
    show_side_edges_debug(debugData.roiGray, debugData.topMask, debugData.botMask);
    stageTimings = append_stage_timing(stageTimings, 'show_side_edges', toc(tStage));
end

if isfield(debugData, 'leftEndData') && isfield(debugData.leftEndData, 'debug')
    tStage = tic;
    show_endface_debug(debugData.leftEndData);
    stageTimings = append_stage_timing(stageTimings, 'show_left_endface', toc(tStage));
end

if isfield(debugData, 'rightEndData') && isfield(debugData.rightEndData, 'debug')
    tStage = tic;
    show_endface_debug(debugData.rightEndData);
    stageTimings = append_stage_timing(stageTimings, 'show_right_endface', toc(tStage));
end

if isfield(debugData, 'rodResult')
    tStage = tic;
    show_rod_fit_debug(debugData.rodResult);
    stageTimings = append_stage_timing(stageTimings, 'show_rod_fit', toc(tStage));
end

end


function stageTimings = append_stage_timing(stageTimings, stageName, elapsedSeconds)
newRow = table(string(stageName), elapsedSeconds, ...
    'VariableNames', stageTimings.Properties.VariableNames);
stageTimings = [stageTimings; newRow];
end


function show_side_edges_debug(roiGray, topMask, botMask)
figure('Name', '侧视边缘提取', 'Units', 'normalized', 'Position', [0.1, 0.08, 0.85, 0.8]);
imshow(roiGray, []);
hold on;
[topY, topX] = find(topMask);
[botY, botX] = find(botMask);
plot(topX, topY, 'g.', 'MarkerSize', 2);
plot(botX, botY, 'b.', 'MarkerSize', 2);
title('上下包络');
legend('上边缘', '下边缘');
end


function show_endface_debug(endData)
dbg = endData.debug;

figure('Name', [endData.label, '端面边缘']);
imshow(dbg.edgeMask);

figure('Name', [endData.label, '内轮廓']);
imshow(dbg.innerMask);

figure('Name', [endData.label, '外轮廓']);
imshow(dbg.outerMask);

if isfield(dbg, 'wall') && ~isempty(dbg.wall)
    show_endface_fit_debug(dbg.img, endData.innerFit, endData.outerFit, dbg.wall, endData.label);
end
end


function show_endface_fit_debug(img, innerFit, outerFit, wall, endLabel)
figure('Name', [endLabel, '壁厚曲线']);
plot(wall.ang, wall.thick);
title([endLabel, '端面圆心角与壁厚关系']);
xlabel('圆心角/rad');
ylabel('壁厚/pixels');

figure('Name', [endLabel, '端面拟合结果']);
imshow(img);
hold on;
rectangle('Position', [innerFit.cx - innerFit.r, innerFit.cy - innerFit.r, 2 * innerFit.r, 2 * innerFit.r], ...
    'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
text(innerFit.cx, innerFit.cy, ...
    ['拟合内圆圆心(', num2str(innerFit.cx), ',', num2str(innerFit.cy), ') 直径=', num2str(innerFit.r * 2), ' pixels'], ...
    'FontSize', 15, 'Color', 'r');

rectangle('Position', [outerFit.cx - outerFit.r, outerFit.cy - outerFit.r, 2 * outerFit.r, 2 * outerFit.r], ...
    'Curvature', [1, 1], 'EdgeColor', 'y', 'LineWidth', 2);
text(outerFit.cx, outerFit.cy + 150, ...
    ['拟合外圆圆心(', num2str(outerFit.cx), ',', num2str(outerFit.cy), ') 直径=', num2str(outerFit.r * 2), ' pixels'], ...
    'FontSize', 15, 'Color', 'y');

line([wall.maxP1(2), wall.maxP2(2)], [wall.maxP1(1), wall.maxP2(1)], 'LineWidth', 4, 'Color', 'g');
line([wall.minP1(2), wall.minP2(2)], [wall.minP1(1), wall.minP2(1)], 'LineWidth', 4, 'Color', 'c');
hold off;
end


function show_rod_fit_debug(rodResult)
cavity = rodResult.cavity;
opt = rodResult.opt;
centerLine = rodResult.centerLine;
rodTop = rodResult.rodTop;
rodBot = rodResult.rodBot;
flatTop = rodResult.flatTop;
flatBot = rodResult.flatBot;
rodDia = rodResult.rodDia;
limitLabel = rodResult.limitLabel;

figure('Name', 'Rod insertion analysis', 'Units', 'normalized', 'Position', [0.08, 0.08, 0.84, 0.82]);

subplot(2, 1, 1);
hold on;
plot(cavity.x, cavity.outerTop, 'Color', [0.55, 0.55, 0.55], 'LineWidth', 1);
plot(cavity.x, cavity.outerBot, 'Color', [0.55, 0.55, 0.55], 'LineWidth', 1);
plot(cavity.x, cavity.innerTop, 'w-', 'LineWidth', 1.5);
plot(cavity.x, cavity.innerBot, 'w-', 'LineWidth', 1.5);
plot(cavity.x, centerLine, 'c--', 'LineWidth', 1.5);
plot(cavity.x, rodTop, 'g-', 'LineWidth', 1.2);
plot(cavity.x, rodBot, 'g-', 'LineWidth', 1.2);
plot([cavity.x(opt.topTouchIdx); cavity.x(opt.botTouchIdx)], ...
    [cavity.innerTop(opt.topTouchIdx); cavity.innerBot(opt.botTouchIdx)], ...
    'rx', 'MarkerSize', 10, 'LineWidth', 2);
title(sprintf('2D cavity model and optimal straight rod (diameter = %.2f mm)', rodDia));
legend('Outer top', 'Outer bottom', 'Estimated inner top', 'Estimated inner bottom', ...
    'Optimal rod axis', 'Rod top', 'Rod bottom', 'Limiting points');
axis equal;
grid on;
set(gca, 'YDir', 'reverse');
xlabel('Tube length (mm)');
ylabel('Height (mm)');

subplot(2, 1, 2);
hold on;
fill([cavity.x; flipud(cavity.x)], [flatTop; flipud(flatBot)], [0.3, 0.3, 0.9], 'EdgeColor', 'none');
plot(cavity.x, flatTop, 'w-', 'LineWidth', 1.2);
plot(cavity.x, flatBot, 'w-', 'LineWidth', 1.2);
rodArea = fill([cavity.x(1); cavity.x(end); cavity.x(end); cavity.x(1)], ...
    [-rodDia / 2; -rodDia / 2; rodDia / 2; rodDia / 2], 'g', 'FaceAlpha', 0.35, 'EdgeColor', 'g');
plot([cavity.x(opt.topTouchIdx); cavity.x(opt.botTouchIdx)], ...
    [flatTop(opt.topTouchIdx); flatBot(opt.botTouchIdx)], ...
    'rx', 'MarkerSize', 10, 'LineWidth', 2);
title(sprintf('Flattened 2D clearance relative to optimal axis (%s-limited)', limitLabel));
ylabel('Relative offset (mm)');
xlabel('Tube length (mm)');
yline(0, 'k--');
legend(rodArea, 'Rod envelope');
grid on;
set(gca, 'YDir', 'reverse');
end
