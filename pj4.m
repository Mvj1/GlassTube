%% 玻璃管图像处理与三维重建
% 顶部相机移速900px/5s，顶部相机缩放比例1060px/30mm，侧边相机缩放比例2427px/30mm

%% 待办：
% 1.定标采用canny算子对标准块分析的方法；
% 3.端面贴图在建模两端的上下左右镜像方向可能存在错误；4.包络图可以采用膨胀腐蚀法或骨架法去除拼接带来的段落痕迹

%% 初始化
clear; clc; close all;

set(groot, 'defaultFigureUnits', 'normalized');
set(groot, 'defaultFigurePosition', [0, 0, 1, 1]);
set(groot, 'defaultFigureWindowState', 'maximized');

cfg = get_cfg();

%% 多图互相关拼接
[stripImg, stepList] = stitch_strip(cfg);
% show_strip(stripImg, stepList, cfg);
imwrite(stripImg, cfg.file.strip);
% fprintf('拼接结果已保存为 %s\n', cfg.file.strip);

%% 侧视边缘提取
[roiGray, topMask, botMask, pathTbl] = extract_side_edges_robust(cfg);
show_side_edges(roiGray, topMask, botMask);
% imwrite(topMask | botMask, cfg.file.side);
writetable(pathTbl, cfg.file.path);
% fprintf('侧视边缘已保存为 %s\n', cfg.file.side);
% fprintf('侧视路径数据已保存为 %s\n', cfg.file.path);

%% 端面轮廓与壁厚分析
% ensure_right_end_image(cfg);
leftEndData = analyze_endface(cfg.end.left, cfg);
rightEndData = analyze_endface(cfg.end.right, cfg);
% warn_scale_consistency(pathTbl, leftEndData, rightEndData, cfg);

%% 玻璃管三维重建
build_tube_model(cfg, pathTbl, leftEndData, rightEndData);

%% 最大插入直径估算
estimate_rod_fit(pathTbl, leftEndData, rightEndData, cfg);

%% 辅助函数
function cfg = get_cfg()
cfg.dir.img = './rare data/3052-1big2935';
cfg.dir.ext = '*.bmp';

cfg.step.nominal = 900;
cfg.step.search = 40;
cfg.rot.angle = -0.39;
cfg.debug = false;

cfg.canny.sigma = 1.5;
cfg.canny.th = [0.15, 0.25];
cfg.canny.low = 0.1;
cfg.canny.high = 0.2;
cfg.canny.iter = 60;

cfg.scale.sidePxPerMm = 1060 / 30;
cfg.scale.endPxPerMm = 2427 / 30;
cfg.warn.scaleRelTol = 0.10;

cfg.file.strip = 'tube_strip.png';
cfg.file.side = 'side_edges.png';
cfg.file.path = 'tube_path.csv';
cfg.file.rod = 'rod_fit.png';
cfg.file.sideCalib = 'side_edge_calibration.mat';

cfg.sideEdge.forceRecalibrate = false;
cfg.sideEdge.baselineTop = [];
cfg.sideEdge.baselineBot = [];
cfg.sideEdge.bgSigma = 25;
cfg.sideEdge.edgeSigma = 1.2;
cfg.sideEdge.searchOutsidePx = 18;
cfg.sideEdge.searchInsidePx = 36;
cfg.sideEdge.edgeWindowPx = 4;
cfg.sideEdge.bandOffsetMinPx = 4;
cfg.sideEdge.bandOffsetMaxPx = 24;
cfg.sideEdge.allowBandWithoutValley = false;
cfg.sideEdge.strongEdgeRatio = 0.35;
cfg.sideEdge.calibColumnStep = 8;
cfg.sideEdge.invalidScore = -2.5;
cfg.sideEdge.candidateNeighborPenalty = 0.20;
cfg.sideEdge.maxJumpPerCol = 5;
cfg.sideEdge.jumpPenalty = 0.08;
cfg.sideEdge.smoothPenalty = 0.18;
cfg.sideEdge.minCandidateScore = 0.05;
cfg.sideEdge.minConfidence = 0.35;
cfg.sideEdge.maxGapToInterp = 80;

cfg.sideEdge.fit.method = 'weighted-pchip-rloess';
cfg.sideEdge.fit.smoothTop = 31;
cfg.sideEdge.fit.smoothBot = 45;
cfg.sideEdge.fit.minSupportCols = 25;
cfg.sideEdge.fit.maxGapFitOnly = 80;
cfg.sideEdge.fit.supportEvidenceThr = 0.35;
cfg.sideEdge.fit.lowEvidenceThr = 0.20;
cfg.sideEdge.fit.normalFitWeightMin = 0.02;
cfg.sideEdge.fit.rawResidualTolPx = 1.5;
cfg.sideEdge.fit.bridgeResidualTolPx = 4.0;
cfg.sideEdge.fit.blendFitWeightCap = 0.65;
cfg.sideEdge.fit.inwardGuardTolPx = 2.0;
cfg.sideEdge.fit.inwardOutlierWindow = 21;
cfg.sideEdge.fit.inwardOutlierTolPx = 4.0;
cfg.sideEdge.fit.inwardOutlierConfMax = 0.90;
cfg.sideEdge.fit.inwardOutlierMinNeighbors = 8;
cfg.sideEdge.fit.topDiameterWindow = 61;
cfg.sideEdge.fit.topDiameterShrinkTolPx = 1.5;
cfg.sideEdge.fit.topDiameterMinNeighbors = 14;
cfg.sideEdge.fit.topDiameterBlendWeightMin = 0.50;
cfg.sideEdge.fit.diameterGuardWindow = 61;
cfg.sideEdge.fit.diameterGuardTolPx = 1.5;
cfg.sideEdge.fit.suspiciousRunMinLen = 5;

cfg.sideEdge.marker.darkThr = 0.18;
cfg.sideEdge.marker.madScale = 3.0;
cfg.sideEdge.marker.structRadius = 7;
cfg.sideEdge.marker.minArea = 60;
cfg.sideEdge.marker.edgeDilatePx = 9;
cfg.sideEdge.marker.localRejectPx = 6;
cfg.sideEdge.marker.columnCoverageThr = 0.30;
cfg.sideEdge.markerPenalty = 1.5;

cfg.sideEdge.top.firstEdgeMinDrop = 0.022;
cfg.sideEdge.top.valleyDepthThr = 0.035;
cfg.sideEdge.top.allowBandWithoutValley = true;
cfg.sideEdge.top.preferOuterEdgeOnlyGapPx = 2;
cfg.sideEdge.top.preferOuterScoreMargin = 0.020;
cfg.sideEdge.top.relaxedOuterDropRatio = 0.60;
cfg.sideEdge.top.relaxedOuterGradRatio = 0.18;
cfg.sideEdge.top.relaxedOuterMaxInwardPx = 12;
cfg.sideEdge.top.bandScoreValleyWeight = 0.45;
cfg.sideEdge.top.bandScoreStrongWeight = 0.12;
cfg.sideEdge.top.edgeOnlyStrongWeight = 0.10;
cfg.sideEdge.top.edgeOnlyLocalWeight = 0.20;
cfg.sideEdge.top.minOuterEvidence = 0.32;
cfg.sideEdge.top.outerBiasWeight = 0.18;
cfg.sideEdge.top.maxInwardOverridePx = 8;
cfg.sideEdge.top.bandEvidenceWeight = 0.28;
cfg.sideEdge.top.outerEvidenceOverrideMargin = 0.08;
cfg.sideEdge.top.bandSelectMargin = 0.04;

cfg.sideEdge.bottom.firstEdgeMinDrop = 0.016;
cfg.sideEdge.bottom.valleyDepthThr = 0.024;
cfg.sideEdge.bottom.allowBandWithoutValley = true;
cfg.sideEdge.bottom.relaxedOuterDropRatio = 0.75;
cfg.sideEdge.bottom.relaxedOuterGradRatio = 0.22;
cfg.sideEdge.bottom.relaxedOuterMaxInwardPx = 10;
cfg.sideEdge.bottom.minOuterEvidence = 0.30;
cfg.sideEdge.bottom.outerBiasWeight = 0.26;
cfg.sideEdge.bottom.maxInwardOverridePx = 8;
cfg.sideEdge.bottom.bandEvidenceWeight = 0.18;
cfg.sideEdge.bottom.outerEvidenceOverrideMargin = 0.08;
cfg.sideEdge.bottom.bandSelectMargin = 0.06;

cfg.end.left.label = '左端';
cfg.end.left.imgAbs = 'C:\Users\11603\Documents\MATLAB\GlassTube\40.12-30.49.bmp';
cfg.end.left.innerCsv = 'left_end_inner.csv';
cfg.end.left.outerCsv = 'left_end_outer.csv';

cfg.end.right.label = '右端';
cfg.end.right.imgAbs = 'C:\Users\11603\Documents\MATLAB\GlassTube\test.bmp';
cfg.end.right.innerCsv = 'right_end_inner.csv';
cfg.end.right.outerCsv = 'right_end_outer.csv';
end


function ensure_right_end_image(cfg)
if exist(cfg.end.right.imgAbs, 'file')
    return;
end

copyfile(cfg.end.left.imgAbs, cfg.end.right.imgAbs);
fprintf('已生成右端占位原图 %s\n', cfg.end.right.imgAbs);
end


function [stripImg, stepList] = stitch_strip(cfg)
files = dir(fullfile(cfg.dir.img, cfg.dir.ext));
[~, idx] = sort({files.name});
files = files(idx);

if numel(files) < 2
    error('需要至少两张图片。');
end

rotImg = @(img) imrotate(img, cfg.rot.angle, 'bicubic', 'crop');

% fprintf('开始拼接，预设旋转角度 %.2f°\n', cfg.rot.angle);

img0 = imread(fullfile(cfg.dir.img, files(1).name));
img0 = rotImg(img0);

[imgH, imgW, imgC] = size(img0);
cutW = cfg.step.nominal;
cutX = floor((imgW - cutW) / 2) + 1;
stripImg = img0(:, cutX:cutX + cutW - 1, :);

if imgC == 3
    prevGray = rgb2gray(img0);
else
    prevGray = img0;
end

stepList = zeros(numel(files) - 1, 1);

for k = 2:numel(files)
    img = imread(fullfile(cfg.dir.img, files(k).name));
    img = rotImg(img);

    if imgC == 3
        currGray = rgb2gray(img);
    else
        currGray = img;
    end

    roiRows = floor(imgH / 3):floor(2 * imgH / 3);
    tplX = floor(imgW / 2) - 200;
    tplW = 150;
    tpl = prevGray(roiRows, tplX:tplX + tplW);

    expX = tplX + cfg.step.nominal;
    findX1 = max(1, expX - cfg.step.search);
    findX2 = min(imgW, expX + tplW + cfg.step.search);
    findImg = currGray(roiRows, findX1:findX2);

    corrMap = normxcorr2(tpl, findImg);
    [peakY, peakX] = find(corrMap == max(corrMap(:)), 1);

    matchX = peakX - size(tpl, 2) + 1;
    matchX = findX1 + matchX - 1;
    dx = matchX - tplX;
    dy = peakY - size(tpl, 1);

    if abs(dx - cfg.step.nominal) > cfg.step.search + 10
        dx = cfg.step.nominal;
        dy = 0;
        warning('第 %d 张图匹配失败，改用默认步长。', k);
    end

    stepList(k - 1) = dx;

    cutW = round(dx);
    cutX1 = floor((imgW - cutW) / 2) + 1;
    cutX2 = cutX1 + cutW - 1;
    patch = img(:, cutX1:cutX2, :);

    if abs(dy) > 0
        patch = imtranslate(patch, [0, -dy], 'FillValues', 0);
    end

    stripImg = [patch, stripImg];
    prevGray = currGray;

    if cfg.debug
        fprintf('第 %02d 张图：步长 %d px，垂直补偿 %+d px\n', k, cutW, round(dy));
    end
end
end


function show_strip(stripImg, stepList, cfg)
figure('Name', '拼接结果', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.6]);

subplot(2, 1, 1);
imshow(stripImg);
title('拼接结果');
xlabel('像素');

subplot(2, 1, 2);
plot(stepList, '-o');
yline(cfg.step.nominal, '--r');
title('实际检测步长');
xlabel('图片序号');
ylabel('像素');
grid on;
end


function [roiGray, topMask, botMask, pathTbl] = extract_side_edges(cfg)
if ~exist(cfg.file.strip, 'file')
    error('未找到文件 %s，请先完成拼接。', cfg.file.strip);
end

img = imread(cfg.file.strip);

fig = figure('Name', 'ROI选择', 'NumberTitle', 'off');
imshow(img);
title('框选提取区域，双击选框内部确认', 'Color', 'r', 'FontSize', 12);
[roiImg, ~] = imcrop;
close(fig);

if isempty(roiImg)
    error('未选择侧视区域。');
end

if size(roiImg, 3) == 3
    roiGray = rgb2gray(roiImg);
else
    roiGray = roiImg;
end

roiGray = imgaussfilt(roiGray, cfg.canny.sigma);
edgeMask = edge(roiGray, 'Canny', cfg.canny.th);

[rowN, colN] = size(edgeMask);
topMask = false(rowN, colN);
botMask = false(rowN, colN);

for col = 1:colN
    rows = find(edgeMask(:, col));
    if isempty(rows)
        continue;
    end
    topMask(min(rows), col) = true;
    botMask(max(rows), col) = true;
end

topMask = bwareaopen(topMask, 30);
botMask = bwareaopen(botMask, 30);

pathTbl = table;
pathTbl.X = (1:colN)';
pathTbl.TopY = nan(colN, 1);
pathTbl.BotY = nan(colN, 1);

for col = 1:colN
    topRow = find(topMask(:, col), 1);
    botRow = find(botMask(:, col), 1);
    if ~isempty(topRow)
        pathTbl.TopY(col) = topRow;
    end
    if ~isempty(botRow)
        pathTbl.BotY(col) = botRow;
    end
end

pathTbl.Dia = pathTbl.BotY - pathTbl.TopY;
end


function show_side_edges(roiGray, topMask, botMask)
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


function endData = analyze_endface(endCfg, cfg)
img = imread(endCfg.imgAbs);
img = im2gray(img);
img = imopen(img, strel('disk', 8));

% figure('Name', [endCfg.label, '端面预处理']);
% imshow(img);

edgeMask = manual_canny(img, cfg.canny.low, cfg.canny.high, cfg.canny.iter);
figure('Name', [endCfg.label, '端面边缘']);
imshow(edgeMask);

innerMask = trace_inner_profile(edgeMask);
outerMask = trace_outer_profile(edgeMask);

figure('Name', [endCfg.label, '内轮廓']);
imshow(innerMask);

figure('Name', [endCfg.label, '外轮廓']);
imshow(outerMask);

writematrix(outerMask, endCfg.outerCsv);
writematrix(innerMask, endCfg.innerCsv);

innerFit = fit_circle_ls(innerMask);
outerFit = fit_circle_ls(outerMask);

wall = measure_wall_thickness(innerMask, outerMask, innerFit, outerFit);
% show_endface_result(img, innerFit, outerFit, wall, endCfg.label);

endData.label = endCfg.label;
endData.imgAbs = endCfg.imgAbs;
endData.outerFile = endCfg.outerCsv;
endData.innerFile = endCfg.innerCsv;
endData.innerDiaPx = innerFit.r * 2;
endData.outerDiaPx = outerFit.r * 2;
endData.innerDiaMm = endData.innerDiaPx / cfg.scale.endPxPerMm;
endData.outerDiaMm = endData.outerDiaPx / cfg.scale.endPxPerMm;

% fprintf('%s端面外轮廓已保存为 %s\n', endCfg.label, endCfg.outerCsv);
% fprintf('%s端面内轮廓已保存为 %s\n', endCfg.label, endCfg.innerCsv);
end


function edgeMask = manual_canny(img, lowThr, highThr, iterN)
flt = [1, 2, 1; 2, 4, 2; 1, 2, 1] / 16;
imgF = conv2(img, flt, 'same');

sobelX = [-1, 0, 1; -1, 0, 1; -1, 0, 1];
sobelY = [-1, -1, -1; 0, 0, 0; 1, 1, 1];

gradX = conv2(imgF, sobelX, 'same');
gradY = conv2(imgF, sobelY, 'same');
grad = sqrt(gradX .^ 2 + gradY .^ 2);
theta = atan2(gradY, gradX);

[rowN, colN] = size(theta);

for row = 1:rowN
    for col = 1:colN
        if theta(row, col) > -pi / 8 && theta(row, col) <= pi / 8
            theta(row, col) = 0;
        elseif theta(row, col) > pi / 8 && theta(row, col) <= 3 * pi / 8
            theta(row, col) = 1;
        elseif theta(row, col) > 3 * pi / 8 && theta(row, col) <= 5 * pi / 8
            theta(row, col) = 2;
        elseif theta(row, col) > 5 * pi / 8 && theta(row, col) <= 7 * pi / 8
            theta(row, col) = 3;
        elseif theta(row, col) > 7 * pi / 8 || theta(row, col) <= -7 * pi / 8
            theta(row, col) = 0;
        elseif theta(row, col) > -7 * pi / 8 && theta(row, col) <= -5 * pi / 8
            theta(row, col) = 1;
        elseif theta(row, col) > -5 * pi / 8 && theta(row, col) <= -3 * pi / 8
            theta(row, col) = 2;
        elseif theta(row, col) > -3 * pi / 8 && theta(row, col) <= -pi / 8
            theta(row, col) = 3;
        end
    end
end

for row = 2:rowN - 1
    for col = 2:colN - 1
        if theta(row, col) == 2
            if ~(grad(row, col) > grad(row + 1, col) && grad(row, col) > grad(row - 1, col))
                grad(row, col) = 0;
            end
        elseif theta(row, col) == 3
            if ~(grad(row, col) > grad(row + 1, col - 1) && grad(row, col) > grad(row - 1, col + 1))
                grad(row, col) = 0;
            end
        elseif theta(row, col) == 0
            if ~(grad(row, col) > grad(row, col - 1) && grad(row, col) > grad(row, col + 1))
                grad(row, col) = 0;
            end
        elseif theta(row, col) == 1
            if ~(grad(row, col) > grad(row - 1, col - 1) && grad(row, col) > grad(row + 1, col + 1))
                grad(row, col) = 0;
            end
        end
    end
end

weak = grad * 0;
strong = grad * 0;
th1 = max(max(grad(2:rowN - 1, 2:colN - 1))) * lowThr;
th2 = max(max(grad(2:rowN - 1, 2:colN - 1))) * highThr;

for row = 1:rowN
    for col = 1:colN
        if grad(row, col) >= th2
            strong(row, col) = grad(row, col);
        elseif grad(row, col) >= th1 && grad(row, col) <= th2
            weak(row, col) = grad(row, col);
        end
    end
end

for row = 1:rowN
    for col = 1:colN
        if weak(row, col) ~= 0
            weak(row, col) = 1;
        end
        if strong(row, col) ~= 0
            strong(row, col) = 1;
        end
    end
end

for k = 1:iterN
    for row = 2:rowN - 1
        for col = 2:colN - 1
            if weak(row, col) == 0
                continue;
            end
            if strong(row - 1, col - 1) ~= 0 || strong(row - 1, col) ~= 0 || ...
               strong(row - 1, col + 1) ~= 0 || strong(row, col - 1) ~= 0 || ...
               strong(row, col + 1) ~= 0 || strong(row + 1, col - 1) ~= 0 || ...
               strong(row + 1, col) ~= 0 || strong(row + 1, col + 1) ~= 0
                strong(row, col) = weak(row, col);
                weak(row, col) = 0;
            end
        end
    end
end

edgeMask = strong;
end


function innerMask = trace_inner_profile(edgeMask)
[rowN, colN] = size(edgeMask);
innerMask = edgeMask * 0;

for row = round(rowN / 2):rowN - 10
    if edgeMask(row, round(colN / 2)) ~= 0
        row2 = row;
        innerMask(row, round(colN / 2)) = 1;
        break;
    end
end

for row = round(rowN / 2):-1:10
    if edgeMask(row, round(colN / 2)) ~= 0
        row1 = row;
        innerMask(row, round(colN / 2)) = 1;
        break;
    end
end

for col = round(colN / 2):colN - 10
    if edgeMask(round(rowN / 2), col) ~= 0
        col2 = col;
        innerMask(round(rowN / 2), col) = 1;
        break;
    end
end

for col = round(colN / 2):-1:10
    if edgeMask(round(rowN / 2), col) ~= 0
        col1 = col;
        innerMask(round(rowN / 2), col) = 1;
        break;
    end
end

for row = row1 + 1:row2 - 1
    for col = round(colN / 2):colN - 10
        if edgeMask(row, col) ~= 0
            innerMask(row, col) = 1;
            break;
        end
    end
end

for row = row1 + 1:row2 - 1
    for col = round(colN / 2):-1:10
        if edgeMask(row, col) ~= 0
            innerMask(row, col) = 1;
            break;
        end
    end
end

for col = col1 + 1:col2 - 1
    for row = round(rowN / 2):rowN - 10
        if edgeMask(row, col) ~= 0
            innerMask(row, col) = 1;
            break;
        end
    end
end

for col = col1 + 1:col2 - 1
    for row = round(rowN / 2):-1:10
        if edgeMask(row, col) ~= 0
            innerMask(row, col) = 1;
            break;
        end
    end
end
end


function outerMask = trace_outer_profile(edgeMask)
[rowN, colN] = size(edgeMask);
outerMask = edgeMask * 0;

for row = 10:rowN - 10
    for col = 10:colN - 10
        if edgeMask(row, col) ~= 0
            outerMask(row, col) = 1;
            break;
        end
    end
end

for row = rowN - 10:-1:10
    for col = colN - 10:-1:10
        if edgeMask(row, col) ~= 0
            outerMask(row, col) = 1;
            break;
        end
    end
end

for col = 10:colN - 10
    for row = 10:rowN - 10
        if edgeMask(row, col) ~= 0
            outerMask(row, col) = 1;
            break;
        end
    end
end

for col = colN - 10:-1:10
    for row = rowN - 10:-1:10
        if edgeMask(row, col) ~= 0
            outerMask(row, col) = 1;
            break;
        end
    end
end
end


function fitData = fit_circle_ls(mask)
[rowN, colN] = size(mask);
sx = 0;
sx2 = 0;
sx3 = 0;
sy = 0;
sy2 = 0;
sy3 = 0;
sxy = 0;
sxy2 = 0;
sx2y = 0;
ptN = 0;

for col = 20:colN - 20
    for row = 20:rowN - 20
        if mask(row, col) == 0
            continue;
        end
        sx = col + sx;
        sx2 = col ^ 2 + sx2;
        sx3 = col ^ 3 + sx3;
        sxy = col * row + sxy;
        sxy2 = col * row ^ 2 + sxy2;
        sx2y = col ^ 2 * row + sx2y;
        sy = row + sy;
        sy2 = row ^ 2 + sy2;
        sy3 = row ^ 3 + sy3;
        ptN = ptN + 1;
    end
end

matM = ptN * sx2 - sx ^ 2;
matN = ptN * sxy - sx * sy;
matH = ptN * sx3 + ptN * sxy2 - sx * (sx2 + sy2);
matP = ptN * sy2 - sy ^ 2;
matQ = ptN * sy3 + ptN * sx2y - sy * (sx2 + sy2);

coefA = (matP * matH - matN * matQ) / (matN ^ 2 - matM * matP);
coefB = (matQ * matM - matN * matH) / (matN ^ 2 - matM * matP);
coefC = -(sx2 + sy2 + coefA * sx + coefB * sy) / ptN;

fitData.cx = -coefA / 2;
fitData.cy = -coefB / 2;
fitData.r = sqrt(coefA ^ 2 + coefB ^ 2 - 4 * coefC) / 2;
end


function wall = measure_wall_thickness(innerMask, outerMask, innerFit, outerFit)
idx = 1;
padR = 20;
innerR = [];
outerR = [];

for theta = 0:0.01:2 * pi
    ang(idx) = theta; %#ok<AGROW>
    for radius = innerFit.r - padR:0.01:innerFit.r + padR
        if innerMask(round(innerFit.cy) + round(radius * cos(theta)), round(innerFit.cx) + round(radius * sin(theta))) ~= 0
            innerR(idx) = radius; %#ok<AGROW>
            break;
        elseif radius == innerFit.r + padR
            innerR(idx) = median(innerR); %#ok<AGROW>
        end
    end

    for radius = outerFit.r - padR:0.01:outerFit.r + padR
        if outerMask(round(innerFit.cy) + round(radius * cos(theta)), round(innerFit.cx) + round(radius * sin(theta))) ~= 0
            outerR(idx) = radius; %#ok<AGROW>
            break;
        elseif radius == outerFit.r + padR
            outerR(idx) = median(outerR); %#ok<AGROW>
        end
    end

    idx = idx + 1;
end

thick = outerR - innerR;
maxMask = thick == max(thick);
minMask = thick == min(thick);

wall.ang = ang;
wall.innerR = innerR;
wall.outerR = outerR;
wall.thick = thick;
wall.maxVal = max(thick);
wall.minVal = min(thick);
wall.maxP1 = [round(innerFit.cy) + innerR(maxMask) * cos(ang(maxMask)), round(innerFit.cx) + innerR(maxMask) * sin(ang(maxMask))];
wall.maxP2 = [round(innerFit.cy) + outerR(maxMask) * cos(ang(maxMask)), round(innerFit.cx) + outerR(maxMask) * sin(ang(maxMask))];
wall.minP1 = [round(innerFit.cy) + innerR(minMask) * cos(ang(minMask)), round(innerFit.cx) + innerR(minMask) * sin(ang(minMask))];
wall.minP2 = [round(innerFit.cy) + outerR(minMask) * cos(ang(minMask)), round(innerFit.cx) + outerR(minMask) * sin(ang(minMask))];
end


function show_endface_result(img, innerFit, outerFit, wall, endLabel)
figure('Name', [endLabel, '壁厚曲线']);
plot(wall.ang, wall.thick);
title([endLabel, '端面圆心角与壁厚关系']);
xlabel('圆心角/rad');
ylabel('壁厚/pixels');
set(get(gca, 'XLabel'), 'FontName', 'Times New Roman', 'FontWeight', 'bold', 'FontSize', 18);
set(get(gca, 'YLabel'), 'FontName', 'Times New Roman', 'FontWeight', 'bold', 'FontSize', 18);
set(gca, 'FontName', 'Times New Roman', 'FontWeight', 'bold', 'FontSize', 13);

figure('Name', [endLabel, '端面拟合结果']);
imshow(img);
hold on;
rectangle('Position', [innerFit.cx - innerFit.r, innerFit.cy - innerFit.r, 2 * innerFit.r, 2 * innerFit.r], ...
    'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
text(innerFit.cx, innerFit.cy, ...
    ['拟合内圆圆心(', num2str(innerFit.cx), ',', num2str(innerFit.cy), ') 直径=', num2str(innerFit.r * 2), ' pixels'], ...
    'FontSize', 15, 'Color', 'r');

rectangle('Position', [outerFit.cx - outerFit.r, outerFit.cy - outerFit.r, 2 * outerFit.r, 2 * outerFit.r], ...
    'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
text(outerFit.cx, outerFit.cy + 150, ...
    ['拟合外圆圆心(', num2str(outerFit.cx), ',', num2str(outerFit.cy), ') 直径=', num2str(outerFit.r * 2), ' pixels'], ...
    'FontSize', 15, 'Color', 'r');

line([wall.maxP1(2), wall.maxP2(2)], [wall.maxP1(1), wall.maxP2(1)], 'LineWidth', 8);
text(wall.maxP1(2), wall.maxP1(1), ['最大壁厚 ', num2str(wall.maxVal), ' pixels'], 'FontSize', 15, 'Color', 'r');

line([wall.minP1(2), wall.minP2(2)], [wall.minP1(1), wall.minP2(1)], 'LineWidth', 8);
text(wall.minP1(2), wall.minP1(1), ['最小壁厚 ', num2str(wall.minVal), ' pixels'], 'FontSize', 15, 'Color', 'r');
hold off;
end


function build_tube_model(cfg, pathTbl, leftEndData, rightEndData)
leftPtsOut = readmatrix(leftEndData.outerFile);
leftPtsIn = readmatrix(leftEndData.innerFile);
rightPtsOut = readmatrix(rightEndData.outerFile);
rightPtsIn = readmatrix(rightEndData.innerFile);

step = 5;
xPath = pathTbl.X(1:step:end) / cfg.scale.sidePxPerMm;
yRaw = (fillmissing(pathTbl.TopY, 'linear') + fillmissing(pathTbl.BotY, 'linear')) / 2;
yPath = smoothdata(yRaw(1:step:end), 'rloess', 50) / cfg.scale.sidePxPerMm;

angleN = 120;
[leftOutRaw, leftInRaw, leftOffRaw, leftMaxD] = analyze_profile_separated(leftPtsOut, leftPtsIn, angleN);
[leftOutFit, leftInFit, leftOffFit] = fit_perfect_separated(leftPtsOut, leftPtsIn, angleN);
[rightOutRaw, rightInRaw, rightOffRaw, rightMaxD] = analyze_profile_separated(rightPtsOut, rightPtsIn, angleN);
[rightOutFit, rightInFit, rightOffFit] = fit_perfect_separated(rightPtsOut, rightPtsIn, angleN);

leftOutRaw = leftOutRaw / cfg.scale.endPxPerMm;
leftInRaw = leftInRaw / cfg.scale.endPxPerMm;
leftOffRaw = leftOffRaw / cfg.scale.endPxPerMm;
leftOutFit = leftOutFit / cfg.scale.endPxPerMm;
leftInFit = leftInFit / cfg.scale.endPxPerMm;
leftOffFit = leftOffFit / cfg.scale.endPxPerMm;
rightOutRaw = rightOutRaw / cfg.scale.endPxPerMm;
rightInRaw = rightInRaw / cfg.scale.endPxPerMm;
rightOffRaw = rightOffRaw / cfg.scale.endPxPerMm;
rightOutFit = rightOutFit / cfg.scale.endPxPerMm;
rightInFit = rightInFit / cfg.scale.endPxPerMm;
rightOffFit = rightOffFit / cfg.scale.endPxPerMm;

len1 = (leftMaxD / cfg.scale.endPxPerMm) / 48;
len2 = (rightMaxD / cfg.scale.endPxPerMm) / 48;

sliceN = numel(xPath);
theta = linspace(0, 2 * pi, angleN);

xMesh = zeros(angleN, sliceN);
yOut = zeros(angleN, sliceN);
zOut = zeros(angleN, sliceN);
yIn = zeros(angleN, sliceN);
zIn = zeros(angleN, sliceN);

stepLen = sqrt([0; diff(xPath)] .^ 2 + [0; diff(yPath)] .^ 2);
arcLen = cumsum(stepLen);
allLen = arcLen(end);

for k = 1:sliceN
    nowLen = arcLen(k);
    endLen = allLen - nowLen;

    if nowLen <= len1
        weight = nowLen / len1;
        weight = weight * weight * (3 - 2 * weight);
        curOut = (1 - weight) * leftOutRaw + weight * leftOutFit;
        curIn = (1 - weight) * leftInRaw + weight * leftInFit;
        curOff = (1 - weight) * leftOffRaw + weight * leftOffFit;
    elseif endLen <= len2
        weight = (len2 - endLen) / len2;
        weight = weight * weight * (3 - 2 * weight);
        curOut = (1 - weight) * rightOutFit + weight * rightOutRaw;
        curIn = (1 - weight) * rightInFit + weight * rightInRaw;
        curOff = (1 - weight) * rightOffFit + weight * rightOffRaw;
    else
        midLen = allLen - len1 - len2;
        if midLen <= 0
            ratio = 0.5;
        else
            ratio = (nowLen - len1) / midLen;
        end
        curOut = (1 - ratio) * leftOutFit + ratio * rightOutFit;
        curIn = (1 - ratio) * leftInFit + ratio * rightInFit;
        curOff = (1 - ratio) * leftOffFit + ratio * rightOffFit;
    end

    locYOut = curOut .* cos(theta);
    locZOut = curOut .* sin(theta);
    locYIn = curIn .* cos(theta) + curOff(1);
    locZIn = curIn .* sin(theta) + curOff(2);

    xMesh(:, k) = xPath(k);
    yOut(:, k) = -yPath(k) + locYOut;
    yIn(:, k) = -yPath(k) + locYIn;
    zOut(:, k) = locZOut;
    zIn(:, k) = locZIn;
end

fig = figure('Color', 'k', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.7]);
ax = axes('Parent', fig, 'Color', 'k');
hold on;
axis equal;
view(3);

hOut = surf(xMesh, zOut, yOut, 'FaceColor', [0.6, 0.8, 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.2, 'SpecularStrength', 0.8);
hIn = surf(xMesh, zIn, yIn, 'FaceColor', [0.8, 0.8, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
alpha(hOut, 0.3);
alpha(hIn, 0.6);

fill3(xMesh(:, 1), zOut(:, 1), yOut(:, 1), [0.6, 0.8, 1], 'FaceAlpha', 0.3);
fill3(xMesh(:, end), zOut(:, end), yOut(:, end), [0.6, 0.8, 1], 'FaceAlpha', 0.3);

light('Position', [0, -500, 500], 'Style', 'local');
light('Position', [2000, 500, 500], 'Style', 'local');
lighting gouraud;
material shiny;

xlabel('X (mm)');
ylabel('Z (mm)');
zlabel('Y (mm)');
title('Eccentric glass tube 3D reconstruction');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
grid on;
ax.GridColor = [1, 1, 1];
ax.GridAlpha = 0.2;
view(0, 5);
end


function [rOut, rIn, offVec, maxD] = analyze_profile_separated(ptsOut, ptsIn, angleN)
ptsOut = fix_dims(ptsOut);
ptsIn = fix_dims(ptsIn);

ctrOut = mean(ptsOut, 1);
ctrIn = mean(ptsIn, 1);
offVec = ctrIn - ctrOut;

ptsOut = bsxfun(@minus, ptsOut, ctrOut);
ptsIn = bsxfun(@minus, ptsIn, ctrIn);

dist = sqrt(sum(ptsOut .^ 2, 2));
maxD = max(dist) * 2;

rOut = resample_polar(ptsOut, angleN);
rIn = resample_polar(ptsIn, angleN);
end


function [rOut, rIn, offVec] = fit_perfect_separated(ptsOut, ptsIn, angleN)
ptsOut = fix_dims(ptsOut);
ptsIn = fix_dims(ptsIn);

ctrOut = mean(ptsOut, 1);
ctrIn = mean(ptsIn, 1);
offVec = ctrIn - ctrOut;

[aOut, bOut, phiOut] = get_ellipse_params(ptsOut);
[aIn, bIn, phiIn] = get_ellipse_params(ptsIn);

theta = linspace(0, 2 * pi, angleN);
ellipseR = @(a, b, phi, ang) sqrt((a * b) ^ 2 ./ ((b * cos(ang - phi)) .^ 2 + (a * sin(ang - phi)) .^ 2));

rOut = ellipseR(aOut, bOut, phiOut, theta);
rIn = ellipseR(aIn, bIn, phiIn, theta);
end


function [a, b, phi] = get_ellipse_params(pts)
ctr = mean(pts, 1);
pts = bsxfun(@minus, pts, ctr);
covMat = cov(pts);
[vec, val] = eig(covMat);
eigVal = diag(val);
[~, idx] = sort(eigVal, 'descend');
vec = vec(:, idx);
pts = pts * vec;

width = max(pts(:, 1)) - min(pts(:, 1));
height = max(pts(:, 2)) - min(pts(:, 2));
a = width / 2;
b = height / 2;

if b > a
    tmp = a;
    a = b;
    b = tmp;
end

phi = atan2(vec(2, 1), vec(1, 1));
end


function pts = fix_dims(pts)
[rowN, colN] = size(pts);

if rowN > 10 && colN > 10
    [yy, zz] = find(pts > 0);
    pts = [yy, zz];
elseif rowN == 2 && colN > 2
    pts = pts';
elseif colN == 3
    pts = pts(:, 2:3);
elseif rowN == 3
    pts = pts(2:3, :)';
end
end


function rFit = resample_polar(pts, angleN)
[theta, radius] = cart2pol(pts(:, 1), pts(:, 2));
theta = mod(theta, 2 * pi);
[theta, idx] = sort(theta);
radius = radius(idx);
[thetaU, idxU] = unique(theta);
radiusU = radius(idxU);

thetaPad = [thetaU - 2 * pi; thetaU; thetaU + 2 * pi];
radiusPad = [radiusU; radiusU; radiusU];
thetaGrid = linspace(0, 2 * pi, angleN);
rFit = interp1(thetaPad, radiusPad, thetaGrid, 'linear', 'extrap');
end


function estimate_rod_fit(pathTbl, leftEndData, rightEndData, cfg)
if ~isfield(leftEndData, 'innerDiaMm') || isempty(leftEndData.innerDiaMm) || ~isfinite(leftEndData.innerDiaMm) || leftEndData.innerDiaMm <= 0
    error('左端端面分析未得到可用的内径数据，无法执行插入分析。');
end

if ~isfield(rightEndData, 'innerDiaMm') || isempty(rightEndData.innerDiaMm) || ~isfinite(rightEndData.innerDiaMm) || rightEndData.innerDiaMm <= 0
    error('右端端面分析未得到可用的内径数据，无法执行插入分析。');
end

tubeID = min(leftEndData.innerDiaMm, rightEndData.innerDiaMm);
cavity = build_analysis_cavity(pathTbl, leftEndData, rightEndData, cfg);
opt = optimize_straight_rod_axis(cavity.x, cavity.innerTop, cavity.innerBot);

rodDia = min(opt.sideLimit, tubeID);
rodDia = max(0, rodDia);

if rodDia == 0
    fprintf('No feasible straight rod diameter was found in the 2D cavity model.\n');
end

if opt.sideLimit <= tubeID
    limitLabel = 'side-view cavity';
else
    limitLabel = 'end-face minimum diameter';
end

centerLine = opt.slope * cavity.x + opt.intercept;
rodTop = centerLine - rodDia / 2;
rodBot = centerLine + rodDia / 2;
flatTop = cavity.innerTop - centerLine;
flatBot = cavity.innerBot - centerLine;

fprintf('================ Results ================\n');
% fprintf('Left end inner diameter: %.2f mm (%.2f px)\n', leftEndData.innerDiaMm, leftEndData.innerDiaPx);
% fprintf('Right end inner diameter: %.2f mm (%.2f px)\n', rightEndData.innerDiaMm, rightEndData.innerDiaPx);
% fprintf('2D side-view cavity limit: %.2f mm\n', opt.sideLimit);
% fprintf('End-face minimum diameter limit: %.2f mm\n', tubeID);
% fprintf('Active limit: %s\n', limitLabel);
% fprintf('Optimal rod axis slope: %.6f mm/mm\n', opt.slope);
% fprintf('Optimal rod axis intercept: %.3f mm\n', opt.intercept);
fprintf('Maximum straight rod diameter: %.2f mm\n', rodDia);
fprintf('=========================================\n');


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

% saveas(gcf, cfg.file.rod);
% fprintf('Rod insertion analysis saved to %s\n', cfg.file.rod);
end


function cavity = build_analysis_cavity(pathTbl, leftEndData, rightEndData, cfg)
xVal = pathTbl.X / cfg.scale.sidePxPerMm;
outerTop = fillmissing(pathTbl.TopY, 'linear') / cfg.scale.sidePxPerMm;
outerBot = fillmissing(pathTbl.BotY, 'linear') / cfg.scale.sidePxPerMm;
outerCtr = smoothdata((outerTop + outerBot) / 2, 'rloess', 50);

leftSupport = extract_end_inner_support(leftEndData, cfg);
rightSupport = extract_end_inner_support(rightEndData, cfg);

if numel(xVal) < 2
    error('Not enough side-view samples for rod analysis.');
end

t = (xVal - xVal(1)) / max(eps, xVal(end) - xVal(1));
topRel = (1 - t) * leftSupport.topRel + t * rightSupport.topRel;
botRel = (1 - t) * leftSupport.botRel + t * rightSupport.botRel;

innerTop = outerCtr + topRel;
innerBot = outerCtr + botRel;

% inner cavity must remain inside the observed outer envelope.
innerTop = max(innerTop, outerTop);
innerBot = min(innerBot, outerBot);

valid = isfinite(xVal) & isfinite(innerTop) & isfinite(innerBot) & (innerBot > innerTop);
if nnz(valid) < 2
    error('Estimated inner cavity is invalid for rod analysis.');
end

cavity.x = xVal(valid);
cavity.outerTop = outerTop(valid);
cavity.outerBot = outerBot(valid);
cavity.outerCtr = outerCtr(valid);
cavity.innerTop = innerTop(valid);
cavity.innerBot = innerBot(valid);
cavity.leftSupport = leftSupport;
cavity.rightSupport = rightSupport;
end


function support = extract_end_inner_support(endData, cfg)
ptsOut = fix_dims(readmatrix(endData.outerFile));
ptsIn = fix_dims(readmatrix(endData.innerFile));
ctrOut = mean(ptsOut, 1);
relY = ptsIn(:, 1) - ctrOut(1);

support.topRel = min(relY) / cfg.scale.endPxPerMm;
support.botRel = max(relY) / cfg.scale.endPxPerMm;
support.projDia = support.botRel - support.topRel;
support.centerRel = mean(relY) / cfg.scale.endPxPerMm;
end


function opt = optimize_straight_rod_axis(xVal, innerTop, innerBot)
centerLine = (innerTop + innerBot) / 2;
localSlope = diff(centerLine) ./ diff(xVal);
localSlope = localSlope(isfinite(localSlope));
baseSlope = polyfit(xVal, centerLine, 1);
baseSlope = baseSlope(1);

if isempty(localSlope)
    slopeAbs = max(abs(baseSlope), 0.02);
else
    slopeAbs = max(abs([localSlope(:); baseSlope]));
    slopeAbs = max(slopeAbs, 0.02);
end

slopeGrid = linspace(-slopeAbs - 0.05, slopeAbs + 0.05, 4001);
clearanceGrid = -inf(size(slopeGrid));
interceptGrid = zeros(size(slopeGrid));
topIdxGrid = ones(size(slopeGrid));
botIdxGrid = ones(size(slopeGrid));

for k = 1:numel(slopeGrid)
    [clearanceGrid(k), interceptGrid(k), topIdxGrid(k), botIdxGrid(k)] = ...
        evaluate_axis_clearance(slopeGrid(k), xVal, innerTop, innerBot);
end

[bestClear, bestIdx] = max(clearanceGrid);
opt.slope = slopeGrid(bestIdx);
opt.intercept = interceptGrid(bestIdx);
opt.sideLimit = bestClear;
opt.topTouchIdx = topIdxGrid(bestIdx);
opt.botTouchIdx = botIdxGrid(bestIdx);
end


function [clearance, intercept, topIdx, botIdx] = evaluate_axis_clearance(slope, xVal, innerTop, innerBot)
topShift = innerTop - slope * xVal;
botShift = innerBot - slope * xVal;

[topLim, topIdx] = max(topShift);
[botLim, botIdx] = min(botShift);

clearance = botLim - topLim;
intercept = (topLim + botLim) / 2;
end


function warn_scale_consistency(pathTbl, leftEndData, rightEndData, cfg)
sideDiaPx = pathTbl.Dia(isfinite(pathTbl.Dia));
if isempty(sideDiaPx)
    fprintf(2, 'WARNING: Skip scale consistency check because side-view diameter data is unavailable.\n');
    return;
end

sideDiaMm = median(sideDiaPx) / cfg.scale.sidePxPerMm;
endDiaMm = min(leftEndData.innerDiaMm, rightEndData.innerDiaMm);
relDiff = abs(endDiaMm - sideDiaMm) / max(endDiaMm, sideDiaMm);

fprintf('Scale check: end inner diameter = %.2f mm, side median diameter = %.2f mm, relative diff = %.2f%%\n', ...
    endDiaMm, sideDiaMm, relDiff * 100);

if relDiff > cfg.warn.scaleRelTol
    fprintf(2, ['WARNING: End-face diameter and side-view diameter differ by %.2f%%. ' ...
        'Please verify calibration, ROI selection, end-face extraction, or placeholder end images.\n'], ...
        relDiff * 100);
end
end
