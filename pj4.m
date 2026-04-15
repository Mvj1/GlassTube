%% 鐜荤拑绠″浘鍍忓鐞嗕笌涓夌淮閲嶅缓
% 椤堕儴鐩告満绉婚€?00px/5s锛岄《閮ㄧ浉鏈虹缉鏀炬瘮渚?060px/30mm锛屼晶杈圭浉鏈虹缉鏀炬瘮渚?427px/30mm

%% 鍒濆鍖?
clear; clc; close all;
cfg = get_cfg();
stageTimings = init_stage_timing_table();

if is_debug_display_enabled(cfg)
    setup_debug_figure_defaults();
end

%% 澶氬浘浜掔浉鍏虫嫾鎺?
tStage = tic;
[stripImg, stepList] = stitch_strip(cfg);
stageTimings = append_stage_timing(stageTimings, 'stitch_strip', toc(tStage));
% show_strip(stripImg, stepList, cfg);

tStage = tic;
imwrite(stripImg, cfg.file.strip);
stageTimings = append_stage_timing(stageTimings, 'write_strip', toc(tStage));
% fprintf('鎷兼帴缁撴灉宸蹭繚瀛樹负 %s\n', cfg.file.strip);

%% 渚ц杈圭紭鎻愬彇
tStage = tic;
[roiGray, topMask, botMask, pathTbl] = extract_side_edges_robust(cfg);
stageTimings = append_stage_timing(stageTimings, 'extract_side_edges_robust', toc(tStage));
%%%%%% % imwrite(topMask | botMask, cfg.file.side);

tStage = tic;
writetable(pathTbl, cfg.file.path);
stageTimings = append_stage_timing(stageTimings, 'write_tube_path', toc(tStage));
%%%%%% % fprintf('渚ц杈圭紭宸蹭繚瀛樹负 %s\n', cfg.file.side);
%%%%%% % fprintf('渚ц璺緞鏁版嵁宸蹭繚瀛樹负 %s\n', cfg.file.path);

%% 绔潰杞粨鍒嗘瀽涓庢渶澶ф彃鍏ョ洿寰勪及绠?
tStage = tic;
ensure_right_end_image(cfg);
stageTimings = append_stage_timing(stageTimings, 'ensure_right_end_image', toc(tStage));

tStage = tic;
leftEndData = analyze_endface(cfg.end.left, cfg);
stageTimings = append_stage_timing(stageTimings, 'analyze_left_endface', toc(tStage));

tStage = tic;
rightEndData = analyze_endface(cfg.end.right, cfg);
stageTimings = append_stage_timing(stageTimings, 'analyze_right_endface', toc(tStage));

tStage = tic;
warn_scale_consistency(pathTbl, leftEndData, rightEndData, cfg);
stageTimings = append_stage_timing(stageTimings, 'warn_scale_consistency', toc(tStage));

tStage = tic;
rodResult = estimate_rod_fit(pathTbl, leftEndData, rightEndData, cfg);
stageTimings = append_stage_timing(stageTimings, 'estimate_rod_fit', toc(tStage));

if is_debug_display_enabled(cfg)
    debugData = struct();
    debugData.roiGray = roiGray;
    debugData.topMask = topMask;
    debugData.botMask = botMask;
    debugData.leftEndData = leftEndData;
    debugData.rightEndData = rightEndData;
    debugData.rodResult = rodResult;

    tStage = tic;
    debugTimings = pj4_debug(cfg, debugData);
    stageTimings = [stageTimings; debugTimings];
    stageTimings = append_stage_timing(stageTimings, 'debug_visuals_total', toc(tStage));
end

if should_dump_stage_timings(cfg)
    writetable(stageTimings, cfg.file.stageTimings);
end

%% 杈呭姪鍑芥暟
function cfg = get_cfg()
cfg.dir.img = './GlassTubeData';
cfg.dir.ext = '*.bmp';

cfg.step.nominal = 900;
cfg.step.search = 40;
cfg.rot.angle = -0.39;
cfg.debug.enable = true;
cfg.debug.verbose = false;
cfg.debug.dumpSideCandidates = false;
cfg.debug.dumpStageTimings = false;

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
cfg.file.stageTimings = 'pj4_stage_timings.csv';

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

cfg.end.left.label = '宸︾';
cfg.end.left.imgAbs = 'C:\Users\11603\Documents\MATLAB\GlassTube\40.12-30.49.bmp';
cfg.end.left.innerCsv = 'left_end_inner.csv';
cfg.end.left.outerCsv = 'left_end_outer.csv';
cfg.end.left.cacheMat = 'left_end_analysis.mat';

cfg.end.right.label = '鍙崇';
cfg.end.right.imgAbs = 'C:\Users\11603\Documents\MATLAB\GlassTube\test.bmp';
cfg.end.right.innerCsv = 'right_end_inner.csv';
cfg.end.right.outerCsv = 'right_end_outer.csv';
cfg.end.right.cacheMat = 'right_end_analysis.mat';
cfg.end.forceRecompute = false;
end


function ensure_right_end_image(cfg)
if exist(cfg.end.right.imgAbs, 'file')
    return;
end

copyfile(cfg.end.left.imgAbs, cfg.end.right.imgAbs);
fprintf('宸茬敓鎴愬彸绔崰浣嶅師鍥?%s\n', cfg.end.right.imgAbs);
end


function [stripImg, stepList] = stitch_strip(cfg)
files = dir(fullfile(cfg.dir.img, cfg.dir.ext));
[~, idx] = sort({files.name});
files = files(idx);

if numel(files) < 2
    error('闇€瑕佽嚦灏戜袱寮犲浘鐗囥€?);
end

rotImg = @(img) imrotate(img, cfg.rot.angle, 'bicubic', 'crop');

% fprintf('寮€濮嬫嫾鎺ワ紝棰勮鏃嬭浆瑙掑害 %.2f掳\n', cfg.rot.angle);

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
        warning('绗?%d 寮犲浘鍖归厤澶辫触锛屾敼鐢ㄩ粯璁ゆ闀裤€?, k);
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

    if is_verbose_debug_enabled(cfg)
        fprintf('绗?%02d 寮犲浘锛氭闀?%d px锛屽瀭鐩磋ˉ鍋?%+d px\n', k, cutW, round(dy));
    end
end
end


function show_strip(stripImg, stepList, cfg)
figure('Name', '鎷兼帴缁撴灉', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.6]);

subplot(2, 1, 1);
imshow(stripImg);
title('鎷兼帴缁撴灉');
xlabel('鍍忕礌');

subplot(2, 1, 2);
plot(stepList, '-o');
yline(cfg.step.nominal, '--r');
title('瀹為檯妫€娴嬫闀?);
xlabel('鍥剧墖搴忓彿');
ylabel('鍍忕礌');
grid on;
end


function endData = analyze_endface(endCfg, cfg)
cachedData = load_endface_cache(endCfg, cfg);
if ~isempty(cachedData)
    write_endface_outputs_if_needed(cachedData, endCfg);
    endData = cachedData;
    return;
end

img = imread(endCfg.imgAbs);
img = im2gray(img);
img = im2double(img);
img = imopen(img, strel('disk', 8));

edgeMask = manual_canny(img, cfg.canny.low, cfg.canny.high, cfg.canny.iter);
innerMask = trace_inner_profile(edgeMask);
outerMask = trace_outer_profile(edgeMask);
innerPts = fix_dims(innerMask);
outerPts = fix_dims(outerMask);

writematrix(outerMask, endCfg.outerCsv);
writematrix(innerMask, endCfg.innerCsv);

innerFit = fit_circle_ls(innerPts);
outerFit = fit_circle_ls(outerPts);
support = compute_end_inner_support(innerPts, outerPts, cfg.scale.endPxPerMm);

wall = [];
if is_debug_display_enabled(cfg)
    wall = measure_wall_thickness(innerMask, outerMask, innerFit, outerFit);
end

endData.label = endCfg.label;
endData.imgAbs = endCfg.imgAbs;
endData.outerFile = endCfg.outerCsv;
endData.innerFile = endCfg.innerCsv;
endData.outerPts = outerPts;
endData.innerPts = innerPts;
endData.innerDiaPx = innerFit.r * 2;
endData.outerDiaPx = outerFit.r * 2;
endData.innerDiaMm = endData.innerDiaPx / cfg.scale.endPxPerMm;
endData.outerDiaMm = endData.outerDiaPx / cfg.scale.endPxPerMm;
endData.innerFit = innerFit;
endData.outerFit = outerFit;
endData.support = support;
endData.sourceSignature = endface_source_signature(endCfg);
endData.configFingerprint = endface_config_fingerprint(cfg);
endData.debug = struct( ...
    'img', img, ...
    'edgeMask', edgeMask, ...
    'innerMask', innerMask, ...
    'outerMask', outerMask, ...
    'wall', wall);

save_endface_cache(endData, endCfg);
end
function edgeMask = manual_canny(img, lowThr, highThr, ~)
sobelX = [-1, 0, 1; -1, 0, 1; -1, 0, 1];
sobelY = sobelX.';

imgF = imgaussfilt(img, 1);
gradX = imfilter(imgF, sobelX, 'replicate', 'conv');
gradY = imfilter(imgF, sobelY, 'replicate', 'conv');
grad = hypot(gradX, gradY);

theta = mod(atan2(gradY, gradX), pi);
dirIdx = mod(round(theta / (pi / 4)), 4);

leftVal = circshift(grad, [0, -1]);
rightVal = circshift(grad, [0, 1]);
upVal = circshift(grad, [-1, 0]);
downVal = circshift(grad, [1, 0]);
upLeftVal = circshift(grad, [-1, -1]);
downRightVal = circshift(grad, [1, 1]);
upRightVal = circshift(grad, [-1, 1]);
downLeftVal = circshift(grad, [1, -1]);

keepMask = false(size(grad));
mask0 = dirIdx == 0;
mask1 = dirIdx == 1;
mask2 = dirIdx == 2;
mask3 = dirIdx == 3;

keepMask(mask0) = grad(mask0) > leftVal(mask0) & grad(mask0) > rightVal(mask0);
keepMask(mask1) = grad(mask1) > upLeftVal(mask1) & grad(mask1) > downRightVal(mask1);
keepMask(mask2) = grad(mask2) > upVal(mask2) & grad(mask2) > downVal(mask2);
keepMask(mask3) = grad(mask3) > upRightVal(mask3) & grad(mask3) > downLeftVal(mask3);

keepMask([1, end], :) = false;
keepMask(:, [1, end]) = false;
grad(~keepMask) = 0;

gradPeak = max(grad(:));
if gradPeak <= 0 || ~isfinite(gradPeak)
    edgeMask = false(size(grad));
    return;
end

th1 = gradPeak * lowThr;
th2 = gradPeak * highThr;
strong = grad >= th2;
weak = grad >= th1 & grad < th2;

edgeMask = imreconstruct(strong, strong | weak) > 0;
edgeMask = logical(edgeMask);
end


function innerMask = trace_inner_profile(edgeMask)
[rowN, colN] = size(edgeMask);
innerMask = false(size(edgeMask));
rowMargin = 10;
colMargin = 10;
ctrRow = round(rowN / 2);
ctrCol = round(colN / 2);

rowDown = ctrRow:(rowN - rowMargin);
rowUp = rowMargin:ctrRow;
colRight = ctrCol:(colN - colMargin);
colLeft = colMargin:ctrCol;

row2 = first_true_in_vector(edgeMask(rowDown, ctrCol), rowDown, true);
row1 = first_true_in_vector(edgeMask(rowUp, ctrCol), rowUp, false);
col2 = first_true_in_vector(edgeMask(ctrRow, colRight), colRight, true);
col1 = first_true_in_vector(edgeMask(ctrRow, colLeft), colLeft, false);

if any(~isfinite([row1, row2, col1, col2]))
    error('Inner profile tracing failed: unable to find the central seed points.');
end

innerMask(row1, ctrCol) = true;
innerMask(row2, ctrCol) = true;
innerMask(ctrRow, col1) = true;
innerMask(ctrRow, col2) = true;

rowRange = (row1 + 1):(row2 - 1);
if ~isempty(rowRange)
    rightSlice = edgeMask(rowRange, ctrCol:(colN - colMargin));
    [hitRight, idxRight] = max(rightSlice, [], 2);
    innerMask = set_true_from_row_hits(innerMask, rowRange(hitRight > 0), ctrCol + idxRight(hitRight > 0) - 1);

    leftSlice = fliplr(edgeMask(rowRange, colMargin:ctrCol));
    [hitLeft, idxLeft] = max(leftSlice, [], 2);
    innerMask = set_true_from_row_hits(innerMask, rowRange(hitLeft > 0), ctrCol - idxLeft(hitLeft > 0) + 1);
end

colRange = (col1 + 1):(col2 - 1);
if ~isempty(colRange)
    downSlice = edgeMask(ctrRow:(rowN - rowMargin), colRange);
    [hitDown, idxDown] = max(downSlice, [], 1);
    innerMask = set_true_from_row_hits(innerMask, ctrRow + idxDown(hitDown > 0).' - 1, colRange(hitDown > 0).');

    upSlice = flipud(edgeMask(rowMargin:ctrRow, colRange));
    [hitUp, idxUp] = max(upSlice, [], 1);
    innerMask = set_true_from_row_hits(innerMask, ctrRow - idxUp(hitUp > 0).' + 1, colRange(hitUp > 0).');
end
end


function outerMask = trace_outer_profile(edgeMask)
[rowN, colN] = size(edgeMask);
outerMask = false(size(edgeMask));
rowRange = 10:(rowN - 10);
colRange = 10:(colN - 10);
scanMask = edgeMask(rowRange, colRange);

[hitLeft, idxLeft] = max(scanMask, [], 2);
outerMask = set_true_from_row_hits(outerMask, rowRange(hitLeft > 0), colRange(1) + idxLeft(hitLeft > 0) - 1);

[hitRight, idxRight] = max(fliplr(scanMask), [], 2);
outerMask = set_true_from_row_hits(outerMask, rowRange(hitRight > 0), colRange(end) - idxRight(hitRight > 0) + 1);

[hitTop, idxTop] = max(scanMask, [], 1);
outerMask = set_true_from_row_hits(outerMask, rowRange(1) + idxTop(hitTop > 0).' - 1, colRange(hitTop > 0).');

[hitBot, idxBot] = max(flipud(scanMask), [], 1);
outerMask = set_true_from_row_hits(outerMask, rowRange(end) - idxBot(hitBot > 0).' + 1, colRange(hitBot > 0).');
end


function fitData = fit_circle_ls(maskOrPts)
if ismatrix(maskOrPts) && size(maskOrPts, 2) == 2 && size(maskOrPts, 1) > 2
    rowIdx = double(maskOrPts(:, 1));
    colIdx = double(maskOrPts(:, 2));
else
    [rowN, colN] = size(maskOrPts);
    validMask = false(size(maskOrPts));
    validMask(20:rowN - 20, 20:colN - 20) = maskOrPts(20:rowN - 20, 20:colN - 20) ~= 0;
    [rowIdx, colIdx] = find(validMask);
    rowIdx = double(rowIdx);
    colIdx = double(colIdx);
end

ptN = numel(rowIdx);

if ptN < 3
    error('Circle fitting failed: not enough valid contour points.');
end

x = colIdx;
y = rowIdx;
sx = sum(x);
sx2 = sum(x .^ 2);
sx3 = sum(x .^ 3);
sy = sum(y);
sy2 = sum(y .^ 2);
sy3 = sum(y .^ 3);
sxy = sum(x .* y);
sxy2 = sum(x .* (y .^ 2));
sx2y = sum((x .^ 2) .* y);

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
padR = 20;
ang = 0:0.01:(2 * pi);
centerRow = round(innerFit.cy);
centerCol = round(innerFit.cx);

innerR = sample_radial_contour(innerMask, centerRow, centerCol, innerFit.r, padR, ang);
outerR = sample_radial_contour(outerMask, centerRow, centerCol, outerFit.r, padR, ang);

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


function rodResult = estimate_rod_fit(pathTbl, leftEndData, rightEndData, cfg)
if ~isfield(leftEndData, 'innerDiaMm') || isempty(leftEndData.innerDiaMm) || ~isfinite(leftEndData.innerDiaMm) || leftEndData.innerDiaMm <= 0
    error('宸︾绔潰鍒嗘瀽鏈緱鍒板彲鐢ㄧ殑鍐呭緞鏁版嵁锛屾棤娉曟墽琛屾彃鍏ュ垎鏋愩€?);
end

if ~isfield(rightEndData, 'innerDiaMm') || isempty(rightEndData.innerDiaMm) || ~isfinite(rightEndData.innerDiaMm) || rightEndData.innerDiaMm <= 0
    error('鍙崇绔潰鍒嗘瀽鏈緱鍒板彲鐢ㄧ殑鍐呭緞鏁版嵁锛屾棤娉曟墽琛屾彃鍏ュ垎鏋愩€?);
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

rodResult = struct( ...
    'rodDia', rodDia, ...
    'limitLabel', limitLabel, ...
    'cavity', cavity, ...
    'opt', opt, ...
    'centerLine', centerLine, ...
    'rodTop', rodTop, ...
    'rodBot', rodBot, ...
    'flatTop', flatTop, ...
    'flatBot', flatBot);
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
if isfield(endData, 'support') && ~isempty(endData.support)
    support = endData.support;
    return;
end

if isfield(endData, 'outerPts') && isfield(endData, 'innerPts') && ...
        ~isempty(endData.outerPts) && ~isempty(endData.innerPts)
    support = compute_end_inner_support(endData.innerPts, endData.outerPts, cfg.scale.endPxPerMm);
    return;
end

ptsOut = fix_dims(readmatrix(endData.outerFile));
ptsIn = fix_dims(readmatrix(endData.innerFile));
support = compute_end_inner_support(ptsIn, ptsOut, cfg.scale.endPxPerMm);
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


function stageTimings = init_stage_timing_table()
stageTimings = table('Size', [0, 2], ...
    'VariableTypes', {'string', 'double'}, ...
    'VariableNames', {'Stage', 'Seconds'});
end


function stageTimings = append_stage_timing(stageTimings, stageName, elapsedSeconds)
newRow = table(string(stageName), elapsedSeconds, ...
    'VariableNames', stageTimings.Properties.VariableNames);
stageTimings = [stageTimings; newRow];
end


function setup_debug_figure_defaults()
set(groot, 'defaultFigureUnits', 'normalized');
set(groot, 'defaultFigurePosition', [0, 0, 1, 1]);
set(groot, 'defaultFigureWindowState', 'maximized');
end


function tf = is_debug_display_enabled(cfg)
tf = isfield(cfg, 'debug') && isstruct(cfg.debug) && ...
    isfield(cfg.debug, 'enable') && logical(cfg.debug.enable);
end


function tf = is_verbose_debug_enabled(cfg)
tf = is_debug_display_enabled(cfg) && isfield(cfg.debug, 'verbose') && logical(cfg.debug.verbose);
end


function tf = should_dump_stage_timings(cfg)
tf = is_debug_display_enabled(cfg) && isfield(cfg.debug, 'dumpStageTimings') && logical(cfg.debug.dumpStageTimings);
end


function cachedData = load_endface_cache(endCfg, cfg)
cachedData = [];
if ~isfield(cfg, 'end') || ~isfield(cfg.end, 'forceRecompute') || logical(cfg.end.forceRecompute)
    return;
end
if ~isfield(endCfg, 'cacheMat') || ~exist(endCfg.cacheMat, 'file')
    return;
end

loaded = load(endCfg.cacheMat, 'endData');
if ~isfield(loaded, 'endData') || ~isstruct(loaded.endData)
    return;
end

sourceSignature = endface_source_signature(endCfg);
configFingerprint = endface_config_fingerprint(cfg);
requiredFields = {'sourceSignature', 'configFingerprint', 'innerPts', 'outerPts', 'support', ...
    'innerFit', 'outerFit', 'innerDiaMm', 'outerDiaMm'};

for k = 1:numel(requiredFields)
    if ~isfield(loaded.endData, requiredFields{k}) || isempty(loaded.endData.(requiredFields{k}))
        return;
    end
end

if ~strcmp(loaded.endData.sourceSignature, sourceSignature)
    return;
end
if ~strcmp(loaded.endData.configFingerprint, configFingerprint)
    return;
end

cachedData = loaded.endData;
cachedData.label = endCfg.label;
cachedData.imgAbs = endCfg.imgAbs;
cachedData.outerFile = endCfg.outerCsv;
cachedData.innerFile = endCfg.innerCsv;
end


function save_endface_cache(endData, endCfg)
if ~isfield(endCfg, 'cacheMat') || isempty(endCfg.cacheMat)
    return;
end

save(endCfg.cacheMat, 'endData');
end


function write_endface_outputs_if_needed(endData, endCfg)
if exist(endCfg.outerCsv, 'file') && exist(endCfg.innerCsv, 'file')
    return;
end

if ~isfield(endData, 'debug') || ~isfield(endData.debug, 'outerMask') || ~isfield(endData.debug, 'innerMask')
    error('Cached end-face outputs are missing matrix masks and cannot be restored.');
end

if ~exist(endCfg.outerCsv, 'file')
    writematrix(endData.debug.outerMask, endCfg.outerCsv);
end
if ~exist(endCfg.innerCsv, 'file')
    writematrix(endData.debug.innerMask, endCfg.innerCsv);
end
end


function support = compute_end_inner_support(innerPts, outerPts, endPxPerMm)
ctrOut = mean(outerPts, 1);
relY = innerPts(:, 1) - ctrOut(1);

support.topRel = min(relY) / endPxPerMm;
support.botRel = max(relY) / endPxPerMm;
support.projDia = support.botRel - support.topRel;
support.centerRel = mean(relY) / endPxPerMm;
end


function signature = endface_source_signature(endCfg)
fileInfo = dir(endCfg.imgAbs);
if isempty(fileInfo)
    error('Missing end-face image: %s', endCfg.imgAbs);
end

signature = sprintf('%s|%d|%s', endCfg.imgAbs, fileInfo.bytes, fileInfo.date);
end


function fp = endface_config_fingerprint(cfg)
parts = [ ...
    cfg.canny.sigma, cfg.canny.low, cfg.canny.high, cfg.canny.iter, ...
    cfg.scale.endPxPerMm];
fp = sprintf('%.12g|', parts);
end


function outIdx = first_true_in_vector(vec, axisValues, searchForward)
if searchForward
    hitIdx = find(vec ~= 0, 1, 'first');
else
    hitIdx = find(vec ~= 0, 1, 'last');
end

if isempty(hitIdx)
    outIdx = nan;
    return;
end

outIdx = axisValues(hitIdx);
end


function mask = set_true_from_row_hits(mask, rowIdx, colIdx)
if isempty(rowIdx) || isempty(colIdx)
    return;
end

linIdx = sub2ind(size(mask), rowIdx(:), colIdx(:));
mask(linIdx) = true;
end


function radiiOut = sample_radial_contour(mask, centerRow, centerCol, baseRadius, padR, ang)
radiusVals = max(1, floor(baseRadius - padR)):max(1, ceil(baseRadius + padR));
[radiusGrid, angGrid] = ndgrid(radiusVals, ang);

rowGrid = round(centerRow + radiusGrid .* cos(angGrid));
colGrid = round(centerCol + radiusGrid .* sin(angGrid));
insideMask = rowGrid >= 1 & rowGrid <= size(mask, 1) & colGrid >= 1 & colGrid <= size(mask, 2);

rowGrid(~insideMask) = 1;
colGrid(~insideMask) = 1;

hitMask = false(size(rowGrid));
linearIdx = sub2ind(size(mask), rowGrid(insideMask), colGrid(insideMask));
hitMask(insideMask) = mask(linearIdx) ~= 0;

[hasHit, hitIdx] = max(hitMask, [], 1);
radiiOut = nan(1, numel(ang));
radiiOut(hasHit > 0) = radiusVals(hitIdx(hasHit > 0));

if any(hasHit > 0)
    fallbackRadius = median(radiiOut(hasHit > 0));
else
    fallbackRadius = round(baseRadius);
end
if ~isfinite(fallbackRadius)
    fallbackRadius = round(baseRadius);
end

radiiOut(hasHit == 0) = fallbackRadius;
end
