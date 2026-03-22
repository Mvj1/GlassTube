function [roiGray, topMask, botMask, pathTbl] = extract_side_edges_robust(cfg)
set(groot, 'defaultFigureUnits', 'normalized');
set(groot, 'defaultFigurePosition', [0, 0, 1, 1]);
set(groot, 'defaultFigureWindowState', 'maximized');
if ~exist(cfg.file.strip, 'file')
    error('Missing strip image: %s', cfg.file.strip);
end

img = imread(cfg.file.strip);
[roiGray, sideModel] = prepare_side_roi_and_model(img, cfg);
sideResult = detect_side_edges(roiGray, sideModel, cfg);

roiGray = sideResult.displayGray;
topMask = rows_to_mask(sideResult.top.finalPath, size(roiGray));
botMask = rows_to_mask(sideResult.bot.finalPath, size(roiGray));
pathTbl = build_side_path_table(sideResult.top, sideResult.bot, size(roiGray, 2));
end


function [roiGray, sideModel] = prepare_side_roi_and_model(img, cfg)
roiRect = [];
sideModel = struct();
needsCalibrate = cfg.sideEdge.forceRecalibrate || ~exist(cfg.file.sideCalib, 'file');

if ~needsCalibrate
    loaded = load(cfg.file.sideCalib, 'sideModel');
    if isfield(loaded, 'sideModel') && is_compatible_side_model(loaded.sideModel)
        sideModel = loaded.sideModel;
        roiRect = sideModel.roiRect;
    else
        needsCalibrate = true;
    end
end

if needsCalibrate
    [roiImg, roiRect] = pick_side_roi(img);
    sideModel = calibrate_side_edge_model(roiImg, cfg);
    sideModel.roiRect = roiRect;
    sideModel.schemaVersion = side_model_version();
    save(cfg.file.sideCalib, 'sideModel');
else
    roiImg = crop_side_roi(img, roiRect);
    if isempty(roiImg)
        error('Saved ROI is outside the current strip image. Delete %s and recalibrate.', cfg.file.sideCalib);
    end
end

roiGray = ensure_gray_double(roiImg);
end


function tf = is_compatible_side_model(sideModel)
requiredFields = {'schemaVersion', 'roiRect', 'baselineTop', 'baselineBot', ...
    'searchOutsidePx', 'searchInsidePx', 'edgeWindowPx', 'allowBandWithoutValley', ...
    'top', 'bottom'};
tf = isstruct(sideModel) && isfield(sideModel, 'schemaVersion') && ...
    sideModel.schemaVersion == side_model_version();

if ~tf
    return;
end

for idx = 1:numel(requiredFields)
    if ~isfield(sideModel, requiredFields{idx}) || isempty(sideModel.(requiredFields{idx}))
        tf = false;
        return;
    end
end

topFields = {'bandOffsetMinPx', 'bandOffsetMaxPx', 'firstEdgeMinDrop', 'valleyDepthThr'};
botFields = topFields;
for idx = 1:numel(topFields)
    if ~isfield(sideModel.top, topFields{idx}) || isempty(sideModel.top.(topFields{idx}))
        tf = false;
        return;
    end
    if ~isfield(sideModel.bottom, botFields{idx}) || isempty(sideModel.bottom.(botFields{idx}))
        tf = false;
        return;
    end
end
end


function version = side_model_version()
version = 3;
end


function [roiImg, roiRect] = pick_side_roi(img)
fig = figure('Name', 'ROI Selection', 'NumberTitle', 'off');
imshow(img);
title('Draw the side-view ROI and double-click inside the box to confirm.', ...
    'Color', 'r', 'FontSize', 12);
[roiImg, roiRect] = imcrop;
close(fig);

if isempty(roiImg)
    error('Side-view ROI was not selected.');
end
end


function roiImg = crop_side_roi(img, roiRect)
if numel(roiRect) ~= 4
    roiImg = [];
    return;
end

x1 = max(1, floor(roiRect(1)) + 1);
y1 = max(1, floor(roiRect(2)) + 1);
w = max(1, round(roiRect(3)));
h = max(1, round(roiRect(4)));
x2 = min(size(img, 2), x1 + w - 1);
y2 = min(size(img, 1), y1 + h - 1);

if x1 > size(img, 2) || y1 > size(img, 1) || x1 > x2 || y1 > y2
    roiImg = [];
    return;
end

roiImg = img(y1:y2, x1:x2, :);
end


function sideModel = calibrate_side_edge_model(roiImg, cfg)
roiGray = ensure_gray_double(roiImg);
prep = preprocess_side_roi(roiGray, cfg);
[rowN, colN] = size(roiGray);

colStart = max(1, round(colN * 0.08));
colStop = max(colStart + 5, round(colN * 0.92));
sampleCols = unique(round(linspace(colStart, colStop, max(15, ceil((colStop - colStart + 1) / cfg.sideEdge.calibColumnStep)))));

topBounds = [max(2, round(rowN * 0.03)), max(8, floor(rowN * 0.45))];
botBounds = [min(rowN - 8, ceil(rowN * 0.55)), max(rowN - 2, ceil(rowN * 0.55) + 5)];
blankMask = false(size(prep.gray));

topParams = edge_params_from_cfg(cfg.sideEdge, 'top');
botParams = edge_params_from_cfg(cfg.sideEdge, 'bottom');
topSeed = collect_edge_measurements(prep, sampleCols, 'top', topBounds, blankMask, topParams);
botSeed = collect_edge_measurements(prep, sampleCols, 'bottom', botBounds, blankMask, botParams);

if nnz(topSeed.valid) < 8 || nnz(botSeed.valid) < 8
    error('Side-edge calibration failed: not enough valid edge columns were found.');
end

sideModel.baselineTop = round(median(topSeed.edgeRows(topSeed.valid)));
sideModel.baselineBot = round(median(botSeed.edgeRows(botSeed.valid)));

if sideModel.baselineBot <= sideModel.baselineTop
    error('Side-edge calibration failed: top/bottom baselines overlap.');
end

topSpread = robust_mad(topSeed.edgeRows(topSeed.valid));
botSpread = robust_mad(botSeed.edgeRows(botSeed.valid));
sideModel.searchOutsidePx = max(cfg.sideEdge.searchOutsidePx, round(max(topSpread, botSpread)) + 6);
sideModel.searchInsidePx = cfg.sideEdge.searchInsidePx;
sideModel.edgeWindowPx = cfg.sideEdge.edgeWindowPx;
sideModel.allowBandWithoutValley = cfg.sideEdge.allowBandWithoutValley;
sideModel.maxJumpPerCol = cfg.sideEdge.maxJumpPerCol;
sideModel.smoothPenalty = cfg.sideEdge.smoothPenalty;
sideModel.marker = cfg.sideEdge.marker;

sideModel.top = calibrate_edge_params(topSeed, topParams);
sideModel.bottom = calibrate_edge_params(botSeed, botParams);
end


function edgeParams = calibrate_edge_params(seed, edgeParams)
dropVals = seed.dropVals(seed.valid);
valleyVals = seed.valleyDepths(seed.valid);
gradVals = seed.gradVals(seed.valid);
offsetVals = seed.bandOffsets(seed.valid);

edgeParams.firstEdgeMinDrop = max(edgeParams.firstEdgeMinDrop, 0.75 * prctile(dropVals, 20));
edgeParams.valleyDepthThr = max(edgeParams.valleyDepthThr, 0.75 * prctile(valleyVals, 20));
edgeParams.bandOffsetMinPx = max(edgeParams.bandOffsetMinPx, round(prctile(offsetVals, 20)));
edgeParams.bandOffsetMaxPx = max(edgeParams.bandOffsetMinPx + 2, round(prctile(offsetVals, 80)));
edgeParams.dropRef = max(prctile(dropVals, 50), edgeParams.firstEdgeMinDrop);
edgeParams.valleyRef = max(prctile(valleyVals, 50), edgeParams.valleyDepthThr);
edgeParams.gradRef = max([prctile(gradVals, 50), 0.5 * edgeParams.firstEdgeMinDrop, eps]);
end


function prep = preprocess_side_roi(roiGray, cfg)
gray = ensure_gray_double(roiGray);
bg = imgaussfilt(gray, cfg.sideEdge.bgSigma);
normGray = rescale(gray - bg);
smoothGray = imgaussfilt(normGray, cfg.sideEdge.edgeSigma);

prep.gray = gray;
prep.normGray = normGray;
prep.smoothGray = smoothGray;
prep.displayGray = smoothGray;
end


function seed = collect_edge_measurements(prep, sampleCols, edgeName, rowBounds, markerMask, edgeParams)
edgeRows = nan(numel(sampleCols), 1);
bandOffsets = nan(numel(sampleCols), 1);
dropVals = nan(numel(sampleCols), 1);
valleyDepths = nan(numel(sampleCols), 1);
gradVals = nan(numel(sampleCols), 1);
valid = false(numel(sampleCols), 1);

for idx = 1:numel(sampleCols)
    col = sampleCols(idx);
    cand = find_first_true_edge(prep.smoothGray(:, col), markerMask(:, col), edgeName, rowBounds, edgeParams);
    if ~cand.valid
        continue;
    end
    edgeRows(idx) = cand.row;
    bandOffsets(idx) = cand.bandOffset;
    dropVals(idx) = cand.drop;
    valleyDepths(idx) = cand.valleyDepth;
    gradVals(idx) = cand.localDelta;
    valid(idx) = true;
end

seed.edgeRows = edgeRows;
seed.bandOffsets = bandOffsets;
seed.dropVals = dropVals;
seed.valleyDepths = valleyDepths;
seed.gradVals = gradVals;
seed.valid = valid;
end


function sideResult = detect_side_edges(roiGray, sideModel, cfg)
prep = preprocess_side_roi(roiGray, cfg);
markerMask = detect_marker_regions(prep, sideModel, cfg);
colN = size(prep.gray, 2);

baseTop = sideModel.baselineTop * ones(1, colN);
baseBot = sideModel.baselineBot * ones(1, colN);

topPass1 = build_edge_candidates(prep, sideModel, 'top', markerMask, false(1, colN), baseTop, cfg);
botPass1 = build_edge_candidates(prep, sideModel, 'bottom', markerMask, false(1, colN), baseBot, cfg);
topTrace1 = trace_edge_path(topPass1, cfg);
botTrace1 = trace_edge_path(botPass1, cfg);

topMarker1 = detect_edge_marker_columns(markerMask, topTrace1.path, sideModel.baselineTop, cfg.sideEdge.marker.localRejectPx);
botMarker1 = detect_edge_marker_columns(markerMask, botTrace1.path, sideModel.baselineBot, cfg.sideEdge.marker.localRejectPx);

topFit1 = fit_edge_curve(topTrace1.path, topTrace1.conf, topTrace1.valid, topMarker1, cfg.sideEdge.fit, baseTop, cfg.sideEdge.fit.smoothTop);
botFit1 = fit_edge_curve(botTrace1.path, botTrace1.conf, botTrace1.valid, botMarker1, cfg.sideEdge.fit, baseBot, cfg.sideEdge.fit.smoothBot);

topPass2 = build_edge_candidates(prep, sideModel, 'top', markerMask, topMarker1, topFit1, cfg);
botPass2 = build_edge_candidates(prep, sideModel, 'bottom', markerMask, botMarker1, botFit1, cfg);
topTrace2 = trace_edge_path(topPass2, cfg);
botTrace2 = trace_edge_path(botPass2, cfg);

topMarker = detect_edge_marker_columns(markerMask, topFit1, sideModel.baselineTop, cfg.sideEdge.marker.localRejectPx);
botMarker = detect_edge_marker_columns(markerMask, botFit1, sideModel.baselineBot, cfg.sideEdge.marker.localRejectPx);
topMarker = topMarker | detect_edge_marker_columns(markerMask, topTrace2.path, sideModel.baselineTop, cfg.sideEdge.marker.localRejectPx);
botMarker = botMarker | detect_edge_marker_columns(markerMask, botTrace2.path, sideModel.baselineBot, cfg.sideEdge.marker.localRejectPx);
topReject = detect_inward_outlier_columns(topTrace2.path, topTrace2.conf, topTrace2.valid, topMarker, 'top', cfg.sideEdge.fit);
botReject = detect_inward_outlier_columns(botTrace2.path, botTrace2.conf, botTrace2.valid, botMarker, 'bottom', cfg.sideEdge.fit);
topBlocked = topMarker | topReject;
botBlocked = botMarker | botReject;

topFit = fit_edge_curve(topTrace2.path, topTrace2.conf, topTrace2.valid, topBlocked, cfg.sideEdge.fit, topFit1, cfg.sideEdge.fit.smoothTop);
botFit = fit_edge_curve(botTrace2.path, botTrace2.conf, botTrace2.valid, botBlocked, cfg.sideEdge.fit, botFit1, cfg.sideEdge.fit.smoothBot);

[topFinal, topConf, topValid, topGapFill, topBlendW, topZone] = fuse_edge_results( ...
    'top', topTrace2.path, topFit, topTrace2.conf, topTrace2.valid, topBlocked, cfg.sideEdge.fit);
[botFinal, botConf, botValid, botGapFill, botBlendW, botZone] = fuse_edge_results( ...
    'bottom', botTrace2.path, botFit, botTrace2.conf, botTrace2.valid, botBlocked, cfg.sideEdge.fit);
[topFinal, topZone, topBlendW] = enforce_top_diameter_floor( ...
    topFinal, botFinal, topValid, botValid, topBlocked, botBlocked, topZone, topBlendW, cfg.sideEdge.fit);

sideResult.displayGray = prep.displayGray;
sideResult.top.rawPath = topTrace2.path(:);
sideResult.top.fitPath = topFit(:);
sideResult.top.finalPath = topFinal(:);
sideResult.top.conf = topConf(:);
sideResult.top.valid = topValid(:);
sideResult.top.marker = topBlocked(:);
sideResult.top.gapFill = topGapFill(:);
sideResult.top.blendWeight = topBlendW(:);
sideResult.top.zone = topZone(:);

sideResult.bot.rawPath = botTrace2.path(:);
sideResult.bot.fitPath = botFit(:);
sideResult.bot.finalPath = botFinal(:);
sideResult.bot.conf = botConf(:);
sideResult.bot.valid = botValid(:);
sideResult.bot.marker = botBlocked(:);
sideResult.bot.gapFill = botGapFill(:);
sideResult.bot.blendWeight = botBlendW(:);
sideResult.bot.zone = botZone(:);
end


function markerMask = detect_marker_regions(prep, sideModel, cfg)
[rowN, colN] = size(prep.gray);
rowMin = max(1, sideModel.baselineTop - sideModel.searchOutsidePx);
rowMax = min(rowN, sideModel.baselineBot + sideModel.searchOutsidePx);

searchMask = false(rowN, colN);
searchMask(rowMin:rowMax, :) = true;

blackHat = imbothat(prep.gray, strel('disk', cfg.sideEdge.marker.structRadius));
vals = blackHat(searchMask);
thr = max(cfg.sideEdge.marker.darkThr, median(vals) + cfg.sideEdge.marker.madScale * robust_mad(vals));
markerMask = searchMask & blackHat >= thr;
markerMask = bwareaopen(markerMask, cfg.sideEdge.marker.minArea);
markerMask = imdilate(markerMask, strel('line', cfg.sideEdge.marker.edgeDilatePx, 0));
end


function markerCols = detect_edge_marker_columns(markerMask, guidePath, fallbackRow, localRejectPx)
colN = size(markerMask, 2);
rowN = size(markerMask, 1);
markerCols = false(1, colN);

for col = 1:colN
    center = fallbackRow;
    if col <= numel(guidePath) && isfinite(guidePath(col))
        center = guidePath(col);
    end
    r1 = max(1, round(center) - localRejectPx);
    r2 = min(rowN, round(center) + localRejectPx);
    markerCols(col) = any(markerMask(r1:r2, col));
end
end


function candidates = build_edge_candidates(prep, sideModel, edgeName, markerMask, blockedCols, priorCenter, cfg)
[rowN, colN] = size(prep.gray);
bandRows = build_runtime_band(edgeName, sideModel, priorCenter, rowN);
rowCount = numel(bandRows);

score = cfg.sideEdge.invalidScore * ones(rowCount, colN);
evidence = zeros(rowCount, colN);
valid = false(rowCount, colN);

for col = 1:colN
    center = priorCenter(min(col, numel(priorCenter)));
    if ~isfinite(center)
        center = default_row_for_edge(edgeName, sideModel);
    end
    fallbackIdx = clamp_row_to_band(center, bandRows);

    searchBounds = local_search_bounds(edgeName, center, sideModel, rowN);
    if mean(markerMask(searchBounds(1):searchBounds(2), col)) > cfg.sideEdge.marker.columnCoverageThr
        score(fallbackIdx, col) = cfg.sideEdge.invalidScore - cfg.sideEdge.markerPenalty;
        continue;
    end

    if blockedCols(min(col, numel(blockedCols)))
        score(fallbackIdx, col) = cfg.sideEdge.invalidScore - cfg.sideEdge.markerPenalty;
        continue;
    end

    edgeParams = edge_params_from_model(sideModel, edgeName);
    cand = find_first_true_edge(prep.smoothGray(:, col), markerMask(:, col), edgeName, searchBounds, edgeParams);

    if ~cand.valid
        score(fallbackIdx, col) = cfg.sideEdge.invalidScore;
        continue;
    end

    idx = clamp_row_to_band(cand.row, bandRows);
    centerPenalty = 0.03 * abs(cand.row - center);
    evidenceVal = compute_edge_evidence(cand.drop, cand.valleyDepth, cand.localDelta, edgeParams);
    score(idx, col) = cand.score - centerPenalty;
    evidence(idx, col) = evidenceVal;
    valid(idx, col) = true;

    if idx > 1
        score(idx - 1, col) = max(score(idx - 1, col), score(idx, col) - cfg.sideEdge.candidateNeighborPenalty);
        evidence(idx - 1, col) = max(evidence(idx - 1, col), max(0, evidenceVal - 0.15));
        valid(idx - 1, col) = true;
    end
    if idx < rowCount
        score(idx + 1, col) = max(score(idx + 1, col), score(idx, col) - cfg.sideEdge.candidateNeighborPenalty);
        evidence(idx + 1, col) = max(evidence(idx + 1, col), max(0, evidenceVal - 0.15));
        valid(idx + 1, col) = true;
    end
end

candidates.rows = bandRows(:);
candidates.score = score;
candidates.evidence = evidence;
candidates.valid = valid;
validCenter = priorCenter(isfinite(priorCenter));
if isempty(validCenter)
    candidates.baseline = default_row_for_edge(edgeName, sideModel);
else
    candidates.baseline = median(validCenter);
end
candidates.maxJumpPerCol = sideModel.maxJumpPerCol;
candidates.smoothPenalty = sideModel.smoothPenalty;
end


function bandRows = build_runtime_band(edgeName, sideModel, priorCenter, rowN)
validCenter = priorCenter(isfinite(priorCenter));
if isempty(validCenter)
    validCenter = default_row_for_edge(edgeName, sideModel);
end

if strcmp(edgeName, 'top')
    rowMin = max(1, floor(min(validCenter)) - sideModel.searchOutsidePx);
    rowMax = min(rowN, ceil(max(validCenter)) + sideModel.searchInsidePx);
else
    rowMin = max(1, floor(min(validCenter)) - sideModel.searchInsidePx);
    rowMax = min(rowN, ceil(max(validCenter)) + sideModel.searchOutsidePx);
end

bandRows = rowMin:rowMax;
end


function bounds = local_search_bounds(edgeName, center, sideModel, rowN)
if strcmp(edgeName, 'top')
    bounds = [max(1, round(center) - sideModel.searchOutsidePx), ...
        min(rowN, round(center) + sideModel.searchInsidePx)];
else
    bounds = [max(1, round(center) - sideModel.searchInsidePx), ...
        min(rowN, round(center) + sideModel.searchOutsidePx)];
end
end


function rowIdx = default_row_for_edge(edgeName, sideModel)
if strcmp(edgeName, 'top')
    rowIdx = sideModel.baselineTop;
else
    rowIdx = sideModel.baselineBot;
end
end


function idx = clamp_row_to_band(rowVal, bandRows)
rowVal = max(bandRows(1), min(bandRows(end), round(rowVal)));
idx = rowVal - bandRows(1) + 1;
end


function edgeParams = edge_params_from_cfg(sideCfg, edgeName)
edgeParams.edgeWindowPx = sideCfg.edgeWindowPx;
edgeParams.allowBandWithoutValley = sideCfg.allowBandWithoutValley;
edgeParams.searchOutsidePx = sideCfg.searchOutsidePx;
edgeParams.searchInsidePx = sideCfg.searchInsidePx;

if strcmp(edgeName, 'top')
    edgeParams.bandOffsetMinPx = sideCfg.bandOffsetMinPx;
    edgeParams.bandOffsetMaxPx = sideCfg.bandOffsetMaxPx;
    edgeParams.firstEdgeMinDrop = sideCfg.top.firstEdgeMinDrop;
    edgeParams.valleyDepthThr = sideCfg.top.valleyDepthThr;
    edgeParams = merge_struct(edgeParams, sideCfg.top);
else
    edgeParams.bandOffsetMinPx = sideCfg.bandOffsetMinPx;
    edgeParams.bandOffsetMaxPx = sideCfg.bandOffsetMaxPx;
    edgeParams.firstEdgeMinDrop = sideCfg.bottom.firstEdgeMinDrop;
    edgeParams.valleyDepthThr = sideCfg.bottom.valleyDepthThr;
    edgeParams = merge_struct(edgeParams, sideCfg.bottom);
end
edgeParams = ensure_edge_param_defaults(edgeParams);
end


function edgeParams = edge_params_from_model(sideModel, edgeName)
edgeParams.edgeWindowPx = sideModel.edgeWindowPx;
edgeParams.allowBandWithoutValley = sideModel.allowBandWithoutValley;

if strcmp(edgeName, 'top')
    edgeParams = merge_struct(edgeParams, sideModel.top);
else
    edgeParams = merge_struct(edgeParams, sideModel.bottom);
end
edgeParams = ensure_edge_param_defaults(edgeParams);
end


function out = merge_struct(a, b)
out = a;
fields = fieldnames(b);
for idx = 1:numel(fields)
    out.(fields{idx}) = b.(fields{idx});
end
end


function cand = find_first_true_edge(profile, markerCol, edgeName, rowBounds, params)
cand = struct('valid', false, 'row', nan, 'bandOffset', nan, ...
    'drop', 0, 'valleyDepth', 0, 'localDelta', 0, 'score', params.firstEdgeMinDrop * 0.25);
edgeOnlyCand = cand;

if strcmp(edgeName, 'top')
    scanRows = rowBounds(1):rowBounds(2);
else
    scanRows = rowBounds(2):-1:rowBounds(1);
end
scanStart = scanRows(1);

for row = scanRows
    if strcmp(edgeName, 'top') && ~edgeOnlyCand.valid && ...
            (row - scanStart) <= params.relaxedOuterMaxInwardPx
        relaxedInfo = compute_edge_windows(profile, markerCol, edgeName, row, params.edgeWindowPx, ...
            params.relaxedOuterDropRatio * params.firstEdgeMinDrop, params.relaxedOuterGradRatio);
        if relaxedInfo.valid
            edgeOnlyCand.valid = true;
            edgeOnlyCand.row = row;
            edgeOnlyCand.bandOffset = nan;
            edgeOnlyCand.drop = relaxedInfo.drop;
            edgeOnlyCand.valleyDepth = 0;
            edgeOnlyCand.localDelta = relaxedInfo.localDelta;
            edgeOnlyCand.score = relaxedInfo.drop + params.edgeOnlyLocalWeight * relaxedInfo.localDelta;
        end
    end

    edgeInfo = compute_edge_windows(profile, markerCol, edgeName, row, params.edgeWindowPx, ...
        params.firstEdgeMinDrop, 0.4);
    if ~edgeInfo.valid
        continue;
    end

    bandInfo = verify_inner_dark_band(profile, markerCol, edgeName, row, edgeInfo.outsideMean, params);
    if bandInfo.hasBand
        fullScore = edgeInfo.drop + params.bandScoreValleyWeight * bandInfo.valleyDepth + ...
            params.bandScoreStrongWeight * bandInfo.strongEdge;
        if edgeOnlyCand.valid && strcmp(edgeName, 'top') && ...
                ((row - edgeOnlyCand.row) >= params.preferOuterEdgeOnlyGapPx || ...
                fullScore <= edgeOnlyCand.score + params.preferOuterScoreMargin)
            cand = edgeOnlyCand;
            return;
        end
        cand.valid = true;
        cand.row = row;
        cand.bandOffset = bandInfo.offset;
        cand.drop = edgeInfo.drop;
        cand.valleyDepth = bandInfo.valleyDepth;
        cand.localDelta = edgeInfo.localDelta;
        cand.score = fullScore;
        return;
    end

    if params.allowBandWithoutValley && ~edgeOnlyCand.valid
        edgeOnlyCand.valid = true;
        edgeOnlyCand.row = row;
        edgeOnlyCand.bandOffset = bandInfo.offset;
        edgeOnlyCand.drop = edgeInfo.drop;
        edgeOnlyCand.valleyDepth = bandInfo.valleyDepth;
        edgeOnlyCand.localDelta = edgeInfo.localDelta;
        edgeOnlyCand.score = edgeInfo.drop + params.edgeOnlyStrongWeight * bandInfo.strongEdge + ...
            params.edgeOnlyLocalWeight * edgeInfo.localDelta;
    end
end

if edgeOnlyCand.valid
    cand = edgeOnlyCand;
end
end


function edgeInfo = compute_edge_windows(profile, markerCol, edgeName, row, windowPx, minDrop, localDeltaRatio)
edgeInfo = struct('valid', false, 'drop', 0, 'outsideMean', 0, 'localDelta', 0);
nRows = numel(profile);

if nargin < 6 || ~isfinite(minDrop)
    minDrop = 0;
end
if nargin < 7 || ~isfinite(localDeltaRatio) || localDeltaRatio < 0
    localDeltaRatio = 0.4;
end

if strcmp(edgeName, 'top')
    outsideRows = max(1, row - windowPx):row - 1;
    insideRows = row + 1:min(nRows, row + windowPx);
    localDelta = profile(max(1, row - 1)) - profile(min(nRows, row + 1));
else
    outsideRows = row + 1:min(nRows, row + windowPx);
    insideRows = max(1, row - windowPx):row - 1;
    localDelta = profile(min(nRows, row + 1)) - profile(max(1, row - 1));
end

if isempty(outsideRows) || isempty(insideRows)
    return;
end
if any(markerCol(outsideRows)) || any(markerCol(insideRows))
    return;
end

outsideMean = mean(profile(outsideRows));
insideMean = mean(profile(insideRows));
drop = outsideMean - insideMean;

if drop < minDrop
    return;
end
if localDelta < localDeltaRatio * drop
    return;
end

edgeInfo.valid = true;
edgeInfo.drop = drop;
edgeInfo.outsideMean = outsideMean;
edgeInfo.localDelta = localDelta;
end


function bandInfo = verify_inner_dark_band(profile, markerCol, edgeName, row, outsideMean, params)
bandInfo = struct('hasBand', false, 'offset', nan, 'valleyDepth', 0, 'strongEdge', 0);
nRows = numel(profile);

if strcmp(edgeName, 'top')
    bandRows = row + params.bandOffsetMinPx:min(nRows, row + params.bandOffsetMaxPx);
else
    bandRows = max(1, row - params.bandOffsetMaxPx):row - params.bandOffsetMinPx;
end

if isempty(bandRows)
    return;
end
if mean(markerCol(bandRows)) > 0.5
    return;
end

[valleyVal, valleyIdx] = min(profile(bandRows));
bandInfo.valleyDepth = outsideMean - valleyVal;
bandInfo.offset = abs(bandRows(valleyIdx) - row);

gradRows = max(1, bandRows(1) - 1):min(nRows, bandRows(end) + 1);
bandInfo.strongEdge = max(abs(diff(profile(gradRows))));
bandInfo.hasBand = bandInfo.valleyDepth >= params.valleyDepthThr && ...
    bandInfo.strongEdge >= params.strongEdgeRatio * params.firstEdgeMinDrop;
end


function trace = trace_edge_path(candidates, cfg)
rows = candidates.rows;
score = candidates.score;
evidenceMap = candidates.evidence;
validMap = candidates.valid;
[rowN, colN] = size(score);

dp = -inf(rowN, colN);
prevIdx = ones(rowN, colN, 'uint16');
initPenalty = abs(rows - candidates.baseline) / max(1, numel(rows));
dp(:, 1) = score(:, 1) - initPenalty;

for col = 2:colN
    prevCost = dp(:, col - 1);
    for row = 1:rowN
        delta = abs((1:rowN)' - row);
        transCost = prevCost - cfg.sideEdge.jumpPenalty * (delta .^ 2);
        overJump = max(0, delta - candidates.maxJumpPerCol);
        transCost = transCost - candidates.smoothPenalty * (overJump .^ 2);
        [bestCost, bestIdx] = max(transCost);
        dp(row, col) = score(row, col) + bestCost;
        prevIdx(row, col) = uint16(bestIdx);
    end
end

rowPath = zeros(1, colN, 'uint16');
rawScore = zeros(1, colN);
pickedValid = false(1, colN);
colIdx = 1:colN;

[~, rowPath(colN)] = max(dp(:, colN));
for col = colN:-1:2
    rowPath(col - 1) = prevIdx(rowPath(col), col);
end

linIdx = sub2ind(size(score), double(rowPath), colIdx);
rawScore(:) = score(linIdx);
pickedValid(:) = validMap(linIdx);

trace.path = rows(double(rowPath)).';
trace.conf = evidenceMap(linIdx);
trace.valid = pickedValid;
end


function fitPath = fit_edge_curve(rawPath, conf, valid, markerCols, fitCfg, fallbackPath, smoothSpan)
rawPath = rawPath(:).';
conf = conf(:).';
valid = valid(:).';
markerCols = markerCols(:).';
fallbackPath = fallbackPath(:).';
xAll = 1:numel(rawPath);

supportMask = valid & ~markerCols & isfinite(rawPath) & conf >= fitCfg.supportEvidenceThr;
if nnz(supportMask) < fitCfg.minSupportCols
    supportMask = valid & ~markerCols & isfinite(rawPath) & conf >= fitCfg.lowEvidenceThr;
end
if nnz(supportMask) < max(8, round(fitCfg.minSupportCols / 2))
    supportMask = valid & isfinite(rawPath) & conf >= fitCfg.lowEvidenceThr;
end
if nnz(supportMask) < 2
    fitPath = fallbackPath;
    return;
end

xSupport = xAll(supportMask);
ySupport = rawPath(supportMask);
fitPath = interp1(xSupport, ySupport, xAll, 'pchip', 'extrap');
fitPath = smoothdata(fitPath, 'rloess', make_odd_span(smoothSpan));
fitPath(supportMask) = 0.65 * ySupport + 0.35 * fitPath(supportMask);
fitPath = smoothdata(fitPath, 'rloess', make_odd_span(max(5, round(smoothSpan * 0.7))));
end


function [fusedPath, fusedConf, fusedValid, gapFill, blendWeight, zoneLabel] = fuse_edge_results(edgeName, rawPath, fitPath, conf, valid, markerCols, fitCfg)
rawPath = rawPath(:).';
fitPath = fitPath(:).';
conf = conf(:).';
valid = valid(:).';
markerCols = markerCols(:).';
residual = abs(rawPath - fitPath);
usableRaw = ~markerCols & isfinite(rawPath);
rawMask = usableRaw & residual <= fitCfg.rawResidualTolPx;
blendMask = usableRaw & residual > fitCfg.rawResidualTolPx & residual <= fitCfg.bridgeResidualTolPx;
gapMask = markerCols | ~usableRaw | residual > fitCfg.bridgeResidualTolPx;
inwardGuardMask = false(size(rawPath));

if strcmp(edgeName, 'top')
    inwardGuardMask = usableRaw & (fitPath - rawPath) >= fitCfg.inwardGuardTolPx;
elseif strcmp(edgeName, 'bottom')
    inwardGuardMask = usableRaw & (rawPath - fitPath) >= fitCfg.inwardGuardTolPx;
end

rawMask = rawMask | inwardGuardMask;
blendMask = blendMask & ~inwardGuardMask;
gapMask = gapMask & ~inwardGuardMask;

fusedPath = fitPath;
fusedConf = conf;
fusedValid = valid;
gapFill = false(size(rawPath));
blendWeight = ones(size(rawPath));
zoneLabel = repmat("gap_long", size(rawPath));

blendWeight(rawMask) = fitCfg.normalFitWeightMin;
fusedPath(rawMask) = (1 - fitCfg.normalFitWeightMin) .* rawPath(rawMask) + ...
    fitCfg.normalFitWeightMin .* fitPath(rawMask);
zoneLabel(rawMask) = "normal";

if any(blendMask)
    wFit = compute_blend_fit_weight(residual(blendMask), conf(blendMask), fitCfg);
    blendWeight(blendMask) = wFit;
    fusedPath(blendMask) = (1 - wFit) .* rawPath(blendMask) + wFit .* fitPath(blendMask);
    zoneLabel(blendMask) = "blend";
end

fusedValid(rawMask | blendMask) = true;
runList = find_invalid_runs(gapMask);

for idx = 1:size(runList, 1)
    startIdx = runList(idx, 1);
    endIdx = runList(idx, 2);
    runIdx = startIdx:endIdx;
    runLen = endIdx - startIdx + 1;

    if runLen <= fitCfg.maxGapFitOnly
        gapFill(runIdx) = true;
        fusedValid(runIdx) = true;
        fusedConf(runIdx) = max(fitCfg.lowEvidenceThr, min_neighbor_conf(conf, startIdx, endIdx) * 0.75);
        zoneLabel(runIdx) = "gap_short";
    else
        gapFill(runIdx) = true;
        fusedValid(runIdx) = false;
        fusedConf(runIdx) = 0.08;
        zoneLabel(runIdx) = "gap_long";
    end
end

zoneLabel(markerCols) = "marker";
blendWeight(gapMask) = 1;

if any(~isfinite(fusedPath))
    fusedPath = fillmissing(fusedPath, 'linear');
end
if any(~isfinite(fusedPath))
    fusedPath = fillmissing(rawPath, 'nearest');
end
fusedPath = fusedPath(:);
fusedConf = fusedConf(:);
fusedValid = fusedValid(:);
gapFill = gapFill(:);
blendWeight = blendWeight(:);
zoneLabel = zoneLabel(:);
end


function evidence = compute_edge_evidence(dropVal, valleyVal, gradVal, edgeParams)
eDrop = clamp_values(dropVal / max(edgeParams.dropRef, eps), 0, 1);
eValley = clamp_values(valleyVal / max(edgeParams.valleyRef, eps), 0, 1);
eGrad = clamp_values(gradVal / max(edgeParams.gradRef, eps), 0, 1);
evidence = 0.45 * eDrop + 0.35 * eValley + 0.20 * eGrad;
end


function rejectCols = detect_inward_outlier_columns(rawPath, conf, valid, blockedCols, edgeName, fitCfg)
rawPath = rawPath(:).';
conf = conf(:).';
valid = valid(:).';
blockedCols = blockedCols(:).';
rejectCols = false(size(rawPath));
halfWin = floor(make_odd_span(fitCfg.inwardOutlierWindow) / 2);

for idx = 1:numel(rawPath)
    if ~valid(idx) || blockedCols(idx) || ~isfinite(rawPath(idx))
        continue;
    end
    leftIdx = max(1, idx - halfWin);
    rightIdx = min(numel(rawPath), idx + halfWin);
    neighMask = valid(leftIdx:rightIdx) & ~blockedCols(leftIdx:rightIdx) & isfinite(rawPath(leftIdx:rightIdx));
    neighMask(idx - leftIdx + 1) = false;
    neighVals = rawPath(leftIdx:rightIdx);
    neighVals = neighVals(neighMask);
    if numel(neighVals) < fitCfg.inwardOutlierMinNeighbors
        continue;
    end

    localMed = median(neighVals);
    if strcmp(edgeName, 'top')
        inwardDelta = rawPath(idx) - localMed;
    else
        inwardDelta = localMed - rawPath(idx);
    end

    if inwardDelta >= fitCfg.inwardOutlierTolPx && conf(idx) <= fitCfg.inwardOutlierConfMax
        rejectCols(idx) = true;
    end
end
end


function [topPath, zoneLabel, blendWeight] = enforce_top_diameter_floor( ...
    topPath, botPath, topValid, botValid, topBlocked, botBlocked, zoneLabel, blendWeight, fitCfg)
topPath = topPath(:).';
botPath = botPath(:).';
topValid = topValid(:).';
botValid = botValid(:).';
topBlocked = topBlocked(:).';
botBlocked = botBlocked(:).';
zoneLabel = zoneLabel(:).';
blendWeight = blendWeight(:).';

dia = botPath - topPath;
halfWin = floor(make_odd_span(fitCfg.topDiameterWindow) / 2);

for idx = 1:numel(topPath)
    if ~topValid(idx) || ~botValid(idx) || topBlocked(idx) || botBlocked(idx) || ...
            ~isfinite(topPath(idx)) || ~isfinite(botPath(idx))
        continue;
    end

    leftIdx = max(1, idx - halfWin);
    rightIdx = min(numel(topPath), idx + halfWin);
    neighMask = topValid(leftIdx:rightIdx) & botValid(leftIdx:rightIdx) & ...
        ~topBlocked(leftIdx:rightIdx) & ~botBlocked(leftIdx:rightIdx) & ...
        isfinite(dia(leftIdx:rightIdx));
    neighMask(idx - leftIdx + 1) = false;
    neighDia = dia(leftIdx:rightIdx);
    neighDia = neighDia(neighMask);
    if numel(neighDia) < fitCfg.topDiameterMinNeighbors
        continue;
    end

    diaFloor = median(neighDia) - fitCfg.topDiameterShrinkTolPx;
    if dia(idx) < diaFloor
        topPath(idx) = botPath(idx) - diaFloor;
        blendWeight(idx) = max(blendWeight(idx), fitCfg.topDiameterBlendWeightMin);
        zoneLabel(idx) = "blend";
    end
end

topPath = topPath(:);
zoneLabel = zoneLabel(:);
blendWeight = blendWeight(:);
end


function edgeParams = ensure_edge_param_defaults(edgeParams)
if ~isfield(edgeParams, 'dropRef') || ~isfinite(edgeParams.dropRef) || edgeParams.dropRef <= 0
    edgeParams.dropRef = max(edgeParams.firstEdgeMinDrop, eps);
end
if ~isfield(edgeParams, 'valleyRef') || ~isfinite(edgeParams.valleyRef) || edgeParams.valleyRef <= 0
    edgeParams.valleyRef = max(edgeParams.valleyDepthThr, eps);
end
if ~isfield(edgeParams, 'gradRef') || ~isfinite(edgeParams.gradRef) || edgeParams.gradRef <= 0
    edgeParams.gradRef = max(0.5 * edgeParams.firstEdgeMinDrop, eps);
end
if ~isfield(edgeParams, 'strongEdgeRatio') || ~isfinite(edgeParams.strongEdgeRatio) || edgeParams.strongEdgeRatio <= 0
    edgeParams.strongEdgeRatio = 0.35;
end
if ~isfield(edgeParams, 'preferOuterEdgeOnlyGapPx') || ~isfinite(edgeParams.preferOuterEdgeOnlyGapPx) || edgeParams.preferOuterEdgeOnlyGapPx < 0
    edgeParams.preferOuterEdgeOnlyGapPx = 4;
end
if ~isfield(edgeParams, 'relaxedOuterDropRatio') || ~isfinite(edgeParams.relaxedOuterDropRatio) || edgeParams.relaxedOuterDropRatio <= 0
    edgeParams.relaxedOuterDropRatio = 0.6;
end
if ~isfield(edgeParams, 'relaxedOuterGradRatio') || ~isfinite(edgeParams.relaxedOuterGradRatio) || edgeParams.relaxedOuterGradRatio < 0
    edgeParams.relaxedOuterGradRatio = 0.18;
end
if ~isfield(edgeParams, 'relaxedOuterMaxInwardPx') || ~isfinite(edgeParams.relaxedOuterMaxInwardPx) || edgeParams.relaxedOuterMaxInwardPx < 0
    edgeParams.relaxedOuterMaxInwardPx = 12;
end
if ~isfield(edgeParams, 'preferOuterScoreMargin') || ~isfinite(edgeParams.preferOuterScoreMargin) || edgeParams.preferOuterScoreMargin < 0
    edgeParams.preferOuterScoreMargin = 0;
end
if ~isfield(edgeParams, 'bandScoreValleyWeight') || ~isfinite(edgeParams.bandScoreValleyWeight)
    edgeParams.bandScoreValleyWeight = 0.8;
end
if ~isfield(edgeParams, 'bandScoreStrongWeight') || ~isfinite(edgeParams.bandScoreStrongWeight)
    edgeParams.bandScoreStrongWeight = 0.2;
end
if ~isfield(edgeParams, 'edgeOnlyStrongWeight') || ~isfinite(edgeParams.edgeOnlyStrongWeight)
    edgeParams.edgeOnlyStrongWeight = 0.15;
end
if ~isfield(edgeParams, 'edgeOnlyLocalWeight') || ~isfinite(edgeParams.edgeOnlyLocalWeight)
    edgeParams.edgeOnlyLocalWeight = 0;
end
end


function wFit = compute_blend_fit_weight(residualVals, confVals, fitCfg)
if isempty(residualVals)
    wFit = zeros(size(residualVals));
    return;
end

residualDen = max(fitCfg.bridgeResidualTolPx - fitCfg.rawResidualTolPx, eps);
residualScale = clamp_values((residualVals - fitCfg.rawResidualTolPx) ./ residualDen, 0, 1);
wFit = min(fitCfg.blendFitWeightCap, 0.15 + 0.50 * residualScale .* (1 - confVals));
end


function confVal = min_neighbor_conf(conf, startIdx, endIdx)
leftVal = 0.25;
rightVal = 0.25;

if startIdx > 1 && isfinite(conf(startIdx - 1))
    leftVal = conf(startIdx - 1);
end
if endIdx < numel(conf) && isfinite(conf(endIdx + 1))
    rightVal = conf(endIdx + 1);
end

confVal = min(leftVal, rightVal);
end


function span = make_odd_span(spanIn)
span = max(5, round(spanIn));
if mod(span, 2) == 0
    span = span + 1;
end
end


function edgeMask = rows_to_mask(path, maskSize)
edgeMask = false(maskSize);
cols = 1:numel(path);
valid = isfinite(path);
rows = round(path(valid));
cols = cols(valid);
rows = max(1, min(maskSize(1), rows));
linIdx = sub2ind(maskSize, rows(:), cols(:));
edgeMask(linIdx) = true;
end


function pathTbl = build_side_path_table(topTrace, botTrace, colN)
pathTbl = table;
pathTbl.X = (1:colN)';
pathTbl.TopRaw = topTrace.rawPath(:);
pathTbl.BotRaw = botTrace.rawPath(:);
pathTbl.TopFit = topTrace.fitPath(:);
pathTbl.BotFit = botTrace.fitPath(:);
pathTbl.TopY = topTrace.finalPath(:);
pathTbl.BotY = botTrace.finalPath(:);
pathTbl.TopMarker = topTrace.marker(:);
pathTbl.BotMarker = botTrace.marker(:);
pathTbl.TopGapFill = topTrace.gapFill(:);
pathTbl.BotGapFill = botTrace.gapFill(:);
pathTbl.TopConf = topTrace.conf(:);
pathTbl.BotConf = botTrace.conf(:);
pathTbl.TopValid = topTrace.valid(:);
pathTbl.BotValid = botTrace.valid(:);
pathTbl.TopBlendW = topTrace.blendWeight(:);
pathTbl.BotBlendW = botTrace.blendWeight(:);
pathTbl.TopZone = topTrace.zone(:);
pathTbl.BotZone = botTrace.zone(:);
pathTbl.Dia = pathTbl.BotY - pathTbl.TopY;
pathTbl.Dia(~(pathTbl.TopValid & pathTbl.BotValid)) = nan;
end


function gray = ensure_gray_double(img)
if size(img, 3) == 3
    gray = rgb2gray(img);
else
    gray = img;
end
gray = im2double(gray);
end


function out = normalize_vector(vec)
vals = vec(isfinite(vec));
if isempty(vals)
    out = zeros(size(vec));
    return;
end

vMin = min(vals);
vMax = max(vals);
if vMax - vMin < eps
    out = ones(size(vec));
else
    out = max(0, min(1, (vec - vMin) / (vMax - vMin)));
end
end


function out = clamp_values(vals, lo, hi)
out = min(hi, max(lo, vals));
end


function runList = find_invalid_runs(mask)
mask = mask(:).';
padMask = [false, mask, false];
startIdx = find(diff(padMask) == 1);
endIdx = find(diff(padMask) == -1) - 1;
runList = [startIdx(:), endIdx(:)];
end


function val = robust_mad(x)
x = x(isfinite(x));
if isempty(x)
    val = 0;
    return;
end

xMed = median(x);
val = 1.4826 * median(abs(x - xMed));
if val < eps
    val = std(x);
end
end
