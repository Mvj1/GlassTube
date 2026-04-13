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
configFingerprint = side_config_fingerprint(cfg.sideEdge);
sourceSignature = side_calibration_source_signature(img, cfg);
needsCalibrate = cfg.sideEdge.forceRecalibrate || ~exist(cfg.file.sideCalib, 'file');

if ~needsCalibrate
    loaded = load(cfg.file.sideCalib, 'sideModel');
    if isfield(loaded, 'sideModel') && is_compatible_side_model(loaded.sideModel, configFingerprint, sourceSignature)
        sideModel = loaded.sideModel;
        roiRect = sideModel.roiRect;
    else
        if isfield(loaded, 'sideModel') && isstruct(loaded.sideModel) && isfield(loaded.sideModel, 'roiRect')
            roiRect = loaded.sideModel.roiRect;
        end
        needsCalibrate = true;
    end
end

if needsCalibrate
    roiImg = crop_side_roi(img, roiRect);
    if isempty(roiImg)
        [roiImg, roiRect] = pick_side_roi(img);
    end
    sideModel = calibrate_side_edge_model(roiImg, cfg);
    sideModel.roiRect = roiRect;
    sideModel.schemaVersion = side_model_version();
    sideModel.configFingerprint = configFingerprint;
    sideModel.sourceSignature = sourceSignature;
    save(cfg.file.sideCalib, 'sideModel');
else
    roiImg = crop_side_roi(img, roiRect);
    if isempty(roiImg)
        error('Saved ROI is outside the current strip image. Delete %s and recalibrate.', cfg.file.sideCalib);
    end
end

roiGray = ensure_gray_double(roiImg);
end


function tf = is_compatible_side_model(sideModel, configFingerprint, sourceSignature)
requiredFields = {'schemaVersion', 'roiRect', 'baselineTop', 'baselineBot', ...
    'searchOutsidePx', 'searchInsidePx', 'edgeWindowPx', 'allowBandWithoutValley', ...
    'top', 'bottom', 'configFingerprint', 'sourceSignature'};
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

tf = tf && strcmp(sideModel.configFingerprint, configFingerprint);
tf = tf && strcmp(sideModel.sourceSignature, sourceSignature);
end


function version = side_model_version()
version = 5;
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
strongVals = seed.strongVals(seed.valid);
offsetVals = seed.bandOffsets(seed.valid & isfinite(seed.bandOffsets));

edgeParams.firstEdgeMinDrop = max(edgeParams.firstEdgeMinDrop, 0.75 * prctile(dropVals, 20));
edgeParams.valleyDepthThr = max(edgeParams.valleyDepthThr, 0.75 * prctile(valleyVals, 20));
if ~isempty(offsetVals)
    edgeParams.bandOffsetMinPx = max(edgeParams.bandOffsetMinPx, round(prctile(offsetVals, 20)));
    edgeParams.bandOffsetMaxPx = max(edgeParams.bandOffsetMinPx + 2, round(prctile(offsetVals, 80)));
end
edgeParams.dropRef = max(prctile(dropVals, 50), edgeParams.firstEdgeMinDrop);
edgeParams.valleyRef = max(prctile(valleyVals, 50), edgeParams.valleyDepthThr);
edgeParams.gradRef = max([prctile(gradVals, 50), 0.5 * edgeParams.firstEdgeMinDrop, eps]);
edgeParams.outerRef = max(prctile(dropVals + 0.5 * gradVals, 50), edgeParams.firstEdgeMinDrop);
edgeParams.strongRef = max([prctile(strongVals, 50), edgeParams.strongEdgeRatio * edgeParams.firstEdgeMinDrop, eps]);
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
strongVals = nan(numel(sampleCols), 1);
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
    strongVals(idx) = cand.strongEdge;
    valid(idx) = true;
end

seed.edgeRows = edgeRows;
seed.bandOffsets = bandOffsets;
seed.dropVals = dropVals;
seed.valleyDepths = valleyDepths;
seed.gradVals = gradVals;
seed.strongVals = strongVals;
seed.valid = valid;
end


function sideResult = detect_side_edges(roiGray, sideModel, cfg)
prep = preprocess_side_roi(roiGray, cfg);
markerMask = detect_marker_regions(prep, sideModel, cfg);
colN = size(prep.gray, 2);
fitCfg = ensure_fit_cfg_defaults(cfg.sideEdge.fit);

baseTop = sideModel.baselineTop * ones(1, colN);
baseBot = sideModel.baselineBot * ones(1, colN);

topPass1 = build_edge_candidates(prep, sideModel, 'top', markerMask, false(1, colN), baseTop, cfg);
botPass1 = build_edge_candidates(prep, sideModel, 'bottom', markerMask, false(1, colN), baseBot, cfg);
topTrace1 = trace_edge_path(topPass1, cfg);
botTrace1 = trace_edge_path(botPass1, cfg);

topMarker1 = detect_edge_marker_columns(markerMask, topTrace1.path, sideModel.baselineTop, cfg.sideEdge.marker.localRejectPx);
botMarker1 = detect_edge_marker_columns(markerMask, botTrace1.path, sideModel.baselineBot, cfg.sideEdge.marker.localRejectPx);

topFit1 = fit_edge_curve(topTrace1.path, topTrace1.conf, topTrace1.valid, topMarker1, fitCfg, baseTop, fitCfg.smoothTop);
botFit1 = fit_edge_curve(botTrace1.path, botTrace1.conf, botTrace1.valid, botMarker1, fitCfg, baseBot, fitCfg.smoothBot);

topPass2 = build_edge_candidates(prep, sideModel, 'top', markerMask, topMarker1, topFit1, cfg);
botPass2 = build_edge_candidates(prep, sideModel, 'bottom', markerMask, botMarker1, botFit1, cfg);
topTrace2 = trace_edge_path(topPass2, cfg);
botTrace2 = trace_edge_path(botPass2, cfg);

topMarker = detect_edge_marker_columns(markerMask, topFit1, sideModel.baselineTop, cfg.sideEdge.marker.localRejectPx);
botMarker = detect_edge_marker_columns(markerMask, botFit1, sideModel.baselineBot, cfg.sideEdge.marker.localRejectPx);
topMarker = topMarker | detect_edge_marker_columns(markerMask, topTrace2.path, sideModel.baselineTop, cfg.sideEdge.marker.localRejectPx);
botMarker = botMarker | detect_edge_marker_columns(markerMask, botTrace2.path, sideModel.baselineBot, cfg.sideEdge.marker.localRejectPx);
topReject = detect_inward_outlier_columns(topTrace2.path, topTrace2.conf, topTrace2.valid, topMarker, 'top', fitCfg);
botReject = detect_inward_outlier_columns(botTrace2.path, botTrace2.conf, botTrace2.valid, botMarker, 'bottom', fitCfg);
topSuspect = detect_suspect_contamination_columns( ...
    topTrace2.path, topTrace2.conf, topTrace2.valid, topMarker, ...
    botTrace2.path, botTrace2.valid, botMarker, 'top', fitCfg);
botSuspect = detect_suspect_contamination_columns( ...
    botTrace2.path, botTrace2.conf, botTrace2.valid, botMarker, ...
    topTrace2.path, topTrace2.valid, topMarker, 'bottom', fitCfg);
topBlocked = topMarker | topReject | topSuspect | topPass2.suspectCols(:).';
botBlocked = botMarker | botReject | botSuspect | botPass2.suspectCols(:).';

topFit = fit_edge_curve(topTrace2.path, topTrace2.conf, topTrace2.valid, topBlocked, fitCfg, topFit1, fitCfg.smoothTop);
botFit = fit_edge_curve(botTrace2.path, botTrace2.conf, botTrace2.valid, botBlocked, fitCfg, botFit1, fitCfg.smoothBot);

[topFinal, topConf, topValid, topGapFill, topBlendW, topZone] = fuse_edge_results( ...
    'top', topTrace2.path, topFit, topTrace2.conf, topTrace2.valid, topBlocked, fitCfg);
[botFinal, botConf, botValid, botGapFill, botBlendW, botZone] = fuse_edge_results( ...
    'bottom', botTrace2.path, botFit, botTrace2.conf, botTrace2.valid, botBlocked, fitCfg);
[topFinal, botFinal, topZone, botZone, topBlendW, botBlendW] = enforce_diameter_guards( ...
    topFinal, botFinal, topValid, botValid, topBlocked, botBlocked, ...
    topZone, botZone, topBlendW, botBlendW, fitCfg);
[topFinal, topZone] = smooth_pixel_kinks(topFinal, topZone, topConf, 'top', fitCfg);
[botFinal, botZone] = smooth_pixel_kinks(botFinal, botZone, botConf, 'bottom', fitCfg);

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

if should_dump_candidate_debug(cfg)
    write_candidate_debug_table(topPass1, 'top', 1, cfg);
    write_candidate_debug_table(topPass2, 'top', 2, cfg);
    write_candidate_debug_table(botPass1, 'bottom', 1, cfg);
    write_candidate_debug_table(botPass2, 'bottom', 2, cfg);
end
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
edgeParams = edge_params_from_model(sideModel, edgeName);

score = cfg.sideEdge.invalidScore * ones(rowCount, colN);
evidence = zeros(rowCount, colN);
valid = false(rowCount, colN);
suspectCols = false(1, colN);
diag = init_candidate_diag(colN);

for col = 1:colN
    center = priorCenter(min(col, numel(priorCenter)));
    if ~isfinite(center)
        center = default_row_for_edge(edgeName, sideModel);
    end
    center = max(1, min(rowN, center));
    fallbackIdx = clamp_row_to_band(center, bandRows);

    searchBounds = local_search_bounds(edgeName, center, sideModel, rowN);
    diag.center(col) = center;
    diag.searchMin(col) = searchBounds(1);
    diag.searchMax(col) = searchBounds(2);
    diag.blocked(col) = blockedCols(min(col, numel(blockedCols)));
    if mean(markerMask(searchBounds(1):searchBounds(2), col)) > cfg.sideEdge.marker.columnCoverageThr
        score(fallbackIdx, col) = cfg.sideEdge.invalidScore - cfg.sideEdge.markerPenalty;
        diag.columnMasked(col) = true;
        continue;
    end

    if blockedCols(min(col, numel(blockedCols)))
        score(fallbackIdx, col) = cfg.sideEdge.invalidScore - cfg.sideEdge.markerPenalty;
        continue;
    end

    [cand, candSet] = find_first_true_edge(prep.smoothGray(:, col), markerMask(:, col), edgeName, searchBounds, edgeParams);
    cand = select_runtime_candidate(cand, candSet, center, cfg.sideEdge, edgeParams);
    retryExpanded = false;
    boundaryLocked = is_boundary_locked_candidate(cand, searchBounds, edgeName, center, edgeParams);
    if ~cand.valid || boundaryLocked
        expandedBounds = expanded_search_bounds(edgeName, center, sideModel, rowN);
        if any(expandedBounds ~= searchBounds)
            retryParams = edgeParams;
            if strcmp(edgeName, 'top')
                retryParams = relaxed_retry_edge_params(edgeParams);
            end
            [candEx, candSetEx] = find_first_true_edge(prep.smoothGray(:, col), markerMask(:, col), edgeName, expandedBounds, retryParams);
            candEx = select_runtime_candidate(candEx, candSetEx, center, cfg.sideEdge, edgeParams);
            if candEx.valid
                cand = candEx;
                candSet = candSetEx;
                searchBounds = expandedBounds;
                diag.searchMin(col) = expandedBounds(1);
                diag.searchMax(col) = expandedBounds(2);
                diag.expandedRetry(col) = true;
                retryExpanded = true;
                boundaryLocked = is_boundary_locked_candidate(cand, searchBounds, edgeName, center, edgeParams);
            end
        end
    end
    if boundaryLocked
        cand = make_empty_edge_candidate();
        candSet = struct('bestOuter', make_empty_edge_candidate(), 'bestBand', make_empty_edge_candidate(), ...
            'inwardGap', inf, 'outerWeak', false, 'bandOverrides', false);
        diag.boundaryRejected(col) = true;
    end
    diag = record_candidate_diag(diag, col, candSet, cand);

    if ~cand.valid
        score(fallbackIdx, col) = cfg.sideEdge.invalidScore;
        [score, evidence, valid, anchorInfo] = stamp_center_anchor(score, evidence, valid, bandRows, col, center, cand, candSet, edgeName, cfg.sideEdge, edgeParams);
        diag.anchorRow(col) = anchorInfo.row;
        diag.anchorScore(col) = anchorInfo.score;
        diag.anchorEvidence(col) = anchorInfo.evidence;
        diag.anchorSource(col) = string(anchorInfo.source);
        continue;
    end

    idx = clamp_row_to_band(cand.row, bandRows);
    bridgeFar = should_bridge_far_candidate(edgeName, cand, center, retryExpanded, cfg.sideEdge, edgeParams);
    centerPenalty = runtime_center_penalty(edgeName, cand.row, center, retryExpanded, bridgeFar, edgeParams);
    suspectPenalty = cfg.sideEdge.markerPenalty * double(cand.suspect);
    evidenceVal = cand.evidence;
    score(idx, col) = cand.score - centerPenalty - suspectPenalty;
    evidence(idx, col) = evidenceVal;
    valid(idx, col) = true;
    suspectCols(col) = cand.suspect;

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
    if bridgeFar
        [score, evidence, valid] = stamp_transition_bridge(score, evidence, valid, bandRows, col, center, cand, edgeName, score(idx, col), evidenceVal, cfg.sideEdge);
    end

    [score, evidence, valid, anchorInfo] = stamp_center_anchor(score, evidence, valid, bandRows, col, center, cand, candSet, edgeName, cfg.sideEdge, edgeParams);
    diag.anchorRow(col) = anchorInfo.row;
    diag.anchorScore(col) = anchorInfo.score;
    diag.anchorEvidence(col) = anchorInfo.evidence;
    diag.anchorSource(col) = string(anchorInfo.source);
end

candidates.rows = bandRows(:);
candidates.score = score;
candidates.evidence = evidence;
candidates.valid = valid;
candidates.suspectCols = suspectCols(:);
candidates.diag = diag;
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
center = max(1, min(rowN, center));
if strcmp(edgeName, 'top')
    bounds = [max(1, round(center) - sideModel.searchOutsidePx), ...
        min(rowN, round(center) + sideModel.searchInsidePx)];
else
    bounds = [max(1, round(center) - sideModel.searchInsidePx), ...
        min(rowN, round(center) + sideModel.searchOutsidePx)];
end
if bounds(1) > bounds(2)
    centerRow = max(1, min(rowN, round(center)));
    bounds = [centerRow, centerRow];
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


function [cand, candSet] = find_first_true_edge(profile, markerCol, edgeName, rowBounds, params)
cand = make_empty_edge_candidate();
bestOuter = cand;
bestBand = cand;
candSet = struct('bestOuter', bestOuter, 'bestBand', bestBand, ...
    'inwardGap', inf, 'outerWeak', false, 'bandOverrides', false);

if strcmp(edgeName, 'top')
    scanRows = rowBounds(1):rowBounds(2);
else
    scanRows = rowBounds(2):-1:rowBounds(1);
end
scanStart = scanRows(1);

for row = scanRows
    inwardOffset = inward_offset_from_outer(row, scanStart, edgeName);
    minDrop = params.firstEdgeMinDrop;
    localDeltaRatio = params.baseLocalDeltaRatio;
    if inwardOffset <= params.relaxedOuterMaxInwardPx
        minDrop = min(minDrop, params.relaxedOuterDropRatio * params.firstEdgeMinDrop);
        localDeltaRatio = min(localDeltaRatio, params.relaxedOuterGradRatio);
    end

    edgeInfo = compute_edge_windows(profile, markerCol, edgeName, row, params.edgeWindowPx, ...
        minDrop, localDeltaRatio);
    if ~edgeInfo.valid
        continue;
    end

    bandInfo = verify_inner_dark_band(profile, markerCol, edgeName, row, edgeInfo.outsideMean, params);
    outerEvidence = compute_outer_edge_evidence(edgeInfo.drop, edgeInfo.localDelta, params);
    if outerEvidence < params.minOuterEvidence
        continue;
    end

    bandEvidence = compute_band_evidence(bandInfo.valleyDepth, bandInfo.strongEdge, params);
    candNow = cand;
    candNow.valid = true;
    candNow.row = row;
    candNow.bandOffset = bandInfo.offset;
    candNow.drop = edgeInfo.drop;
    candNow.valleyDepth = bandInfo.valleyDepth;
    candNow.localDelta = edgeInfo.localDelta;
    candNow.strongEdge = bandInfo.strongEdge;
    candNow.outerEvidence = outerEvidence;
    candNow.bandEvidence = bandEvidence;
    candNow.hasBand = bandInfo.hasBand;
    candNow.inwardOffset = inwardOffset;

    outerScore = outerEvidence + outward_position_bonus(inwardOffset, params);
    candNow.evidence = outerEvidence;
    candNow.score = outerScore;
    if ~bestOuter.valid || is_better_candidate(candNow, bestOuter, true)
        bestOuter = candNow;
    end

    if bandInfo.hasBand || params.allowBandWithoutValley
        candNow.evidence = min(1, 0.7 * outerEvidence + 0.3 * bandEvidence);
        candNow.score = outerScore + params.bandEvidenceWeight * bandEvidence;
        if ~bestBand.valid || is_better_candidate(candNow, bestBand, false)
            bestBand = candNow;
        end
    end
end

if bestOuter.valid && ~bestBand.valid
    cand = bestOuter;
    candSet.bestOuter = bestOuter;
    return;
end
if bestBand.valid && ~bestOuter.valid
    cand = bestBand;
    cand.suspect = true;
    candSet.bestBand = cand;
    return;
end
if ~bestOuter.valid && ~bestBand.valid
    return;
end

inwardGap = inward_distance_between_rows(bestOuter.row, bestBand.row, edgeName);
outerWeak = bestOuter.outerEvidence <= (params.minOuterEvidence + params.outerEvidenceOverrideMargin);
bandOverrides = bestBand.score >= (bestOuter.score + params.bandSelectMargin);
bestBand.suspect = inwardGap > max(2, round(0.6 * params.maxInwardOverridePx));
candSet.bestOuter = bestOuter;
candSet.bestBand = bestBand;
candSet.inwardGap = inwardGap;
candSet.outerWeak = outerWeak;
candSet.bandOverrides = bandOverrides;

if inwardGap > params.maxInwardOverridePx && ~outerWeak
    cand = bestOuter;
    return;
end

if bandOverrides && (inwardGap <= params.maxInwardOverridePx || outerWeak)
    cand = bestBand;
else
    cand = bestOuter;
end
end


function bounds = expanded_search_bounds(edgeName, center, sideModel, rowN)
center = max(1, min(rowN, center));
extraInward = max(round(2.2 * sideModel.searchInsidePx), sideModel.searchInsidePx + 42);
extraOutside = max(sideModel.searchOutsidePx, round(1.3 * sideModel.searchOutsidePx));
if strcmp(edgeName, 'top')
    bounds = [max(1, round(center) - extraOutside), ...
        min(rowN, round(center) + extraInward)];
else
    bounds = [max(1, round(center) - extraInward), ...
        min(rowN, round(center) + extraOutside)];
end
if bounds(1) > bounds(2)
    centerRow = max(1, min(rowN, round(center)));
    bounds = [centerRow, centerRow];
end
end


function edgeParams = relaxed_retry_edge_params(edgeParams)
edgeParams.firstEdgeMinDrop = 0.72 * edgeParams.firstEdgeMinDrop;
edgeParams.minOuterEvidence = max(0.20, 0.78 * edgeParams.minOuterEvidence);
edgeParams.baseLocalDeltaRatio = max(0.14, 0.72 * edgeParams.baseLocalDeltaRatio);
edgeParams.relaxedOuterDropRatio = max(0.45, 0.85 * edgeParams.relaxedOuterDropRatio);
edgeParams.relaxedOuterGradRatio = max(0.10, 0.85 * edgeParams.relaxedOuterGradRatio);
end


function cand = select_runtime_candidate(cand, candSet, center, sideCfg, edgeParams)
choices = [candSet.bestOuter, candSet.bestBand];
bestIdx = 0;
bestScore = -inf;
scoreList = -inf(1, numel(choices));
distList = inf(1, numel(choices));

for idx = 1:numel(choices)
    candNow = choices(idx);
    if ~candNow.valid || ~isfinite(candNow.row)
        continue;
    end

    centerDist = abs(candNow.row - center);
    centerPenalty = 0.03 * centerDist;
    suspectPenalty = sideCfg.markerPenalty * double(candNow.suspect);
    scoreNow = candNow.score - centerPenalty - suspectPenalty;
    scoreList(idx) = scoreNow;
    distList(idx) = centerDist;
    if bestIdx == 0 || scoreNow > bestScore + 1e-6
        bestIdx = idx;
        bestScore = scoreNow;
        cand = candNow;
    elseif abs(scoreNow - bestScore) <= 1e-6 && candNow.inwardOffset < cand.inwardOffset
        bestIdx = idx;
        bestScore = scoreNow;
        cand = candNow;
    end
end

validIdx = find(isfinite(distList));
if numel(validIdx) < 2
    return;
end

[~, closePos] = min(distList(validIdx));
closeIdx = validIdx(closePos);
farIdx = validIdx(validIdx ~= closeIdx);
if isempty(farIdx)
    return;
end
farIdx = farIdx(1);

closeCand = choices(closeIdx);
farCand = choices(farIdx);
closeDist = distList(closeIdx);
farDist = distList(farIdx);
distGap = farDist - closeDist;

if closeIdx == 1 && closeCand.valid && isfinite(closeCand.row) && ...
        closeDist <= max(2, edgeParams.preferOuterEdgeOnlyGapPx - 1) && ...
        closeCand.evidence >= max(0.92, sideCfg.minConfidence + 0.45) && ...
        distGap >= max(2, edgeParams.preferOuterEdgeOnlyGapPx - 1)
    scoreGap = scoreList(farIdx) - scoreList(closeIdx);
    outerHoldMargin = edgeParams.preferOuterScoreMargin + 0.08 + 0.02 * distGap;
    if scoreGap <= outerHoldMargin
        cand = closeCand;
        return;
    end
end

if distGap < edgeParams.preferOuterEdgeOnlyGapPx
    return;
end

farTooFar = farDist >= max(edgeParams.maxInwardOverridePx, edgeParams.preferOuterEdgeOnlyGapPx + 1);
if ~farTooFar
    return;
end

scoreGap = scoreList(farIdx) - scoreList(closeIdx);
escapeMargin = edgeParams.preferOuterScoreMargin + 0.02 * distGap + 0.10 * double(farCand.suspect);
if scoreGap <= escapeMargin
    cand = closeCand;
end
end


function penalty = runtime_center_penalty(edgeName, rowVal, center, retryExpanded, bridgeFar, edgeParams)
gap = abs(rowVal - center);
if retryExpanded && strcmp(edgeName, 'top')
    penalty = 0.03 * min(gap, max(edgeParams.maxInwardOverridePx + 4, 12));
elseif bridgeFar && strcmp(edgeName, 'bottom')
    penalty = 0.03 * min(gap, max(edgeParams.maxInwardOverridePx + 4, 12));
else
    penalty = 0.03 * gap;
end
end


function tf = should_bridge_far_candidate(edgeName, cand, center, retryExpanded, sideCfg, edgeParams)
tf = false;
if ~cand.valid || ~isfinite(cand.row) || ~isfinite(center)
    return;
end

gap = abs(cand.row - center);
gapThr = max(sideCfg.maxJumpPerCol + 3, edgeParams.maxInwardOverridePx + 2);
if gap < gapThr
    return;
end

if retryExpanded && strcmp(edgeName, 'top')
    tf = true;
    return;
end

if strcmp(edgeName, 'bottom') && ~retryExpanded && cand.evidence >= max(sideCfg.minConfidence, 0.45)
    tf = true;
end
end


function [score, evidence, valid, anchorInfo] = stamp_center_anchor(score, evidence, valid, bandRows, col, center, cand, candSet, edgeName, sideCfg, edgeParams)
anchorInfo = struct('row', nan, 'score', nan, 'evidence', nan, 'source', "none");
if ~isfinite(center)
    return;
end

if ~cand.valid || ~isfinite(cand.row)
    anchorRow = center;
    anchorEvidence = max(0.18, 0.55 * sideCfg.minConfidence);
    anchorScore = max(0.16, sideCfg.minCandidateScore + 0.08);
    anchorIdx = clamp_row_to_band(anchorRow, bandRows);
    if anchorScore > score(anchorIdx, col) + 1e-6
        score(anchorIdx, col) = anchorScore;
        evidence(anchorIdx, col) = max(evidence(anchorIdx, col), anchorEvidence);
        valid(anchorIdx, col) = true;
        anchorInfo.row = anchorRow;
        anchorInfo.score = anchorScore;
        anchorInfo.evidence = anchorEvidence;
        anchorInfo.source = "center_gap";
    end
    return;
end

centerGap = abs(cand.row - center);
anchorGapThr = max(edgeParams.preferOuterEdgeOnlyGapPx + 1, edgeParams.maxInwardOverridePx);
if centerGap < anchorGapThr
    return;
end

anchorRow = nan;
anchorEvidence = 0;
anchorScore = -inf;
choices = [candSet.bestOuter, candSet.bestBand];
for idx = 1:numel(choices)
    candNow = choices(idx);
    if ~candNow.valid || ~isfinite(candNow.row)
        continue;
    end
    distNow = abs(candNow.row - center);
    if distNow > 2
        continue;
    end
    scoreNow = candNow.score - 0.03 * distNow - sideCfg.markerPenalty * double(candNow.suspect);
    if scoreNow > anchorScore
        anchorRow = candNow.row;
        anchorEvidence = candNow.evidence;
        anchorScore = scoreNow;
        anchorInfo.source = string(candidate_source_label(candNow, candSet));
    end
end

if ~isfinite(anchorRow)
    anchorRow = center;
    anchorEvidence = max(0.18, 0.65 * cand.evidence);
    anchorScore = cand.score - 0.20 - 0.02 * centerGap;
    if strcmp(edgeName, 'bottom') && cand.evidence >= max(0.80, sideCfg.minConfidence + 0.35) && centerGap >= anchorGapThr
        anchorScore = min(anchorScore, cand.score - 0.55 - 0.03 * centerGap);
    end
    anchorInfo.source = "center";
end

anchorIdx = clamp_row_to_band(anchorRow, bandRows);
if anchorScore <= score(anchorIdx, col) + 1e-6
    return;
end

score(anchorIdx, col) = anchorScore;
evidence(anchorIdx, col) = max(evidence(anchorIdx, col), anchorEvidence);
valid(anchorIdx, col) = true;
anchorInfo.row = anchorRow;
anchorInfo.score = anchorScore;
anchorInfo.evidence = anchorEvidence;
if anchorInfo.source == "none"
    anchorInfo.source = "center";
end
end


function [score, evidence, valid] = stamp_transition_bridge(score, evidence, valid, bandRows, col, center, cand, edgeName, candScore, candEvidence, sideCfg)
if ~cand.valid || ~isfinite(cand.row) || ~isfinite(center)
    return;
end

rowStart = round(center);
rowStop = round(cand.row);
if abs(rowStop - rowStart) <= max(2, sideCfg.maxJumpPerCol)
    return;
end

stepDir = sign(rowStop - rowStart);
if strcmp(edgeName, 'bottom')
    bridgeRows = rowStart + stepDir : stepDir : rowStop - stepDir;
else
    bridgeRows = rowStart + stepDir * sideCfg.maxJumpPerCol : stepDir * sideCfg.maxJumpPerCol : rowStop - stepDir;
end
gapTotal = max(abs(rowStop - rowStart), 1);
for rowVal = bridgeRows
    idx = clamp_row_to_band(rowVal, bandRows);
    distToTarget = abs(rowStop - rowVal);
    if strcmp(edgeName, 'bottom')
        distFromCenter = abs(rowVal - rowStart);
        bridgeScore = candScore - 0.02 - 0.008 * distToTarget - 0.002 * distFromCenter;
        evidenceScale = 1 - 0.20 * (distToTarget / gapTotal);
        bridgeEvidence = max(0.22, evidenceScale * candEvidence);
    else
        bridgeScore = candScore - 0.03 - 0.012 * distToTarget;
        bridgeEvidence = max(0.20, 0.72 * candEvidence);
    end
    if bridgeScore <= score(idx, col) + 1e-6
        continue;
    end
    score(idx, col) = bridgeScore;
    evidence(idx, col) = max(evidence(idx, col), bridgeEvidence);
    valid(idx, col) = true;
end
end


function diag = init_candidate_diag(colN)
diag = struct();
diag.center = nan(colN, 1);
diag.searchMin = nan(colN, 1);
diag.searchMax = nan(colN, 1);
diag.blocked = false(colN, 1);
diag.columnMasked = false(colN, 1);
diag.expandedRetry = false(colN, 1);
diag.boundaryRejected = false(colN, 1);
diag.bestOuterRow = nan(colN, 1);
diag.bestOuterScore = nan(colN, 1);
diag.bestOuterEvidence = nan(colN, 1);
diag.bestOuterInward = nan(colN, 1);
diag.bestBandRow = nan(colN, 1);
diag.bestBandScore = nan(colN, 1);
diag.bestBandEvidence = nan(colN, 1);
diag.bestBandInward = nan(colN, 1);
diag.bestBandSuspect = false(colN, 1);
diag.inwardGap = nan(colN, 1);
diag.outerWeak = false(colN, 1);
diag.bandOverrides = false(colN, 1);
diag.chosenRow = nan(colN, 1);
diag.chosenScore = nan(colN, 1);
diag.chosenEvidence = nan(colN, 1);
diag.chosenInward = nan(colN, 1);
diag.chosenSuspect = false(colN, 1);
diag.chosenSource = strings(colN, 1);
diag.anchorRow = nan(colN, 1);
diag.anchorScore = nan(colN, 1);
diag.anchorEvidence = nan(colN, 1);
diag.anchorSource = strings(colN, 1);
end


function diag = record_candidate_diag(diag, col, candSet, cand)
if candSet.bestOuter.valid
    diag.bestOuterRow(col) = candSet.bestOuter.row;
    diag.bestOuterScore(col) = candSet.bestOuter.score;
    diag.bestOuterEvidence(col) = candSet.bestOuter.evidence;
    diag.bestOuterInward(col) = candSet.bestOuter.inwardOffset;
end
if candSet.bestBand.valid
    diag.bestBandRow(col) = candSet.bestBand.row;
    diag.bestBandScore(col) = candSet.bestBand.score;
    diag.bestBandEvidence(col) = candSet.bestBand.evidence;
    diag.bestBandInward(col) = candSet.bestBand.inwardOffset;
    diag.bestBandSuspect(col) = candSet.bestBand.suspect;
end
diag.inwardGap(col) = candSet.inwardGap;
diag.outerWeak(col) = candSet.outerWeak;
diag.bandOverrides(col) = candSet.bandOverrides;
if cand.valid
    diag.chosenRow(col) = cand.row;
    diag.chosenScore(col) = cand.score;
    diag.chosenEvidence(col) = cand.evidence;
    diag.chosenInward(col) = cand.inwardOffset;
    diag.chosenSuspect(col) = cand.suspect;
    diag.chosenSource(col) = string(candidate_source_label(cand, candSet));
end
end


function tf = is_boundary_locked_candidate(cand, searchBounds, edgeName, center, edgeParams)
tf = false;
if ~cand.valid || ~isfinite(cand.row) || ~isfinite(center)
    return;
end

gapThr = max(edgeParams.maxInwardOverridePx, edgeParams.preferOuterEdgeOnlyGapPx + 2);
if abs(cand.row - center) < gapThr
    return;
end

if strcmp(edgeName, 'top')
    inwardBoundary = searchBounds(2);
else
    inwardBoundary = searchBounds(1);
end

tf = abs(cand.row - inwardBoundary) <= 1;
end


function label = candidate_source_label(cand, candSet)
label = "other";
if cand.valid && candSet.bestOuter.valid && cand.row == candSet.bestOuter.row && abs(cand.score - candSet.bestOuter.score) <= 1e-6
    label = "outer";
    return;
end
if cand.valid && candSet.bestBand.valid && cand.row == candSet.bestBand.row && abs(cand.score - candSet.bestBand.score) <= 1e-6
    label = "band";
end
end


function tf = should_dump_candidate_debug(cfg)
tf = isfield(cfg, 'sideEdge') && isfield(cfg.sideEdge, 'debug') && ...
    isfield(cfg.sideEdge.debug, 'dumpCandidates') && cfg.sideEdge.debug.dumpCandidates;
end


function write_candidate_debug_table(candidates, edgeName, passIdx, cfg)
if ~isfield(candidates, 'diag')
    return;
end

diag = candidates.diag;
tbl = table;
colN = numel(diag.center);
tbl.X = (1:colN)';
tbl.Center = diag.center;
tbl.SearchMin = diag.searchMin;
tbl.SearchMax = diag.searchMax;
tbl.Blocked = diag.blocked;
tbl.ColumnMasked = diag.columnMasked;
tbl.BestOuterRow = diag.bestOuterRow;
tbl.BestOuterScore = diag.bestOuterScore;
tbl.BestOuterEvidence = diag.bestOuterEvidence;
tbl.BestOuterInward = diag.bestOuterInward;
tbl.BestBandRow = diag.bestBandRow;
tbl.BestBandScore = diag.bestBandScore;
tbl.BestBandEvidence = diag.bestBandEvidence;
tbl.BestBandInward = diag.bestBandInward;
tbl.BestBandSuspect = diag.bestBandSuspect;
tbl.InwardGap = diag.inwardGap;
tbl.OuterWeak = diag.outerWeak;
tbl.BandOverrides = diag.bandOverrides;
tbl.ChosenRow = diag.chosenRow;
tbl.ChosenScore = diag.chosenScore;
tbl.ChosenEvidence = diag.chosenEvidence;
tbl.ChosenInward = diag.chosenInward;
tbl.ChosenSuspect = diag.chosenSuspect;
tbl.ChosenSource = diag.chosenSource;
tbl.AnchorRow = diag.anchorRow;
tbl.AnchorScore = diag.anchorScore;
tbl.AnchorEvidence = diag.anchorEvidence;
tbl.AnchorSource = diag.anchorSource;

baseName = cfg.sideEdge.debug.candidatePrefix;
filePath = sprintf('%s_%s_pass%d.csv', baseName, edgeName, passIdx);
writetable(tbl, filePath);
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
fitRef = [ySupport(:); fallbackPath(isfinite(fallbackPath)).'];
if ~isempty(fitRef)
    fitMin = min(fitRef) - 2 * fitCfg.bridgeResidualTolPx;
    fitMax = max(fitRef) + 2 * fitCfg.bridgeResidualTolPx;
    fitPath = max(fitMin, min(fitMax, fitPath));
end
end


function [fusedPath, fusedConf, fusedValid, gapFill, blendWeight, zoneLabel] = fuse_edge_results(edgeName, rawPath, fitPath, conf, valid, blockedCols, fitCfg)
rawPath = rawPath(:).';
fitPath = fitPath(:).';
conf = conf(:).';
valid = valid(:).';
blockedCols = blockedCols(:).';
residual = abs(rawPath - fitPath);
usableRaw = isfinite(rawPath);
rawMask = usableRaw & residual <= fitCfg.rawResidualTolPx;
blendMask = usableRaw & residual > fitCfg.rawResidualTolPx & residual <= fitCfg.bridgeResidualTolPx;
gapMask = blockedCols | ~usableRaw | residual > fitCfg.bridgeResidualTolPx;
inwardGuardMask = false(size(rawPath));
trendProtectMask = detect_trend_protected_cols(edgeName, rawPath, fitPath, conf, blockedCols, fitCfg);

if strcmp(edgeName, 'top')
    inwardGuardMask = usableRaw & (fitPath - rawPath) >= fitCfg.inwardGuardTolPx;
elseif strcmp(edgeName, 'bottom')
    inwardGuardMask = usableRaw & (fitPath - rawPath) >= fitCfg.inwardGuardTolPx;
end

rawMask = rawMask | inwardGuardMask;
rawMask = rawMask | trendProtectMask;
blendMask = blendMask & ~rawMask;
gapMask = gapMask & ~rawMask;

fusedPath = fitPath;
fusedConf = conf;
fusedValid = valid;
gapFill = false(size(rawPath));
blendWeight = ones(size(rawPath));
zoneLabel = repmat("gap_long", size(rawPath));

protectedRawMask = inwardGuardMask | trendProtectMask;
normalRawMask = rawMask & ~protectedRawMask;
blendWeight(normalRawMask) = fitCfg.normalFitWeightMin;
fusedPath(normalRawMask) = (1 - fitCfg.normalFitWeightMin) .* rawPath(normalRawMask) + ...
    fitCfg.normalFitWeightMin .* fitPath(normalRawMask);
zoneLabel(normalRawMask) = "normal";

if any(inwardGuardMask)
    fusedPath(inwardGuardMask) = rawPath(inwardGuardMask);
    blendWeight(inwardGuardMask) = 0;
    fusedValid(inwardGuardMask) = true;
    zoneLabel(inwardGuardMask) = "raw_protected";
end

trendOnlyMask = trendProtectMask & ~inwardGuardMask;
if any(trendOnlyMask)
    fusedPath(trendOnlyMask) = rawPath(trendOnlyMask);
    blendWeight(trendOnlyMask) = 0;
    fusedValid(trendOnlyMask) = true;
    zoneLabel(trendOnlyMask) = "trend_protected";
end

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

zoneLabel(blockedCols & ~protectedRawMask) = "marker";
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


function cand = make_empty_edge_candidate()
cand = struct('valid', false, 'row', nan, 'bandOffset', nan, ...
    'drop', 0, 'valleyDepth', 0, 'localDelta', 0, 'strongEdge', 0, ...
    'outerEvidence', 0, 'bandEvidence', 0, 'evidence', 0, 'score', 0, ...
    'hasBand', false, 'suspect', false, 'inwardOffset', inf);
end


function evidence = compute_outer_edge_evidence(dropVal, gradVal, edgeParams)
eDrop = clamp_values(dropVal / max(edgeParams.dropRef, eps), 0, 1);
eGrad = clamp_values(gradVal / max(edgeParams.gradRef, eps), 0, 1);
evidence = 0.72 * eDrop + 0.28 * eGrad;
end


function evidence = compute_band_evidence(valleyVal, strongVal, edgeParams)
eValley = clamp_values(valleyVal / max(edgeParams.valleyRef, eps), 0, 1);
eStrong = clamp_values(strongVal / max(edgeParams.strongRef, eps), 0, 1);
evidence = 0.70 * eValley + 0.30 * eStrong;
end


function bonus = outward_position_bonus(inwardOffset, edgeParams)
bonus = edgeParams.outerBiasWeight * max(0, 1 - inwardOffset / max(edgeParams.maxInwardOverridePx, 1));
end


function tf = is_better_candidate(candNow, candRef, preferOuter)
if candNow.score > candRef.score + 1e-6
    tf = true;
    return;
end
if candNow.score < candRef.score - 1e-6
    tf = false;
    return;
end

if preferOuter
    if candNow.inwardOffset ~= candRef.inwardOffset
        tf = candNow.inwardOffset < candRef.inwardOffset;
        return;
    end
end

if candNow.outerEvidence ~= candRef.outerEvidence
    tf = candNow.outerEvidence > candRef.outerEvidence;
else
    tf = candNow.bandEvidence > candRef.bandEvidence;
end
end


function offset = inward_offset_from_outer(row, scanStart, edgeName)
if strcmp(edgeName, 'top')
    offset = row - scanStart;
else
    offset = scanStart - row;
end
offset = max(0, offset);
end


function offset = inward_distance_between_rows(outerRow, innerRow, edgeName)
if strcmp(edgeName, 'top')
    offset = innerRow - outerRow;
else
    offset = outerRow - innerRow;
end
offset = max(0, offset);
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


function suspectCols = detect_suspect_contamination_columns( ...
    rawPath, conf, valid, markerCols, otherPath, otherValid, otherBlocked, edgeName, fitCfg)
rawPath = rawPath(:).';
conf = conf(:).';
valid = valid(:).';
markerCols = markerCols(:).';
otherPath = otherPath(:).';
otherValid = otherValid(:).';
otherBlocked = otherBlocked(:).';
suspectCols = false(size(rawPath));
halfWin = floor(make_odd_span(fitCfg.diameterGuardWindow) / 2);

for idx = 1:numel(rawPath)
    if ~valid(idx) || ~isfinite(rawPath(idx))
        continue;
    end
    leftIdx = max(1, idx - halfWin);
    rightIdx = min(numel(rawPath), idx + halfWin);

    neighMask = valid(leftIdx:rightIdx) & ~markerCols(leftIdx:rightIdx) & isfinite(rawPath(leftIdx:rightIdx));
    neighMask(idx - leftIdx + 1) = false;
    neighVals = rawPath(leftIdx:rightIdx);
    neighVals = neighVals(neighMask);
    if numel(neighVals) < fitCfg.diameterGuardMinNeighbors
        continue;
    end

    localMed = median(neighVals);
    if strcmp(edgeName, 'top')
        inwardDelta = rawPath(idx) - localMed;
    else
        inwardDelta = localMed - rawPath(idx);
    end
    if inwardDelta < fitCfg.inwardGuardTolPx
        continue;
    end

    diaShrink = 0;
    if idx <= numel(otherPath) && otherValid(idx) && ~otherBlocked(idx) && isfinite(otherPath(idx))
        localOtherMask = otherValid(leftIdx:rightIdx) & ~otherBlocked(leftIdx:rightIdx) & ...
            isfinite(otherPath(leftIdx:rightIdx));
        localOtherMask = localOtherMask & neighMask;
        if strcmp(edgeName, 'top')
            diaVals = otherPath(leftIdx:rightIdx) - rawPath(leftIdx:rightIdx);
            curDia = otherPath(idx) - rawPath(idx);
        else
            diaVals = rawPath(leftIdx:rightIdx) - otherPath(leftIdx:rightIdx);
            curDia = rawPath(idx) - otherPath(idx);
        end
        diaVals = diaVals(localOtherMask);
        if numel(diaVals) >= fitCfg.diameterGuardMinNeighbors
            diaShrink = median(diaVals) - curDia;
        end
    end

    markerNear = markerCols(idx);
    weakConf = conf(idx) <= fitCfg.inwardOutlierConfMax;
    if (markerNear || weakConf || diaShrink >= fitCfg.diameterGuardTolPx) && ...
            inwardDelta >= fitCfg.inwardGuardTolPx
        suspectCols(idx) = true;
    end
end

suspectCols = merge_close_runs(suspectCols, fitCfg.suspiciousBridgeGap);
suspectCols = keep_min_run_length(suspectCols, fitCfg.suspiciousRunMinLen);
suspectCols = suspectCols(:).';
end


function [topPath, botPath, topZone, botZone, topBlendWeight, botBlendWeight] = enforce_diameter_guards( ...
    topPath, botPath, topValid, botValid, topBlocked, botBlocked, ...
    topZone, botZone, topBlendWeight, botBlendWeight, fitCfg)
topPath = topPath(:).';
botPath = botPath(:).';
topValid = topValid(:).';
botValid = botValid(:).';
topBlocked = topBlocked(:).';
botBlocked = botBlocked(:).';
topZone = topZone(:).';
botZone = botZone(:).';
topBlendWeight = topBlendWeight(:).';
botBlendWeight = botBlendWeight(:).';

dia = botPath - topPath;
halfWin = floor(make_odd_span(fitCfg.diameterGuardWindow) / 2);

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
    if numel(neighDia) < fitCfg.diameterGuardMinNeighbors
        continue;
    end

    diaFloor = median(neighDia) - fitCfg.diameterGuardTolPx;
    if dia(idx) < diaFloor
        neighTop = topPath(leftIdx:rightIdx);
        neighBot = botPath(leftIdx:rightIdx);
        neighTop = neighTop(neighMask);
        neighBot = neighBot(neighMask);
        topInward = topPath(idx) - median(neighTop);
        botInward = median(neighBot) - botPath(idx);
        if botInward >= topInward
            botPath(idx) = topPath(idx) + diaFloor;
            botBlendWeight(idx) = max(botBlendWeight(idx), fitCfg.diameterGuardBlendWeightMin);
            botZone(idx) = "blend";
        else
            topPath(idx) = botPath(idx) - diaFloor;
            topBlendWeight(idx) = max(topBlendWeight(idx), fitCfg.diameterGuardBlendWeightMin);
            topZone(idx) = "blend";
        end
    end
end

topPath = topPath(:);
botPath = botPath(:);
topZone = topZone(:);
botZone = botZone(:);
topBlendWeight = topBlendWeight(:);
botBlendWeight = botBlendWeight(:);
end


function [pathVals, zoneLabel] = smooth_pixel_kinks(pathVals, zoneLabel, conf, edgeName, fitCfg)
pathVals = pathVals(:).';
zoneLabel = string(zoneLabel(:).');
conf = conf(:).';

specialZones = ["marker", "raw_protected", "blend", "gap_short"];
specialMask = ismember(zoneLabel, specialZones);
lowConfMask = conf <= fitCfg.kinkConfMax;
candidateMask = isfinite(pathVals) & (specialMask | lowConfMask);
runList = find_invalid_runs(candidateMask);

for idx = 1:size(runList, 1)
    startIdx = runList(idx, 1);
    endIdx = runList(idx, 2);
    runLen = endIdx - startIdx + 1;
    if runLen > fitCfg.kinkRunMaxLen
        continue;
    end
    if startIdx <= 1 || endIdx >= numel(pathVals)
        continue;
    end

    leftVal = pathVals(startIdx - 1);
    rightVal = pathVals(endIdx + 1);
    if ~isfinite(leftVal) || ~isfinite(rightVal)
        continue;
    end

    xRun = startIdx:endIdx;
    interpVals = interp1([startIdx - 1, endIdx + 1], [leftVal, rightVal], xRun);
    rawVals = pathVals(xRun);
    if any(~isfinite(rawVals))
        continue;
    end

    rawTv = abs(rawVals(1) - leftVal) + abs(rightVal - rawVals(end));
    if runLen > 1
        rawTv = rawTv + sum(abs(diff(rawVals)));
    end
    interpTv = abs(interpVals(1) - leftVal) + abs(rightVal - interpVals(end));
    if runLen > 1
        interpTv = interpTv + sum(abs(diff(interpVals)));
    end

    maxDelta = max(abs(rawVals - interpVals));
    if maxDelta < fitCfg.kinkInterpMinDeltaPx
        continue;
    end
    if (rawTv - interpTv) < fitCfg.kinkTvGainMin
        continue;
    end

    if ~any(specialMask(xRun)) && all(conf(xRun) > fitCfg.kinkConfMax)
        continue;
    end

    pathVals(xRun) = interpVals;
    zoneLabel(xRun) = edgeName + "_smoothed";
end

for idx = 2:numel(pathVals) - 1
    if ~candidateMask(idx)
        continue;
    end
    leftVal = pathVals(idx - 1);
    midVal = pathVals(idx);
    rightVal = pathVals(idx + 1);
    if ~isfinite(leftVal) || ~isfinite(midVal) || ~isfinite(rightVal)
        continue;
    end
    interpVal = 0.5 * (leftVal + rightVal);
    rawTv = abs(midVal - leftVal) + abs(rightVal - midVal);
    interpTv = abs(interpVal - leftVal) + abs(rightVal - interpVal);
    if abs(midVal - interpVal) < fitCfg.kinkInterpMinDeltaPx
        continue;
    end
    if (rawTv - interpTv) < fitCfg.kinkTvGainMin
        continue;
    end
    pathVals(idx) = interpVal;
    zoneLabel(idx) = edgeName + "_smoothed";
end

for idx = 2:numel(pathVals) - 2
    if ~(candidateMask(idx) && candidateMask(idx + 1))
        continue;
    end
    leftVal = pathVals(idx - 1);
    rightVal = pathVals(idx + 2);
    midVals = pathVals(idx:idx + 1);
    if ~all(isfinite([leftVal, midVals, rightVal]))
        continue;
    end
    interpVals = interp1([idx - 1, idx + 2], [leftVal, rightVal], idx:idx + 1);
    rawTv = abs(midVals(1) - leftVal) + abs(midVals(2) - midVals(1)) + abs(rightVal - midVals(2));
    interpTv = abs(interpVals(1) - leftVal) + abs(interpVals(2) - interpVals(1)) + abs(rightVal - interpVals(2));
    if max(abs(midVals - interpVals)) < fitCfg.kinkInterpMinDeltaPx
        continue;
    end
    if (rawTv - interpTv) < fitCfg.kinkTvGainMin
        continue;
    end
    pathVals(idx:idx + 1) = interpVals;
    zoneLabel(idx:idx + 1) = edgeName + "_smoothed";
end

pathVals = pathVals(:);
zoneLabel = zoneLabel(:);
end


function edgeParams = ensure_edge_param_defaults(edgeParams)
if ~isfield(edgeParams, 'dropRef') || ~isfinite(edgeParams.dropRef) || edgeParams.dropRef <= 0
    edgeParams.dropRef = max(edgeParams.firstEdgeMinDrop, eps);
end
if ~isfield(edgeParams, 'outerRef') || ~isfinite(edgeParams.outerRef) || edgeParams.outerRef <= 0
    edgeParams.outerRef = edgeParams.dropRef;
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
if ~isfield(edgeParams, 'strongRef') || ~isfinite(edgeParams.strongRef) || edgeParams.strongRef <= 0
    edgeParams.strongRef = max(edgeParams.strongEdgeRatio * edgeParams.firstEdgeMinDrop, eps);
end
if ~isfield(edgeParams, 'baseLocalDeltaRatio') || ~isfinite(edgeParams.baseLocalDeltaRatio) || edgeParams.baseLocalDeltaRatio <= 0
    edgeParams.baseLocalDeltaRatio = 0.4;
end
if ~isfield(edgeParams, 'minOuterEvidence') || ~isfinite(edgeParams.minOuterEvidence) || edgeParams.minOuterEvidence <= 0
    edgeParams.minOuterEvidence = 0.32;
end
if ~isfield(edgeParams, 'outerBiasWeight') || ~isfinite(edgeParams.outerBiasWeight)
    edgeParams.outerBiasWeight = 0.18;
end
if ~isfield(edgeParams, 'maxInwardOverridePx') || ~isfinite(edgeParams.maxInwardOverridePx) || edgeParams.maxInwardOverridePx < 0
    edgeParams.maxInwardOverridePx = 8;
end
if ~isfield(edgeParams, 'bandEvidenceWeight') || ~isfinite(edgeParams.bandEvidenceWeight)
    edgeParams.bandEvidenceWeight = 0.35;
end
if ~isfield(edgeParams, 'outerEvidenceOverrideMargin') || ~isfinite(edgeParams.outerEvidenceOverrideMargin) || edgeParams.outerEvidenceOverrideMargin < 0
    edgeParams.outerEvidenceOverrideMargin = 0.10;
end
if ~isfield(edgeParams, 'bandSelectMargin') || ~isfinite(edgeParams.bandSelectMargin)
    edgeParams.bandSelectMargin = 0.04;
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


function fitCfg = ensure_fit_cfg_defaults(fitCfg)
if ~isfield(fitCfg, 'diameterGuardWindow') || ~isfinite(fitCfg.diameterGuardWindow) || fitCfg.diameterGuardWindow < 5
    if isfield(fitCfg, 'topDiameterWindow') && isfinite(fitCfg.topDiameterWindow)
        fitCfg.diameterGuardWindow = fitCfg.topDiameterWindow;
    else
        fitCfg.diameterGuardWindow = 61;
    end
end
if ~isfield(fitCfg, 'diameterGuardTolPx') || ~isfinite(fitCfg.diameterGuardTolPx) || fitCfg.diameterGuardTolPx <= 0
    if isfield(fitCfg, 'topDiameterShrinkTolPx') && isfinite(fitCfg.topDiameterShrinkTolPx)
        fitCfg.diameterGuardTolPx = fitCfg.topDiameterShrinkTolPx;
    else
        fitCfg.diameterGuardTolPx = 1.5;
    end
end
if ~isfield(fitCfg, 'diameterGuardMinNeighbors') || ~isfinite(fitCfg.diameterGuardMinNeighbors) || fitCfg.diameterGuardMinNeighbors < 3
    if isfield(fitCfg, 'topDiameterMinNeighbors') && isfinite(fitCfg.topDiameterMinNeighbors)
        fitCfg.diameterGuardMinNeighbors = fitCfg.topDiameterMinNeighbors;
    else
        fitCfg.diameterGuardMinNeighbors = 14;
    end
end
if ~isfield(fitCfg, 'diameterGuardBlendWeightMin') || ~isfinite(fitCfg.diameterGuardBlendWeightMin)
    if isfield(fitCfg, 'topDiameterBlendWeightMin') && isfinite(fitCfg.topDiameterBlendWeightMin)
        fitCfg.diameterGuardBlendWeightMin = fitCfg.topDiameterBlendWeightMin;
    else
        fitCfg.diameterGuardBlendWeightMin = 0.50;
    end
end
if ~isfield(fitCfg, 'suspiciousRunMinLen') || ~isfinite(fitCfg.suspiciousRunMinLen) || fitCfg.suspiciousRunMinLen < 1
    fitCfg.suspiciousRunMinLen = 5;
end
if ~isfield(fitCfg, 'suspiciousBridgeGap') || ~isfinite(fitCfg.suspiciousBridgeGap) || fitCfg.suspiciousBridgeGap < 0
    fitCfg.suspiciousBridgeGap = 2;
end
if ~isfield(fitCfg, 'kinkRunMaxLen') || ~isfinite(fitCfg.kinkRunMaxLen) || fitCfg.kinkRunMaxLen < 1
    fitCfg.kinkRunMaxLen = 3;
end
if ~isfield(fitCfg, 'kinkInterpMinDeltaPx') || ~isfinite(fitCfg.kinkInterpMinDeltaPx) || fitCfg.kinkInterpMinDeltaPx <= 0
    fitCfg.kinkInterpMinDeltaPx = 1.2;
end
if ~isfield(fitCfg, 'kinkTvGainMin') || ~isfinite(fitCfg.kinkTvGainMin) || fitCfg.kinkTvGainMin <= 0
    fitCfg.kinkTvGainMin = 1.0;
end
if ~isfield(fitCfg, 'kinkConfMax') || ~isfinite(fitCfg.kinkConfMax) || fitCfg.kinkConfMax <= 0
    fitCfg.kinkConfMax = 0.90;
end
if ~isfield(fitCfg, 'trendProtectConfMin') || ~isfinite(fitCfg.trendProtectConfMin) || fitCfg.trendProtectConfMin <= 0
    fitCfg.trendProtectConfMin = 0.45;
end
if ~isfield(fitCfg, 'trendProtectResidualPx') || ~isfinite(fitCfg.trendProtectResidualPx) || fitCfg.trendProtectResidualPx <= 0
    fitCfg.trendProtectResidualPx = 6.0;
end
if ~isfield(fitCfg, 'trendProtectStepMax') || ~isfinite(fitCfg.trendProtectStepMax) || fitCfg.trendProtectStepMax <= 0
    fitCfg.trendProtectStepMax = 8.0;
end
if ~isfield(fitCfg, 'trendProtectSpanPx') || ~isfinite(fitCfg.trendProtectSpanPx) || fitCfg.trendProtectSpanPx <= 0
    fitCfg.trendProtectSpanPx = 4.0;
end
if ~isfield(fitCfg, 'trendProtectRunMinLen') || ~isfinite(fitCfg.trendProtectRunMinLen) || fitCfg.trendProtectRunMinLen < 1
    fitCfg.trendProtectRunMinLen = 3;
end
end


function mask = detect_trend_protected_cols(edgeName, rawPath, fitPath, conf, blockedCols, fitCfg)
mask = false(size(rawPath));
if ~strcmp(edgeName, 'top')
    return;
end

rawPath = rawPath(:).';
fitPath = fitPath(:).';
conf = conf(:).';
blockedCols = blockedCols(:).';
residual = abs(rawPath - fitPath);
usable = isfinite(rawPath) & isfinite(fitPath) & ~blockedCols & ...
    conf >= fitCfg.trendProtectConfMin & residual >= fitCfg.trendProtectResidualPx;

for idx = 2:numel(rawPath) - 1
    if ~(usable(idx - 1) && usable(idx) && usable(idx + 1))
        continue;
    end
    d1 = rawPath(idx) - rawPath(idx - 1);
    d2 = rawPath(idx + 1) - rawPath(idx);
    if abs(d1) > fitCfg.trendProtectStepMax || abs(d2) > fitCfg.trendProtectStepMax
        continue;
    end
    s1 = sign(d1);
    s2 = sign(d2);
    if s1 == 0
        s1 = s2;
    end
    if s2 == 0
        s2 = s1;
    end
    if s1 == 0 || s1 ~= s2
        continue;
    end
    if abs(rawPath(idx + 1) - rawPath(idx - 1)) < fitCfg.trendProtectSpanPx
        continue;
    end
    mask(idx - 1:idx + 1) = true;
end

mask = keep_min_run_length(mask, fitCfg.trendProtectRunMinLen);
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


function mask = merge_close_runs(mask, maxGap)
mask = logical(mask(:).');
if maxGap <= 0 || ~any(mask)
    return;
end
runList = find_invalid_runs(~mask);
for idx = 1:size(runList, 1)
    startIdx = runList(idx, 1);
    endIdx = runList(idx, 2);
    runLen = endIdx - startIdx + 1;
    if runLen <= maxGap && startIdx > 1 && endIdx < numel(mask) && mask(startIdx - 1) && mask(endIdx + 1)
        mask(startIdx:endIdx) = true;
    end
end
end


function mask = keep_min_run_length(mask, minLen)
mask = logical(mask(:).');
if minLen <= 1 || ~any(mask)
    return;
end
runList = find_invalid_runs(mask);
for idx = 1:size(runList, 1)
    startIdx = runList(idx, 1);
    endIdx = runList(idx, 2);
    if (endIdx - startIdx + 1) < minLen
        mask(startIdx:endIdx) = false;
    end
end
end


function span = make_odd_span(spanIn)
span = max(5, round(spanIn));
if mod(span, 2) == 0
    span = span + 1;
end
end


function fp = side_config_fingerprint(sideCfg)
fitCfg = ensure_fit_cfg_defaults(sideCfg.fit);
topCfg = ensure_edge_param_defaults(merge_struct(struct( ...
    'edgeWindowPx', sideCfg.edgeWindowPx, ...
    'allowBandWithoutValley', sideCfg.allowBandWithoutValley, ...
    'searchOutsidePx', sideCfg.searchOutsidePx, ...
    'searchInsidePx', sideCfg.searchInsidePx, ...
    'bandOffsetMinPx', sideCfg.bandOffsetMinPx, ...
    'bandOffsetMaxPx', sideCfg.bandOffsetMaxPx, ...
    'firstEdgeMinDrop', sideCfg.top.firstEdgeMinDrop, ...
    'valleyDepthThr', sideCfg.top.valleyDepthThr), sideCfg.top));
botCfg = ensure_edge_param_defaults(merge_struct(struct( ...
    'edgeWindowPx', sideCfg.edgeWindowPx, ...
    'allowBandWithoutValley', sideCfg.allowBandWithoutValley, ...
    'searchOutsidePx', sideCfg.searchOutsidePx, ...
    'searchInsidePx', sideCfg.searchInsidePx, ...
    'bandOffsetMinPx', sideCfg.bandOffsetMinPx, ...
    'bandOffsetMaxPx', sideCfg.bandOffsetMaxPx, ...
    'firstEdgeMinDrop', sideCfg.bottom.firstEdgeMinDrop, ...
    'valleyDepthThr', sideCfg.bottom.valleyDepthThr), sideCfg.bottom));

fpStruct = struct( ...
    'bgSigma', sideCfg.bgSigma, ...
    'edgeSigma', sideCfg.edgeSigma, ...
    'searchOutsidePx', sideCfg.searchOutsidePx, ...
    'searchInsidePx', sideCfg.searchInsidePx, ...
    'edgeWindowPx', sideCfg.edgeWindowPx, ...
    'bandOffsetMinPx', sideCfg.bandOffsetMinPx, ...
    'bandOffsetMaxPx', sideCfg.bandOffsetMaxPx, ...
    'allowBandWithoutValley', sideCfg.allowBandWithoutValley, ...
    'strongEdgeRatio', sideCfg.strongEdgeRatio, ...
    'maxJumpPerCol', sideCfg.maxJumpPerCol, ...
    'smoothPenalty', sideCfg.smoothPenalty, ...
    'markerPenalty', sideCfg.markerPenalty, ...
    'marker', sideCfg.marker, ...
    'fit', fitCfg, ...
    'top', topCfg, ...
    'bottom', botCfg);

jsonText = jsonencode(fpStruct);
md = java.security.MessageDigest.getInstance('MD5');
md.update(uint8(jsonText));
fp = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2).', 1, []));
end


function signature = side_calibration_source_signature(img, cfg)
imgSize = size(img);
if numel(imgSize) < 3
    imgSize(3) = 1;
end
signature = sprintf('%s|%d|%d|%d', cfg.dir.img, imgSize(1), imgSize(2), imgSize(3));
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
