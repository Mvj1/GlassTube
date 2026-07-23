function result = detect_side_inner_walls(stripImg, sideCfg, sidePxPerMm, expectedWallMm, seamCols)
%DETECT_SIDE_INNER_WALLS Track upper/lower bore surfaces using Canny evidence.

arguments
    stripImg
    sideCfg (1,1) struct
    sidePxPerMm (1,1) double {mustBePositive}
    expectedWallMm (1,1) double {mustBeNonnegative} = 0
    seamCols double = []
end

tStart = tic;
gray = to_gray_single(stripImg);
[roiGray, roiOrigin] = crop_roi(gray, sideCfg.roiRect);
[rowN, colN] = size(roiGray);
cfg = apply_defaults(sideCfg, rowN);

sampleCols = round(linspace(max(1, round(0.08 * colN)), ...
    min(colN, round(0.92 * colN)), min(401, colN)));
profile = median(roiGray(:, sampleCols), 2);
profile = imgaussfilt(profile, cfg.profileSigma);
profileGrad = gradient(profile);
[topCenter, botCenter, initialization] = initialize_inner_rows(profileGrad, cfg, expectedWallMm, sidePxPerMm, rowN);
seamMask = build_seam_mask(seamCols, roiOrigin(2), colN, cfg.seamHalfWidthPx);

[topTrace, topDebug] = track_one_wall(roiGray, "top", topCenter, cfg, seamMask);
[botTrace, botDebug] = track_one_wall(roiGray, "bottom", botCenter, cfg, seamMask);

if median(botTrace.path - topTrace.path, 'omitnan') <= 0
    error('GlassTube:SideWallsCrossed', 'The detected lower inner wall is not below the upper inner wall.');
end
observedPair = topTrace.observed & botTrace.observed;
validPair = topTrace.valid & botTrace.valid;
if mean(observedPair) < cfg.minObservedFraction
    error('GlassTube:InsufficientSideEvidence', ...
        'Only %.2f%% of columns contain directly observed upper/lower Canny walls.', 100 * mean(observedPair));
end
longestInvalid = longest_true_run(~validPair);
if longestInvalid > cfg.maxInvalidGapCols
    error('GlassTube:LongSideGap', ...
        'The inner-wall track contains an unsupported gap of %d columns.', longestInvalid);
end

centerRawPx = (topTrace.path + botTrace.path) / 2;
apparentHalfGapPx = (botTrace.path - topTrace.path) / 2;
centerFitPx = smoothdata(centerRawPx, 'rlowess', odd_span(min(cfg.centerSmoothSpan, colN)), 'omitnan');
gapBaselinePx = smoothdata(apparentHalfGapPx, 'rlowess', odd_span(min(cfg.gapBaselineSpan, colN)), 'omitnan');
localShrinkPx = max(0, gapBaselinePx - apparentHalfGapPx);
centerResidualPx = abs(centerRawPx - centerFitPx);
uncertaintyPx = cfg.baseUncertaintyPx + cfg.centerResidualWeight * centerResidualPx;
uncertaintyPx = uncertaintyPx + double(~observedPair) * cfg.interpolatedUncertaintyPx;
uncertaintyPx = uncertaintyPx + double(seamMask(:)) * cfg.seamUncertaintyPx;

xMm = ((1:colN)' - 1) / sidePxPerMm;
centerMm = centerFitPx(:) / sidePxPerMm;
centerMm = centerMm - median(centerMm, 'omitnan');

result = struct();
result.xMm = xMm;
result.topPx = topTrace.path(:);
result.bottomPx = botTrace.path(:);
result.centerRawPx = centerRawPx(:);
result.centerFitPx = centerFitPx(:);
result.centerMm = centerMm;
result.apparentHalfGapPx = apparentHalfGapPx(:);
result.localShrinkMm = localShrinkPx(:) / sidePxPerMm;
result.uncertaintyMm = uncertaintyPx(:) / sidePxPerMm;
result.observed = observedPair(:);
result.valid = validPair(:);
result.top = topTrace;
result.bottom = botTrace;
result.roiRect = sideCfg.roiRect;
result.roiOrigin = roiOrigin;
result.initialRows = [topCenter, botCenter];
result.initialization = initialization;
result.quality = struct( ...
    'observedFraction', mean(observedPair), ...
    'topObservedFraction', mean(topTrace.observed), ...
    'bottomObservedFraction', mean(botTrace.observed), ...
    'medianApparentGapPx', median(2 * apparentHalfGapPx, 'omitnan'), ...
    'maxUnsupportedGapCols', longest_true_run(~observedPair), ...
    'medianConfidence', median(min(topTrace.confidence, botTrace.confidence), 'omitnan'));
result.timingSeconds = toc(tStart);
if cfg.returnDebug
    result.debug = struct('roiGray', roiGray, 'top', topDebug, 'bottom', botDebug, ...
        'profile', profile, 'profileGradient', profileGrad, 'seamMask', seamMask);
else
    result.debug = struct();
end
end

function [trace, debug] = track_one_wall(roiGray, wallName, centerRow, cfg, seamMask)
rowN = size(roiGray, 1);
colN = size(roiGray, 2);
rowLo = max(2, floor(centerRow - cfg.bandHalfWidthPx));
rowHi = min(rowN - 1, ceil(centerRow + cfg.bandHalfWidthPx));
rows = rowLo:rowHi;
band = roiGray(rows, :);

colLow = prctile(band, 5, 1);
colHigh = prctile(band, 95, 1);
bandNorm = (band - colLow) ./ max(colHigh - colLow, single(1e-4));
bandNorm = min(1, max(0, bandNorm));
if isempty(cfg.cannyThreshold)
    [cannyMask, cannyThreshold] = edge(bandNorm, 'Canny', [], cfg.cannySigma);
else
    [cannyMask, cannyThreshold] = edge(bandNorm, 'Canny', cfg.cannyThreshold, cfg.cannySigma);
end
[~, gradY] = imgradientxy(bandNorm, 'sobel');
if wallName == "top"
    polarityStrength = max(gradY, 0);
else
    polarityStrength = max(-gradY, 0);
end
candidateMask = cannyMask & polarityStrength > 0;
strengthVals = polarityStrength(candidateMask);
strengthScale = prctile(strengthVals, 99);
if isempty(strengthScale) || ~isfinite(strengthScale) || strengthScale <= eps('single')
    error('GlassTube:NoSideCannyCandidates', 'No polarity-correct Canny candidates were found for the %s wall.', wallName);
end
strength = min(1, polarityStrength / strengthScale);
rowDistance = abs(single(rows(:)) - single(centerRow)) / max(single(cfg.bandHalfWidthPx), 1);
centerCost = repmat(cfg.centerPenalty * rowDistance .^ 2, 1, colN);
localScore = -cfg.missingCandidatePenalty - centerCost;
candidateScore = cfg.edgeStrengthWeight * strength - centerCost;
localScore(candidateMask) = candidateScore(candidateMask);
if any(seamMask)
    localScore(:, seamMask) = localScore(:, seamMask) - cfg.seamCandidatePenalty;
end

[pathIdx, pathScore] = banded_viterbi(single(localScore), cfg.maxJumpPx, cfg.jumpPenalty);
cols = (1:colN)';
lin = sub2ind(size(candidateMask), pathIdx(:), cols);
observed = candidateMask(lin);
confidence = zeros(colN, 1, 'single');
confidence(observed) = strength(lin(observed));
path = double(rows(pathIdx)).';
path = path(:);
for col = find(observed).'
    r = pathIdx(col);
    if r <= 1 || r >= numel(rows), continue; end
    y1 = double(polarityStrength(r - 1, col));
    y2 = double(polarityStrength(r, col));
    y3 = double(polarityStrength(r + 1, col));
    denom = y1 - 2 * y2 + y3;
    if denom < -eps
        delta = 0.5 * (y1 - y3) / denom;
        path(col) = path(col) + max(-0.5, min(0.5, delta));
    end
end
[valid, interpolated] = classify_gaps(observed, cfg.maxInterpolatedGapCols);
trace = struct('path', path, 'observed', observed, 'interpolated', interpolated, ...
    'valid', valid, 'confidence', double(confidence), 'pathScore', pathScore, ...
    'bandRows', [rowLo, rowHi], 'cannyThreshold', cannyThreshold);
debug = struct('cannyMask', cannyMask, 'candidateMask', candidateMask, ...
    'strength', strength, 'bandRows', rows);
end

function [pathIdx, bestScore] = banded_viterbi(localScore, maxJump, jumpPenalty)
[rowCount, colCount] = size(localScore);
score = -inf(rowCount, colCount, 'single');
parentShift = zeros(rowCount, colCount, 'int8');
score(:, 1) = localScore(:, 1);
for col = 2:colCount
    prev = score(:, col - 1);
    best = -inf(rowCount, 1, 'single');
    bestShift = zeros(rowCount, 1, 'int8');
    for shift = -maxJump:maxJump
        if shift >= 0
            dst = (1 + shift):rowCount; src = 1:(rowCount - shift);
        else
            dst = 1:(rowCount + shift); src = (1 - shift):rowCount;
        end
        candidate = prev(src) - single(jumpPenalty * abs(shift));
        improve = candidate > best(dst);
        if any(improve)
            dstImprove = dst(improve);
            best(dstImprove) = candidate(improve);
            bestShift(dstImprove) = int8(shift);
        end
    end
    score(:, col) = localScore(:, col) + best;
    score(:, col) = score(:, col) - max(score(:, col));
    parentShift(:, col) = bestShift;
end
[bestScore, idx] = max(score(:, end));
pathIdx = zeros(colCount, 1);
pathIdx(end) = idx;
for col = colCount:-1:2
    pathIdx(col - 1) = pathIdx(col) - double(parentShift(pathIdx(col), col));
end
end

function [valid, interpolated] = classify_gaps(observed, maxGap)
observed = logical(observed(:));
valid = observed;
interpolated = false(size(observed));
runs = false_runs(observed);
for k = 1:size(runs, 1)
    idx = runs(k, 1):runs(k, 2);
    if numel(idx) <= maxGap && runs(k, 1) > 1 && runs(k, 2) < numel(observed)
        valid(idx) = true;
        interpolated(idx) = true;
    end
end
end

function [topCenter, botCenter, info] = initialize_inner_rows(profileGrad, cfg, wallMm, pxPerMm, rowN)
% Start from a physical prior when it is still supported, otherwise use the
% strongest polarity-correct full-profile transition in each ROI half.
topGlobalRows = 3:floor(rowN * 0.48);
botGlobalRows = ceil(rowN * 0.52):(rowN - 2);
if isempty(topGlobalRows) || isempty(botGlobalRows)
    error('GlassTube:InvalidSideInitialization', 'The side ROI is too short for upper/lower wall initialization.');
end

[topGlobalStrength, topGlobalIdx] = max(profileGrad(topGlobalRows));
[botGlobalValue, botGlobalIdx] = min(profileGrad(botGlobalRows));
botGlobalStrength = -botGlobalValue;
topGlobal = topGlobalRows(topGlobalIdx);
botGlobal = botGlobalRows(botGlobalIdx);
if topGlobalStrength <= 0 || botGlobalStrength <= 0
    error('GlassTube:InvalidSideInitialization', ...
        'The longitudinal profile does not contain polarity-correct upper/lower wall transitions.');
end

topCenter = topGlobal;
botCenter = botGlobal;
topSource = "global";
botSource = "global";
topPrior = nan;
botPrior = nan;
topPriorStrength = nan;
botPriorStrength = nan;

if isfinite(cfg.outerBaselineTopPx) && isfinite(cfg.outerBaselineBottomPx) && wallMm > 0
    expectedOffset = wallMm * pxPerMm;
    topExpected = cfg.outerBaselineTopPx + expectedOffset;
    botExpected = cfg.outerBaselineBottomPx - expectedOffset;
    searchHalf = cfg.initialSearchHalfWidthPx;
    topRows = max(3, round(topExpected - searchHalf)):min(round(rowN * 0.48), round(topExpected + searchHalf));
    botRows = max(round(rowN * 0.52), round(botExpected - searchHalf)):min(rowN - 2, round(botExpected + searchHalf));

    if ~isempty(topRows)
        [topPriorStrength, topPriorIdx] = max(profileGrad(topRows));
        topPrior = topRows(topPriorIdx);
        if topPriorStrength >= cfg.initializationPriorStrengthFraction * topGlobalStrength
            topCenter = topPrior;
            topSource = "prior";
        end
    end
    if ~isempty(botRows)
        [botPriorValue, botPriorIdx] = min(profileGrad(botRows));
        botPriorStrength = -botPriorValue;
        botPrior = botRows(botPriorIdx);
        if botPriorStrength >= cfg.initializationPriorStrengthFraction * botGlobalStrength
            botCenter = botPrior;
            botSource = "prior";
        end
    end
end

if botCenter - topCenter < cfg.minimumInnerGapFraction * rowN
    error('GlassTube:InvalidSideInitialization', ...
        'The initialized inner-wall pair is not separated by the minimum plausible ROI opening.');
end
info = struct('topSource', topSource, 'bottomSource', botSource, ...
    'topGlobalRow', topGlobal, 'bottomGlobalRow', botGlobal, ...
    'topGlobalStrength', topGlobalStrength, 'bottomGlobalStrength', botGlobalStrength, ...
    'topPriorRow', topPrior, 'bottomPriorRow', botPrior, ...
    'topPriorStrength', topPriorStrength, 'bottomPriorStrength', botPriorStrength);
end

function mask = build_seam_mask(seamCols, roiX1, colN, halfWidth)
mask = false(1, colN);
localCols = round(seamCols(:) - roiX1 + 1);
for col = localCols.'
    lo = max(1, col - halfWidth); hi = min(colN, col + halfWidth);
    if lo <= hi, mask(lo:hi) = true; end
end
end

function [roi, origin] = crop_roi(gray, rect)
if numel(rect) ~= 4, error('GlassTube:InvalidSideROI', 'sideCfg.roiRect must be [x y width height].'); end
x1 = max(1, floor(rect(1)) + 1); y1 = max(1, floor(rect(2)) + 1);
x2 = min(size(gray, 2), x1 + round(rect(3)) - 1);
y2 = min(size(gray, 1), y1 + round(rect(4)) - 1);
if x2 <= x1 || y2 <= y1, error('GlassTube:InvalidSideROI', 'The configured side ROI is outside the strip.'); end
roi = gray(y1:y2, x1:x2); origin = [y1, x1];
end

function gray = to_gray_single(img)
if ndims(img) == 3, img = im2gray(img); end
gray = im2single(img);
end

function cfg = apply_defaults(cfg, rowN)
defaults = struct('roiRect', [0 0 inf rowN], 'profileSigma', 2.0, ...
    'initialSearchHalfWidthPx', 90, 'bandHalfWidthPx', 125, 'cannySigma', 1.2, ...
    'cannyThreshold', [], 'maxJumpPx', 5, 'jumpPenalty', 0.18, ...
    'edgeStrengthWeight', 4.0, 'centerPenalty', 0.20, ...
    'missingCandidatePenalty', 1.8, 'seamCandidatePenalty', 0.35, ...
    'seamHalfWidthPx', 4, 'maxInterpolatedGapCols', 40, 'maxInvalidGapCols', 80, ...
    'minObservedFraction', 0.90, 'centerSmoothSpan', 31, 'gapBaselineSpan', 301, ...
    'baseUncertaintyPx', 0.75, 'centerResidualWeight', 0.50, ...
    'interpolatedUncertaintyPx', 1.5, 'seamUncertaintyPx', 1.0, ...
    'returnDebug', false, 'outerBaselineTopPx', nan, 'outerBaselineBottomPx', nan, ...
    'initializationPriorStrengthFraction', 0.65, 'minimumInnerGapFraction', 0.10);
names = fieldnames(defaults);
for k = 1:numel(names)
    if ~isfield(cfg, names{k}) || isempty(cfg.(names{k})), cfg.(names{k}) = defaults.(names{k}); end
end
end

function span = odd_span(value)
span = max(3, round(value)); if mod(span, 2) == 0, span = span - 1; end
end

function runs = false_runs(mask)
mask = logical(mask(:)).'; padded = [true, mask, true];
starts = find(diff(padded) == -1); stops = find(diff(padded) == 1) - 1;
runs = [starts(:), stops(:)];
end

function value = longest_true_run(mask)
mask = logical(mask(:)).'; padded = [false, mask, false];
starts = find(diff(padded) == 1); stops = find(diff(padded) == -1) - 1;
if isempty(starts), value = 0; else, value = max(stops - starts + 1); end
end
