function result = solve_max_rod_diameter(sideData, leftEnd, rightEnd, fitCfg)
%SOLVE_MAX_ROD_DIAMETER Compute a conservative straight-rod diameter.
%   The rod is represented as a Euclidean-width strip around a straight axis.
%   Side-view evidence may shrink the interpolated end-face radius, but it can
%   never enlarge it.

arguments
    sideData (1,1) struct
    leftEnd (1,1) struct
    rightEnd (1,1) struct
    fitCfg (1,1) struct = struct()
end

cfg = defaults(fitCfg);
x = double(sideData.xMm(:));
center = double(sideData.centerMm(:));
localShrink = double(sideData.localShrinkMm(:));
uncertainty = double(sideData.uncertaintyMm(:));
valid = logical(sideData.valid(:)) & isfinite(x) & isfinite(center) & ...
    isfinite(localShrink) & isfinite(uncertainty);
if nnz(valid) < 3
    error('GlassTube:InsufficientRodConstraints', 'At least three valid side-view constraints are required.');
end
x = x(valid); center = center(valid);
localShrink = localShrink(valid); uncertainty = uncertainty(valid);
x = x - x(1);

leftRadius = safe_radius(leftEnd);
rightRadius = safe_radius(rightEnd);
t = x / max(x(end), eps);
baseRadius = (1 - t) * leftRadius + t * rightRadius;
radius = baseRadius - localShrink - uncertainty - cfg.additionalSafetyMarginMm;
if any(radius <= 0)
    error('GlassTube:NonpositiveCavityRadius', 'Conservative uncertainty consumes the full bore radius.');
end

top = center - radius;
bottom = center + radius;
localSlope = diff(center) ./ max(diff(x), eps);
localSlope = localSlope(isfinite(localSlope));
if isempty(localSlope)
    slopeBound = cfg.minimumSlopeBound;
else
    robustAbs = prctile(abs(localSlope), cfg.slopePercentile);
    linearFit = polyfit(x, center, 1);
    slopeBound = max(cfg.minimumSlopeBound, cfg.slopeBoundFactor * max(robustAbs, abs(linearFit(1))) + cfg.slopePadding);
end
slopeBound = min(cfg.maximumSlopeBound, slopeBound);

coarseSlope = linspace(-slopeBound, slopeBound, cfg.coarseSlopeCount);
coarseDiameter = arrayfun(@(m) diameter_for_slope(m, x, top, bottom), coarseSlope);
[~, order] = sort(coarseDiameter, 'descend');
seedIdx = unique(order(1:min(cfg.refineSeedCount, numel(order))));

candidateSlope = nan(1, numel(seedIdx) * 2 + 3);
candidateCount = numel(seedIdx) + 3;
candidateSlope(1:candidateCount) = [coarseSlope(seedIdx), 0, -slopeBound, slopeBound];
for idx = seedIdx(:).'
    loIdx = max(1, idx - 1); hiIdx = min(numel(coarseSlope), idx + 1);
    lo = coarseSlope(loIdx); hi = coarseSlope(hiIdx);
    if hi > lo
        objective = @(m) -diameter_for_slope(m, x, top, bottom);
        candidateCount = candidateCount + 1;
        candidateSlope(candidateCount) = fminbnd(objective, lo, hi, ...
            optimset('TolX', cfg.slopeTolerance, 'Display', 'off'));
    end
end
candidateSlope = unique(candidateSlope(1:candidateCount));
candidateDiameter = arrayfun(@(m) diameter_for_slope(m, x, top, bottom), candidateSlope);
[rodDiameter, bestIdx] = max(candidateDiameter);
bestSlope = candidateSlope(bestIdx);
[~, bestIntercept, topIdx, bottomIdx, verticalGap] = evaluate_slope(bestSlope, x, top, bottom);

if ~isfinite(rodDiameter) || rodDiameter <= 0
    error('GlassTube:NoFeasibleRod', 'No positive-diameter straight rod fits the conservative cavity.');
end

axisY = bestSlope * x + bestIntercept;
halfVertical = 0.5 * rodDiameter * sqrt(1 + bestSlope ^ 2);
result = struct();
result.diameterMm = rodDiameter;
result.radiusMm = rodDiameter / 2;
result.axisSlope = bestSlope;
result.axisInterceptMm = bestIntercept;
result.axisXmm = x;
result.axisYmm = axisY;
result.rodTopMm = axisY - halfVertical;
result.rodBottomMm = axisY + halfVertical;
result.cavityTopMm = top;
result.cavityBottomMm = bottom;
result.cavityRadiusMm = radius;
result.verticalClearanceMm = verticalGap;
result.topContactIndex = topIdx;
result.bottomContactIndex = bottomIdx;
result.topContactXmm = x(topIdx);
result.bottomContactXmm = x(bottomIdx);
result.leftSafeRadiusMm = leftRadius;
result.rightSafeRadiusMm = rightRadius;
result.activeLimit = classify_limit(radius, localShrink, uncertainty, cfg);
result.optimization = struct('slopeBound', slopeBound, ...
    'coarseSlopeCount', cfg.coarseSlopeCount, 'candidateCount', numel(candidateSlope));
end

function value = diameter_for_slope(slope, x, top, bottom)
[value, ~] = evaluate_slope(slope, x, top, bottom);
end

function [diameter, intercept, topIdx, bottomIdx, verticalGap] = evaluate_slope(slope, x, top, bottom)
topShift = top - slope * x;
bottomShift = bottom - slope * x;
[topLimit, topIdx] = max(topShift);
[bottomLimit, bottomIdx] = min(bottomShift);
verticalGap = bottomLimit - topLimit;
diameter = verticalGap / sqrt(1 + slope ^ 2);
intercept = (topLimit + bottomLimit) / 2;
end

function radius = safe_radius(endData)
fieldOrder = {'safeInnerRadiusMm', 'conservativeRadiusMm', 'conservativeInnerRadiusMm', 'innerRadiusMm'};
radius = nan;
for k = 1:numel(fieldOrder)
    if isfield(endData, fieldOrder{k}) && isfinite(endData.(fieldOrder{k}))
        radius = double(endData.(fieldOrder{k}));
        break;
    end
end
if ~isfinite(radius) && isfield(endData, 'innerDiaMm') && isfinite(endData.innerDiaMm)
    radius = double(endData.innerDiaMm) / 2;
end
if ~isfinite(radius) || radius <= 0
    error('GlassTube:InvalidEndfaceRadius', 'End-face analysis did not provide a positive safe inner radius.');
end
end

function label = classify_limit(radius, shrink, uncertainty, cfg)
minimumBaseLoss = min(radius);
[~, idx] = min(radius);
if shrink(idx) > max(uncertainty(idx), cfg.additionalSafetyMarginMm)
    label = 'side local constriction';
elseif uncertainty(idx) > cfg.additionalSafetyMarginMm
    label = 'measurement uncertainty';
elseif minimumBaseLoss > 0
    label = 'end-face radius / curvature';
else
    label = 'unknown';
end
end

function cfg = defaults(cfg)
defaultValues = struct('additionalSafetyMarginMm', 0.02, ...
    'minimumSlopeBound', 0.08, 'maximumSlopeBound', 2.0, ...
    'slopePercentile', 99.0, 'slopeBoundFactor', 2.0, ...
    'slopePadding', 0.05, 'coarseSlopeCount', 513, ...
    'refineSeedCount', 12, 'slopeTolerance', 1e-10);
names = fieldnames(defaultValues);
for k = 1:numel(names)
    if ~isfield(cfg, names{k}) || isempty(cfg.(names{k}))
        cfg.(names{k}) = defaultValues.(names{k});
    end
end
cfg.coarseSlopeCount = max(33, round(cfg.coarseSlopeCount));
if mod(cfg.coarseSlopeCount, 2) == 0, cfg.coarseSlopeCount = cfg.coarseSlopeCount + 1; end
end


