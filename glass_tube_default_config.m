function cfg = glass_tube_default_config()
%GLASS_TUBE_DEFAULT_CONFIG Reproducible configuration for the analysis pipeline.
rootDir = fileparts(mfilename('fullpath'));

cfg.algorithmVersion = '2.0.0';
cfg.input.sideDir = fullfile(rootDir, 'data', '1');
cfg.input.sidePattern = '*.bmp';
cfg.input.leftEndface = fullfile(rootDir, 'data', 'lef.bmp');
% A distinct right-end image is intentionally required for production use.
cfg.input.rightEndface = fullfile(rootDir, 'data', 'rig.bmp');

% Fallback values measured on 2026-03-16. A generated calibration file,
% when present, is the source of truth.
cfg.scale.sidePxPerMm = 1062.42252 / 30;
cfg.scale.endPxPerMm = 2430.99996 / 30;
calibrationFile = fullfile(rootDir, 'results', 'glass_tube_calibration.mat');
if isfile(calibrationFile)
    loadedCalibration = load(calibrationFile, 'calibration');
    if isfield(loadedCalibration, 'calibration') && ...
            isfield(loadedCalibration.calibration, 'sidePxPerMm') && ...
            isfield(loadedCalibration.calibration, 'endPxPerMm')
        cfg.scale.sidePxPerMm = loadedCalibration.calibration.sidePxPerMm;
        cfg.scale.endPxPerMm = loadedCalibration.calibration.endPxPerMm;
    end
end
cfg.calibration = cfg.scale;
cfg.calibration.file = calibrationFile;
cfg.calibration.source = struct('sideImage', fullfile(rootDir, 'data', 'biaozhun2.bmp'), ...
    'endImage', fullfile(rootDir, 'data', 'biaozhun1.bmp'), 'knownLengthMm', 30);

cfg.stitch.filePattern = cfg.input.sidePattern;
cfg.stitch.nominalStepPx = 900;
cfg.stitch.searchRadiusPx = 40;
cfg.stitch.rotationDeg = -0.39;
cfg.stitch.registrationScale = 0.35;
cfg.stitch.registrationRowFraction = [0.32, 0.68];
cfg.stitch.templateWidthPx = 180;
cfg.stitch.maxVerticalShiftPx = 8;
cfg.stitch.minimumPeakCorrelation = 0.95;
cfg.stitch.minimumPsr = 1.7;
cfg.stitch.allowNominalFallback = false;

cfg.endface.coarse = struct('targetShortSide', 1024, 'claheNumTiles', [8 8], ...
    'claheClipLimit', 0.01, 'innerRadiusFraction', [0.20 0.30], ...
    'outerRadiusFraction', [0.29 0.38], 'sensitivity', 0.99, ...
    'edgeThreshold', 0.01, 'maxCandidates', 12, ...
    'centerToleranceFraction', 0.035, 'minimumWallFraction', 0.025);
cfg.endface.canny = struct('threshold', [], 'sigma', 1.5);
cfg.endface.trace = struct('angleCount', 1440, 'searchHalfWidthFraction', 0.018, ...
    'minimumSearchHalfWidthPx', 20, 'cannyRadialTolerancePx', 2, ...
    'gradientScaleQuantile', 0.90, 'cannyScore', 3.0, ...
    'gradientScore', 1.5, 'radiusPriorScore', 0.35, ...
    'unsupportedPenalty', 3.5, 'transitionPenalty', 0.20, ...
    'maxJumpPx', 5, 'minimumGradientScore', 0.05, ...
    'strongGradientScore', 0.45, 'continuityWindow', 11, ...
    'continuityOutlierPx', 5, 'subpixelLimitPx', 0.75);
cfg.endface.fit = struct('maxIterations', 30, 'tukeyConstant', 4.685, ...
    'minimumScalePx', 0.10, 'minimumInlierTolerancePx', 0.75, ...
    'convergenceTolerance', 1e-7);
cfg.endface.conservative = struct('quantile', 0.01, 'useMinimum', false, ...
    'safetyMarginMm', 0.02, 'safetyMarginPx', 0);
cfg.endface.quality = struct('minInnerCoverage', 0.80, 'minOuterCoverage', 0.65, ...
    'maxAngularGapDeg', 20, 'minFitInlierFraction', 0.80, ...
    'maxRobustRmseFraction', 0.004, 'maxCenterOffsetFraction', 0.025, ...
    'maxRoundnessFraction', 0.020);
cfg.endface.cache = struct('enable', true, 'file', '');
cfg.endface.debug = struct('enable', false);

cfg.side.roiRect = [726.51, 1540.51, 17255, 1559.98];
cfg.side.outerBaselineTopPx = 146;
cfg.side.outerBaselineBottomPx = 1387;
cfg.side.profileSigma = 2.0;
cfg.side.initialSearchHalfWidthPx = 90;
cfg.side.initializationPriorStrengthFraction = 0.65;
cfg.side.minimumInnerGapFraction = 0.10;
cfg.side.bandHalfWidthPx = 125;
cfg.side.cannySigma = 1.2;
cfg.side.cannyThreshold = [];
cfg.side.maxJumpPx = 5;
cfg.side.jumpPenalty = 0.18;
cfg.side.edgeStrengthWeight = 4.0;
cfg.side.centerPenalty = 0.20;
cfg.side.missingCandidatePenalty = 1.8;
cfg.side.seamCandidatePenalty = 0.35;
cfg.side.seamHalfWidthPx = 4;
cfg.side.maxInterpolatedGapCols = 40;
cfg.side.maxInvalidGapCols = 80;
cfg.side.minObservedFraction = 0.90;
cfg.side.centerSmoothSpan = 31;
cfg.side.gapBaselineSpan = 301;
cfg.side.baseUncertaintyPx = 0.75;
cfg.side.centerResidualWeight = 0.50;
cfg.side.interpolatedUncertaintyPx = 1.5;
cfg.side.seamUncertaintyPx = 1.0;
cfg.side.returnDebug = false;

cfg.rod.additionalSafetyMarginMm = 0.02;
cfg.rod.minimumSlopeBound = 0.08;
cfg.rod.maximumSlopeBound = 2.0;
cfg.rod.slopePercentile = 99.0;
cfg.rod.slopeBoundFactor = 2.0;
cfg.rod.slopePadding = 0.05;
cfg.rod.coarseSlopeCount = 513;
cfg.rod.refineSeedCount = 12;
cfg.rod.slopeTolerance = 1e-10;

cfg.validation.requireDistinctEndfaces = true;
cfg.validation.minimumEndfaceDiameterMm = 1;
cfg.validation.maximumEndfaceDiameterMm = 100;
cfg.validation.maximumEndfaceDiameterDifferenceMm = 1.0;

cfg.output.dir = fullfile(rootDir, 'results');
cfg.output.saveResultMat = true;
cfg.output.saveDiagnosticFigure = true;
cfg.output.saveStripImage = false;
cfg.output.savePathCsv = false;
cfg.output.figureName = 'glass_tube_result.png';
cfg.output.resultName = 'glass_tube_result.mat';
cfg.output.pathName = 'inner_wall_path.csv';
cfg.output.stripName = 'tube_strip.png';
end


