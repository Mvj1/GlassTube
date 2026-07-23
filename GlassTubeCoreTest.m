classdef GlassTubeCoreTest < matlab.unittest.TestCase
    % Fast synthetic regression tests for the glass-tube core algorithms.

    methods (TestClassSetup)
        function addProjectRoot(testCase)
            rootDir = fileparts(mfilename('fullpath'));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(rootDir));
        end
    end

    methods (Test)
        function testSideWallPathsAndGapSemantics(testCase)
            [img, truth, cfg, pxPerMm] = GlassTubeCoreTest.syntheticSideView();

            actual = detect_side_inner_walls(img, cfg, pxPerMm, 0, []);
            observed = actual.top.observed & actual.bottom.observed;
            compareMask = observed & ~truth.occlusionMask(:);
            topError = abs(actual.topPx(compareMask) - truth.topPx(compareMask));
            bottomError = abs(actual.bottomPx(compareMask) - truth.bottomPx(compareMask));

            testCase.verifyLessThanOrEqual(prctile(topError, 95), 1.0, ...
                'The upper Canny wall should be localized to within one pixel at the 95th percentile.');
            testCase.verifyLessThanOrEqual(prctile(bottomError, 95), 1.0, ...
                'The lower Canny wall should be localized to within one pixel at the 95th percentile.');
            testCase.verifyGreaterThan(mean(observed(~truth.occlusionMask(:))), 0.96, ...
                'Unoccluded columns should normally be directly supported by Canny evidence.');

            gapCore = truth.occlusionCols(3:end-2);
            testCase.verifyFalse(any(actual.top.observed(gapCore)), ...
                'The deliberately hidden upper wall must not be labelled observed.');
            testCase.verifyFalse(any(actual.bottom.observed(gapCore)), ...
                'The deliberately hidden lower wall must not be labelled observed.');
            testCase.verifyTrue(all(actual.top.interpolated(gapCore)), ...
                'A bounded internal upper-wall gap must be labelled interpolated.');
            testCase.verifyTrue(all(actual.bottom.interpolated(gapCore)), ...
                'A bounded internal lower-wall gap must be labelled interpolated.');
            testCase.verifyTrue(all(actual.valid(gapCore)), ...
                'Short interpolated gaps remain usable but must stay distinguishable from observations.');
            testCase.verifyFalse(any(actual.observed(gapCore)), ...
                'Combined observed status must require direct evidence for both walls.');

            detectedGapPx = actual.bottomPx - actual.topPx;
            neck = truth.neckCoreMask(:);
            trueMinimumGapPx = min(truth.bottomPx(neck) - truth.topPx(neck));
            detectedMinimumGapPx = min(detectedGapPx(neck));
            testCase.verifyLessThanOrEqual(detectedMinimumGapPx, trueMinimumGapPx + 1.0, ...
                'The clearance path must not erase or materially widen the local constriction.');
            testCase.verifyGreaterThanOrEqual(detectedMinimumGapPx, trueMinimumGapPx - 1.5, ...
                'The constriction estimate should remain geometrically accurate.');
            testCase.verifyGreaterThan(max(actual.localShrinkMm(neck)), 4.5 / pxPerMm, ...
                'The known constriction must propagate into the conservative local-shrink signal.');
        end

        function testSideWallMirrorSymmetry(testCase)
            [img, truth, cfg, pxPerMm] = GlassTubeCoreTest.syntheticSideView();
            original = detect_side_inner_walls(img, cfg, pxPerMm, 0, []);
            mirrored = detect_side_inner_walls(flipud(img), cfg, pxPerMm, 0, []);
            imageHeight = size(img, 1);

            mirroredTopBack = imageHeight + 1 - mirrored.bottomPx;
            mirroredBottomBack = imageHeight + 1 - mirrored.topPx;
            stable = ~truth.occlusionMask(:);

            testCase.verifyLessThanOrEqual(prctile(abs(original.topPx(stable) - mirroredTopBack(stable)), 99), 0.25, ...
                'Upper and lower tracking must be symmetric under a vertical image reflection.');
            testCase.verifyLessThanOrEqual(prctile(abs(original.bottomPx(stable) - mirroredBottomBack(stable)), 99), 0.25, ...
                'Mirroring must not introduce a bottom-wall sign bias.');
            testCase.verifyLessThanOrEqual(max(abs( ...
                (original.bottomPx - original.topPx) - ...
                (mirrored.bottomPx - mirrored.topPx))), 0.35, ...
                'The inferred aperture must be invariant under vertical mirroring.');
            testCase.verifyLessThan(mean(xor(original.observed, mirrored.observed)), 0.01, ...
                'Observed/interpolated classification should be mirror invariant.');
            testCase.verifyLessThan(mean(xor(original.valid, mirrored.valid)), 0.01, ...
                'Validity classification should be mirror invariant.');
        end

        function testSideWallInitializationFallsBackFromStalePrior(testCase)
            [img, truth, cfg, pxPerMm] = GlassTubeCoreTest.syntheticSideView();
            cfg.outerBaselineTopPx = 8;
            cfg.outerBaselineBottomPx = 172;
            cfg.initialSearchHalfWidthPx = 8;
            cfg.initializationPriorStrengthFraction = 0.65;

            actual = detect_side_inner_walls(img, cfg, pxPerMm, 1, []);

            testCase.verifyEqual(actual.initialization.topSource, "global", ...
                'A stale upper baseline must yield to stronger full-profile evidence.');
            testCase.verifyEqual(actual.initialization.bottomSource, "global", ...
                'A stale lower baseline must yield to stronger full-profile evidence.');
            testCase.verifyLessThanOrEqual(abs(actual.initialRows(1) - median(truth.topPx)), 2.0);
            testCase.verifyLessThanOrEqual(abs(actual.initialRows(2) - median(truth.bottomPx)), 2.0);
            testCase.verifyGreaterThan(mean(actual.observed), 0.90);
        end

        function testStraightSolverAgainstDenseReference(testCase)
            x = linspace(0, 40, 201).';
            side = GlassTubeCoreTest.makeSideData(x, zeros(size(x)), zeros(size(x)));
            actual = solve_max_rod_diameter(side, struct('safeInnerRadiusMm', 15), ...
                struct('safeInnerRadiusMm', 15), GlassTubeCoreTest.solverConfig());
            reference = GlassTubeCoreTest.denseRodReference(side, 15, 15, 0, [-0.75, 0.75]);

            testCase.verifyEqual(actual.diameterMm, 30, 'AbsTol', 1e-10);
            testCase.verifyLessThanOrEqual(abs(actual.diameterMm - reference.diameterMm), 0.01);
            testCase.verifyEqual(actual.axisSlope, 0, 'AbsTol', 1e-8);
        end

        function testInclinedSolverUsesPhysicalNormalWidth(testCase)
            x = linspace(0, 40, 201).';
            slope = 0.25;
            side = GlassTubeCoreTest.makeSideData(x, slope * x, zeros(size(x)));
            actual = solve_max_rod_diameter(side, struct('safeInnerRadiusMm', 15), ...
                struct('safeInnerRadiusMm', 15), GlassTubeCoreTest.solverConfig());
            reference = GlassTubeCoreTest.denseRodReference(side, 15, 15, 0, [-0.75, 0.75]);
            expectedNormalDiameter = 30 / sqrt(1 + slope^2);

            testCase.verifyLessThanOrEqual(abs(actual.diameterMm - reference.diameterMm), 0.01);
            testCase.verifyEqual(actual.diameterMm, expectedNormalDiameter, 'AbsTol', 1e-7);
            testCase.verifyEqual(actual.axisSlope, slope, 'AbsTol', 1e-7);
            testCase.verifyEqual(actual.diameterMm, ...
                actual.verticalClearanceMm / sqrt(1 + actual.axisSlope^2), 'AbsTol', 1e-10, ...
                'Reported diameter must be the physical width normal to the rod axis.');
            testCase.verifyLessThan(actual.diameterMm, actual.verticalClearanceMm, ...
                'A nonzero rod slope requires a physical normal-width correction.');
        end

        function testSinusoidalSolverAgainstDenseReference(testCase)
            x = linspace(0, 40, 201).';
            center = 0.8 * sin(2 * pi * x / x(end));
            side = GlassTubeCoreTest.makeSideData(x, center, zeros(size(x)));
            actual = solve_max_rod_diameter(side, struct('safeInnerRadiusMm', 15), ...
                struct('safeInnerRadiusMm', 15), GlassTubeCoreTest.solverConfig());
            reference = GlassTubeCoreTest.denseRodReference(side, 15, 15, 0, [-0.75, 0.75]);

            testCase.verifyLessThanOrEqual(abs(actual.diameterMm - reference.diameterMm), 0.01, ...
                'The optimized diameter must agree with an independent dense slope search.');
            testCase.verifyLessThan(actual.diameterMm, 30, ...
                'Curvature must reduce the diameter of a fitting straight rod.');
        end

        function testLocalConstrictionSolverDoesNotEnlargeBore(testCase)
            x = linspace(0, 40, 201).';
            shrink = 1.2 * exp(-((x - 20) / 1.3).^8);
            side = GlassTubeCoreTest.makeSideData(x, zeros(size(x)), shrink);
            actual = solve_max_rod_diameter(side, struct('safeInnerRadiusMm', 15), ...
                struct('safeInnerRadiusMm', 15), GlassTubeCoreTest.solverConfig());
            reference = GlassTubeCoreTest.denseRodReference(side, 15, 15, 0, [-0.75, 0.75]);
            conservativeMinimumDiameter = 2 * (15 - max(shrink));

            testCase.verifyLessThanOrEqual(abs(actual.diameterMm - reference.diameterMm), 0.01);
            testCase.verifyLessThanOrEqual(actual.diameterMm, conservativeMinimumDiameter + 1e-8, ...
                'A local inward wall feature must never be smoothed into a larger passable diameter.');
            testCase.verifyEqual(actual.diameterMm, conservativeMinimumDiameter, 'AbsTol', 1e-7);
            testCase.verifyEqual(actual.activeLimit, 'side local constriction');
        end
    end

    methods (Static, Access = private)
        function [img, truth, cfg, pxPerMm] = syntheticSideView()
            imageHeight = 180;
            imageWidth = 640;
            pxPerMm = 10;
            x = 1:imageWidth;
            bend = 2.2 * sin(2 * pi * (x - 1) / (imageWidth - 1));
            neck = 6.0 * exp(-((x - 455) / 22).^8);
            topPx = 55 + bend + neck;
            bottomPx = 125 + bend - neck;

            [rowGrid, ~] = ndgrid(single((1:imageHeight).'), single(1:imageWidth));
            edgeWidth = single(0.65);
            upperTransition = 1 ./ (1 + exp(-(rowGrid - single(topPx)) / edgeWidth));
            lowerTransition = 1 ./ (1 + exp((rowGrid - single(bottomPx)) / edgeWidth));
            bore = upperTransition .* lowerTransition;
            texture = single(0.006 * sin(0.17 * double(rowGrid)) + ...
                0.004 * cos(0.11 * double(rowGrid) + 0.07 * x));
            img = single(0.12) + single(0.76) * bore + texture;
            img = min(single(1), max(single(0), img));

            occlusionCols = 275:284;
            img(:, occlusionCols) = single(0.50);

            cfg = struct( ...
                'roiRect', [0, 0, imageWidth, imageHeight], ...
                'profileSigma', 1.2, ...
                'initialSearchHalfWidthPx', 22, ...
                'bandHalfWidthPx', 20, ...
                'cannySigma', 1.0, ...
                'cannyThreshold', [], ...
                'maxJumpPx', 3, ...
                'jumpPenalty', 0.20, ...
                'edgeStrengthWeight', 5.0, ...
                'centerPenalty', 0.08, ...
                'missingCandidatePenalty', 1.8, ...
                'seamCandidatePenalty', 0.35, ...
                'seamHalfWidthPx', 3, ...
                'maxInterpolatedGapCols', 16, ...
                'maxInvalidGapCols', 24, ...
                'minObservedFraction', 0.90, ...
                'centerSmoothSpan', 31, ...
                'gapBaselineSpan', 151, ...
                'baseUncertaintyPx', 0.50, ...
                'centerResidualWeight', 0.25, ...
                'interpolatedUncertaintyPx', 1.5, ...
                'seamUncertaintyPx', 1.0, ...
                'returnDebug', false, ...
                'outerBaselineTopPx', nan, ...
                'outerBaselineBottomPx', nan);

            truth = struct();
            truth.topPx = topPx(:);
            truth.bottomPx = bottomPx(:);
            truth.occlusionCols = occlusionCols;
            truth.occlusionMask = false(imageWidth, 1);
            truth.occlusionMask(occlusionCols) = true;
            truth.neckCoreMask = abs(x(:) - 455) <= 10;
        end

        function side = makeSideData(x, center, localShrink)
            side = struct( ...
                'xMm', x(:), ...
                'centerMm', center(:), ...
                'localShrinkMm', localShrink(:), ...
                'uncertaintyMm', zeros(numel(x), 1), ...
                'valid', true(numel(x), 1));
        end

        function cfg = solverConfig()
            cfg = struct( ...
                'additionalSafetyMarginMm', 0, ...
                'minimumSlopeBound', 0.08, ...
                'maximumSlopeBound', 0.8, ...
                'slopePercentile', 99, ...
                'slopeBoundFactor', 2, ...
                'slopePadding', 0.05, ...
                'coarseSlopeCount', 129, ...
                'refineSeedCount', 8, ...
                'slopeTolerance', 1e-11);
        end

        function reference = denseRodReference(side, leftRadius, rightRadius, safetyMargin, slopeRange)
            x = double(side.xMm(:));
            center = double(side.centerMm(:));
            shrink = double(side.localShrinkMm(:));
            uncertainty = double(side.uncertaintyMm(:));
            valid = logical(side.valid(:)) & isfinite(x) & isfinite(center) & ...
                isfinite(shrink) & isfinite(uncertainty);
            x = x(valid);
            x = x - x(1);
            center = center(valid);
            shrink = shrink(valid);
            uncertainty = uncertainty(valid);
            t = x / max(x(end), eps);
            radius = (1 - t) * leftRadius + t * rightRadius - shrink - uncertainty - safetyMargin;
            top = center - radius;
            bottom = center + radius;

            slopes = linspace(slopeRange(1), slopeRange(2), 100001);
            diameters = -inf(size(slopes));
            chunkSize = 2000;
            for first = 1:chunkSize:numel(slopes)
                last = min(numel(slopes), first + chunkSize - 1);
                m = slopes(first:last);
                shifted = x * m;
                verticalGap = min(bottom - shifted, [], 1) - max(top - shifted, [], 1);
                diameters(first:last) = verticalGap ./ sqrt(1 + m.^2);
            end
            [diameterMm, idx] = max(diameters);
            reference = struct('diameterMm', diameterMm, 'axisSlope', slopes(idx));
        end
    end
end
