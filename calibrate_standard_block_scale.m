close all;
clear;
clc;

set(groot, 'defaultFigureUnits', 'normalized');
set(groot, 'defaultFigurePosition', [0, 0, 1, 1]);
set(groot, 'defaultFigureWindowState', 'maximized');

knownLengthMm = 30;
cannySigma = 1.5;
cannyTh = [0.15, 0.25];
minValidRows = 50;
minComponentArea = 30;
angleTolDeg = 15;
trimFrac = 0.05;
maxParallelAngleDeg = 3;

cameras = {
    struct('label', 'top camera',  'file', 'biaozhun2.bmp', 'angle', -0.39), ...
    struct('label', 'side camera', 'file', 'biaozhun1.bmp', 'angle', 0.00)
};

for idx = 1:numel(cameras)
    cam = cameras{idx};
    grayImg = preprocess_standard_block(cam.file, cam.angle, cannySigma);
    [gradX, gradY] = imgradientxy(grayImg, 'sobel');
    edgeMask = edge(grayImg, 'Canny', cannyTh);
    edgeMask = bwareaopen(edgeMask, minComponentArea);

    [leftPts, rightPts] = sample_left_right_points( ...
        edgeMask, gradX, gradY, minValidRows, angleTolDeg, trimFrac);
    pixelWidth = measure_parallel_distance( ...
        leftPts, rightPts, minValidRows, maxParallelAngleDeg);
    pxPerMm = pixelWidth / knownLengthMm;

    fprintf('%s (%s): pixelWidth = %.2f px, scale = %.6f px/mm\n', ...
        cam.label, cam.file, pixelWidth, pxPerMm);
end


function grayImg = preprocess_standard_block(fileName, rotateAngle, cannySigma)
img = imread(fileName);

if ndims(img) == 3
    grayImg = rgb2gray(img);
else
    grayImg = img;
end

grayImg = im2double(grayImg);

if rotateAngle ~= 0
    grayImg = imrotate(grayImg, rotateAngle, 'bicubic', 'crop');
end

grayImg = imgaussfilt(grayImg, cannySigma);
end


function [leftPts, rightPts] = sample_left_right_points( ...
    edgeMask, gradX, gradY, minValidRows, angleTolDeg, trimFrac)
[~, colN] = size(edgeMask);
borderMargin = max(10, round(colN * 0.01));
angleTol = angleTolDeg * pi / 180;

if colN <= 2 * borderMargin + 1
    error('Image width is too small after applying the border margin.');
end

gradAngle = atan2(abs(gradY), abs(gradX));
verticalMask = gradAngle <= angleTol;
candidateMask = edgeMask & verticalMask;

leftPts = nan(size(edgeMask, 1), 2);
rightPts = nan(size(edgeMask, 1), 2);
validN = 0;

for row = 1:size(edgeMask, 1)
    rowMask = candidateMask(row, borderMargin + 1:colN - borderMargin);
    xs = find(rowMask);
    if numel(xs) < 2
        continue;
    end

    validN = validN + 1;
    leftX = xs(1) + borderMargin;
    rightX = xs(end) + borderMargin;
    leftPts(validN, :) = [leftX, row];
    rightPts(validN, :) = [rightX, row];
end

leftPts = leftPts(1:validN, :);
rightPts = rightPts(1:validN, :);

if size(leftPts, 1) < minValidRows || size(rightPts, 1) < minValidRows
    error('Calibration failed: only %d valid rows were found.', size(leftPts, 1));
end

yVals = leftPts(:, 2);
yLow = prctile(yVals, trimFrac * 100);
yHigh = prctile(yVals, (1 - trimFrac) * 100);
keepMask = yVals >= yLow & yVals <= yHigh;

leftPts = leftPts(keepMask, :);
rightPts = rightPts(keepMask, :);

if size(leftPts, 1) < minValidRows || size(rightPts, 1) < minValidRows
    error('Calibration failed: only %d valid rows remained after trimming.', size(leftPts, 1));
end
end


function pixelWidth = measure_parallel_distance( ...
    leftPts, rightPts, minValidRows, maxParallelAngleDeg)
leftLine = fit_line_tls(leftPts);
rightLine = fit_line_tls(rightPts);

[leftPts, leftLine] = refine_line_fit(leftPts, leftLine, minValidRows);
[rightPts, rightLine] = refine_line_fit(rightPts, rightLine, minValidRows);

dirLeft = leftLine.dir;
dirRight = rightLine.dir;
if dot(dirLeft, dirRight) < 0
    dirRight = -dirRight;
end

cosTheta = max(-1, min(1, dot(dirLeft, dirRight)));
angleDeg = acosd(cosTheta);
if angleDeg > maxParallelAngleDeg
    error('Calibration failed: left/right edge angle mismatch is %.3f deg.', angleDeg);
end

dirCommon = dirLeft + dirRight;
dirNorm = norm(dirCommon);
if dirNorm < eps
    error('Calibration failed: unable to build a common edge direction.');
end
dirCommon = dirCommon / dirNorm;
nCommon = [-dirCommon(2), dirCommon(1)];
nCommon = nCommon / norm(nCommon);

dLeft = median(leftPts * nCommon.');
dRight = median(rightPts * nCommon.');
pixelWidth = abs(dRight - dLeft);

if ~isfinite(pixelWidth) || pixelWidth <= 0
    error('Calibration failed: measured pixel width is invalid.');
end
end


function lineModel = fit_line_tls(points)
centroid = mean(points, 1);
centered = points - centroid;
[~, ~, basis] = svd(centered, 0);
dir = basis(:, 1).';
dir = dir / norm(dir);
normal = [-dir(2), dir(1)];
normal = normal / norm(normal);

lineModel.centroid = centroid;
lineModel.dir = dir;
lineModel.normal = normal;
lineModel.d = dot(normal, centroid);
end


function [inlierPts, lineModel] = refine_line_fit(points, lineModel, minValidRows)
residuals = abs(points * lineModel.normal.' - lineModel.d);
resMed = median(residuals);
resMad = median(abs(residuals - resMed));
thr = max(resMed + 3 * resMad, 1.0);
inlierMask = residuals <= thr;
inlierPts = points(inlierMask, :);

if size(inlierPts, 1) < minValidRows
    error('Calibration failed: only %d inlier points remained after line fitting.', size(inlierPts, 1));
end

lineModel = fit_line_tls(inlierPts);
end
