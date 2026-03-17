close all;
clear;
clc;

knownLengthMm = 30;
cannySigma = 1.5;
cannyTh = [0.15, 0.25];
minValidRows = 50;
minComponentArea = 30;

cameras = {
    struct('label', 'top camera',  'file', 'biaozhun2.bmp', 'angle', -0.39), ...
    struct('label', 'side camera', 'file', 'biaozhun1.bmp', 'angle', 0.00)
};

for idx = 1:numel(cameras)
    cam = cameras{idx};
    grayImg = preprocess_standard_block(cam.file, cam.angle, cannySigma);
    edgeMask = edge(grayImg, 'Canny', cannyTh);
    edgeMask = bwareaopen(edgeMask, minComponentArea);

    pixelWidth = measure_left_right_width(edgeMask, minValidRows);
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


function pixelWidth = measure_left_right_width(edgeMask, minValidRows)
[~, colN] = size(edgeMask);
borderMargin = max(10, round(colN * 0.01));

if colN <= 2 * borderMargin + 1
    error('Image width is too small after applying the border margin.');
end

rowSpans = nan(size(edgeMask, 1), 1);

for row = 1:size(edgeMask, 1)
    rowMask = edgeMask(row, borderMargin + 1:colN - borderMargin);
    xs = find(rowMask);
    if numel(xs) < 2
        continue;
    end

    leftX = xs(1) + borderMargin;
    rightX = xs(end) + borderMargin;
    rowSpans(row) = rightX - leftX;
end

rowSpans = rowSpans(~isnan(rowSpans));

if numel(rowSpans) < minValidRows
    error('Calibration failed: only %d valid rows were found.', numel(rowSpans));
end

pixelWidth = median(rowSpans);
end
