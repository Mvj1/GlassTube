function calibration = calibrate_standard_block_scale(outputFile)
%CALIBRATE_STANDARD_BLOCK_SCALE Calibrate both camera scales with Canny edges.
%   CALIBRATION = CALIBRATE_STANDARD_BLOCK_SCALE() analyzes the two standard
%   block images and saves a versioned calibration MAT file under results/.
%   TLS line fitting removes the need to rotate and resample calibration images.

rootDir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(outputFile)
    outputFile = fullfile(rootDir, 'results', 'glass_tube_calibration.mat');
end
knownLengthMm = 30;
opts = struct('cannySigma', 1.5, 'cannyThreshold', [0.15 0.25], ...
    'minComponentArea', 30, 'minimumRows', 50, 'angleToleranceDeg', 15, ...
    'trimFraction', 0.05, 'maximumParallelAngleDeg', 3);

cameras = [ ...
    struct('role', 'side', 'label', 'longitudinal side view', ...
        'file', fullfile(rootDir, 'biaozhun2.bmp')), ...
    struct('role', 'end', 'label', 'end face', ...
        'file', fullfile(rootDir, 'biaozhun1.bmp'))];

records = repmat(struct(), numel(cameras), 1);
for idx = 1:numel(cameras)
    cam = cameras(idx);
    if ~isfile(cam.file)
        error('GlassTube:CalibrationImageMissing', 'Missing calibration image: %s', cam.file);
    end
    img = imread(cam.file);
    if ndims(img) == 3, img = im2gray(img); end
    gray = im2single(img);
    [gradX, gradY] = imgradientxy(gray, 'sobel');
    edgeMask = edge(gray, 'Canny', opts.cannyThreshold, opts.cannySigma);
    edgeMask = bwareaopen(edgeMask, opts.minComponentArea);
    [leftPts, rightPts] = sample_parallel_edges(edgeMask, gradX, gradY, opts);
    measurement = measure_parallel_distance(leftPts, rightPts, opts);

    records(idx).role = cam.role;
    records(idx).label = cam.label;
    records(idx).image = cam.file;
    records(idx).knownLengthMm = knownLengthMm;
    records(idx).pixelWidth = measurement.pixelWidth;
    records(idx).pxPerMm = measurement.pixelWidth / knownLengthMm;
    records(idx).distanceMadPx = measurement.distanceMadPx;
    records(idx).fitAngleDifferenceDeg = measurement.angleDifferenceDeg;
    records(idx).validRowCount = measurement.validRowCount;
    records(idx).sha256 = file_sha256(cam.file);
    fprintf('%s: %.3f px, %.6f px/mm, MAD %.4f px\n', ...
        cam.label, measurement.pixelWidth, records(idx).pxPerMm, measurement.distanceMadPx);
end

calibration = struct();
calibration.algorithmVersion = 'calibrate-standard-block-2.0.0';
calibration.createdAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
calibration.sidePxPerMm = records(strcmp({records.role}, 'side')).pxPerMm;
calibration.endPxPerMm = records(strcmp({records.role}, 'end')).pxPerMm;
calibration.records = records;
calibration.options = opts;

outDir = fileparts(outputFile);
if ~isempty(outDir) && ~isfolder(outDir), mkdir(outDir); end
save(outputFile, 'calibration');
end

function [leftPts, rightPts] = sample_parallel_edges(edgeMask, gradX, gradY, opts)
[rowN, colN] = size(edgeMask);
border = max(10, round(colN * 0.01));
angleTolerance = deg2rad(opts.angleToleranceDeg);
gradAngle = atan2(abs(gradY), abs(gradX));
candidates = edgeMask & gradAngle <= angleTolerance;
leftPts = nan(rowN, 2); rightPts = nan(rowN, 2); count = 0;
for row = 1:rowN
    xs = find(candidates(row, border + 1:colN - border));
    if numel(xs) < 2, continue; end
    count = count + 1;
    leftPts(count, :) = [xs(1) + border, row];
    rightPts(count, :) = [xs(end) + border, row];
end
leftPts = leftPts(1:count, :); rightPts = rightPts(1:count, :);
if count < opts.minimumRows
    error('GlassTube:CalibrationInsufficientEdges', 'Only %d usable rows were found.', count);
end
y = leftPts(:, 2);
keep = y >= prctile(y, 100 * opts.trimFraction) & ...
    y <= prctile(y, 100 * (1 - opts.trimFraction));
leftPts = leftPts(keep, :); rightPts = rightPts(keep, :);
end

function measurement = measure_parallel_distance(leftPts, rightPts, opts)
[leftPts, leftLine] = robust_line(leftPts, opts.minimumRows);
[rightPts, rightLine] = robust_line(rightPts, opts.minimumRows);
dirLeft = leftLine.direction; dirRight = rightLine.direction;
if dot(dirLeft, dirRight) < 0, dirRight = -dirRight; end
angleDifference = acosd(max(-1, min(1, dot(dirLeft, dirRight))));
if angleDifference > opts.maximumParallelAngleDeg
    error('GlassTube:CalibrationNonparallelEdges', ...
        'Calibration edges differ by %.3f degrees.', angleDifference);
end
commonDirection = dirLeft + dirRight;
commonDirection = commonDirection / norm(commonDirection);
normal = [-commonDirection(2), commonDirection(1)];
leftDistance = leftPts * normal.'; rightDistance = rightPts * normal.';
pixelWidth = abs(median(rightDistance) - median(leftDistance));
edgeResidual = [leftDistance - median(leftDistance); rightDistance - median(rightDistance)];
measurement = struct('pixelWidth', pixelWidth, ...
    'distanceMadPx', median(abs(edgeResidual - median(edgeResidual))), ...
    'angleDifferenceDeg', angleDifference, ...
    'validRowCount', min(size(leftPts, 1), size(rightPts, 1)));
end

function [inliers, line] = robust_line(points, minimumRows)
line = fit_tls(points);
residual = abs(points * line.normal.' - line.offset);
med = median(residual); scale = median(abs(residual - med));
keep = residual <= max(1, med + 3 * scale);
inliers = points(keep, :);
if size(inliers, 1) < minimumRows
    error('GlassTube:CalibrationInsufficientInliers', 'Only %d line inliers remain.', size(inliers, 1));
end
line = fit_tls(inliers);
end

function line = fit_tls(points)
center = mean(points, 1);
[~, ~, basis] = svd(points - center, 0);
direction = basis(:, 1).'; direction = direction / norm(direction);
normal = [-direction(2), direction(1)];
line = struct('center', center, 'direction', direction, ...
    'normal', normal, 'offset', dot(normal, center));
end

function digest = file_sha256(path)
md = java.security.MessageDigest.getInstance('SHA-256');
fid = fopen(path, 'rb');
if fid < 0, error('GlassTube:FileHashFailed', 'Cannot open %s.', path); end
try
    bytes = fread(fid, inf, '*uint8'); fclose(fid);
catch cause
    fclose(fid); rethrow(cause);
end
md.update(bytes);
digest = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2).', 1, []));
end
