function [stripImg, transforms, quality, timing] = stitch_tube_strip(cfg)
%STITCH_TUBE_STRIP Register and stitch a naturally sorted tube image sequence.
%
%   [STRIP, TRANSFORMS, QUALITY, TIMING] = STITCH_TUBE_STRIP(CFG)
%   estimates adjacent-frame translation on a downsampled grayscale ROI and
%   composes a full-resolution strip in a single preallocated canvas.
%
%   Required configuration (flat names shown; legacy nested aliases are
%   accepted for imageDir/extension/rotationAngle/nominal/search):
%       cfg.imageDir              image directory
%       cfg.extension             '*.bmp', '.bmp', 'bmp', or a cell array
%       cfg.rotationAngle         rotation in degrees
%       cfg.nominal               nominal horizontal step in full-res pixels
%       cfg.search                scalar X search radius or [X Y] pixels
%       cfg.quality.minPeak       minimum normalized-correlation peak
%       cfg.quality.minPSR        minimum peak-to-sidelobe ratio
%       cfg.quality.onFailure     'error' or 'nominal'
%
%   Optional configuration:
%       cfg.downsample                         default 0.50
%       cfg.registration.rowRange              default [1/3 2/3]
%           Fractions when both values are <= 1, otherwise full-res rows.
%       cfg.registration.templateXOffset       default -200 pixels
%       cfg.registration.templateWidth         default 160 pixels
%       cfg.quality.psrExclusionRadius         default 2 downsampled pixels
%       cfg.rotationMethod                     default 'bicubic'
%       cfg.verbose                            default false
%
%   TRANSFORMS is a table with pairwise and accumulated full-resolution
%   translations. CumulativeY is the vertical translation applied to each
%   frame (positive is downward). The output preserves the legacy project
%   orientation: the first natural-order frame is at the right-hand end.
%
%   QUALITY contains peak, PSR, pass/fallback flags, thresholds, and a pair
%   table. TIMING reports stage and per-frame/per-pair wall-clock timings.
%
%   The function intentionally performs no disk writes.

    totalTimer = tic;
    cfg = normalize_config(cfg);

    stageTimer = tic;
    files = discover_images(cfg.imageDir, cfg.extension);
    timing.fileDiscovery = toc(stageTimer);
    frameCount = numel(files);
    if frameCount < 2
        error('stitch_tube_strip:TooFewImages', ...
            'At least two images are required; found %d in "%s".', ...
            frameCount, cfg.imageDir);
    end

    pairCount = frameCount - 1;
    timing.perFrameReadRotate = zeros(frameCount, 1);
    timing.perFrameDownsample = zeros(frameCount, 1);
    timing.perFrameCompose = zeros(frameCount, 1);
    timing.perPairRegistration = zeros(pairCount, 1);

    pairDxRaw = nan(pairCount, 1);
    pairDyRaw = nan(pairCount, 1);
    pairDx = nan(pairCount, 1);
    pairDy = nan(pairCount, 1);
    peak = nan(pairCount, 1);
    psr = nan(pairCount, 1);
    passed = false(pairCount, 1);
    usedFallback = false(pairCount, 1);
    subpixelX = nan(pairCount, 1);
    subpixelY = nan(pairCount, 1);

    frameTimer = tic;
    firstImg = read_and_rotate(files(1).path, cfg.rotationAngle, cfg.rotationMethod);
    timing.perFrameReadRotate(1) = toc(frameTimer);

    [imgH, imgW, imgC] = image_shape(firstImg);
    referenceSize = size(firstImg);
    referenceClass = class(firstImg);
    cfg = finalize_geometry(cfg, imgH, imgW, frameCount);

    downTimer = tic;
    prevGraySmall = downsample_gray(firstImg, cfg.downsample);
    timing.perFrameDownsample(1) = toc(downTimer);

    % Allocate once using the strict configured displacement bounds. The
    % final crop makes the returned array exact without repeated growth.
    canvasW = round(cfg.nominal) + pairCount * ...
        (ceil(cfg.nominal + cfg.searchX) + 2);
    verticalMargin = pairCount * (ceil(cfg.searchY) + 1) + 3;
    canvasH = imgH + 2 * verticalMargin + 2;
    canvas = zeros([canvasH, canvasW, imgC], 'like', firstImg);

    cumulativeX = zeros(frameCount, 1);
    cumulativeY = zeros(frameCount, 1);
    patchWidth = zeros(frameCount, 1);
    bufferX1 = zeros(frameCount, 1);
    bufferX2 = zeros(frameCount, 1);
    bufferY1 = zeros(frameCount, 1);
    bufferY2 = zeros(frameCount, 1);

    rightCursor = canvasW;
    topUsed = canvasH;
    bottomUsed = 1;
    baseRow = verticalMargin + 2;

    firstWidth = round(cfg.nominal);
    firstPatch = centered_patch(firstImg, firstWidth);
    composeTimer = tic;
    x1 = rightCursor - firstWidth + 1;
    x2 = rightCursor;
    [canvas, y1, y2] = place_patch(canvas, firstPatch, baseRow, 0, x1, x2);
    timing.perFrameCompose(1) = toc(composeTimer);
    patchWidth(1) = firstWidth;
    bufferX1(1) = x1;
    bufferX2(1) = x2;
    bufferY1(1) = y1;
    bufferY2(1) = y2;
    rightCursor = x1 - 1;
    topUsed = min(topUsed, y1);
    bottomUsed = max(bottomUsed, y2);
    clear firstImg firstPatch;

    for k = 2:frameCount
        frameTimer = tic;
        currImg = read_and_rotate(files(k).path, cfg.rotationAngle, cfg.rotationMethod);
        timing.perFrameReadRotate(k) = toc(frameTimer);
        validate_frame(currImg, referenceSize, referenceClass, files(k).name);

        downTimer = tic;
        currGraySmall = downsample_gray(currImg, cfg.downsample);
        timing.perFrameDownsample(k) = toc(downTimer);

        registrationTimer = tic;
        match = estimate_pair(prevGraySmall, currGraySmall, cfg, imgH, imgW);
        timing.perPairRegistration(k - 1) = toc(registrationTimer);

        pairDxRaw(k - 1) = match.dx;
        pairDyRaw(k - 1) = match.dy;
        peak(k - 1) = match.peak;
        psr(k - 1) = match.psr;
        subpixelX(k - 1) = match.subpixelX;
        subpixelY(k - 1) = match.subpixelY;

        isGood = isfinite(match.dx) && isfinite(match.dy) && ...
            isfinite(match.peak) && isfinite(match.psr) && ...
            match.peak >= cfg.quality.minPeak && ...
            match.psr >= cfg.quality.minPSR && ...
            abs(match.dx - cfg.nominal) <= cfg.searchX + cfg.boundTolerance && ...
            abs(match.dy) <= cfg.searchY + cfg.boundTolerance;
        passed(k - 1) = isGood;

        if isGood
            dxUsed = match.dx;
            dyUsed = match.dy;
        else
            failureText = sprintf([ ...
                'Low-quality match %d -> %d (%s -> %s): peak=%.5f ' ...
                '(min %.5f), PSR=%.3f (min %.3f), dx=%.3f, dy=%.3f.'], ...
                k - 1, k, files(k - 1).name, files(k).name, ...
                match.peak, cfg.quality.minPeak, match.psr, ...
                cfg.quality.minPSR, match.dx, match.dy);
            switch cfg.quality.onFailure
                case "error"
                    error('stitch_tube_strip:LowQualityMatch', '%s', failureText);
                case "nominal"
                    warning('stitch_tube_strip:Fallback', ...
                        '%s Using nominal dx=%.3f and dy=0.', ...
                        failureText, cfg.nominal);
                    dxUsed = cfg.nominal;
                    dyUsed = 0;
                    usedFallback(k - 1) = true;
                otherwise
                    error('stitch_tube_strip:InternalConfig', ...
                        'Unsupported quality.onFailure value "%s".', ...
                        cfg.quality.onFailure);
            end
        end

        pairDx(k - 1) = dxUsed;
        pairDy(k - 1) = dyUsed;
        cumulativeX(k) = cumulativeX(k - 1) + dxUsed;
        % A feature at y+dy in the current frame is aligned by translating
        % the current frame by -dy. Accumulating this fixes the old local-dy
        % placement error for frame 3 and later.
        cumulativeY(k) = cumulativeY(k - 1) - dyUsed;

        currentWidth = round(cumulativeX(k)) - round(cumulativeX(k - 1));
        if currentWidth < 1 || currentWidth > imgW
            error('stitch_tube_strip:InvalidPatchWidth', ...
                'Frame %d produced invalid patch width %d pixels.', ...
                k, currentWidth);
        end
        if currentWidth > rightCursor
            error('stitch_tube_strip:CanvasOverflow', ...
                'Configured horizontal bounds were exceeded at frame %d.', k);
        end

        patch = centered_patch(currImg, currentWidth);
        composeTimer = tic;
        x1 = rightCursor - currentWidth + 1;
        x2 = rightCursor;
        [canvas, y1, y2] = place_patch( ...
            canvas, patch, baseRow, cumulativeY(k), x1, x2);
        timing.perFrameCompose(k) = toc(composeTimer);

        patchWidth(k) = currentWidth;
        bufferX1(k) = x1;
        bufferX2(k) = x2;
        bufferY1(k) = y1;
        bufferY2(k) = y2;
        rightCursor = x1 - 1;
        topUsed = min(topUsed, y1);
        bottomUsed = max(bottomUsed, y2);

        if cfg.verbose
            fprintf(['Frame %02d/%02d: dx=%8.3f px, dy=%+7.3f px, ' ...
                'peak=%.4f, PSR=%6.2f, %s\n'], ...
                k, frameCount, dxUsed, dyUsed, match.peak, match.psr, ...
                char(quality_label(isGood, usedFallback(k - 1))));
        end

        prevGraySmall = currGraySmall;
        clear currImg patch;
    end

    finalizeTimer = tic;
    usedX1 = rightCursor + 1;
    stripImg = canvas(topUsed:bottomUsed, usedX1:canvasW, :);
    timing.finalize = toc(finalizeTimer);
    clear canvas;

    outputX1 = bufferX1 - usedX1 + 1;
    outputX2 = bufferX2 - usedX1 + 1;
    outputY1 = bufferY1 - topUsed + 1;
    outputY2 = bufferY2 - topUsed + 1;

    frameIndex = (1:frameCount).';
    fileName = string({files.name}).';
    framePairDx = [0; pairDx];
    framePairDy = [0; pairDy];
    transforms = table(frameIndex, fileName, framePairDx, framePairDy, ...
        cumulativeX, cumulativeY, patchWidth, outputX1, outputX2, ...
        outputY1, outputY2, ...
        'VariableNames', {'Frame', 'File', 'PairDx', 'PairDy', ...
        'CumulativeX', 'CumulativeY', 'PatchWidth', 'OutputX1', ...
        'OutputX2', 'OutputY1', 'OutputY2'});

    pairIndex = (1:pairCount).';
    previousFile = string({files(1:end - 1).name}).';
    currentFile = string({files(2:end).name}).';
    status = quality_label(passed, usedFallback);
    quality.pairs = table(pairIndex, previousFile, currentFile, ...
        pairDxRaw, pairDyRaw, pairDx, pairDy, peak, psr, ...
        subpixelX, subpixelY, passed, usedFallback, status, ...
        'VariableNames', {'Pair', 'PreviousFile', 'CurrentFile', ...
        'RawDx', 'RawDy', 'Dx', 'Dy', 'Peak', 'PSR', ...
        'SubpixelX', 'SubpixelY', 'Passed', 'UsedFallback', 'Status'});
    quality.peak = peak;
    quality.psr = psr;
    quality.passed = passed;
    quality.usedFallback = usedFallback;
    quality.allPassed = all(passed);
    quality.numFallbacks = nnz(usedFallback);
    quality.thresholds = struct( ...
        'minPeak', cfg.quality.minPeak, ...
        'minPSR', cfg.quality.minPSR, ...
        'onFailure', cfg.quality.onFailure);

    timing.readRotate = sum(timing.perFrameReadRotate);
    timing.downsample = sum(timing.perFrameDownsample);
    timing.registration = sum(timing.perPairRegistration);
    timing.composition = sum(timing.perFrameCompose);
    timing.total = toc(totalTimer);
    timing.frameCount = frameCount;
    timing.outputSize = size(stripImg);
end


function cfg = normalize_config(cfg)
    if ~isstruct(cfg) || ~isscalar(cfg)
        error('stitch_tube_strip:InvalidConfig', ...
            'cfg must be a scalar structure.');
    end

    cfg.imageDir = get_required(cfg, 'imageDir', {'dir', 'img'});
    cfg.extension = get_required(cfg, 'extension', {'dir', 'ext'});
    cfg.rotationAngle = get_required(cfg, 'rotationAngle', {'rot', 'angle'});
    cfg.nominal = get_required(cfg, 'nominal', {'step', 'nominal'});
    searchValue = get_required(cfg, 'search', {'step', 'search'});

    cfg.imageDir = char(string(cfg.imageDir));
    if ~isfolder(cfg.imageDir)
        error('stitch_tube_strip:InvalidImageDirectory', ...
            'Image directory does not exist: "%s".', cfg.imageDir);
    end
    validateattributes(cfg.rotationAngle, {'numeric'}, ...
        {'scalar', 'real', 'finite'}, mfilename, 'rotationAngle');
    validateattributes(cfg.nominal, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, 'nominal');
    validateattributes(searchValue, {'numeric'}, ...
        {'vector', 'real', 'finite', 'nonnegative', 'nonempty'}, ...
        mfilename, 'search');
    if isscalar(searchValue)
        cfg.searchX = double(searchValue);
        cfg.searchY = min(8, double(searchValue));
    elseif numel(searchValue) == 2
        cfg.searchX = double(searchValue(1));
        cfg.searchY = double(searchValue(2));
    else
        error('stitch_tube_strip:InvalidSearch', ...
            'cfg.search must be a scalar or [searchX searchY].');
    end

    if ~isfield(cfg, 'quality') || ~isstruct(cfg.quality)
        error('stitch_tube_strip:MissingQualityConfig', ...
            'cfg.quality with minPeak, minPSR, and onFailure is required.');
    end
    requiredQuality = {'minPeak', 'minPSR', 'onFailure'};
    for i = 1:numel(requiredQuality)
        if ~isfield(cfg.quality, requiredQuality{i})
            error('stitch_tube_strip:MissingQualityConfig', ...
                'cfg.quality.%s is required.', requiredQuality{i});
        end
    end
    validateattributes(cfg.quality.minPeak, {'numeric'}, ...
        {'scalar', 'real', 'finite', '>=', -1, '<=', 1}, ...
        mfilename, 'quality.minPeak');
    validateattributes(cfg.quality.minPSR, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'}, ...
        mfilename, 'quality.minPSR');
    cfg.quality.onFailure = lower(string(cfg.quality.onFailure));
    if ~isscalar(cfg.quality.onFailure) || ...
            ~ismember(cfg.quality.onFailure, ["error", "nominal"])
        error('stitch_tube_strip:InvalidFailurePolicy', ...
            'cfg.quality.onFailure must be ''error'' or ''nominal''.');
    end

    cfg.downsample = get_optional(cfg, 'downsample', 0.50);
    validateattributes(cfg.downsample, {'numeric'}, ...
        {'scalar', 'real', 'finite', '>', 0, '<=', 1}, ...
        mfilename, 'downsample');

    if ~isfield(cfg, 'registration') || ~isstruct(cfg.registration)
        cfg.registration = struct();
    end
    cfg.registration.rowRange = get_optional( ...
        cfg.registration, 'rowRange', [1/3, 2/3]);
    cfg.registration.templateXOffset = get_optional( ...
        cfg.registration, 'templateXOffset', -200);
    cfg.registration.templateWidth = get_optional( ...
        cfg.registration, 'templateWidth', 160);
    cfg.quality.psrExclusionRadius = get_optional( ...
        cfg.quality, 'psrExclusionRadius', 2);
    cfg.rotationMethod = char(string(get_optional( ...
        cfg, 'rotationMethod', 'bicubic')));
    cfg.verbose = logical(get_optional(cfg, 'verbose', false));
    cfg.boundTolerance = max(1, 1 / cfg.downsample);

    validateattributes(cfg.registration.rowRange, {'numeric'}, ...
        {'vector', 'numel', 2, 'real', 'finite'}, ...
        mfilename, 'registration.rowRange');
    validateattributes(cfg.registration.templateXOffset, {'numeric'}, ...
        {'scalar', 'real', 'finite'}, ...
        mfilename, 'registration.templateXOffset');
    validateattributes(cfg.registration.templateWidth, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, ...
        mfilename, 'registration.templateWidth');
    validateattributes(cfg.quality.psrExclusionRadius, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'integer', 'nonnegative'}, ...
        mfilename, 'quality.psrExclusionRadius');
    if ~ismember(lower(string(cfg.rotationMethod)), ...
            ["nearest", "bilinear", "bicubic"])
        error('stitch_tube_strip:InvalidRotationMethod', ...
            'rotationMethod must be nearest, bilinear, or bicubic.');
    end
end


function value = get_required(cfg, flatName, nestedPath)
    if isfield(cfg, flatName)
        value = cfg.(flatName);
        return;
    end
    if numel(nestedPath) == 2 && isfield(cfg, nestedPath{1}) && ...
            isstruct(cfg.(nestedPath{1})) && ...
            isfield(cfg.(nestedPath{1}), nestedPath{2})
        value = cfg.(nestedPath{1}).(nestedPath{2});
        return;
    end
    error('stitch_tube_strip:MissingConfig', ...
        'Missing required configuration field cfg.%s.', flatName);
end


function value = get_optional(cfg, name, defaultValue)
    if isfield(cfg, name)
        value = cfg.(name);
    else
        value = defaultValue;
    end
end


function cfg = finalize_geometry(cfg, imgH, imgW, frameCount)
    rowRange = double(cfg.registration.rowRange(:).');
    if all(rowRange <= 1)
        rows = round(rowRange .* (imgH - 1) + 1);
    else
        rows = round(rowRange);
    end
    rows(1) = max(1, rows(1));
    rows(2) = min(imgH, rows(2));
    if rows(2) - rows(1) + 1 < 32
        error('stitch_tube_strip:InvalidRegistrationRows', ...
            'Registration row range must contain at least 32 full-res rows.');
    end

    tplWidth = round(cfg.registration.templateWidth);
    tplX = floor(imgW / 2) + round(cfg.registration.templateXOffset);
    expectedX = tplX + cfg.nominal;
    if tplX < 1 || tplX + tplWidth - 1 > imgW
        error('stitch_tube_strip:InvalidTemplateROI', ...
            'Template ROI [%d, %d] is outside the image width %d.', ...
            tplX, tplX + tplWidth - 1, imgW);
    end
    if expectedX - cfg.searchX < 1 || ...
            expectedX + tplWidth - 1 + cfg.searchX > imgW
        error('stitch_tube_strip:InvalidSearchROI', ...
            'Search ROI exceeds the image bounds. Reduce nominal/search or move the template.');
    end
    if cfg.nominal + cfg.searchX > imgW
        error('stitch_tube_strip:InvalidNominalStep', ...
            'nominal + search must be smaller than the image width.');
    end
    if round(cfg.nominal) + (frameCount - 1) * ...
            (ceil(cfg.nominal + cfg.searchX) + 2) > flintmax
        error('stitch_tube_strip:OutputTooLarge', ...
            'Configured output width is not representable.');
    end

    cfg.registration.rowsFull = rows;
    cfg.registration.templateXFull = tplX;
    cfg.registration.templateWidthFull = tplWidth;
end


function files = discover_images(imageDir, extensions)
    if ischar(extensions) || (isstring(extensions) && isscalar(extensions))
        extensions = cellstr(extensions);
    elseif isstring(extensions)
        extensions = cellstr(extensions(:));
    elseif ~iscell(extensions)
        error('stitch_tube_strip:InvalidExtension', ...
            'extension must be text or a cell/string array of extensions.');
    end

    found = struct('name', {}, 'folder', {}, 'path', {});
    for i = 1:numel(extensions)
        pattern = char(string(extensions{i}));
        if isempty(pattern)
            continue;
        end
        exactPath = fullfile(imageDir, pattern);
        if ~contains(pattern, '*') && ~contains(pattern, '?') && isfile(exactPath)
            entries = dir(exactPath);
        else
            if ~contains(pattern, '*') && ~contains(pattern, '?')
                if startsWith(pattern, '.')
                    pattern = strcat('*', pattern);
                else
                    pattern = strcat('*.', pattern);
                end
            end
            entries = dir(fullfile(imageDir, pattern));
        end
        entries = entries(~[entries.isdir]);
        for j = 1:numel(entries)
            item.name = entries(j).name;
            item.folder = entries(j).folder;
            item.path = fullfile(entries(j).folder, entries(j).name);
            found(end + 1) = item; %#ok<AGROW>
        end
    end
    if isempty(found)
        error('stitch_tube_strip:NoImages', ...
            'No images matching the configured extension were found in "%s".', ...
            imageDir);
    end

    allPaths = lower(string({found.path}));
    [~, uniqueIdx] = unique(allPaths, 'stable');
    found = found(uniqueIdx);
    names = string({found.name});
    keys = strings(size(names));
    for i = 1:numel(names)
        keys(i) = natural_key(names(i));
    end
    [~, order] = sort(keys + "|" + lower(names));
    files = found(order);
end


function key = natural_key(name)
    tokens = regexp(lower(char(name)), '\d+|\D+', 'match');
    parts = strings(1, numel(tokens));
    for i = 1:numel(tokens)
        token = tokens{i};
        if all(isstrprop(token, 'digit'))
            digits = regexprep(token, '^0+(?=\d)', '');
            parts(i) = "#" + compose('%08d', strlength(digits)) + ...
                ":" + string(digits) + ":" + ...
                compose('%08d', strlength(token));
        else
            parts(i) = "@" + string(token);
        end
    end
    key = join(parts, "");
end


function img = read_and_rotate(path, angle, method)
    img = imread(path);
    if ~(isnumeric(img) || islogical(img))
        error('stitch_tube_strip:UnsupportedImageType', ...
            'Unsupported image type in "%s".', path);
    end
    if ndims(img) > 3 || ~(size(img, 3) == 1 || size(img, 3) == 3)
        error('stitch_tube_strip:UnsupportedChannels', ...
            'Image "%s" must be grayscale or RGB.', path);
    end
    if angle ~= 0
        img = imrotate(img, angle, method, 'crop');
    end
end


function [h, w, c] = image_shape(img)
    h = size(img, 1);
    w = size(img, 2);
    c = size(img, 3);
end


function validate_frame(img, referenceSize, referenceClass, fileName)
    if ~isequal(size(img), referenceSize) || ~strcmp(class(img), referenceClass)
        error('stitch_tube_strip:InconsistentFrames', ...
            'Frame "%s" differs in size, channels, or numeric class.', fileName);
    end
end


function graySmall = downsample_gray(img, scale)
    if size(img, 3) == 3
        gray = rgb2gray(img);
    else
        gray = img;
    end
    targetSize = max([32, 32], round([size(gray, 1), size(gray, 2)] * scale));
    graySmall = imresize(gray, targetSize, 'bilinear', 'Antialiasing', true);
end


function match = estimate_pair(prevGray, currGray, cfg, fullH, fullW)
    smallH = size(prevGray, 1);
    smallW = size(prevGray, 2);
    scaleY = (smallH - 1) / max(fullH - 1, 1);
    scaleX = (smallW - 1) / max(fullW - 1, 1);

    rowsFull = cfg.registration.rowsFull;
    tplXFull = cfg.registration.templateXFull;
    tplWFull = cfg.registration.templateWidthFull;

    r1 = full_to_small(rowsFull(1), scaleY, smallH);
    r2 = full_to_small(rowsFull(2), scaleY, smallH);
    tplX = full_to_small(tplXFull, scaleX, smallW);
    tplW = max(8, round(tplWFull * scaleX));
    tplX2 = min(smallW, tplX + tplW - 1);
    tplX = tplX2 - tplW + 1;

    searchX = max(0, ceil(cfg.searchX * scaleX));
    searchY = max(0, ceil(cfg.searchY * scaleY));
    expectedX = tplX + cfg.nominal * scaleX;
    sx1 = max(1, floor(expectedX - searchX));
    sx2 = min(smallW, ceil(expectedX + searchX) + tplW - 1);
    sy1 = max(1, r1 - searchY);
    sy2 = min(smallH, r2 + searchY);

    template = im2single(prevGray(r1:r2, tplX:tplX2));
    searchImage = im2single(currGray(sy1:sy2, sx1:sx2));
    if std(template(:)) <= eps('single')
        error('stitch_tube_strip:FlatTemplate', ...
            'Registration template has insufficient intensity variation.');
    end
    if any(size(searchImage) < size(template))
        error('stitch_tube_strip:SearchSmallerThanTemplate', ...
            'Registration search ROI is smaller than the template ROI.');
    end

    corrMap = normxcorr2(template, searchImage);
    validMap = corrMap(size(template, 1):size(searchImage, 1), ...
        size(template, 2):size(searchImage, 2));
    [peakValue, linearIndex] = max(validMap(:));
    [peakRow, peakCol] = ind2sub(size(validMap), linearIndex);

    [offsetX, fittedPeakX] = quadratic_offset(validMap(peakRow, :), peakCol);
    [offsetY, fittedPeakY] = quadratic_offset(validMap(:, peakCol), peakRow);
    peakValue = max([peakValue, fittedPeakX, fittedPeakY]);

    matchStartX = sx1 + (peakCol - 1) + offsetX;
    matchStartY = sy1 + (peakRow - 1) + offsetY;
    match.dx = (matchStartX - tplX) / scaleX;
    match.dy = (matchStartY - r1) / scaleY;
    match.peak = double(peakValue);
    match.psr = compute_psr(validMap, peakRow, peakCol, ...
        cfg.quality.psrExclusionRadius);
    match.subpixelX = offsetX / scaleX;
    match.subpixelY = offsetY / scaleY;
end


function index = full_to_small(fullIndex, scale, upperBound)
    index = round((double(fullIndex) - 1) * scale) + 1;
    index = min(max(index, 1), upperBound);
end


function [offset, fittedPeak] = quadratic_offset(values, index)
    offset = 0;
    fittedPeak = double(values(index));
    if index <= 1 || index >= numel(values)
        return;
    end
    left = double(values(index - 1));
    center = double(values(index));
    right = double(values(index + 1));
    denominator = left - 2 * center + right;
    if ~isfinite(denominator) || denominator >= -eps(max(1, abs(center)))
        return;
    end
    offset = 0.5 * (left - right) / denominator;
    offset = max(-0.75, min(0.75, offset));
    fittedPeak = center - 0.25 * (left - right) * offset;
end


function value = compute_psr(corrMap, peakRow, peakCol, radius)
    mask = true(size(corrMap));
    rowRange = max(1, peakRow - radius):min(size(corrMap, 1), peakRow + radius);
    colRange = max(1, peakCol - radius):min(size(corrMap, 2), peakCol + radius);
    mask(rowRange, colRange) = false;
    sidelobes = double(corrMap(mask));
    sidelobes = sidelobes(isfinite(sidelobes));
    if numel(sidelobes) < 2
        value = NaN;
        return;
    end
    sigma = std(sidelobes, 0);
    if sigma <= eps(max(1, abs(mean(sidelobes))))
        value = Inf;
    else
        value = (double(corrMap(peakRow, peakCol)) - mean(sidelobes)) / sigma;
    end
end


function patch = centered_patch(img, width)
    x1 = floor((size(img, 2) - width) / 2) + 1;
    x2 = x1 + width - 1;
    patch = img(:, x1:x2, :);
end


function [canvas, y1, y2] = place_patch(canvas, patch, baseRow, yShift, x1, x2)
    integerShift = round(yShift);
    fractionalShift = yShift - integerShift;
    if abs(fractionalShift) < 1e-6
        shiftedPatch = patch;
        y1 = baseRow + integerShift;
    else
        paddedSize = size(patch);
        paddedSize(1) = paddedSize(1) + 2;
        shiftedPatch = zeros(paddedSize, 'like', patch);
        shiftedPatch(2:end - 1, :, :) = patch;
        shiftedPatch = imtranslate(shiftedPatch, [0, fractionalShift], ...
            'linear', 'OutputView', 'same', 'FillValues', 0);
        y1 = baseRow + integerShift - 1;
    end
    y2 = y1 + size(shiftedPatch, 1) - 1;
    if y1 < 1 || y2 > size(canvas, 1)
        error('stitch_tube_strip:VerticalCanvasOverflow', ...
            'Accumulated vertical shift exceeded the configured canvas bound.');
    end
    canvas(y1:y2, x1:x2, :) = shiftedPatch;
end


function labels = quality_label(passed, fallback)
    labels = repmat("failed", size(passed));
    labels(passed) = "ok";
    labels(fallback) = "fallback";
end





