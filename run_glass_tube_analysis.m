function result = run_glass_tube_analysis(cfg)
%RUN_GLASS_TUBE_ANALYSIS Measure the conservative straight-rod passage.
%   RESULT = RUN_GLASS_TUBE_ANALYSIS(CFG) performs end-face validation,
%   subpixel stitching, Canny-supported inner-wall tracking and continuous
%   straight-rod optimization. The function never silently substitutes a
%   missing end face.

arguments
    cfg (1,1) struct = glass_tube_default_config()
end

totalTimer = tic;
validate_pipeline_config(cfg);
ensure_output_dirs(cfg);
[leftPath, rightPath, sameContent] = validate_endface_inputs(cfg);

t = tic;
leftCfg = endface_config(cfg, 'left');
leftEnd = analyze_endface_canny(leftPath, leftCfg);
if sameContent
    rightEnd = leftEnd;
    rightEnd.source.path = rightPath;
else
    rightCfg = endface_config(cfg, 'right');
    rightEnd = analyze_endface_canny(rightPath, rightCfg);
end
stageTiming.endfaces = toc(t);
validate_endface_results(leftEnd, rightEnd, cfg.validation);

stitchCfg = build_stitch_config(cfg);
t = tic;
[stripImg, transforms, stitchQuality, stitchTiming] = stitch_tube_strip(stitchCfg);
stageTiming.stitch = toc(t);
seamCols = internal_seams(transforms, size(stripImg, 2));

leftWallMm = (leftEnd.outer.meanRadiusPx - leftEnd.inner.meanRadiusPx) / cfg.scale.endPxPerMm;
rightWallMm = (rightEnd.outer.meanRadiusPx - rightEnd.inner.meanRadiusPx) / cfg.scale.endPxPerMm;
expectedWallMm = median([leftWallMm, rightWallMm]);

t = tic;
side = detect_side_inner_walls(stripImg, cfg.side, cfg.scale.sidePxPerMm, expectedWallMm, seamCols);
stageTiming.sideWalls = toc(t);

t = tic;
rod = solve_max_rod_diameter(side, leftEnd, rightEnd, cfg.rod);
stageTiming.rodFit = toc(t);

result = struct();
result.algorithmVersion = cfg.algorithmVersion;
result.maximumRodDiameterMm = rod.diameterMm;
result.maximumRodRadiusMm = rod.radiusMm;
result.rod = rod;
result.leftEnd = leftEnd;
result.rightEnd = rightEnd;
result.side = side;
result.stitch = struct('transforms', transforms, 'quality', stitchQuality, ...
    'timing', stitchTiming, 'seamColumns', seamCols(:));
result.calibration = cfg.calibration;
result.input = cfg.input;
result.quality = struct('endfaceDiameterDifferenceMm', ...
    abs(leftEnd.conservativeDiameterMm - rightEnd.conservativeDiameterMm), ...
    'sideObservedFraction', side.quality.observedFraction, ...
    'stitchAllPassed', stitchQuality.allPassed, ...
    'endfacesDistinct', ~sameContent, ...
    'certified', ~sameContent && stitchQuality.allPassed);
result.timing = stageTiming;
result.timing.total = toc(totalTimer);

if cfg.output.saveStripImage
    imwrite(stripImg, fullfile(cfg.output.dir, cfg.output.stripName));
end
if cfg.output.savePathCsv
    pathTable = table(side.xMm, side.topPx, side.bottomPx, side.centerMm, ...
        side.localShrinkMm, side.uncertaintyMm, side.observed, side.valid, ...
        'VariableNames', {'Xmm','UpperInnerPx','LowerInnerPx','CenterMm', ...
        'LocalShrinkMm','UncertaintyMm','Observed','Valid'});
    writetable(pathTable, fullfile(cfg.output.dir, cfg.output.pathName));
end
if cfg.output.saveDiagnosticFigure
    save_glass_tube_diagnostic(stripImg, result, cfg, ...
        fullfile(cfg.output.dir, cfg.output.figureName));
end
if cfg.output.saveResultMat
    save(fullfile(cfg.output.dir, cfg.output.resultName), 'result', '-v7.3');
end

fprintf('==============================================\n');
fprintf('Maximum conservative straight-rod diameter: %.3f mm\n', result.maximumRodDiameterMm);
fprintf('Active limit: %s\n', rod.activeLimit);
fprintf('Side Canny observed columns: %.2f%%\n', 100 * side.quality.observedFraction);
fprintf('Certified result: %s\n', string(result.quality.certified));
fprintf('Total processing time: %.3f s\n', result.timing.total);
fprintf('==============================================\n');
end

function validate_pipeline_config(cfg)
required = {'input','scale','stitch','endface','side','rod','validation','output'};
for k = 1:numel(required)
    if ~isfield(cfg, required{k}) || ~isstruct(cfg.(required{k}))
        error('GlassTube:InvalidConfig', 'Missing configuration section cfg.%s.', required{k});
    end
end
validateattributes(cfg.scale.sidePxPerMm, {'numeric'}, {'scalar','finite','positive'});
validateattributes(cfg.scale.endPxPerMm, {'numeric'}, {'scalar','finite','positive'});
if ~isfolder(cfg.input.sideDir)
    error('GlassTube:MissingSideDirectory', 'Missing side-image directory: %s', cfg.input.sideDir);
end
end

function ensure_output_dirs(cfg)
if ~isfolder(cfg.output.dir), mkdir(cfg.output.dir); end
cacheDir = fullfile(cfg.output.dir, 'cache');
if ~isfolder(cacheDir), mkdir(cacheDir); end
end

function [leftPath, rightPath, sameContent] = validate_endface_inputs(cfg)
leftPath = char(string(cfg.input.leftEndface));
rightPath = char(string(cfg.input.rightEndface));
if ~isfile(leftPath)
    error('GlassTube:MissingLeftEndface', 'Missing left end-face image: %s', leftPath);
end
if ~isfile(rightPath)
    error('GlassTube:MissingRightEndface', ['Missing right end-face image: %s. ' ...
        'A left-image copy is intentionally not created.'], rightPath);
end
samePath = strcmpi(canonical_path(leftPath), canonical_path(rightPath));
sameContent = strcmp(file_sha256(leftPath), file_sha256(rightPath));
if cfg.validation.requireDistinctEndfaces && (samePath || sameContent)
    error('GlassTube:DuplicateEndfaces', ...
        'Left and right end-face images must be independent; their path or SHA-256 content is identical.');
end
end

function validate_endface_results(leftEnd, rightEnd, validationCfg)
diameters = [leftEnd.conservativeDiameterMm, rightEnd.conservativeDiameterMm];
if any(diameters < validationCfg.minimumEndfaceDiameterMm | ...
        diameters > validationCfg.maximumEndfaceDiameterMm)
    error('GlassTube:EndfaceDiameterOutOfRange', ...
        'Conservative end-face diameters [%.3f %.3f] mm are outside the configured range.', diameters);
end
if abs(diff(diameters)) > validationCfg.maximumEndfaceDiameterDifferenceMm
    error('GlassTube:EndfaceMismatch', ...
        'Left/right conservative diameters differ by %.3f mm.', abs(diff(diameters)));
end
end

function out = endface_config(cfg, label)
out = struct('scale', struct('endPxPerMm', cfg.scale.endPxPerMm), ...
    'endface', cfg.endface);
out.endface.cache.enable = cfg.endface.cache.enable;
out.endface.cache.file = fullfile(cfg.output.dir, 'cache', sprintf('%s_endface_cache.mat', label));
end

function out = build_stitch_config(cfg)
out = struct();
out.imageDir = cfg.input.sideDir;
out.extension = cfg.input.sidePattern;
out.rotationAngle = cfg.stitch.rotationDeg;
out.nominal = cfg.stitch.nominalStepPx;
out.search = [cfg.stitch.searchRadiusPx, cfg.stitch.maxVerticalShiftPx];
out.downsample = cfg.stitch.registrationScale;
out.registration = struct('rowRange', cfg.stitch.registrationRowFraction, ...
    'templateWidth', cfg.stitch.templateWidthPx);
out.quality = struct('minPeak', cfg.stitch.minimumPeakCorrelation, ...
    'minPSR', cfg.stitch.minimumPsr, ...
    'onFailure', ternary(cfg.stitch.allowNominalFallback, 'nominal', 'error'), ...
    'psrExclusionRadius', 2);
out.verbose = false;
end

function seams = internal_seams(transforms, stripWidth)
seams = unique([transforms.OutputX1; transforms.OutputX2]);
seams = seams(seams > 1 & seams < stripWidth);
end

function value = ternary(condition, trueValue, falseValue)
if condition, value = trueValue; else, value = falseValue; end
end

function pathOut = canonical_path(pathIn)
info = dir(pathIn);
if isempty(info), pathOut = char(string(pathIn)); else, pathOut = info(1).folder + string(filesep) + info(1).name; end
pathOut = char(pathOut);
end

function digest = file_sha256(path)
md = java.security.MessageDigest.getInstance('SHA-256');
fid = fopen(path, 'rb');
if fid < 0, error('GlassTube:FileHashFailed', 'Cannot open %s.', path); end
try
    bytes = fread(fid, inf, '*uint8');
    fclose(fid);
catch cause
    fclose(fid);
    rethrow(cause);
end
md.update(bytes);
digest = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2).', 1, []));
end



