function summary = review_side_robustness(outputDir)
%REVIEW_SIDE_ROBUSTNESS Stitch and annotate every remaining side-view group.
rootDir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(outputDir)
    outputDir = fullfile(rootDir, 'results', 'robustness_review');
end
if ~isfolder(outputDir), mkdir(outputDir); end
cfg = glass_tube_default_config();

endCfg = struct('scale', struct('endPxPerMm', cfg.scale.endPxPerMm), ...
    'endface', cfg.endface);
endCfg.endface.cache.file = fullfile(outputDir, 'reference_endface_cache.mat');
referenceEnd = analyze_endface_canny(cfg.input.leftEndface, endCfg);
expectedWallMm = (referenceEnd.outer.meanRadiusPx - referenceEnd.inner.meanRadiusPx) / cfg.scale.endPxPerMm;

groups = build_groups(rootDir);
rows = repmat(struct('Group',"",'FrameCount',0,'Status',"",'ObservedFraction',nan, ...
    'TopObservedFraction',nan,'BottomObservedFraction',nan,'MedianConfidence',nan, ...
    'MinPeak',nan,'MedianPeak',nan,'MinPSR',nan,'MedianPSR',nan, ...
    'FallbackCount',nan,'StitchSeconds',nan,'SideSeconds',nan,'OutputFile',"", ...
    'Message',""), numel(groups), 1);

for k = 1:numel(groups)
    group = groups(k);
    fprintf('\n[%d/%d] %s (%d frames)\n', k, numel(groups), group.name, numel(group.files));
    rows(k).Group = string(group.name);
    rows(k).FrameCount = numel(group.files);
    try
        stitchCfg = build_stitch_cfg(cfg, group);
        [strip, transforms, stitchQuality, stitchTiming] = stitch_tube_strip(stitchCfg);
        seams = unique([transforms.OutputX1; transforms.OutputX2]);
        seams = seams(seams > 1 & seams < size(strip,2));
        sideCfg = cfg.side;
        sideCfg.returnDebug = false;
        side = detect_side_inner_walls(strip, sideCfg, cfg.scale.sidePxPerMm, expectedWallMm, seams);
        outputFile = fullfile(outputDir, sanitize_name(group.name) + "_full_review.jpg");
        save_review_figure(strip, side, stitchQuality, group.name, outputFile);

        rows(k).Status = quality_status(stitchQuality);
        rows(k).ObservedFraction = side.quality.observedFraction;
        rows(k).TopObservedFraction = side.quality.topObservedFraction;
        rows(k).BottomObservedFraction = side.quality.bottomObservedFraction;
        rows(k).MedianConfidence = side.quality.medianConfidence;
        rows(k).MinPeak = min(stitchQuality.peak);
        rows(k).MedianPeak = median(stitchQuality.peak);
        rows(k).MinPSR = min(stitchQuality.psr);
        rows(k).MedianPSR = median(stitchQuality.psr);
        rows(k).FallbackCount = stitchQuality.numFallbacks;
        rows(k).StitchSeconds = stitchTiming.total;
        rows(k).SideSeconds = side.timingSeconds;
        rows(k).OutputFile = string(outputFile);
        fprintf('  observed %.2f%%, peak min %.4f, PSR min %.3f, output %s\n', ...
            100*side.quality.observedFraction, min(stitchQuality.peak), min(stitchQuality.psr), outputFile);
        clear strip side transforms stitchQuality;
    catch cause
        rows(k).Status = "FAILED";
        rows(k).Message = string(cause.identifier) + ": " + string(cause.message);
        fprintf(2, '  FAILED: %s\n', rows(k).Message);
        % Export an explicitly non-certified overlay whenever tracking can
        % still be visualized. The original quality gates remain unchanged.
        try
            stitchCfg = build_stitch_cfg(cfg, group);
            [strip, transforms, stitchQuality, stitchTiming] = stitch_tube_strip(stitchCfg);
            seams = unique([transforms.OutputX1; transforms.OutputX2]);
            seams = seams(seams > 1 & seams < size(strip,2));
            diagnosticCfg = cfg.side;
            diagnosticCfg.returnDebug = false;
            diagnosticCfg.minObservedFraction = 0;
            diagnosticCfg.maxInvalidGapCols = inf;
            side = detect_side_inner_walls(strip, diagnosticCfg, cfg.scale.sidePxPerMm, expectedWallMm, seams);
            outputFile = fullfile(outputDir, sanitize_name(group.name) + "_rejected_review.jpg");
            save_review_figure(strip, side, stitchQuality, group.name, outputFile, ...
                "REJECTED: " + string(cause.identifier));
            rows(k).Status = "REJECTED";
            rows(k).ObservedFraction = side.quality.observedFraction;
            rows(k).TopObservedFraction = side.quality.topObservedFraction;
            rows(k).BottomObservedFraction = side.quality.bottomObservedFraction;
            rows(k).MedianConfidence = side.quality.medianConfidence;
            rows(k).MinPeak = min(stitchQuality.peak);
            rows(k).MedianPeak = median(stitchQuality.peak);
            rows(k).MinPSR = min(stitchQuality.psr);
            rows(k).MedianPSR = median(stitchQuality.psr);
            rows(k).FallbackCount = stitchQuality.numFallbacks;
            rows(k).StitchSeconds = stitchTiming.total;
            rows(k).SideSeconds = side.timingSeconds;
            rows(k).OutputFile = string(outputFile);
            clear strip side transforms stitchQuality;
        catch diagnosticCause
            rows(k).Message = rows(k).Message + " | diagnostic export: " + ...
                string(diagnosticCause.identifier) + ": " + string(diagnosticCause.message);
        end
    end
end
summary = struct2table(rows);
writetable(summary, fullfile(outputDir, 'robustness_summary.csv'));
save(fullfile(outputDir, 'robustness_summary.mat'), 'summary', 'groups');
end

function groups = build_groups(rootDir)
rare = fullfile(rootDir, 'raredata');
ranges = [0 21; 22 42; 44 64; 66 86; 91 111; 112 132; 133 153; 154 174];
groups = repmat(struct('name', '', 'dir', '', 'files', strings(1,0)), 13, 1);
groups(1) = full_directory_group(rare, '30.89-1big30.92');
groups(2) = full_directory_group(rare, '30.89-2');
groups(3) = full_directory_group(rare, '3052-2small3092');
groups(4) = suffix_group(rare, 'ImagesAndVideos', 'IAV_04-23', 4, 23);
groups(5) = suffix_group(rare, 'ImagesAndVideos', 'IAV_28-49', 28, 49);
for k = 1:size(ranges,1)
    groups(k+5) = suffix_group(rare, 'ImagesAndVideos2', ...
        sprintf('IAV2_%03d-%03d', ranges(k,1), ranges(k,2)), ranges(k,1), ranges(k,2));
end
end

function group = full_directory_group(rare, name)
dirPath = fullfile(rare, name);
files = dir(fullfile(dirPath, '*.bmp'));
files = natural_file_sort(files);
group = struct('name', name, 'dir', dirPath, 'files', {string({files.name})});
end

function group = suffix_group(rare, dirName, groupName, firstSuffix, lastSuffix)
dirPath = fullfile(rare, dirName);
files = dir(fullfile(dirPath, '*.bmp'));
selected = false(numel(files),1);
for k = 1:numel(files)
    token = regexp(files(k).name, '-(\d+)\.bmp$', 'tokens', 'once');
    if isempty(token), continue; end
    value = str2double(token{1});
    selected(k) = value >= firstSuffix && value <= lastSuffix;
end
files = natural_file_sort(files(selected));
expected = lastSuffix-firstSuffix+1;
if numel(files) ~= expected
    error('GlassTube:ReviewGroupIncomplete', '%s expected %d files but found %d.', groupName, expected, numel(files));
end
group = struct('name', groupName, 'dir', dirPath, 'files', {string({files.name})});
end

function files = natural_file_sort(files)
if isempty(files), return; end
numbers = nan(numel(files),1);
for k = 1:numel(files)
    token = regexp(files(k).name, '-(\d+)\.bmp$', 'tokens', 'once');
    if ~isempty(token), numbers(k) = str2double(token{1}); end
end
if all(isfinite(numbers))
    [~,order] = sort(numbers);
else
    [~,order] = sort({files.name});
end
files = files(order);
end

function out = build_stitch_cfg(cfg, group)
out = struct('imageDir', group.dir, 'extension', {cellstr(group.files)}, ...
    'rotationAngle', cfg.stitch.rotationDeg, 'nominal', cfg.stitch.nominalStepPx, ...
    'search', [180 12], ...
    'downsample', cfg.stitch.registrationScale, ...
    'registration', struct('rowRange', cfg.stitch.registrationRowFraction, ...
        'templateWidth', cfg.stitch.templateWidthPx), ...
    'quality', struct('minPeak', 0.85, ...
        'minPSR', 1.8, 'onFailure', 'nominal', ...
        'psrExclusionRadius', 2), 'verbose', false);
end

function save_review_figure(strip, side, stitchQuality, groupName, outputFile, statusNote)
if nargin < 6, statusNote = ""; end
if ndims(strip)==3, gray=im2gray(strip); else, gray=strip; end
maxWidth = 5200;
scale = min(1, maxWidth/size(gray,2));
fullSmall = imresize(gray, scale, 'bilinear');
rect = side.roiRect;
x1 = max(1,floor(rect(1))+1); y1=max(1,floor(rect(2))+1);
x2 = min(size(gray,2),x1+round(rect(3))-1); y2=min(size(gray,1),y1+round(rect(4))-1);
roi = gray(y1:y2,x1:x2);
roiScale = min(1,maxWidth/size(roi,2));
roiSmall = imresize(roi,roiScale,'bilinear');
fullX = (side.roiOrigin(2)+(0:numel(side.topPx)-1))*scale;
fullTop = (side.roiOrigin(1)-1+side.topPx)*scale;
fullBottom = (side.roiOrigin(1)-1+side.bottomPx)*scale;
roiX = (1:numel(side.topPx))*roiScale;

fig=figure('Visible','off','Color','w','Position',[20 20 2600 1250]);
tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax1=nexttile; imshow(fullSmall,[],'Parent',ax1); hold(ax1,'on');
plot(ax1,fullX,fullTop,'r-','LineWidth',1.6); plot(ax1,fullX,fullBottom,'g-','LineWidth',1.6);
title(ax1,sprintf('%s ? %s | Canny support %.2f%% | peak min %.4f | PSR min %.3f | fallback %d', ...
    groupName, status_label(statusNote), 100*side.quality.observedFraction, min(stitchQuality.peak), min(stitchQuality.psr), stitchQuality.numFallbacks), ...
    'Interpreter','none');
legend(ax1,{'Upper inner wall','Lower inner wall'},'Location','southoutside','Orientation','horizontal');
ax2=nexttile; imshow(roiSmall,[],'Parent',ax2); hold(ax2,'on');
plot(ax2,roiX,side.topPx*roiScale,'r-','LineWidth',1.8);
plot(ax2,roiX,side.bottomPx*roiScale,'g-','LineWidth',1.8);
unsupported=~side.observed;
if any(unsupported)
    scatter(ax2,roiX(unsupported),side.topPx(unsupported)*roiScale,5,[1 .65 0],'filled');
    scatter(ax2,roiX(unsupported),side.bottomPx(unsupported)*roiScale,5,[1 .65 0],'filled');
end
title(ax2,'Full-length side ROI (orange = not directly supported in both walls)');
xlabel(ax2,'Stitched axial direction'); ylabel(ax2,'ROI row');
exportgraphics(fig,outputFile,'Resolution',130,'BackgroundColor','white'); close(fig);
end


function value = status_label(statusNote)
if strlength(statusNote) == 0
    value = "full stitched image";
else
    value = statusNote;
end
end

function value = quality_status(q)
if q.numFallbacks>0, value="PASS_WITH_FALLBACK"; else, value="PASS"; end
end

function value = sanitize_name(name)
value = regexprep(string(name),'[^A-Za-z0-9._-]','_');
end


