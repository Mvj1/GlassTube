function result = analyze_endface_canny(imagePath, cfg)
%ANALYZE_ENDFACE_CANNY Robust, conservative Canny-based end-face analysis.
% RESULT = ANALYZE_ENDFACE_CANNY(IMAGEPATH, CFG)
% Required: cfg.scale.endPxPerMm. Optional settings are under cfg.endface.
% Full-resolution contours always use MATLAB edge(...,'Canny',...).

versionTag = 'analyze_endface_canny-1.0.0';
if nargin < 2 || ~isstruct(cfg)
    error('GlassTube:Endface:InvalidConfig', ...
        'cfg must contain cfg.endface and cfg.scale.endPxPerMm.');
end
imagePath = char(string(imagePath));
if isempty(imagePath) || ~isfile(imagePath)
    error('GlassTube:Endface:MissingImage', 'Missing end-face image: %s', imagePath);
end
if ~isfield(cfg,'scale') || ~isstruct(cfg.scale) || ...
        ~isfield(cfg.scale,'endPxPerMm') || ...
        ~isscalar(cfg.scale.endPxPerMm) || ~isfinite(cfg.scale.endPxPerMm) || ...
        cfg.scale.endPxPerMm <= 0
    error('GlassTube:Endface:InvalidConfig', ...
        'cfg.scale.endPxPerMm must be a finite positive scalar.');
end
opts = default_options();
if isfield(cfg,'endface') && ~isempty(cfg.endface)
    if ~isstruct(cfg.endface)
        error('GlassTube:Endface:InvalidConfig','cfg.endface must be a struct.');
    end
    opts = merge_struct(opts,cfg.endface);
end
opts = aliases(opts,cfg);
validate_options(opts);
pxPerMm = double(cfg.scale.endPxPerMm);
sourceHash = sha256_file(imagePath);
fpData = struct('version',versionTag,'endPxPerMm',pxPerMm, ...
    'options',rmfield(opts,'cache'));
configHash = sha256_text(jsonencode(canonicalize(fpData)));
cacheFile = cache_path(imagePath,opts.cache.file);
if opts.cache.enable
    [cached,hit] = load_cache(cacheFile,sourceHash,configHash,versionTag);
    if hit
        cached.cache.hit = true;
        result = cached;
        return
    end
end
try
    raw = imread(imagePath);
catch cause
    error('GlassTube:Endface:ImageReadFailed', ...
        'Cannot read "%s": %s',imagePath,cause.message);
end
gray = to_uint8_gray(raw);
imageSize = size(gray);
shortSide = min(imageSize(1:2));
if shortSide < 128
    error('GlassTube:Endface:ImageTooSmall','Image short side is only %d px.',shortSide);
end

% Coarse localization: approximately 1024 px, CLAHE, then PhaseCode.
coarseScale = min(1,opts.coarse.targetShortSide/shortSide);
small = imresize(gray,coarseScale,'bicubic');
small = adapthisteq(small,'NumTiles',opts.coarse.claheNumTiles, ...
    'ClipLimit',opts.coarse.claheClipLimit,'Distribution','rayleigh');
smallSide = min(size(small,1),size(small,2));
innerRange = radius_range(smallSide*opts.coarse.innerRadiusFraction);
outerRange = radius_range(smallSide*opts.coarse.outerRadiusFraction);
[ci,ri,mi] = circle_candidates(small,innerRange,'dark',opts.coarse);
[co,ro,mo] = circle_candidates(small,outerRange,'bright',opts.coarse);
if isempty(ri)
    error('GlassTube:Endface:InnerCircleNotFound', ...
        'No dark PhaseCode inner circle in [%d %d] px.',innerRange(1),innerRange(2));
end
if isempty(ro)
    error('GlassTube:Endface:OuterCircleNotFound', ...
        'No bright PhaseCode outer circle in [%d %d] px.',outerRange(1),outerRange(2));
end
pair = choose_pair(ci,ri,mi,co,ro,mo,smallSide,opts.coarse);
innerSeedCenter = (pair.innerCenter-0.5)/coarseScale+0.5;
outerSeedCenter = (pair.outerCenter-0.5)/coarseScale+0.5;
innerSeedRadius = pair.innerRadius/coarseScale;
outerSeedRadius = pair.outerRadius/coarseScale;

% Mandatory built-in full-resolution Canny. Do not replace with manual Canny.
if isempty(opts.canny.threshold)
    [edgeMask,cannyThreshold] = edge(gray,'Canny',[],opts.canny.sigma);
else
    [edgeMask,cannyThreshold] = edge(gray,'Canny', ...
        opts.canny.threshold,opts.canny.sigma);
end
innerTrace = trace_contour(gray,edgeMask,innerSeedCenter,innerSeedRadius, ...
    +1,opts.trace,'inner');
outerTrace = trace_contour(gray,edgeMask,outerSeedCenter,outerSeedRadius, ...
    -1,opts.trace,'outer');
innerFit = robust_circle(innerTrace.pointsPx,opts.fit);
outerFit = robust_circle(outerTrace.pointsPx,opts.fit);
centerDistance = hypot(innerFit.centerPx(1)-outerFit.centerPx(1), ...
    innerFit.centerPx(2)-outerFit.centerPx(2));
if outerFit.meanRadiusPx <= innerFit.meanRadiusPx
    error('GlassTube:Endface:InvalidCircleGeometry', ...
        'Outer radius %.3f px is not greater than inner radius %.3f px.', ...
        outerFit.meanRadiusPx,innerFit.meanRadiusPx);
end

% Conservative radius uses inward residuals and can never exceed mean radius.
innerR = hypot(innerTrace.pointsPx(:,1)-innerFit.centerPx(1), ...
    innerTrace.pointsPx(:,2)-innerFit.centerPx(2));
if opts.conservative.useMinimum
    boundaryRadius = min(innerR);
    conservativeMethod = 'minimum';
else
    boundaryRadius = quantile_local(innerR,opts.conservative.quantile);
    conservativeMethod = sprintf('quantile-%.6g',opts.conservative.quantile);
end
safetyPx = opts.conservative.safetyMarginMm*pxPerMm + ...
    opts.conservative.safetyMarginPx;
conservativeRadius = max(0,min(innerFit.meanRadiusPx,boundaryRadius)-safetyPx);
innerFit.radiusQuantilePx = quantile_local(innerR,opts.conservative.quantile);
innerFit.minimumRadiusPx = min(innerR);
innerFit.maximumRadiusPx = max(innerR);
innerFit.radialResidualPx = innerR-innerFit.meanRadiusPx;
innerFit.roundnessPx = quantile_local(innerR,0.99)-quantile_local(innerR,0.01);
innerFit.fullRangeRoundnessPx = max(innerR)-min(innerR);
outerR = hypot(outerTrace.pointsPx(:,1)-outerFit.centerPx(1), ...
    outerTrace.pointsPx(:,2)-outerFit.centerPx(2));
outerFit.roundnessPx = quantile_local(outerR,0.99)-quantile_local(outerR,0.01);
outerFit.fullRangeRoundnessPx = max(outerR)-min(outerR);
quality = quality_metrics(innerTrace,outerTrace,innerFit,outerFit, ...
    centerDistance,shortSide,opts.quality);
if ~quality.pass
    error('GlassTube:Endface:QualityFailed','%s',quality.message);
end

result = struct();
result.algorithmVersion = versionTag;
result.source = struct('path',imagePath,'sha256',sourceHash, ...
    'imageSize',imageSize(1:2));
result.scale = struct('endPxPerMm',pxPerMm);
result.centerPx = innerFit.centerPx;
result.centerMm = innerFit.centerPx/pxPerMm;
result.meanRadiusPx = innerFit.meanRadiusPx;
result.meanRadiusMm = innerFit.meanRadiusPx/pxPerMm;
result.meanDiameterPx = 2*innerFit.meanRadiusPx;
result.meanDiameterMm = 2*innerFit.meanRadiusPx/pxPerMm;
result.conservativeRadiusPx = conservativeRadius;
result.conservativeRadiusMm = conservativeRadius/pxPerMm;
result.conservativeDiameterPx = 2*conservativeRadius;
result.conservativeDiameterMm = 2*conservativeRadius/pxPerMm;
result.conservativeMethod = conservativeMethod;
result.safetyMarginPx = safetyPx;
result.safetyMarginMm = safetyPx/pxPerMm;
result.roundnessPx = innerFit.roundnessPx;
result.roundnessMm = innerFit.roundnessPx/pxPerMm;
result.coverage = innerTrace.coverage;
result.innerDiaPx = result.meanDiameterPx;
result.innerDiaMm = result.meanDiameterMm;
result.conservativeInnerDiaPx = result.conservativeDiameterPx;
result.conservativeInnerDiaMm = result.conservativeDiameterMm;
result.inner = innerFit;
result.outer = outerFit;
result.innerTrace = compact_trace(innerTrace);
result.outerTrace = compact_trace(outerTrace);
result.quality = quality;
result.coarse = pair;
result.canny = struct('method','MATLAB edge(...,''Canny'',...)', ...
    'sigma',opts.canny.sigma,'thresholdConfigured',opts.canny.threshold, ...
    'thresholdUsed',cannyThreshold);
result.cache = struct('hit',false,'file',cacheFile, ...
    'sourceHash',sourceHash,'configHash',configHash);
if opts.debug.enable
    result.debug = struct('coarseImage',small,'edgeMask',edgeMask, ...
        'innerTrace',innerTrace,'outerTrace',outerTrace, ...
        'innerCandidates',struct('centers',ci,'radii',ri,'metrics',mi), ...
        'outerCandidates',struct('centers',co,'radii',ro,'metrics',mo));
else
    result.debug = struct();
end
if opts.cache.enable
    cacheRecord = struct('version',versionTag,'sourceHash',sourceHash, ...
        'configHash',configHash,'result',result);
    save_cache(cacheFile,cacheRecord);
end
end

function o = default_options()
o.coarse = struct('targetShortSide',1024,'claheNumTiles',[8 8], ...
    'claheClipLimit',0.01,'innerRadiusFraction',[0.20 0.30], ...
    'outerRadiusFraction',[0.29 0.38],'sensitivity',0.98, ...
    'edgeThreshold',0.10,'maxCandidates',12, ...
    'centerToleranceFraction',0.035,'minimumWallFraction',0.025);
o.canny = struct('threshold',[],'sigma',1.5);
o.trace = struct('angleCount',1440,'searchHalfWidthFraction',0.018, ...
    'minimumSearchHalfWidthPx',20,'cannyRadialTolerancePx',2, ...
    'gradientScaleQuantile',0.90,'cannyScore',3.0,'gradientScore',1.5, ...
    'radiusPriorScore',0.35,'unsupportedPenalty',3.5, ...
    'transitionPenalty',0.20,'maxJumpPx',5,'minimumGradientScore',0.05, ...
    'strongGradientScore',0.45,'continuityWindow',11, ...
    'continuityOutlierPx',5,'subpixelLimitPx',0.75);
o.fit = struct('maxIterations',30,'tukeyConstant',4.685, ...
    'minimumScalePx',0.10,'minimumInlierTolerancePx',0.75, ...
    'convergenceTolerance',1e-7);
o.conservative = struct('quantile',0.01,'useMinimum',false, ...
    'safetyMarginMm',0.02,'safetyMarginPx',0);
o.quality = struct('minInnerCoverage',0.80,'minOuterCoverage',0.65, ...
    'maxAngularGapDeg',20,'minFitInlierFraction',0.80, ...
    'maxRobustRmseFraction',0.003,'maxCenterOffsetFraction',0.025, ...
    'maxRoundnessFraction',0.020);
o.cache = struct('enable',true,'file','');
o.debug = struct('enable',false);
end

function o = aliases(o,cfg)
if ~isfield(cfg,'endface') || ~isstruct(cfg.endface), return; end
e = cfg.endface;
if isfield(e,'safetyMarginMm'),o.conservative.safetyMarginMm=e.safetyMarginMm;end
if isfield(e,'safetyMarginPx'),o.conservative.safetyMarginPx=e.safetyMarginPx;end
if isfield(e,'debug') && islogical(e.debug) && isscalar(e.debug),o.debug.enable=e.debug;end
if isfield(e,'cacheFile'),o.cache.file=e.cacheFile;end
if isfield(e,'cacheEnable'),o.cache.enable=e.cacheEnable;end
end

function validate_options(o)
try
    validateattributes(o.coarse.targetShortSide,{'numeric'},{'scalar','finite','>=',256});
    validateattributes(o.coarse.claheNumTiles,{'numeric'},{'vector','numel',2,'integer','positive'});
    validateattributes(o.coarse.claheClipLimit,{'numeric'},{'scalar','finite','positive','<=',1});
    fraction_range(o.coarse.innerRadiusFraction);
    fraction_range(o.coarse.outerRadiusFraction);
    validateattributes(o.coarse.sensitivity,{'numeric'},{'scalar','>',0,'<=',1});
    validateattributes(o.coarse.edgeThreshold,{'numeric'},{'scalar','>=',0,'<=',1});
    validateattributes(o.canny.sigma,{'numeric'},{'scalar','finite','positive'});
    if ~isempty(o.canny.threshold)
        validateattributes(o.canny.threshold,{'numeric'},{'vector','numel',2,'>=',0,'<=',1});
        assert(o.canny.threshold(1)<o.canny.threshold(2));
    end
    validateattributes(o.trace.angleCount,{'numeric'},{'scalar','integer','>=',180});
    validateattributes(o.trace.maxJumpPx,{'numeric'},{'scalar','integer','positive'});
    validateattributes(o.conservative.quantile,{'numeric'},{'scalar','>=',0,'<=',0.5});
    validateattributes(o.conservative.safetyMarginMm,{'numeric'},{'scalar','nonnegative'});
    validateattributes(o.conservative.safetyMarginPx,{'numeric'},{'scalar','nonnegative'});
catch cause
    error('GlassTube:Endface:InvalidConfig','Invalid cfg.endface: %s',cause.message);
end
end

function fraction_range(v)
validateattributes(v,{'numeric'},{'vector','numel',2,'finite','>',0,'<',0.5});
assert(v(1)<v(2));
end

function out = merge_struct(base,override)
out=base; names=fieldnames(override);
for k=1:numel(names)
    n=names{k}; v=override.(n);
    if isfield(out,n)&&isstruct(out.(n))&&isstruct(v)&&isscalar(v)
        out.(n)=merge_struct(out.(n),v);
    else
        out.(n)=v;
    end
end
end

function out = canonicalize(in)
if isstruct(in)
    in=orderfields(in); names=fieldnames(in);
    for j=1:numel(in)
        for k=1:numel(names),in(j).(names{k})=canonicalize(in(j).(names{k}));end
    end
elseif iscell(in)
    for k=1:numel(in),in{k}=canonicalize(in{k});end
end
out=in;
end

function p = cache_path(imagePath,configured)
if ~isempty(configured),p=char(string(configured));return;end
[f,n]=fileparts(imagePath); p=fullfile(f,[n '.endface_canny_cache.mat']);
end

function [r,hit] = load_cache(file,sourceHash,configHash,versionTag)
r=[];hit=false;if ~isfile(file),return;end
try
    s=load(file,'cacheRecord');
    if ~isfield(s,'cacheRecord'),return;end
    c=s.cacheRecord;
    if all(isfield(c,{'version','sourceHash','configHash','result'})) && ...
            strcmp(c.version,versionTag)&&strcmp(c.sourceHash,sourceHash)&& ...
            strcmp(c.configHash,configHash)
        r=c.result;hit=true;
    end
catch
    r=[];hit=false;
end
end

function save_cache(file,cacheRecord)
folder=fileparts(file);
if ~isempty(folder)&&~isfolder(folder)
    [ok,msg]=mkdir(folder);
    if ~ok,warning('GlassTube:Endface:CacheWriteFailed','%s',msg);return;end
end
tmp=[file '.' char(java.util.UUID.randomUUID()) '.tmp.mat'];
cleanup=onCleanup(@()delete_if_file(tmp));
try
    save(tmp,'cacheRecord','-v7.3');
    [ok,msg]=movefile(tmp,file,'f');
    if ~ok,warning('GlassTube:Endface:CacheWriteFailed','%s',msg);end
catch cause
    warning('GlassTube:Endface:CacheWriteFailed','%s',cause.message);
end
end

function delete_if_file(p)
if isfile(p),delete(p);end
end

function h = sha256_file(p)
fid=fopen(p,'rb');
if fid<0,error('GlassTube:Endface:ImageReadFailed','Cannot hash: %s',p);end
cleanup=onCleanup(@()fclose(fid));
md=java.security.MessageDigest.getInstance('SHA-256');
while true
    b=fread(fid,1024*1024,'*uint8');if isempty(b),break;end;md.update(b);
end
h=hex_digest(md.digest());
end

function h = sha256_text(t)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(unicode2native(char(t),'UTF-8'));h=hex_digest(md.digest());
end

function h = hex_digest(d)
b=typecast(d,'uint8');h=lower(reshape(dec2hex(b,2).',1,[]));
end

function g = to_uint8_gray(im)
if ndims(im)==3,im=im2gray(im);end
if isa(im,'uint8'),g=im;
elseif isinteger(im),g=im2uint8(im);
else
    v=im(isfinite(im));
    if isempty(v),error('GlassTube:Endface:InvalidImage','No finite pixels.');end
    g=im2uint8(mat2gray(im,double([min(v),max(v)])));
end
end

function r = radius_range(v)
r=round(v);r(1)=max(5,r(1));r(2)=max(r(1)+2,r(2));
end

function [c,r,m] = circle_candidates(im,range,polarity,o)
sens=unique(min(0.995,[o.sensitivity o.sensitivity+0.01 o.sensitivity+0.015]),'stable');
c=zeros(0,2);r=zeros(0,1);m=zeros(0,1);
for s=sens
    [c,r,m]=imfindcircles(im,range,'ObjectPolarity',polarity, ...
        'Method','PhaseCode','Sensitivity',s,'EdgeThreshold',o.edgeThreshold);
    if ~isempty(r),break;end
end
n=min(numel(r),o.maxCandidates);c=c(1:n,:);r=r(1:n);m=m(1:n);
end

function p = choose_pair(ci,ri,mi,co,ro,mo,side,o)
maxD=o.centerToleranceFraction*side;minWall=o.minimumWallFraction*side;
best=-inf;ii=0;oo=0;
for i=1:numel(ri)
    for j=1:numel(ro)
        d=hypot(ci(i,1)-co(j,1),ci(i,2)-co(j,2));gap=ro(j)-ri(i);
        if d<=maxD&&gap>minWall
            score=mi(i)+mo(j)-0.5*d/maxD;
            if score>best,best=score;ii=i;oo=j;end
        end
    end
end
if ii==0
    error('GlassTube:Endface:NoConcentricCirclePair', ...
        'No nearby-center pair with outer radius greater than inner radius.');
end
p=struct('innerCenter',ci(ii,:),'innerRadius',ri(ii),'innerMetric',mi(ii), ...
    'outerCenter',co(oo,:),'outerRadius',ro(oo),'outerMetric',mo(oo), ...
    'centerDistancePx',hypot(ci(ii,1)-co(oo,1),ci(ii,2)-co(oo,2)), ...
    'score',best);
end

function t = trace_contour(gray,edges,center,seedR,polarity,o,name)
[rows,cols]=size(gray);short=min(rows,cols);
half=max(o.minimumSearchHalfWidthPx,round(o.searchHalfWidthFraction*short));
r0=max(3,floor(seedR-half));
r1=min(ceil(seedR+half),floor(min([center(1)-1,cols-center(1),center(2)-1,rows-center(2)])));
if r1-r0<8,error('GlassTube:Endface:TraceSearchInvalid','%s search band invalid.',name);end
angles=(0:o.angleCount-1)*(2*pi/o.angleCount);rv=single((r0:r1).');
xq=single(center(1))+rv.*single(cos(angles));
yq=single(center(2))+rv.*single(sin(angles));
intensity=interp2(single(gray),xq,yq,'linear',NaN);
es=interp2(single(edges),xq,yq,'nearest',0)>0.5;
grad=zeros(size(intensity),'single');
grad(2:end-1,:)=0.5*(intensity(3:end,:)-intensity(1:end-2,:));
grad(1,:)=intensity(2,:)-intensity(1,:);grad(end,:)=intensity(end,:)-intensity(end-1,:);
polar=single(polarity)*grad;positive=polar(isfinite(polar)&polar>0);
if isempty(positive),error('GlassTube:Endface:TraceGradientMissing','No %s polarity gradient.',name);end
gscale=max(single(1e-6),single(quantile_local(positive,o.gradientScaleQuantile)));
ge=min(2,max(0,polar/gscale));
near=conv2(single(es),ones(2*o.cannyRadialTolerancePx+1,1,'single'),'same')>0;
prior=-((double(rv)-seedR)/max(half,1)).^2;
score=o.gradientScore*ge+o.cannyScore*single(near)+o.radiusPriorScore*single(prior);
score(~near)=score(~near)-o.unsupportedPenalty;score(~isfinite(intensity))=-inf;
[idx,pathScore]=radial_dp(score,o.maxJumpPx,o.transitionPenalty);
lin=sub2ind(size(score),idx(:),(1:o.angleCount).');
selected=double(rv(idx));selGrad=double(ge(lin));selCanny=near(lin);
localMed=movmedian(selected,o.continuityWindow,'omitmissing','Endpoints','shrink');
dev=abs(selected-localMed);
observed=selCanny&selGrad>=o.minimumGradientScore& ...
    (dev<=o.continuityOutlierPx|selGrad>=o.strongGradientScore);
offset=zeros(o.angleCount,1);
for a=1:o.angleCount
    k=idx(a);
    if ~observed(a)||k<=1||k>=numel(rv),continue;end
    y1=double(polar(k-1,a));y2=double(polar(k,a));y3=double(polar(k+1,a));
    den=y1-2*y2+y3;
    if isfinite(den)&&abs(den)>eps&&y2>0
        offset(a)=max(-o.subpixelLimitPx,min(o.subpixelLimitPx,0.5*(y1-y3)/den));
    end
end
refined=selected+offset;aa=angles(observed).';rr=refined(observed);
points=[center(1)+rr.*cos(aa),center(2)+rr.*sin(aa)];
if size(points,1)<30
    error('GlassTube:Endface:InsufficientContour','Only %d %s points.',size(points,1),name);
end
t=struct('name',name,'seedCenterPx',center,'seedRadiusPx',seedR, ...
    'searchRadiusPx',[r0 r1],'anglesRad',angles(:),'radiusPx',refined, ...
    'observed',observed,'selectedCanny',selCanny, ...
    'selectedGradientScore',selGrad,'continuityDeviationPx',dev, ...
    'subpixelOffsetPx',offset,'pointsPx',points,'coverage',mean(observed), ...
    'maxAngularGapDeg',circular_gap(observed)*360/o.angleCount, ...
    'pathScore',double(pathScore),'gradientScale',double(gscale));
end

function [path,finalScore] = radial_dp(score,jump,penalty)
[nr,nc]=size(score);[~,start]=max(max(score,[],1));order=[start:nc 1:start-1];s=score(:,order);
cost=-inf(nr,nc,'single');parent=zeros(nr,nc,'int16');cost(:,1)=s(:,1);
for c=2:nc
    prev=cost(:,c-1);best=-inf(nr,1,'single');bestD=zeros(nr,1,'int16');
    for d=-jump:jump
        shifted=-inf(nr,1,'single');
        if d>=0,shifted(1+d:end)=prev(1:end-d);else,shifted(1:end+d)=prev(1-d:end);end
        cand=shifted-single(penalty*d*d);improve=cand>best;
        best(improve)=cand(improve);bestD(improve)=int16(d);
    end
    cost(:,c)=s(:,c)+best;parent(:,c)=bestD;
end
[finalScore,row]=max(cost(:,end));rot=zeros(nc,1);rot(end)=row;
for c=nc:-1:2
    row=row-double(parent(row,c));row=min(nr,max(1,row));rot(c-1)=row;
end
path=zeros(nc,1);path(order)=rot;
end

function g = circular_gap(observed)
observed=logical(observed(:));
if all(observed),g=0;return;elseif ~any(observed),g=numel(observed);return;end
runs=regionprops(~[observed;observed],'Area');g=min(numel(observed),max([runs.Area]));
end

function f = robust_circle(points,o)
x=double(points(:,1));y=double(points(:,2));
A=[2*x 2*y ones(size(x))];b=x.^2+y.^2;
if numel(x)<3||rank(A)<3,error('GlassTube:Endface:CircleFitFailed','Degenerate contour.');end
p=A\b;cx=p(1);cy=p(2);r=sqrt(max(eps,p(3)+cx^2+cy^2));w=ones(size(x));
for iteration=1:o.maxIterations
    dx=x-cx;dy=y-cy;dist=max(hypot(dx,dy),eps);res=dist-r;
    scale=max(o.minimumScalePx,1.4826*median(abs(res-median(res))));
    u=res/(o.tukeyConstant*scale);w=(abs(u)<1).*(1-u.^2).^2;
    if nnz(w)>2
        J=[(cx-x)./dist (cy-y)./dist -ones(size(x))];sw=sqrt(w);
        JW=J.*sw;
        step=-(JW.'*JW)\(JW.'*(res.*sw));
    else
        step=zeros(3,1);
    end
    if any(~isfinite(step)),error('GlassTube:Endface:CircleFitFailed','Singular fit.');end
    cx=cx+step(1);cy=cy+step(2);r=r+step(3);
    if norm(step)<=o.convergenceTolerance*max(1,r),break;end
end
dist=hypot(x-cx,y-cy);res=dist-r;
scale=max(o.minimumScalePx,1.4826*median(abs(res-median(res))));
tol=max(o.minimumInlierTolerancePx,o.tukeyConstant*scale);inlier=abs(res)<=tol;
f=struct('centerPx',[cx cy],'cx',cx,'cy',cy,'meanRadiusPx',r,'r',r, ...
    'residualPx',res,'robustScalePx',scale, ...
    'robustRmsePx',sqrt(sum(w.*res.^2)/max(sum(w),eps)), ...
    'rmsePx',sqrt(mean(res.^2)),'medianAbsoluteResidualPx',median(abs(res)), ...
    'p95AbsoluteResidualPx',quantile_local(abs(res),0.95), ...
    'inlierMask',inlier,'inlierFraction',mean(inlier), ...
    'pointCount',numel(x),'iterations',iteration);
end

function q = quality_metrics(it,ot,ifit,ofit,centerD,side,o)
checks=struct('innerCoverage',it.coverage>=o.minInnerCoverage, ...
    'outerCoverage',ot.coverage>=o.minOuterCoverage, ...
    'innerAngularGap',it.maxAngularGapDeg<=o.maxAngularGapDeg, ...
    'outerAngularGap',ot.maxAngularGapDeg<=o.maxAngularGapDeg, ...
    'innerInlierFraction',ifit.inlierFraction>=o.minFitInlierFraction, ...
    'outerInlierFraction',ofit.inlierFraction>=o.minFitInlierFraction, ...
    'innerRmse',ifit.robustRmsePx<=o.maxRobustRmseFraction*side, ...
    'outerRmse',ofit.robustRmsePx<=o.maxRobustRmseFraction*side, ...
    'centerAgreement',centerD<=o.maxCenterOffsetFraction*side, ...
    'innerRoundness',ifit.roundnessPx<=o.maxRoundnessFraction*side, ...
    'outerRoundness',ofit.roundnessPx<=o.maxRoundnessFraction*side);
names=fieldnames(checks);failed=names(~cellfun(@(n)checks.(n),names));
q=struct('pass',isempty(failed),'failedChecks',{failed},'checks',checks, ...
    'innerCoverage',it.coverage,'outerCoverage',ot.coverage, ...
    'innerMaxAngularGapDeg',it.maxAngularGapDeg, ...
    'outerMaxAngularGapDeg',ot.maxAngularGapDeg, ...
    'innerInlierFraction',ifit.inlierFraction,'outerInlierFraction',ofit.inlierFraction, ...
    'innerRobustRmsePx',ifit.robustRmsePx,'outerRobustRmsePx',ofit.robustRmsePx, ...
    'centerDistancePx',centerD,'innerRoundnessPx',ifit.roundnessPx, ...
    'outerRoundnessPx',ofit.roundnessPx,'thresholds',o);
if q.pass
    q.message='End-face analysis passed all quality checks.';
else
    q.message=sprintf(['Failed: %s. inner/outer coverage %.3f/%.3f, gaps %.2f/%.2f deg, ', ...
        'RMSE %.3f/%.3f px, center distance %.3f px.'],strjoin(failed,', '), ...
        it.coverage,ot.coverage,it.maxAngularGapDeg,ot.maxAngularGapDeg, ...
        ifit.robustRmsePx,ofit.robustRmsePx,centerD);
end
end

function c = compact_trace(t)
c=rmfield(t,{'selectedCanny','selectedGradientScore', ...
    'continuityDeviationPx','subpixelOffsetPx'});
end

function q = quantile_local(data,p)
data=sort(double(data(isfinite(data))));
if isempty(data),q=NaN;return;elseif isscalar(data),q=data;return;end
pos=1+(numel(data)-1)*p;lo=floor(pos);hi=ceil(pos);
q=data(lo)+(pos-lo)*(data(hi)-data(lo));
end

